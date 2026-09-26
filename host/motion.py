"""Offline, pairwise motion candidate diagnostics. No trajectory or map accumulation.

T_current_previous maps points from previous rectified left optical coordinates
into current rectified left coordinates. Its inverse is the camera pose change.
PnP fit is not independent motion truth; static-scene and depth errors remain.
"""
import cv2
import numpy as np
from host.corner_quality import sample_supported

FEATURE_QUALITY_LEVEL = .001


def transform_disagreement(first,second):
    """Compare two rigid transforms that describe the same coordinate mapping."""
    first=np.asarray(first,dtype=np.float64);second=np.asarray(second,dtype=np.float64)
    if first.shape!=(4,4) or second.shape!=(4,4) or not np.isfinite(first).all() or not np.isfinite(second).all():
        raise ValueError('Expected two finite 4x4 transforms')
    for transform in (first,second):
        if not np.allclose(transform[3],[0,0,0,1],atol=1e-8):
            raise ValueError('Invalid homogeneous transform')
        rotation=transform[:3,:3]
        if not np.allclose(rotation.T@rotation,np.eye(3),atol=1e-5) or np.linalg.det(rotation)<.99999:
            raise ValueError('Invalid rotation')
    residual=np.linalg.inv(first)@second
    cosine=np.clip((np.trace(residual[:3,:3])-1)/2,-1,1)
    return dict(rotationDegrees=float(np.degrees(np.arccos(cosine))),
                translationCheckerSquare=float(np.linalg.norm(residual[:3,3])))


def fit_motion(xyz, pixels, k, image_size):
    """Robust 3D->2D fit; rejected candidates never expose a transform."""
    xyz=np.asarray(xyz,dtype=np.float64);pixels=np.asarray(pixels,dtype=np.float64)
    k=np.asarray(k,dtype=np.float64)
    if xyz.ndim!=2 or xyz.shape[1]!=3 or pixels.shape!=(len(xyz),2):
        raise ValueError('Expected matching Nx3 and Nx2 arrays')
    if k.shape!=(3,3) or not np.isfinite(k).all() or k[0,0]<=0 or k[1,1]<=0 or not np.allclose(k[2],[0,0,1]):
        raise ValueError('Invalid rectified camera matrix')
    w,h=image_size
    if w<=0 or h<=0:raise ValueError('Invalid image size')
    good=np.isfinite(xyz).all(axis=1)&(xyz[:,2]>0)&np.isfinite(pixels).all(axis=1)
    good&=(pixels[:,0]>=0)&(pixels[:,0]<w)&(pixels[:,1]>=0)&(pixels[:,1]<h)
    xyz=np.ascontiguousarray(xyz[good]);pixels=np.ascontiguousarray(pixels[good])
    result=dict(status='rejected',reason='insufficient_correspondences',correspondences=len(xyz),
                T_current_previous=None,cameraPositionInPrevious=None,unit='checker_square',
                metricAccuracyValidated=False,motionAccuracyValidated=False)
    if len(xyz)<30:return result
    def coverage(p):
        cells=np.floor(p/np.array([w,h])*[4,3]).astype(int)
        return len(np.unique(cells,axis=0)),np.ptp(p,axis=0)/[w,h]
    cells,span=coverage(pixels)
    if cells<4 or min(span)<.15:
        result['reason']='poor_image_coverage';return result
    singular=np.linalg.svd(xyz-xyz.mean(axis=0),compute_uv=False)
    if singular[0]<=0 or singular[1]/singular[0]<.01:
        result['reason']='degenerate_3d_support';return result
    result['nearPlanarSupport']=bool(singular[2]/singular[0]<.01)
    try:
        ok,r,t,inliers=cv2.solvePnPRansac(xyz,pixels,k,None,iterationsCount=200,
            reprojectionError=2.,confidence=.999,flags=cv2.SOLVEPNP_EPNP)
        if not ok or inliers is None or len(inliers)<30:
            result['reason']='pnp_failed';return result
        idx=inliers.ravel()
        r,t=cv2.solvePnPRefineLM(xyz[idx],pixels[idx],k,None,r,t)
        rotation=cv2.Rodrigues(r)[0];transformed=xyz@rotation.T+t.reshape(3)
        projected=cv2.projectPoints(xyz,r,t,k,None)[0].reshape(-1,2)
        error=np.linalg.norm(projected-pixels,axis=1)
        good=np.isfinite(error)&(error<=2)&(transformed[:,2]>0)
        count=int(good.sum());ratio=count/len(xyz)
        result.update(inliers=count,inlierRatio=ratio)
        if count<30 or ratio<.6:
            result['reason']='insufficient_inliers';return result
        cells,span=coverage(pixels[good])
        if cells<4 or min(span)<.15:
            result['reason']='poor_inlier_coverage';return result
        if not np.isfinite(rotation).all() or not np.isfinite(t).all():
            result['reason']='nonfinite_pose';return result
        transform=np.eye(4);transform[:3,:3]=rotation;transform[:3,3]=t.ravel()
        result.update(status='candidate',reason=None,T_current_previous=transform.tolist(),
            cameraPositionInPrevious=(-rotation.T@t).ravel().tolist(),
            inlierReprojectionP95Px=float(np.percentile(error[good],95)),
            rotationDegrees=float(np.linalg.norm(r)*180/np.pi))
    except cv2.error:
        result['reason']='pnp_error'
    return result


def estimate_pair(previous,current,z,valid,k):
    """Track source-depth-supported corners forward/backward, then fit PnP.

    Input images and depth must already be rectified to the same K. Caller owns
    session, calibration, timestamp and static-scene validation.
    """
    if previous.ndim!=2 or previous.dtype!=np.uint8 or current.shape!=previous.shape or current.dtype!=np.uint8:
        raise ValueError('Expected equal uint8 rectified grayscale images')
    if min(previous.shape)<2 or z.shape!=previous.shape or valid.shape!=z.shape or valid.dtype!=bool:
        raise ValueError('Depth and boolean validity must match images')
    supported=valid&np.isfinite(z)&(z>0)
    mask=cv2.erode(supported.astype(np.uint8),np.ones((3,3),np.uint8))*255
    # Bright, high-contrast details must not suppress weaker usable corners.
    # Admission still requires depth support, bidirectional LK and robust PnP.
    points=cv2.goodFeaturesToTrack(previous,maxCorners=800,qualityLevel=FEATURE_QUALITY_LEVEL,minDistance=8,mask=mask)
    stats=dict(detected=0,forwardBackward=0,depthSupported=0)
    def empty(reason):
        return dict(status='rejected',reason=reason,T_current_previous=None,**stats)
    if points is None:return empty('no_features')
    stats['detected']=len(points)
    options=dict(winSize=(21,21),maxLevel=3,criteria=(cv2.TERM_CRITERIA_EPS|cv2.TERM_CRITERIA_COUNT,30,.01))
    target,ok,_=cv2.calcOpticalFlowPyrLK(previous,current,points,None,**options)
    if target is None:return empty('tracking_failed')
    # Do not pass failed/nonfinite tracks into the backward tracker.
    indices=np.flatnonzero(ok.ravel().astype(bool)&np.isfinite(target.reshape(-1,2)).all(axis=1))
    if not len(indices):return empty('tracking_failed')
    source=points[indices];target=target[indices]
    back,ok_back,_=cv2.calcOpticalFlowPyrLK(current,previous,target,None,**options)
    if back is None:return empty('tracking_failed')
    good=ok_back.ravel().astype(bool)&(np.linalg.norm((back-source).reshape(-1,2),axis=1)<=1.)
    source=source.reshape(-1,2)[good];target=target.reshape(-1,2)[good]
    stats['forwardBackward']=len(source)
    depth,good=sample_supported(z,supported,source)
    # Avoid interpolating across substantial depth jumps at object boundaries.
    xy=np.floor(source).astype(int);xy[:,0]=np.clip(xy[:,0],0,z.shape[1]-2);xy[:,1]=np.clip(xy[:,1],0,z.shape[0]-2)
    neighbors=np.array([z[xy[:,1]+dy,xy[:,0]+dx] for dx,dy in [(0,0),(1,0),(0,1),(1,1)]])
    good&=neighbors.max(axis=0)<=1.1*neighbors.min(axis=0)
    source=source[good];target=target[good];depth=depth[good]
    stats['depthSupported']=len(source)
    rays=np.column_stack([source,np.ones(len(source))])@np.linalg.inv(k).T
    xyz=rays*depth[:,None]
    return dict(fit_motion(xyz,target,k,previous.shape[::-1]),**stats)

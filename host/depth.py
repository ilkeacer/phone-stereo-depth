"""Horizontal StereoSGBM; positive optical-axis Z in the calibration's length unit."""
import argparse,json,time
from pathlib import Path
import cv2
import numpy as np


def reproject(disparity,q,valid):
    xyz=cv2.reprojectImageTo3D(disparity.astype(np.float32),q)
    mask=valid & np.isfinite(xyz).all(axis=2) & (xyz[:,:,2]>0)
    xyz[~mask]=np.nan
    return xyz,mask


def consistency_mask(left,right,tolerance=1.):
    yy,xx=np.indices(left.shape)
    finite=np.isfinite(left)
    xr=np.rint(xx-np.where(finite,left,0)).astype(np.int32)
    inside=(xr>=0)&(xr<left.shape[1]);clip=np.clip(xr,0,left.shape[1]-1)
    return finite&inside&np.isfinite(right[yy,clip])&(np.abs(left+right[yy,clip])<=tolerance)


def scaled_rectification(calibration, scale):
    """Keep source K/D and rectified viewing directions; change output pixel units.

    Q maps [u,v,disparity,1], so all three pixel coordinates must be unscaled.
    This changes sampling density, not the calibration or its physical scale.
    """
    if not np.isfinite(scale) or not 0 < scale <= 1:
        raise ValueError('Output scale must be finite and in (0,1]')
    c={k:np.array(calibration[k],copy=True) for k in calibration.keys()}
    source=tuple(int(v) for v in c['size'])
    size=tuple(int(round(v*scale)) for v in source)
    if min(size)<1 or any(abs(n/v-scale)>1e-9 for n,v in zip(size,source)):
        raise ValueError('Scale must produce integral dimensions with equal X/Y scale')
    c['inputSize']=np.array(source);c['size']=np.array(size)
    for i in (1,2):
        c[f'P{i}'][:2]*=scale
        x,y,w,h=c[f'roi{i}']
        low=np.ceil(np.array([x,y])*scale).astype(int)
        high=np.floor(np.array([x+w,y+h])*scale).astype(int)
        c[f'roi{i}']=np.r_[low,np.maximum(0,high-low)]
    c['Q']=c['Q']@np.diag([1/scale,1/scale,1/scale,1.])
    return c


class StereoProcessor:
    def __init__(self,calibration,output_scale=1.):
        self.c=calibration if output_scale==1 else scaled_rectification(calibration,output_scale)
        calibration=self.c
        self.size=tuple(int(v) for v in calibration['size'])
        self.input_size=tuple(int(v) for v in calibration.get('inputSize',calibration['size']))
        if abs(calibration['P2'][1,3])>abs(calibration['P2'][0,3]):
            raise ValueError('Horizontal rectification required')
        self.maps=[cv2.initUndistortRectifyMap(calibration[f'K{i}'],calibration[f'D{i}'],calibration[f'R{i}'],calibration[f'P{i}'],self.size,cv2.CV_32FC1) for i in (1,2)]

    def rectify(self,images):
        if bool(self.c['rotate90']):images=[cv2.rotate(im,cv2.ROTATE_90_CLOCKWISE) for im in images]
        if len(images)!=2 or any(im is None or im.shape[::-1]!=self.input_size for im in images):
            raise ValueError('Image dimensions differ from calibration')
        return [cv2.remap(im,*maps,cv2.INTER_LINEAR) for im,maps in zip(images,self.maps)]

    def compute(self,rect,disparities=128):
        return self._compute(rect,disparities,False)

    def compute_diagnostics(self,rect,disparities=128,*,lr_tolerance=1.,speckle_window=100,uniqueness=10):
        """Append cumulative mask counts; backend count already includes OpenCV filters."""
        return self._compute(rect,disparities,True,lr_tolerance,speckle_window,uniqueness)

    def _compute(self,rect,disparities,diagnostics,lr_tolerance=1.,speckle_window=100,uniqueness=10):
        if disparities<=0 or disparities%16:raise ValueError('Disparities must be a positive multiple of16')
        if not np.isfinite(lr_tolerance) or lr_tolerance<0 or speckle_window<0 or not 0<=uniqueness<=100:
            raise ValueError('Invalid matcher diagnostic parameters')
        if len(rect)!=2 or any(im is None or im.dtype!=np.uint8 or im.shape[::-1]!=self.size for im in rect):
            raise ValueError('Matcher requires two calibrated-size uint8 grayscale images')
        size=self.size
        baseline_term=float(self.c['P2'][0,3])
        minimum=0 if baseline_term<0 else -disparities
        common=dict(numDisparities=disparities,blockSize=5,P1=8*25,P2=32*25,uniquenessRatio=uniqueness,speckleWindowSize=speckle_window,speckleRange=2,disp12MaxDiff=1,mode=cv2.STEREO_SGBM_MODE_SGBM_3WAY)
        t=time.perf_counter()
        dl=cv2.StereoSGBM_create(minDisparity=minimum,**common).compute(*rect).astype(np.float32)/16
        right_min=-minimum-disparities+1
        dr=cv2.StereoSGBM_create(minDisparity=right_min,**common).compute(rect[1],rect[0]).astype(np.float32)/16
        model_ms=(time.perf_counter()-t)*1000
        yy,xx=np.indices(dl.shape);xr=np.rint(xx-dl).astype(np.int32)
        inside=(xr>=0)&(xr<size[0]);clip=np.clip(xr,0,size[0]-1)
        counts={}
        def count(name,mask):
            if diagnostics:counts[name]=int(np.count_nonzero(mask))
        valid=dl>minimum-1;count('leftMatcher',valid)
        valid &= inside;count('rightInBounds',valid)
        valid &= dr[yy,clip]>right_min-1;count('rightMatcher',valid)
        valid &= np.abs(dl+dr[yy,clip])<=lr_tolerance;count('leftRight',valid)
        roi=cv2.getValidDisparityROI(tuple(self.c['roi1']),tuple(self.c['roi2']),minimum,disparities,5)
        spatial=np.zeros_like(valid);x,y,w,h=roi;spatial[y:y+h,x:x+w]=True;valid &= spatial
        count('disparityROI',valid)
        xyz,valid=reproject(dl,self.c['Q'],valid)
        count('positiveFiniteZ',valid)
        if diagnostics:
            stats=dict(totalPixels=int(dl.size),stages=counts,
                       roi=list(map(int,roi)),roiPixels=int(spatial.sum()),
                       finalRatioInROI=float(valid.sum()/max(1,spatial.sum())),
                       minDisparity=minimum,numDisparities=disparities,blockSize=5,
                       lrTolerancePx=lr_tolerance,speckleWindowSize=speckle_window,uniquenessRatio=uniqueness,
                       leftMatcherIncludes='OpenCV uniqueness, internal LR, speckle and search boundary rejection')
            return dl,xyz[:,:,2],valid,model_ms,stats
        return dl,xyz[:,:,2],valid,model_ms


def main():
    ap=argparse.ArgumentParser();ap.add_argument('calibration',type=Path);ap.add_argument('left',type=Path);ap.add_argument('right',type=Path)
    ap.add_argument('--output',type=Path,required=True);ap.add_argument('--disparities',type=int,default=128)
    args=ap.parse_args()
    c=np.load(args.calibration);processor=StereoProcessor(c)
    images=[cv2.imread(str(p),cv2.IMREAD_GRAYSCALE) for p in [args.left,args.right]]
    if any(im is None for im in images):raise SystemExit('Image missing')
    rect=processor.rectify(images)
    dl,z,valid,model_ms=processor.compute(rect,args.disparities)
    args.output.mkdir(parents=True,exist_ok=True)
    unit=str(c['lengthUnit']) if 'lengthUnit' in c else 'unknown'
    np.save(args.output/f'depth_z_{unit}.npy',z);np.save(args.output/'disparity_px.npy',dl)
    cv2.imwrite(str(args.output/'valid-mask.png'),valid.astype(np.uint8)*255)
    cv2.imwrite(str(args.output/'rectified-left.png'),rect[0]);cv2.imwrite(str(args.output/'rectified-right.png'),rect[1])
    report=dict(inputLeft=str(args.left),inputRight=str(args.right),calibrationFile=str(args.calibration),method='StereoSGBM',validPixelRatio=float(valid.mean()),twoDirectionMatcherMs=model_ms,
        quantity=f'rectified left-camera optical-axis Z, unit={unit}; not Euclidean range',metricScaleVerified=unit=='m',metricAccuracy='not validated with measured distances',endToEndLatencyMeasured=False)
    (args.output/'metrics.json').write_text(json.dumps(report,indent=2));print(json.dumps(report,indent=2))
if __name__=='__main__':main()

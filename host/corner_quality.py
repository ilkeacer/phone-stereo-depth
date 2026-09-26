"""Compare dense disparity to held-out checker correspondences, not metric ground truth."""
import argparse,json
from pathlib import Path
import cv2
import numpy as np
from host.depth import StereoProcessor
from host.quality import distribution
from host.pointcloud import disparity_support_mask


def sample_supported(disparity,valid,xy):
    """Bilinear sample, conservatively requiring all four neighbors to be valid."""
    xy=np.asarray(xy,dtype=float).reshape(-1,2)
    finite=np.isfinite(xy).all(axis=1)
    safe=np.where(np.isfinite(xy),xy,0)
    x,y=np.floor(safe).astype(int).T
    inside=finite&(x>=0)&(y>=0)&(x<disparity.shape[1]-1)&(y<disparity.shape[0]-1)
    x=np.clip(x,0,disparity.shape[1]-2);y=np.clip(y,0,disparity.shape[0]-2)
    good=inside.copy();value=np.zeros(len(xy))
    dx=safe[:,0]-x;dy=safe[:,1]-y
    for ox,oy,weight in [(0,0,(1-dx)*(1-dy)),(1,0,dx*(1-dy)),(0,1,(1-dx)*dy),(1,1,dx*dy)]:
        v=disparity[y+oy,x+ox]
        good &= valid[y+oy,x+ox]&np.isfinite(v)
        value+=np.where(np.isfinite(v),v,0)*weight
    value[~good]=np.nan
    return value,good


def compare(calibration,output,project,min_disparity=0.):
    cal=np.load(calibration);report=json.loads(calibration.with_suffix('.json').read_text())
    folder=project/Path(report['validationTrial'])
    cols,rows=report['innerCorners']
    cache=json.loads((folder/f"corners_v3_anchored_{cols}x{rows}_rot{int(bool(cal['rotate90']))}.json").read_text())
    processors={s:StereoProcessor(cal,s) for s in (1.,.5)}
    reference=[];dense={s:[] for s in processors};per_pose=[]
    for pair_index,names in enumerate(report['heldoutFiles']):
        corners=[np.array(cache[n],np.float32) for n in names]
        if any(p.shape!=(cols*rows,1,2) for p in corners):raise ValueError('Malformed held-out corners')
        if np.dot((corners[0][-1]-corners[0][0]).ravel(),(corners[1][-1]-corners[1][0]).ravel())<0:
            corners[1]=corners[1][::-1].copy()
        rect_corners=[cv2.undistortPoints(p,cal[f'K{i}'],cal[f'D{i}'],R=cal[f'R{i}'],P=cal[f'P{i}']).reshape(-1,2)
                      for i,p in enumerate(corners,1)]
        ref=rect_corners[0][:,0]-rect_corners[1][:,0];reference.extend(ref.tolist())
        images=[cv2.imread(str(folder/n),0) for n in names]
        row={'pairIndex':pair_index}
        for scale,processor in processors.items():
            rect=processor.rectify(images)
            d,z,valid,ms,q=processor.compute_diagnostics(rect,disparities=int(128*scale),
                                                       lr_tolerance=scale,speckle_window=int(100*scale**2))
            original_ratio=float(valid.mean())
            if min_disparity:valid,_=disparity_support_mask(z,valid,processor.c['P2'],scale,min_disparity)
            values,good=sample_supported(d,valid,rect_corners[0]*scale)
            dense[scale].extend((values/scale).tolist())
            row[str(scale)]={'supportedCorners':int(good.sum()),'fullImageValidRatio':float(valid.mean()),'originalMaskRatio':original_ratio}
        per_pose.append(row)
    reference=np.array(reference);dense={s:np.array(v) for s,v in dense.items()}
    reference_good=np.isfinite(reference)&(reference>0)
    common=reference_good.copy()
    for values in dense.values():common &= np.isfinite(values)&(values>0)
    def errors(values,mask):
        delta=np.abs(values[mask]-reference[mask])
        return dict(count=int(mask.sum()),absoluteDisparityErrorFullResolutionPx=distribution(delta),
                    relativeZDisagreement=distribution(np.abs(reference[mask]/values[mask]-1)),
                    fractionOver2FullResolutionPx=float((delta>2).mean()) if mask.any() else None)
    variants={}
    for scale,values in dense.items():
        good=reference_good&np.isfinite(values)&(values>0)
        variants[str(scale)]={'allSupported':errors(values,good),'sameCornersBothResolutions':errors(values,common)}
    result=dict(reference='held-out corner x disparity after existing calibration; correlated geometric reference, not independent metric ground truth',
                totalCorners=len(reference),positiveReferenceCorners=int(reference_good.sum()),variants=variants,perPose=per_pose,
                scope='same 10 checkerboard captures; bilinear sampling needs four valid neighbors; original-pixel disparity units',
                metricAccuracyValidated=False,calibrationChanged=False)
    result['cloudFilterOriginalPixelThreshold']=min_disparity
    output.parent.mkdir(parents=True,exist_ok=True);output.write_text(json.dumps(result,indent=2,allow_nan=False)+'\n')
    return result


if __name__=='__main__':
    ap=argparse.ArgumentParser(description=__doc__);ap.add_argument('--calibration',type=Path,required=True)
    ap.add_argument('--output',type=Path,required=True)
    ap.add_argument('--min-disparity',type=float,default=0.)
    args=ap.parse_args();cv2.setNumThreads(2)
    r=compare(args.calibration,args.output,Path(__file__).resolve().parents[1],args.min_disparity)
    print(json.dumps(r['variants'],indent=2))

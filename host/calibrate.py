"""Offline checkerboard calibration. Requires measured printed target; no guessed scale.

Run on a --fixed --save-all capture. The saved camera coordinates are used unchanged.
A vertical result is rejected with instructions to rotate BOTH inputs before recalibration.
"""
import argparse,json,bisect
from pathlib import Path
import cv2
import numpy as np
from host.analyze import read_jsonl,nearest_unique


def load_view(path,rotate):
    image=cv2.imread(str(path),cv2.IMREAD_GRAYSCALE)
    if image is None: raise ValueError(f'Cannot read {path}')
    return cv2.rotate(image,cv2.ROTATE_90_CLOCKWISE) if rotate else image


def detect_board(image,shape,seed=None):
    """Classic localization + subpixel refinement; SB on a local ROI reduces clutter failures."""
    if seed is not None:
        # Screen previews can contain another checkerboard. Preserve the recorded target
        # identity; never let a global redetection silently switch to its smaller copy.
        grid=np.mgrid[:shape[0],:shape[1]].T.reshape(-1,2).astype(np.float32)
        h,_=cv2.findHomography(grid,seed.reshape(-1,2),0)
        if h is None:return False,None
        boundary=np.array([[[-1.25,-1.25],[shape[0]+.25,-1.25],[shape[0]+.25,shape[1]+.25],[-1.25,shape[1]+.25]]],np.float32)
        polygon=cv2.perspectiveTransform(boundary,h).reshape(-1,2)
        mask=np.zeros_like(image);cv2.fillConvexPoly(mask,np.rint(polygon).astype(np.int32),255)
        masked=np.where(mask,image,255).astype(np.uint8)
        low=np.maximum(0,np.floor(polygon.min(axis=0))).astype(int)
        high=np.minimum(image.shape[::-1],np.ceil(polygon.max(axis=0))).astype(int)
        x0,y0=low;x1,y1=high
        if x1<=x0 or y1<=y0:return False,None
        ok,points=cv2.findChessboardCornersSB(masked[y0:y1,x0:x1],shape,flags=cv2.CALIB_CB_NORMALIZE_IMAGE|cv2.CALIB_CB_ACCURACY)
        if not ok:return False,None
        points+=np.array([x0,y0],np.float32)
        if np.linalg.norm(points[::-1]-seed)<np.linalg.norm(points-seed):points=points[::-1].copy()
        if np.linalg.norm(points-seed,axis=2).mean()>3:return False,None
        return True,points
    ok,points=cv2.findChessboardCorners(image,shape,cv2.CALIB_CB_ADAPTIVE_THRESH|cv2.CALIB_CB_NORMALIZE_IMAGE)
    if ok:
        points=cv2.cornerSubPix(image,points,(5,5),(-1,-1),(cv2.TERM_CRITERIA_EPS|cv2.TERM_CRITERIA_MAX_ITER,40,.001))
        low=points.min(axis=0).ravel();high=points.max(axis=0).ravel()
        pad=max((high[0]-low[0])/max(1,shape[0]-1),(high[1]-low[1])/max(1,shape[1]-1))*2
        x0,y0=np.maximum(0,np.floor(low-pad)).astype(int);x1,y1=np.minimum([image.shape[1],image.shape[0]],np.ceil(high+pad)).astype(int)
        found,sb=cv2.findChessboardCornersSB(image[y0:y1,x0:x1],shape,flags=cv2.CALIB_CB_NORMALIZE_IMAGE|cv2.CALIB_CB_ACCURACY)
        if found:sb+=np.array([x0,y0],np.float32);return True,sb
        return True,points
    return cv2.findChessboardCornersSB(image,shape,flags=cv2.CALIB_CB_NORMALIZE_IMAGE|cv2.CALIB_CB_EXHAUSTIVE|cv2.CALIB_CB_ACCURACY)


def candidates(directory,ids,threshold_ms):
    streams=[]
    for camera in ids:
        images=read_jsonl(directory/f'camera_{camera}_images.jsonl')
        meta={r['sensorTimestampNs']:r for r in read_jsonl(directory/f'camera_{camera}_metadata.jsonl')}
        rows=[dict(r,capture=meta[r['imageTimestampNs']]) for r in images if r.get('sampleFile') and r['imageTimestampNs'] in meta]
        if not rows:raise ValueError(f'No saved images with exact metadata match for {camera}')
        if (directory/'selected-poses.jsonl').exists():
            # Guided files are already selected snapshots, often minutes after startup.
            # Measure settling from the camera frame sequence, not the first saved pose.
            rows=[r for r in rows if r.get('sequence',0)>=60]
        else:
            start=rows[0]['imageTimestampNs']
            rows=[r for r in rows if r['imageTimestampNs']-start>=3_000_000_000]
        if not rows:raise ValueError('Need more than 3 seconds after focus settles')
        crops={r['capture']['crop'] for r in rows}
        focus=[r['capture']['focusDiopters'] for r in rows]
        if len(crops)!=1 or None in focus or max(focus)-min(focus)>.01:raise ValueError('Crop/focus changes: acquire with --fixed and allow settling')
        streams.append(rows)
    a,b=streams
    for i,j,d in nearest_unique([r['imageTimestampNs'] for r in a],[r['imageTimestampNs'] for r in b],int(threshold_ms*1e6)):
        yield a[i],b[j],d


def solve(object_points,left,right,size):
    _,k1,d1,_,_=cv2.calibrateCamera(object_points,left,size,None,None)
    _,k2,d2,_,_=cv2.calibrateCamera(object_points,right,size,None,None)
    rms,k1,d1,k2,d2,r,t,e,f=cv2.stereoCalibrate(object_points,left,right,k1,d1,k2,d2,size,
        flags=cv2.CALIB_FIX_INTRINSIC,criteria=(cv2.TERM_CRITERIA_EPS|cv2.TERM_CRITERIA_MAX_ITER,100,1e-7))
    r1,r2,p1,p2,q,roi1,roi2=cv2.stereoRectify(k1,d1,k2,d2,size,r,t,flags=cv2.CALIB_ZERO_DISPARITY,alpha=0)
    if abs(p2[1,3])>abs(p2[0,3]):raise ValueError('Vertical rectification: rerun with --rotate90. Both input images AND calibration coordinates must rotate together.')
    return dict(K1=k1,D1=d1,K2=k2,D2=d2,R=r,T=t,E=e,F=f,R1=r1,R2=r2,P1=p1,P2=p2,Q=q,roi1=np.array(roi1),roi2=np.array(roi2)),float(rms)


def heldout_error(corners1,corners2,cal):
    a=cv2.undistortPoints(corners1,cal['K1'],cal['D1'],R=cal['R1'],P=cal['P1']).reshape(-1,2)
    b=cv2.undistortPoints(corners2,cal['K2'],cal['D2'],R=cal['R2'],P=cal['P2']).reshape(-1,2)
    return np.abs(a[:,1]-b[:,1])


def board_contained(points,shape,size,margin=12):
    grid=np.mgrid[:shape[0],:shape[1]].T.reshape(-1,2).astype(np.float32)
    h,_=cv2.findHomography(grid,points.reshape(-1,2),0)
    if h is None:return False
    boundary=np.array([[[-1,-1],[shape[0],-1],[shape[0],shape[1]],[-1,shape[1]]]],np.float32)
    xy=cv2.perspectiveTransform(boundary,h).reshape(-1,2)
    return bool((xy>=margin).all() and (xy<np.array(size)-margin).all())


def motion_score(directory,row,neighbors):
    """Median adjacent-frame flow in native left-camera pixels; no residual-based selection."""
    files=[neighbors[0],row,neighbors[1]]
    images=[cv2.resize(cv2.imread(str(directory/r['sampleFile']),0),None,fx=.5,fy=.5,interpolation=cv2.INTER_AREA) for r in files]
    points=cv2.goodFeaturesToTrack(images[1],maxCorners=120,qualityLevel=.04,minDistance=8,blockSize=7)
    if points is None or len(points)<15:return 1e9
    scores=[]
    for idx in [0,2]:
        target,valid,_=cv2.calcOpticalFlowPyrLK(images[1],images[idx],points,None,winSize=(21,21),maxLevel=3)
        back,valid2,_=cv2.calcOpticalFlowPyrLK(images[idx],images[1],target,None,winSize=(21,21),maxLevel=3)
        good=(valid.ravel()>0)&(valid2.ravel()>0)&(np.linalg.norm(back-points,axis=2).ravel()<.5)
        if good.sum()<15:return 1e9
        dt=abs(files[idx]['imageTimestampNs']-row['imageTimestampNs'])
        if not 20_000_000<dt<150_000_000:return 1e9
        scores.append(float(np.median(np.linalg.norm(target-points,axis=2).ravel()[good]))*2*66_666_667/dt)
    return max(scores)


def collect_views(directory,ids,shape,threshold_ms=20,rotate90=False):
    views=[];last=-10**30;last_attempt=-10**30;size=None;quadrants=set();signature=None;used_phases=set()
    guided=read_jsonl(directory/'selected-poses.jsonl')
    seeds={}
    for row in guided:
        for i,camera in enumerate(ids):
            timestamp=row['leftTimestampNs' if i==0 else 'rightTimestampNs']
            for filename in directory.glob(f'camera_{camera}_*_{timestamp}.jpg'):
                seeds[filename.name]=np.array(row['corners'][i],np.float32)
    version='v3_anchored' if guided else 'v2'
    cache_path=directory/f'corners_{version}_{shape[0]}x{shape[1]}_rot{int(rotate90)}.json'
    cache=json.loads(cache_path.read_text()) if cache_path.exists() else {}
    def detect_cached(filename,image):
        if filename not in cache:
            seed=seeds.get(filename)
            if seed is not None and rotate90:
                transformed=seed.copy();transformed[:,:,0]=image.shape[1]-1-seed[:,:,1];transformed[:,:,1]=seed[:,:,0];seed=transformed
            ok,pts=detect_board(image,shape,seed);cache[filename]=pts.tolist() if ok else None
        value=cache[filename]
        return value is not None,np.array(value,np.float32) if value is not None else None
    phase_times=[e['elapsedNs'] for e in read_jsonl(directory/'events.jsonl') if e['event']=='calibration_pose_prompt']
    candidate_rows=list(candidates(directory,ids,threshold_ms))
    motion_path=directory/'motion_v1.json'
    motion=json.loads(motion_path.read_text()) if motion_path.exists() else {}
    if phase_times:
        raw=read_jsonl(directory/f'camera_{ids[0]}_images.jsonl');lookup={r.get('sampleFile'):i for i,r in enumerate(raw)}
        groups={}
        for a,b,delta in candidate_rows:
            phase=bisect.bisect_right(phase_times,a['arrivalElapsedNs'])-1
            if phase<0 or a['sequence']%3:continue
            age=a['arrivalElapsedNs']-phase_times[phase]
            if not 2_500_000_000<=age<=5_800_000_000:continue
            filename=a['sampleFile'];i=lookup[filename]
            if i<1 or i>=len(raw)-1 or not raw[i-1].get('sampleFile') or not raw[i+1].get('sampleFile'):continue
            if filename not in motion:motion[filename]=motion_score(directory,a,(raw[i-1],raw[i+1]))
            groups.setdefault(phase,[]).append((motion[filename],a,b,delta))
        motion_path.write_text(json.dumps(motion))
        candidate_rows=[(a,b,delta) for phase in sorted(groups) for score,a,b,delta in sorted(groups[phase],key=lambda v:v[0])[:5] if score<=1.5]
    for a,b,delta in candidate_rows:
        if signature is None:signature=[(v['capture']['crop'],v['capture']['focusDiopters']) for v in [a,b]]
        if not phase_times and a['imageTimestampNs']-last_attempt<500_000_000:continue
        phase=None
        if phase_times:
            phase=bisect.bisect_right(phase_times,a['arrivalElapsedNs'])-1
            if phase<0 or phase in used_phases:continue
            phase_age=a['arrivalElapsedNs']-phase_times[phase]
            if not 2_500_000_000<=phase_age<=5_800_000_000:continue
        if not phase_times and a['imageTimestampNs']-last<1_000_000_000:continue
        last_attempt=a['imageTimestampNs']
        l=load_view(directory/a['sampleFile'],rotate90);r=load_view(directory/b['sampleFile'],rotate90)
        if l.shape!=r.shape:raise ValueError('Different stream sizes need explicit resampling and recalibration')
        current=(l.shape[1],l.shape[0])
        if size and current!=size:raise ValueError('Image dimensions changed')
        size=current
        ok1,c1=detect_cached(a['sampleFile'],l)
        ok2,c2=detect_cached(b['sampleFile'],r)
        if not(ok1 and ok2):continue
        if not board_contained(c1,shape,size) or not board_contained(c2,shape,size):continue
        # These adjacent back sensors look in the same direction; resolve 180 degree board ordering.
        if np.dot((c1[-1]-c1[0]).ravel(),(c2[-1]-c2[0]).ravel())<0:c2=c2[::-1].copy()
        if views and min(float(np.linalg.norm(c1-v[0],axis=2).mean()) for v in views)<8:continue
        center=c1.mean(axis=0).ravel();quadrants.add((int(center[0]>=size[0]/2),int(center[1]>=size[1]/2)))
        views.append((c1,c2,a['sampleFile'],b['sampleFile'],delta));last=a['imageTimestampNs']
        if phase is not None:used_phases.add(phase)
    cache_path.write_text(json.dumps(cache))
    policy=('guided target identity + masked SB refinement within3px of recorded corners; full-board margin12px; no residual-based selection' if guided else 'full-board boundary margin12px; minimum-motion candidates per pose when phase prompts exist, <=1.5 native pixels/frame; no residual-based selection')
    (directory/'board-detection-summary.json').write_text(json.dumps(dict(selectionPolicy=policy,acceptedViews=len(views),quadrants=list(quadrants),size=size,geometrySignature=signature,files=[v[2:4] for v in views]),indent=2))
    return views,size,quadrants,signature

def main():
    ap=argparse.ArgumentParser();ap.add_argument('trial',type=Path);ap.add_argument('--ids',nargs=2,default=['20','21'])
    ap.add_argument('--columns',type=int,default=9);ap.add_argument('--rows',type=int,default=6)
    scale=ap.add_mutually_exclusive_group(required=True)
    scale.add_argument('--square-mm',type=float,help='ACTUALLY MEASURED printed square edge')
    scale.add_argument('--scale-free',action='store_true',help='Geometry in checker-square units; metric depth prohibited')
    ap.add_argument('--threshold-ms',type=float,default=20);ap.add_argument('--rotate90',action='store_true')
    ap.add_argument('--output',type=Path,required=True)
    ap.add_argument('--training-extra',type=Path,action='append',default=[],help='Additional training capture with identical geometry')
    ap.add_argument('--validation-trial',type=Path,required=True,help='Separate capture with different held-out poses')
    args=ap.parse_args()
    if args.square_mm is not None and args.square_mm<=0:raise SystemExit('Measured square size must be positive')
    if args.validation_trial.resolve() in [p.resolve() for p in [args.trial]+args.training_extra]:raise SystemExit('Validation must be a separate capture with unseen poses')
    shape=(args.columns,args.rows)
    obj=np.zeros((args.rows*args.columns,3),np.float32);obj[:,:2]=np.mgrid[:args.columns,:args.rows].T.reshape(-1,2)*(args.square_mm/1000 if args.square_mm is not None else 1.0)
    train,size,train_quadrants,train_signature=collect_views(args.trial,args.ids,shape,args.threshold_ms,args.rotate90)
    for directory in args.training_extra:
        extra,extra_size,extra_quadrants,extra_signature=collect_views(directory,args.ids,shape,args.threshold_ms,args.rotate90)
        if extra_size!=size or extra_signature!=train_signature:raise SystemExit('Additional training size/crop/focus differs')
        for v in extra:
            if min(float(np.linalg.norm(v[0]-old[0],axis=2).mean()) for old in train)>=8:train.append(v)
        train_quadrants|=extra_quadrants
    test,test_size,test_quadrants,test_signature=collect_views(args.validation_trial,args.ids,shape,args.threshold_ms,args.rotate90)
    if len(train)<25 or len(test)<5 or len(train_quadrants)<4 or len(test_quadrants)<4:
        raise SystemExit(f'Need >=25 training and >=5 separate validation poses, each covering 4 quadrants; found {len(train)}/{len(test)} poses and {len(train_quadrants)}/{len(test_quadrants)} quadrants. No calibration written.')
    if size!=test_size or train_signature!=test_signature:raise SystemExit('Training/validation size, crop or focus differ')
    # Exclude held-out near-duplicates, retaining enough independent coverage.
    validation_before=len(test)
    test=[view for view in test if min(float(np.linalg.norm(view[0]-v[0],axis=2).mean()) for v in train)>=5]
    test_quadrants={tuple((v[0].mean(axis=0).ravel()>=np.array(size)/2).astype(int).tolist()) for v in test}
    if len(test)<5 or len(test_quadrants)<4:raise SystemExit('Too few independent validation poses after excluding training near-duplicates')
    cal,rms=solve([obj.copy() for _ in train],[v[0] for v in train],[v[1] for v in train],size)
    errors=np.concatenate([heldout_error(v[0],v[1],cal) for v in test])
    report=dict(status='estimated; independent metric validation still required',ids=args.ids,size=size,rotate90=args.rotate90,
        geometrySignature=train_signature,measuredSquareMm=args.square_mm,innerCorners=shape,trainingTrials=[str(p) for p in [args.trial]+args.training_extra],validationTrial=str(args.validation_trial),excludedValidationNearDuplicates=validation_before-len(test),trainViews=len(train),heldoutViews=len(test),rmsPx=rms,
        heldoutVerticalErrorPx=dict(median=float(np.median(errors)),p95=float(np.percentile(errors,95)),maximum=float(errors.max())),
        translationNorm=float(np.linalg.norm(cal['T'])),lengthUnit=('m' if args.square_mm is not None else 'checker_square'),metricScaleVerified=args.square_mm is not None,trainFiles=[v[2:4] for v in train],heldoutFiles=[v[2:4] for v in test])
    # A bad fit must not silently become a metric depth calibration.
    if np.percentile(errors,95)>1.5:raise SystemExit(f'Held-out p95 epipolar error {np.percentile(errors,95):.2f}px exceeds 1.5px; collect better poses/model fit. No calibration written.')
    args.output.mkdir(parents=True,exist_ok=True)
    np.savez(args.output/'calibration.npz',**cal,size=np.array(size),rotate90=np.array(args.rotate90),lengthUnit=np.array('m' if args.square_mm is not None else 'checker_square'))
    (args.output/'calibration.json').write_text(json.dumps(report,indent=2))
    print(json.dumps(report,indent=2))
if __name__=='__main__':main()

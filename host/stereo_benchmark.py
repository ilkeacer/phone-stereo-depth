"""Paired offline stereo profiles; coverage is not ground-truth accuracy."""
import argparse
import hashlib
import json
from pathlib import Path
import cv2
import numpy as np
from host.depth import StereoProcessor
from host.motion_recording import atomic_json
from host.ros_stereo import inspect_recording


def matcher(profile,minimum):
    common=dict(minDisparity=minimum,numDisparities=128,blockSize=15,uniquenessRatio=15,
                speckleWindowSize=100,speckleRange=4,disp12MaxDiff=1,preFilterCap=31)
    if profile=='rtabmap_bm':common['disp12MaxDiff']=-1
    if profile in ('rtabmap_bm','controlled_bm'):
        m=cv2.StereoBM_create(numDisparities=128,blockSize=15)
        for key,value in common.items():getattr(m,'set'+key[0].upper()+key[1:])(value)
        m.setPreFilterSize(9);m.setTextureThreshold(10)
    else:
        if profile=='host_sgbm':
            common.update(blockSize=5,uniquenessRatio=10,speckleRange=2)
            common.pop('preFilterCap')  # Preserve host.depth's OpenCV default.
        m=cv2.StereoSGBM_create(**common,P1=8*common['blockSize']**2,P2=32*common['blockSize']**2,mode=cv2.STEREO_SGBM_MODE_SGBM_3WAY)
    # Dump actual getter values, not a hand-written settings label.
    names=['MinDisparity','NumDisparities','BlockSize','UniquenessRatio','SpeckleWindowSize','SpeckleRange','Disp12MaxDiff','PreFilterCap']
    names+=['PreFilterSize','TextureThreshold','PreFilterType'] if profile.endswith('bm') and not profile.endswith('sgbm') else ['P1','P2','Mode']
    return m,{name:getattr(m,'get'+name)() for name in names}


def measure(rect,cal,profile,regions):
    matchers=[matcher(profile,v) for v in (0,-127)]
    raw=[matchers[0][0].compute(*rect),matchers[1][0].compute(rect[1],rect[0])]
    left,right=[a.astype(np.float32)/16 for a in raw]
    yy,xx=np.indices(left.shape);xr=np.rint(xx-left).astype(int);inside=(xr>=0)&(xr<left.shape[1]);clip=xr.clip(0,left.shape[1]-1)
    backend=left>-1
    lr=backend&inside&(right[yy,clip]>-128)&(np.abs(left+right[yy,clip])<=1)
    xyz=cv2.reprojectImageTo3D(left,cal['Q']);final=lr&np.isfinite(xyz).all(2)&(xyz[:,:,2]>0)&regions['commonSupport']
    metrics={}
    for name,region in regions.items():
        n=int(region.sum())
        metrics[name]=dict(pixels=n,backendPixels=int((backend&region).sum()),lrPixels=int((lr&region).sum()),finalPixels=int((final&region).sum()))
        for key in ['backend','lr','final']:metrics[name][key+'Percent']=100*metrics[name][key+'Pixels']/n
    return dict(matcherSettings=[v[1] for v in matchers],regions=metrics),raw,final


def main():
    ap=argparse.ArgumentParser(description=__doc__);ap.add_argument('--trial',type=Path,required=True)
    ap.add_argument('--calibration',type=Path,required=True);ap.add_argument('--output',type=Path,required=True)
    ap.add_argument('--samples-per-phase',type=int,default=4);args=ap.parse_args()
    if not 1<=args.samples_per_phase<=50:ap.error('samples-per-phase must be 1..50')
    processor,pairs,evidence=inspect_recording(args.trial,args.calibration)
    rows=json.loads((args.trial/'committed-pairs.json').read_text());selected=[]
    for phase in dict.fromkeys(r['phase'] for r in rows):
        indices=[i for i,r in enumerate(rows) if r['phase']==phase]
        selected.extend(indices[i] for i in np.unique(np.linspace(0,len(indices)-1,min(len(indices),args.samples_per_phase),dtype=int)))
    paths=[args.calibration,args.calibration.with_suffix('.json'),args.trial/'manifest.json',args.trial/'committed-pairs.json',Path(__file__),Path(__file__).with_name('depth.py')]
    paths += [args.trial/side['sampleFile'] for i in selected for side in pairs[i][:2]]
    before={str(p.resolve()):hashlib.sha256(p.read_bytes()).hexdigest() for p in paths}
    args.output.mkdir(parents=True,exist_ok=False);atomic_json(args.output/'input-before.json',before)
    cv2.setNumThreads(2);cv2.setRNGSeed(0);w,h=processor.size
    x,y,rw,rh=cv2.getValidDisparityROI(tuple(processor.c['roi1']),tuple(processor.c['roi2']),0,128,15)
    common=np.zeros((h,w),bool);common[y:y+rh,x:x+rw]=True
    for mx,my in processor.maps:common&=(mx>=0)&(my>=0)&(mx<processor.input_size[0]-1)&(my<processor.input_size[1]-1)
    central=np.zeros_like(common);central[h//2-100:h//2+100,w//2-100:w//2+100]=True;central&=common
    regions=dict(full=np.ones_like(common),commonSupport=common,central200=central)
    if not central.any():raise ValueError('Empty central comparison region')
    geometry=dict(nativeFx=[float(processor.c[f'K{i}'][0,0]) for i in (1,2)],rectifiedFx=[float(processor.c[f'P{i}'][0,0]) for i in (1,2)],sourceSize=list(processor.input_size),outputSize=list(processor.size),regions={k:int(v.sum()) for k,v in regions.items()},maps=[])
    for mx,my in processor.maps:
        dxv,dxu=np.gradient(mx);dyv,dyu=np.gradient(my);det=dxu*dyv-dxv*dyu
        scale=1/np.sqrt(np.maximum(np.abs(det[central]),1e-12))
        geometry['maps'].append(dict(centralSourceX=[float(mx[central].min()),float(mx[central].max())],centralSourceY=[float(my[central].min()),float(my[central].max())],medianOutputPixelsPerSourcePixel=float(np.median(scale))))
    atomic_json(args.output/'geometry.json',geometry)
    records=[]
    with (args.output/'frames.jsonl').open('x') as log:
        for i in selected:
            rect=processor.rectify([cv2.imread(str(args.trial/p['sampleFile']),0) for p in pairs[i][:2]])
            clahe=cv2.createCLAHE(clipLimit=2.,tileGridSize=(8,8))
            variants=dict(original=rect,teleBlurSigma1=[cv2.GaussianBlur(rect[0],(0,0),1),rect[1]],claheBoth=[clahe.apply(v) for v in rect])
            for variant,images in variants.items():
                for profile in ('rtabmap_bm','controlled_bm','controlled_sgbm','host_sgbm'):
                    stats,raw,valid=measure(images,processor.c,profile,regions)
                    row=dict(frameIndex=i,phase=rows[i]['phase'],stereoDeltaMs=pairs[i][2]/1e6,variant=variant,profile=profile,**stats)
                    log.write(json.dumps(row)+'\n');log.flush();records.append(row)
                    # All selected frames retain full raw disparities and final masks.
                    np.savez_compressed(args.output/f'{i:04d}-{variant}-{profile}.npz',left_raw=raw[0],right_raw=raw[1],final=valid)
            print(f'Processed selected frame {i}',flush=True)
    summary=[]
    for variant in variants:
        for profile in ('rtabmap_bm','controlled_bm','controlled_sgbm','host_sgbm'):
            chosen=[r for r in records if r['variant']==variant and r['profile']==profile]
            summary.append(dict(variant=variant,profile=profile,frames=len(chosen),regions={region:{key:float(np.median([r['regions'][region][key] for r in chosen])) for key in ['backendPercent','lrPercent','finalPercent']} for region in regions}))
    after={p:hashlib.sha256(Path(p).read_bytes()).hexdigest() for p in before};atomic_json(args.output/'input-after.json',after)
    if before!=after:raise RuntimeError('Source changed during benchmark')
    result=dict(opencv=cv2.__version__,numpy=np.__version__,selectedFrameIndices=selected,selection='Evenly spaced indices per phase, before seeing matcher results',samplesPerPhase=args.samples_per_phase,geometry=geometry,summary=summary,inputUnchanged=True,accuracyMeasured=False,scope='Coverage ablations do not identify a unique cause; matched profiles still differ in algorithm-specific controls.',provenance=evidence)
    atomic_json(args.output/'summary.json',result);print(json.dumps(result,indent=2),flush=True)


if __name__=='__main__':main()

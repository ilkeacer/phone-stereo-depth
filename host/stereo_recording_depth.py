"""Cache checked SGBM depth for complete recorded stereo; never changes calibration."""
import argparse
import json
from pathlib import Path
import cv2
import numpy as np
from host.ros_stereo import inspect_recording
from host.stereo_benchmark import measure
from host.monocular_depth import file_sha
from host.motion_recording import atomic_json


def prepare(trial,calibration,output,square_mm):
    if not np.isfinite(square_mm) or square_mm<=0:raise ValueError('Positive explicit estimated scale required')
    processor,pairs,evidence=inspect_recording(trial,calibration)
    paths=[calibration,calibration.with_suffix('.json'),trial/'manifest.json',trial/'committed-pairs.json',Path(__file__),Path(__file__).with_name('stereo_benchmark.py'),Path(__file__).with_name('depth.py')]
    paths += [trial/item['sampleFile'] for pair in pairs for item in pair[:2]]
    before={str(p.resolve()):file_sha(p) for p in paths}
    output.mkdir(parents=True,exist_ok=False);atomic_json(output/'input-before.json',before)
    atomic_json(output/'manifest.json',dict(status='preparing',frames=0))
    cv2.setNumThreads(2);cv2.setRNGSeed(0);w,h=processor.size
    common=np.zeros((h,w),bool)
    x,y,rw,rh=cv2.getValidDisparityROI(tuple(processor.c['roi1']),tuple(processor.c['roi2']),0,128,15);common[y:y+rh,x:x+rw]=True
    for mx,my in processor.maps:common&=(mx>=0)&(my>=0)&(mx<processor.input_size[0]-1)&(my<processor.input_size[1]-1)
    regions=dict(full=np.ones_like(common),commonSupport=common)
    origin=pairs[0][0]['imageTimestampNs'];records=[]
    with (output/'frames.jsonl').open('x') as log:
        for i,(left,right,_) in enumerate(pairs):
            rect=processor.rectify([cv2.imread(str(trial/item['sampleFile']),0) for item in (left,right)])
            stats,raw,mask=measure(rect,processor.c,'controlled_sgbm',regions)
            depth=cv2.reprojectImageTo3D(raw[0].astype(np.float32)/16,processor.c['Q'])[:,:,2]*(square_mm/1000)
            depth[~mask]=np.nan;name=f'{i:06d}.npz'
            np.savez_compressed(output/name,image=rect[0],depth=depth.astype(np.float32))
            records.append(dict(file=name,sha256=file_sha(output/name),stampNs=left['imageTimestampNs']-origin+10**9))
            log.write(json.dumps(dict(frameIndex=i,**stats))+'\n');log.flush()
            if i%25==0 or i==len(pairs)-1:print(f'SGBM derinlik kaydı: {i+1}/{len(pairs)}',flush=True)
    after={p:file_sha(p) for p in before};atomic_json(output/'input-after.json',after)
    if before!=after:raise RuntimeError('Source changed')
    result=dict(status='complete',inputKind='rgbd',frames=len(records),records=records,projection=processor.c['P1'].tolist(),imageSize=list(processor.size),
                model=dict(method='StereoSGBM',profile='controlled_sgbm',matcherSettings=stats['matcherSettings'],opencv=cv2.__version__,externalLRMaxDifferencePx=1.),
                scaleSource='nominal_display_estimate',estimatedSquareMm=square_mm,metricAccuracyValidated=False,
                sourceSpanSeconds=(records[-1]['stampNs']-records[0]['stampNs'])/1e9,sourceOriginNs=origin,sourceEvidence=evidence,inputUnchanged=True,
                scope='Geometric stereo depth, finite positive + external LR + common support; scale remains an estimate')
    atomic_json(output/'manifest.json',result);return {k:v for k,v in result.items() if k!='records'}

if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__)
    for name in ('trial','calibration','output'):p.add_argument('--'+name,type=Path,required=True)
    p.add_argument('--estimated-square-mm',type=float,required=True)
    a=p.parse_args();print(json.dumps(prepare(a.trial,a.calibration,a.output,a.estimated_square_mm),indent=2))

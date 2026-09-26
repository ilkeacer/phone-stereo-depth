"""Offline monocular-vs-stereo evidence. Dense predictions are not valid matches."""
import argparse
import json
from pathlib import Path
import time
import cv2
import numpy as np
from host.ros_stereo import inspect_recording
from host.monocular_depth import MetricDepthAnything,file_sha,comparison_stats,tracked_depth_change
from host.stereo_benchmark import measure
from host.motion_recording import atomic_json


def main():
    p=argparse.ArgumentParser(description=__doc__)
    for name in ('trial','calibration','repository','checkpoint','output'):p.add_argument('--'+name,type=Path,required=True)
    p.add_argument('--estimated-square-mm',type=float,required=True,help='Explicitly unmeasured comparison scale; never ground truth')
    p.add_argument('--samples-per-phase',type=int,default=4)
    p.add_argument('--device',choices=['auto','cuda','cpu'],default='auto')
    p.add_argument('--input-size',type=int,default=518)
    p.add_argument('--rotation',type=int,choices=[0,90,180,270],default=0,help='Clockwise model input rotation; predictions are rotated back')
    a=p.parse_args()
    if not np.isfinite(a.estimated_square_mm) or a.estimated_square_mm<=0:p.error('Positive finite estimated square size required')
    if not 1<=a.samples_per_phase<=20:p.error('samples-per-phase must be 1..20')
    processor,pairs,provenance=inspect_recording(a.trial,a.calibration)
    rows=json.loads((a.trial/'committed-pairs.json').read_text());anchors=[]
    for phase in dict.fromkeys(r['phase'] for r in rows):
        ids=[i for i,r in enumerate(rows) if r['phase']==phase]
        anchors.extend(ids[i] for i in np.unique(np.linspace(0,len(ids)-1,min(a.samples_per_phase,len(ids)),dtype=int)))
    transitions=[(i,i+1) for i in anchors if i+1<len(rows) and rows[i]['phase']==rows[i+1]['phase']]
    selected=sorted(set(anchors)|{j for _,j in transitions})
    inputs=[a.calibration,a.calibration.with_suffix('.json'),a.trial/'manifest.json',a.trial/'committed-pairs.json']
    inputs+=[a.trial/item['sampleFile'] for i in selected for item in pairs[i][:2]]
    inputs+=[Path(__file__),Path(__file__).with_name('monocular_depth.py'),Path(__file__).with_name('stereo_benchmark.py'),Path(__file__).with_name('depth.py')]
    before={str(f.resolve()):file_sha(f) for f in inputs}
    a.output.mkdir(parents=True,exist_ok=False);atomic_json(a.output/'input-before.json',before)
    cv2.setNumThreads(2);cv2.setRNGSeed(0)
    import torch
    torch.set_num_threads(4)
    model=MetricDepthAnything(a.repository,a.checkpoint,a.device,a.input_size,a.rotation)
    w,h=processor.size
    x,y,rw,rh=cv2.getValidDisparityROI(tuple(processor.c['roi1']),tuple(processor.c['roi2']),0,128,15)
    common=np.zeros((h,w),bool);common[y:y+rh,x:x+rw]=True
    for mx,my in processor.maps:common&=(mx>=0)&(my>=0)&(mx<processor.input_size[0]-1)&(my<processor.input_size[1]-1)
    regions=dict(full=np.ones_like(common),commonSupport=common)
    rectified={};predictions={};records=[];stereo_maps={}
    first=processor.rectify([cv2.imread(str(a.trial/item['sampleFile']),0) for item in pairs[selected[0]][:2]])[0]
    _,warmup=model.predict(cv2.cvtColor(first,cv2.COLOR_GRAY2BGR))
    if model.device=='cuda':torch.cuda.reset_peak_memory_stats()
    with (a.output/'frames.jsonl').open('x') as log:
        for i in selected:
            rect=processor.rectify([cv2.imread(str(a.trial/item['sampleFile']),0) for item in pairs[i][:2]])
            pred,ms=model.predict(cv2.cvtColor(rect[0],cv2.COLOR_GRAY2BGR))
            rectified[i]=rect[0];predictions[i]=pred
            item=dict(frameIndex=i,phase=rows[i]['phase'],isAnchor=i in anchors,inferenceMs=ms,
                      positiveFinitePredictionPercent=100*float((np.isfinite(pred)&(pred>0)).mean()),
                      predictedDepthP5P50P95Metres=np.percentile(pred,[5,50,95]).tolist())
            arrays=dict(predicted_depth_m=pred)
            if i in anchors:
                start=time.perf_counter();stats,raw,mask=measure(rect,processor.c,'controlled_sgbm',regions)
                stereo_ms=(time.perf_counter()-start)*1000
                z=cv2.reprojectImageTo3D(raw[0].astype(np.float32)/16,processor.c['Q'])[:,:,2]*(a.estimated_square_mm/1000)
                z[~mask]=np.nan
                item.update(stereo=stats,stereoMs=stereo_ms,agreement=comparison_stats(pred,z,mask))
                arrays.update(stereo_depth_estimated_m=z,stereo_valid=mask,left_raw=raw[0],right_raw=raw[1]);stereo_maps[i]=z
            np.savez_compressed(a.output/f'{i:04d}.npz',**arrays)
            log.write(json.dumps(item,allow_nan=False)+'\n');log.flush();records.append(item)
            print(json.dumps(item,allow_nan=False),flush=True)
    temporal=[]
    for i,j in transitions:
        temporal.append(dict(fromFrame=i,toFrame=j,phase=rows[i]['phase'],
                             deltaSeconds=(pairs[j][0]['imageTimestampNs']-pairs[i][0]['imageTimestampNs'])/1e9,
                             **tracked_depth_change(rectified[i],rectified[j],predictions[i],predictions[j])))
    anchors_results=[v for v in records if v['isAnchor']]
    result=dict(model=model.metadata,provenance=provenance,anchors=anchors,allInferredFrames=selected,
                inputImage='Rectified telephoto grayscale replicated into BGR, 640x480; same view as stereo depth',
                scaleSource='nominal_display_estimate',estimatedSquareMm=a.estimated_square_mm,
                accuracyMeasured=False,slamEvaluated=False,predictionCoverageIsNotStereoValidity=True,
                warmupMs=warmup,inferenceMedianMs=float(np.median([r['inferenceMs'] for r in records])),
                inferenceP95Ms=float(np.percentile([r['inferenceMs'] for r in records],95)),
                latencyScope='Image preprocessing, inference and CPU output; excludes disk, rectification, ROS and model loading',
                stereoMedianMs=float(np.median([r['stereoMs'] for r in anchors_results])),
                medianStereoFinalPercent=float(np.median([r['stereo']['regions']['full']['finalPercent'] for r in anchors_results])),
                medianPredictionStereoRatio=float(np.median([r['agreement']['medianRatio'] for r in anchors_results if r['agreement']['pixels']])),
                medianAbsoluteRelativeDisagreementPercent=float(np.median([r['agreement']['medianAbsoluteRelativeDisagreementPercent'] for r in anchors_results if r['agreement']['pixels']])),
                temporal=temporal,temporalScope='Uncompensated optical-flow tracked depth change, includes real camera/scene motion; not temporal error or a SLAM test',
                peakCudaAllocatedMiB=torch.cuda.max_memory_allocated()/1024**2 if model.device=='cuda' else None)
    after={f:file_sha(f) for f in before};atomic_json(a.output/'input-after.json',after)
    if before!=after:raise RuntimeError('An input changed during evaluation')
    result['inputUnchanged']=True;atomic_json(a.output/'summary.json',result)
    # A representative frame is selected by its fixed index, never by appearance.
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    i=anchors[len(anchors)//2];fig,axes=plt.subplots(1,3,figsize=(15,4.6),layout='constrained')
    axes[0].imshow(rectified[i],cmap='gray');axes[0].set_title(f'Recorded telephoto\nFrame {i}',fontsize=11)
    cmap=plt.get_cmap('turbo').copy();cmap.set_bad('black')
    axes[1].imshow(stereo_maps[i],cmap=cmap,vmin=0,vmax=5);axes[1].set_title('SGBM\nEstimated metres',fontsize=11)
    im=axes[2].imshow(predictions[i],cmap=cmap,vmin=0,vmax=5);axes[2].set_title(f'DA V2 Small / input rotation {a.rotation}°\nPredicted metres',fontsize=11)
    for ax in axes:ax.axis('off')
    fig.colorbar(im,ax=axes,shrink=.7,label='Shared 0–5 m scale; unverified')
    fig.savefig(a.output/'comparison.png',dpi=140,bbox_inches='tight');plt.close(fig)
    print(json.dumps(result,indent=2,allow_nan=False),flush=True)


if __name__=='__main__':main()

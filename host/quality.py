"""Offline quality diagnostics on existing held-out captures; never refits calibration.

Writes numeric reports plus PRIVATE scene diagnostics to --output. Coverage and
cross-resolution agreement are not ground-truth depth accuracy measurements.
"""
import argparse
import hashlib
import json
import time
from pathlib import Path

import cv2
import numpy as np

from host.calibrate import heldout_error
from host.depth import StereoProcessor


def distribution(values):
    values=np.asarray(values)
    return dict(count=int(values.size),median=float(np.median(values)),
                p95=float(np.percentile(values,95)),maximum=float(values.max())) if values.size else dict(count=0)


def corner_diagnostics(cal, report, directory):
    """Use exactly the saved held-out file list and anchored corner cache."""
    cols,rows=report['innerCorners']
    cache_path=directory/f"corners_v3_anchored_{cols}x{rows}_rot{int(bool(cal['rotate90']))}.json"
    cache=json.loads(cache_path.read_text())
    errors=[];coordinates=[[],[]];by_pose=[]
    for names in report['heldoutFiles']:
        points=[np.array(cache[name],np.float32) for name in names]
        if any(p.shape!=(cols*rows,1,2) for p in points):
            raise ValueError('Missing or malformed held-out corners')
        if np.dot((points[0][-1]-points[0][0]).ravel(),(points[1][-1]-points[1][0]).ravel())<0:
            points[1]=points[1][::-1].copy()
        error=heldout_error(*points,cal)
        errors.extend(error.tolist())
        by_pose.append(distribution(error))
        for i in (0,1):coordinates[i].extend(points[i].reshape(-1,2).tolist())
    errors=np.array(errors);size=np.array(cal['size'])
    spatial={}
    for i in (0,1):
        xy=np.array(coordinates[i]);fraction=xy/size
        # Geometric image center and half-diagonal, not fitted principal point.
        radius=np.linalg.norm(xy-(size-1)/2,axis=1)/np.linalg.norm((size-1)/2)
        radial=[]
        for low,high in zip((0,.25,.5,.75),(.25,.5,.75,1.01)):
            radial.append(dict(radiusFraction=[low,high],**distribution(errors[(radius>=low)&(radius<high)])))
        grid=[]
        for y in range(3):
            for x in range(3):
                take=(fraction[:,0]>=x/3)&(fraction[:,0]<(x+1)/3)&(fraction[:,1]>=y/3)&(fraction[:,1]<(y+1)/3)
                grid.append(dict(cellXY=[x,y],**distribution(errors[take])))
        spatial[str(20+i)]=dict(cornerBounds=[xy.min(axis=0).tolist(),xy.max(axis=0).tolist()],
                                radialBins=radial,grid3x3=grid)
    result=dict(all=distribution(errors),byPose=by_pose,byCameraPosition=spatial,
                meaning='absolute rectified vertical residual in original calibrated pixels; empty bins are unobserved')
    reference=report['heldoutVerticalErrorPx']
    if abs(result['all']['p95']-reference['p95'])>.001:
        raise ValueError('Saved held-out corner evaluation does not reproduce the accepted report')
    return result


def run(calibration_path,output,project_root):
    output.mkdir(parents=True,exist_ok=True)
    cal=np.load(calibration_path);report=json.loads(calibration_path.with_suffix('.json').read_text())
    directory=Path(report['validationTrial'])
    if not directory.is_absolute():directory=project_root/directory
    specs=[('baseline',1.,1.,100,10),('half',.5,.5,25,10),('quarter',.25,.25,6,10),
           ('lr2',1.,2.,100,10),('no_speckle',1.,1.,0,10),('uniqueness0',1.,1.,100,0)]
    processors={scale:StereoProcessor(cal,scale) for _,scale,*_ in specs}
    rows=[]
    for pair_index,names in enumerate(report['heldoutFiles']):
        images=[cv2.imread(str(directory/name),cv2.IMREAD_GRAYSCALE) for name in names]
        if any(im is None for im in images):raise ValueError('Held-out source images missing')
        baseline_z=None;baseline_mask=None
        for name,scale,tolerance,speckle,unique in specs:
            processor=processors[scale]
            started=time.perf_counter();rect=processor.rectify(images)
            disparity,z,valid,matcher_ms,quality=processor.compute_diagnostics(rect,
                disparities=int(np.ceil(128*scale/16))*16,lr_tolerance=tolerance,
                speckle_window=speckle,uniqueness=unique)
            elapsed=(time.perf_counter()-started)*1000
            if name=='baseline':baseline_z=z;baseline_mask=valid
            # Compare at identical full-resolution pixel centers (0,s,2s...),
            # with no interpolation of invalid depths across mask boundaries.
            stride=int(round(1/scale))
            reference_z=baseline_z[::stride,::stride];reference_mask=baseline_mask[::stride,::stride]
            common=valid&reference_mask
            relative=np.abs(z[common]-reference_z[common])/reference_z[common]
            row=dict(pairIndex=pair_index,variant=name,scale=scale,size=list(processor.size),
                     quality=quality,validRatio=float(valid.mean()),matcherMs=matcher_ms,
                     rectifyComputeMs=elapsed,agreementWithBaseline=distribution(relative),
                     commonPixels=int(common.sum()))
            rows.append(row)
            if pair_index==0:
                dest=output/name;dest.mkdir(exist_ok=True)
                cv2.imwrite(str(dest/'valid-mask.png'),valid.astype(np.uint8)*255)
                cv2.imwrite(str(dest/'rectified-left.png'),rect[0]);cv2.imwrite(str(dest/'rectified-right.png'),rect[1])
                np.save(dest/'disparity_px.npy',disparity)
                np.save(dest/'depth_z.npy',z)
        print(f'Quality: {pair_index+1}/{len(report["heldoutFiles"])} recorded pairs',flush=True)
    summaries={}
    for name,*_ in specs:
        subset=[row for row in rows if row['variant']==name]
        summaries[name]=dict(pairs=len(subset),medianValidRatio=float(np.median([r['validRatio'] for r in subset])),
            medianMatcherMs=float(np.median([r['matcherMs'] for r in subset])),
            medianRectifyComputeMs=float(np.median([r['rectifyComputeMs'] for r in subset])),
            medianStageRatios={k:float(np.median([r['quality']['stages'][k]/r['quality']['totalPixels'] for r in subset]))
                               for k in subset[0]['quality']['stages']})
    result=dict(opencv=cv2.__version__,calibrationSha256=hashlib.sha256(calibration_path.read_bytes()).hexdigest(),
        source='accepted held-out checkerboard captures; no new phone capture; not a controlled room-motion test',
        unit=str(cal['lengthUnit']),accuracyEvaluated=False,liveDefaultsChanged=False,
        experimentNotes=[
            'K/D/R unchanged. P and Q scaled together. Source images are remapped again; no refit or widened FOV.',
            'Resolution experiments also change the angular footprint of the fixed 5px block. Not a pure interpolation ablation.',
            'Disparity search and external LR tolerance scale with output; speckle area scales approximately. Internal LR and speckleRange remain in output pixels.',
            'Valid ratio denominator is each full output grid, not just textured/common surfaces. Agreement is only on both-valid matching pixel centers.',
            'Uniqueness or speckle relaxation can retain wrong matches. More coverage is not proof of improvement.',
            'Timings are one ordered run per pair/variant on this host, exclude decode and are not live throughput benchmarks.'],
        heldout=corner_diagnostics(cal,report,directory),summaries=summaries,rows=rows)
    (output/'quality.json').write_text(json.dumps(result,indent=2,allow_nan=False)+'\n')
    return result


def main():
    ap=argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--calibration',type=Path,required=True)
    ap.add_argument('--output',type=Path,required=True)
    ap.add_argument('--project-root',type=Path,default=Path(__file__).resolve().parents[1])
    args=ap.parse_args();cv2.setNumThreads(2)
    result=run(args.calibration,args.output,args.project_root)
    print(json.dumps(result['summaries'],indent=2))


if __name__=='__main__':main()

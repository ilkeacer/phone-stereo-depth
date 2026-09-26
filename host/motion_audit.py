"""Bounded offline pairwise motion audit of an existing raw camera trial."""
import argparse
import hashlib
import json
from pathlib import Path
import cv2
import numpy as np
from host.calibrate import candidates
from host.depth import StereoProcessor
from host.motion import estimate_pair,transform_disagreement,FEATURE_QUALITY_LEVEL
from host.pointcloud import disparity_support_mask
from host.motion_recording import committed_pairs


def temporal_reason(previous,current,max_gap_ns=500_000_000):
    if any(b<=a for a,b in zip(previous,current)):return 'non_increasing_timestamp'
    if any(b-a>max_gap_ns for a,b in zip(previous,current)):return 'temporal_gap'
    return None


def candidate_transform(result):
    if result.get('status')!='candidate':return None
    transform=result.get('T_current_previous')
    return np.asarray(transform,dtype=np.float64) if transform is not None else None


def interior_disparity_mask(disparity,minimum,count):
    """Search endpoints do not have support on both sides of their match."""
    return np.isfinite(disparity)&(disparity>minimum)&(disparity<minimum+count-1)


def audit(trial,calibration,limit=60,start_pair=0):
    if not 2<=limit<=300:raise ValueError('Limit must be 2..300 stereo pairs')
    if type(start_pair) is not int or start_pair<0:raise ValueError('Start pair must be a non-negative integer')
    report=json.loads(calibration.with_suffix('.json').read_text())
    if report['ids']!=['20','21'] or report['lengthUnit']!='checker_square':
        raise ValueError('Expected camera20/21 checker-square calibration')
    with np.load(calibration) as loaded:
        cal={key:loaded[key] for key in loaded.files}
    if str(cal['lengthUnit'])!=report['lengthUnit']:
        raise ValueError('Calibration unit differs from JSON sidecar')
    signature=report['geometrySignature']
    if len(signature)!=2 or any(len(s)!=2 or not isinstance(s[1],(int,float)) or not np.isfinite(s[1]) for s in signature):
        raise ValueError('Invalid calibration geometry signature')
    processor=StereoProcessor(cal,.5)
    # New guide captures already passed warmup; preserve their committed stereo pairs.
    source=committed_pairs(trial) if (trial/'committed-pairs.json').exists() else candidates(trial,['20','21'],20)
    pairs=sorted(source,key=lambda p:p[0]['imageTimestampNs'])
    rows=[];history=[];processed=0
    for a,b,delta in pairs[start_pair:start_pair+limit]:
        timestamp=tuple(x['imageTimestampNs'] for x in (a,b))
        row=dict(pairIndex=start_pair+processed,sourceTimestampsNs=timestamp,stereoDeltaNs=abs(timestamp[0]-timestamp[1]))
        images=[];reason=None
        for item,signature in zip((a,b),report['geometrySignature']):
            meta=item['capture']
            focus=meta.get('focusDiopters')
            if (meta.get('crop')!=signature[0] or not isinstance(focus,(int,float))
                    or not np.isfinite(focus) or abs(focus-signature[1])>.01):
                reason='calibration_geometry_mismatch';break
            if (item.get('width'),item.get('height'))!=processor.input_size:
                reason='source_size_mismatch';break
            # Saved paths must stay within the supplied trial.
            path=(trial/item['sampleFile']).resolve()
            if not path.is_relative_to(trial.resolve()):raise ValueError('Sample path escapes trial')
            im=cv2.imread(str(path),0)
            if im is None:reason='missing_image';break
            images.append(im)
        processed+=1
        if reason:
            row.update(status='rejected',reason=reason,T_current_previous=None);rows.append(row);history=[];continue
        rect=processor.rectify(images)
        disparity,z,valid,_,depth_stats=processor.compute_diagnostics(rect,disparities=64,lr_tolerance=.5,speckle_window=25)
        interior=interior_disparity_mask(disparity,depth_stats['minDisparity'],depth_stats['numDisparities'])
        row['depthSearchBoundaryRejectedPixels']=int(np.count_nonzero(valid&~interior))
        valid &= interior
        valid,_=disparity_support_mask(z,valid,processor.c['P2'],.5,8.)
        row['depthSupportedPixels']=int(np.count_nonzero(valid))
        current=(timestamp,rect[0],z,valid)
        if not history:
            row.update(status='reference',reason='new_segment',T_current_previous=None)
        else:
            old_time,old_image,old_z,old_mask=history[-1]['frame']
            reason=temporal_reason(old_time,timestamp)
            if reason:
                row.update(status='rejected',reason=reason,T_current_previous=None);history=[]
            else:
                k=processor.c['P1'][:,:3]
                row.update(estimate_pair(old_image,rect[0],old_z,old_mask,k))
                # Keep the LK count before forwardBackward becomes pose closure.
                row['trackedForwardBackward']=row.get('forwardBackward',0)
                reverse=estimate_pair(rect[0],old_image,z,valid,k)
                row['reverse']=reverse
                forward_transform=candidate_transform(row);reverse_transform=candidate_transform(reverse)
                if forward_transform is not None and reverse_transform is not None:
                    identity=np.eye(4)
                    row['forwardBackward']=transform_disagreement(identity,reverse_transform@forward_transform)
                else:row['forwardBackward']=None
                if len(history)>=2 and candidate_transform(history[-1]['row']) is not None:
                    older=history[-2]['frame']
                    direct=estimate_pair(older[1],rect[0],older[2],older[3],k)
                    row['threeFrameDirect']=direct
                    direct_transform=candidate_transform(direct)
                    if forward_transform is not None and direct_transform is not None:
                        composed=forward_transform@candidate_transform(history[-1]['row'])
                        row['threeFrame']=transform_disagreement(direct_transform,composed)
                    else:row['threeFrame']=None
                else:
                    row['threeFrameDirect']=None;row['threeFrame']=None
        rows.append(row);history.append(dict(frame=current,row=row));history=history[-2:]
    return dict(scope='offline independent adjacent-pair candidates; not a trajectory or a map',
        calibrationSha256=hashlib.sha256(calibration.read_bytes()).hexdigest(),
        calibrationReportSha256=hashlib.sha256(calibration.with_suffix('.json').read_bytes()).hexdigest(),
        sourceTrial=str(trial),
        processedPairs=processed,availablePairs=len(pairs),startPair=start_pair,pairLimit=limit,outputScale=.5,
        motionGroundTruthAvailable=False,metricAccuracyValidated=False,unit='checker_square',
        gates=dict(maxStereoDeltaMs=20,maxTemporalGapMs=500,minCorrespondences=30,
                   minInlierRatio=.6,maxReprojectionPx=2,maxForwardBackwardPx=1,
                   minOccupiedCellsOf4x3=4,minImageAxisSpan=.15,minOriginalDisparityPx=8,
                   featureQualityLevel=FEATURE_QUALITY_LEVEL,stereoDisparities=64,
                   rejectDisparitySearchEndpoints=True),
        candidateCount=sum(x['status']=='candidate' for x in rows),
        forwardBackwardComparable=sum(x.get('forwardBackward') is not None for x in rows),
        threeFrameComparable=sum(x.get('threeFrame') is not None for x in rows),rows=rows)


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--trial',type=Path,required=True)
    parser.add_argument('--calibration',type=Path,required=True)
    parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--limit',type=int,default=60)
    parser.add_argument('--start-pair',type=int,default=0)
    args=parser.parse_args();cv2.setNumThreads(2);cv2.setRNGSeed(0)
    result=audit(args.trial,args.calibration,args.limit,args.start_pair)
    args.output.parent.mkdir(parents=True,exist_ok=True)
    with args.output.open('x') as f:json.dump(result,f,indent=2,allow_nan=False)
    print(json.dumps({k:v for k,v in result.items() if k!='rows'},indent=2))

if __name__=='__main__':main()

"""Aggregate bounded motion audits without claiming trajectory accuracy."""
import argparse
from collections import Counter
import json
from pathlib import Path
import numpy as np
from host.tracking import MotionTrackingState


def distribution(values):
    values=np.asarray(values,dtype=np.float64)
    if not len(values):return dict(count=0,p50=None,p95=None,maximum=None)
    if not np.isfinite(values).all():raise ValueError('Non-finite consistency metric')
    p50,p95=np.percentile(values,[50,95])
    return dict(count=len(values),p50=float(p50),p95=float(p95),maximum=float(values.max()))


def summarize(paths):
    if not paths:raise ValueError('At least one audit is required')
    audits=[json.loads(Path(path).read_text()) for path in paths]
    expected=('calibrationSha256','calibrationReportSha256','outputScale','unit','gates')
    reference={key:audits[0][key] for key in expected}
    rows=[];sources=[];tracking_events=[]
    for path,audit in zip(paths,audits):
        if any(audit.get(key)!=value for key,value in reference.items()):
            raise ValueError('Audit calibration, scale, unit or gates differ')
        if audit.get('motionGroundTruthAvailable') is not False or audit.get('metricAccuracyValidated') is not False:
            raise ValueError('Audit scope is not unverified checker-square motion')
        part=audit.get('rows')
        if not isinstance(part,list) or len(part)!=audit.get('processedPairs'):
            raise ValueError('Audit row count differs from metadata')
        if part:
            span=(part[-1]['sourceTimestampsNs'][0]-part[0]['sourceTimestampsNs'][0])/1e9
        else:span=0.
        sources.append(dict(auditFile=Path(path).name,startPair=audit.get('startPair',0),
                            processedPairs=len(part),sourceSpanSeconds=span))
        tracker=MotionTrackingState(max_gap_ns=int(audit['gates']['maxTemporalGapMs']*1e6))
        source_run=Path(path).name
        for row in part:
            frame=dict(sourceRun=source_run,sourceTimestampsNs=row['sourceTimestampsNs'])
            motion=row if row.get('status')!='reference' else None
            tracking_events.extend(tracker.observe(frame,motion))
        rows.extend(part)
    statuses=Counter(row.get('status') for row in rows)
    rejects=Counter(row.get('reason') for row in rows if row.get('status')=='rejected')
    reverse_rejects=Counter(row['reverse'].get('reason') for row in rows
                            if row.get('reverse') and row['reverse'].get('status')=='rejected')
    tracking_statuses=Counter(event['status'] for event in tracking_events)
    tracking_losses=Counter(event['reason'] for event in tracking_events if event['status']=='tracking_lost')
    def metrics(name):
        values=[row[name] for row in rows if row.get(name) is not None]
        return {key:distribution([value[key] for value in values])
                for key in ('rotationDegrees','translationCheckerSquare')}
    return dict(scope='aggregate internal consistency of independent motion candidates; not a trajectory or accuracy proof',
                motionGroundTruthAvailable=False,metricAccuracyValidated=False,
                calibrationSha256=reference['calibrationSha256'],
                calibrationReportSha256=reference['calibrationReportSha256'],
                outputScale=reference['outputScale'],unit=reference['unit'],gates=reference['gates'],
                sources=sources,totalPairs=len(rows),statusCounts=dict(statuses),
                rejectionReasons={str(k):v for k,v in rejects.items()},
                reverseRejectionReasons={str(k):v for k,v in reverse_rejects.items()},
                trackingStateCounts=dict(tracking_statuses),
                trackingLossReasons={str(k):v for k,v in tracking_losses.items()},
                forwardBackward=metrics('forwardBackward'),threeFrame=metrics('threeFrame'))


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('audits',type=Path,nargs='+')
    parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args();result=summarize(args.audits)
    args.output.parent.mkdir(parents=True,exist_ok=True)
    with args.output.open('x') as stream:json.dump(result,stream,indent=2,allow_nan=False)
    print(json.dumps(result,indent=2))


if __name__=='__main__':main()

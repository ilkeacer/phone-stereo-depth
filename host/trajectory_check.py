"""Endpoint drift from raw TUM odometry; optimized closure is not ground truth."""
import argparse
import json
from pathlib import Path
import numpy as np


def tracked_pose_values(msg,info):
    """Only accept a pose with same-stamp, non-lost tracking evidence."""
    a,b=msg.header.stamp,info.header.stamp
    if (a.sec,a.nanosec)!=(b.sec,b.nanosec) or info.lost:
        raise ValueError('Pose has no same-stamp successful tracking evidence')
    p=msg.pose.pose.position;q=msg.pose.pose.orientation
    values=[p.x,p.y,p.z,q.x,q.y,q.z,q.w]
    if not np.isfinite(values).all() or abs(sum(v*v for v in values[3:])-1)>1e-3:
        raise ValueError('Invalid tracked pose')
    return values


def endpoint_metrics(trajectory):
    a=np.asarray(trajectory,dtype=float)
    if a.ndim!=2 or a.shape[1] not in (8,9) or len(a)<2 or not np.isfinite(a).all():raise ValueError('At least two finite TUM poses required')
    if np.any(np.diff(a[:,0])<=0):raise ValueError('Timestamps must increase')
    q=a[:,4:8];norm=np.linalg.norm(q,axis=1)
    if not np.allclose(norm,1,atol=1e-3):raise ValueError('Invalid quaternion')
    q=q/norm[:,None]
    angle=float(np.degrees(2*np.arccos(np.clip(abs(np.dot(q[0],q[-1])),0,1))))
    return dict(poses=len(a),durationSeconds=float(a[-1,0]-a[0,0]),
                endpointTranslationInputUnits=float(np.linalg.norm(a[-1,1:4]-a[0,1:4])),
                endpointRotationDegrees=angle,pathLengthInputUnits=float(np.linalg.norm(np.diff(a[:,1:4],axis=0),axis=1).sum()))


def assess(odometry,scale_source,repeatable_endpoint=False):
    m=endpoint_metrics(odometry)
    assessable=scale_source=='measured' and repeatable_endpoint
    m.update(scaleSource=scale_source,repeatablePhysicalEndpointConfirmed=repeatable_endpoint,
             translationThresholdCm=10.,rotationThresholdDegrees=5.,
             endpointTranslationCm=m['endpointTranslationInputUnits']*100 if scale_source=='measured' else None,
             endpointTranslationEstimatedCm=m['endpointTranslationInputUnits']*100 if scale_source=='nominal_display_estimate' else None,
             endpointTranslationPredictedCm=m['endpointTranslationInputUnits']*100 if scale_source=='model_predicted_metres' else None,
             endpointGate='not_assessable',fullSlamAcceptanceAssessed=False)
    if assessable:m['endpointGate']='pass' if m['endpointTranslationCm']<=10 and m['endpointRotationDegrees']<=5 else 'fail'
    return m


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--odometry',type=Path,required=True,help='Raw pre-loop-optimization TUM poses, in measured or estimated metres')
    p.add_argument('--optimized',type=Path)
    p.add_argument('--scale-source',choices=['measured','nominal_display_estimate','model_predicted_metres','unknown'],required=True)
    p.add_argument('--repeatable-endpoint',action='store_true',help='Operator confirms same marked physical position AND orientation')
    p.add_argument('--output',type=Path,required=True)
    a=p.parse_args();result=dict(rawOdometry=assess(np.loadtxt(a.odometry,ndmin=2),a.scale_source,a.repeatable_endpoint))
    if a.optimized:result['optimizedInformationalOnly']=endpoint_metrics(np.loadtxt(a.optimized,ndmin=2))
    with a.output.open('x') as f:json.dump(result,f,indent=2,allow_nan=False)
    print(json.dumps(result,indent=2))


if __name__=='__main__':main()

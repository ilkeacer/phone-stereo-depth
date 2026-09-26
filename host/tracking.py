"""Motion-candidate lifecycle. It deliberately does not accumulate a trajectory."""
import numpy as np
from host.motion import transform_disagreement


def validate_frame(frame):
    if not isinstance(frame,dict) or not isinstance(frame.get('sourceRun'),str) or not frame['sourceRun']:
        raise ValueError('Invalid source run')
    timestamps=frame.get('sourceTimestampsNs')
    if (not isinstance(timestamps,(list,tuple)) or len(timestamps)!=2
            or any(type(value) is not int or value<0 for value in timestamps)):
        raise ValueError('Invalid source timestamps')
    return dict(sourceRun=frame['sourceRun'],sourceTimestampsNs=tuple(timestamps))


def boundary_reason(previous,current,max_gap_ns=500_000_000):
    if previous['sourceRun']!=current['sourceRun']:return 'source_changed'
    before=previous['sourceTimestampsNs'];after=current['sourceTimestampsNs']
    if any(new<=old for old,new in zip(before,after)):return 'non_increasing_timestamp'
    if any(new-old>max_gap_ns for old,new in zip(before,after)):return 'temporal_gap'
    return None


def valid_candidate(result):
    if not isinstance(result,dict) or result.get('status')!='candidate':return False
    transform=result.get('T_current_previous')
    try:transform_disagreement(transform,transform)
    except (ValueError,TypeError,np.linalg.LinAlgError):return False
    return True


class MotionTrackingState:
    """Emit explicit segment/loss events while retaining only the latest frame identity."""
    def __init__(self,max_gap_ns=500_000_000):
        if type(max_gap_ns) is not int or max_gap_ns<=0:raise ValueError('Invalid maximum gap')
        self.max_gap_ns=max_gap_ns;self.last=None;self.segment=0;self.pending_new_segment=False

    def reset(self):
        self.last=None;self.pending_new_segment=False

    def observe(self,frame,motion=None):
        current=validate_frame(frame);events=[]
        if self.last is None:
            self.segment+=1;self.last=current;self.pending_new_segment=False
            return [dict(status='new_segment',reason='initial_reference',segmentId=self.segment,
                         T_current_previous=None,frame=current)]
        boundary=boundary_reason(self.last,current,self.max_gap_ns)
        if boundary:
            events.append(dict(status='tracking_lost',reason=boundary,segmentId=self.segment,
                               T_current_previous=None,frame=current))
            self.segment+=1;self.last=current;self.pending_new_segment=False
            events.append(dict(status='new_segment',reason=boundary,segmentId=self.segment,
                               T_current_previous=None,frame=current))
            return events
        if not valid_candidate(motion):
            reason=motion.get('reason') if isinstance(motion,dict) else 'missing_motion_candidate'
            if not reason:reason='invalid_motion_candidate'
            events.append(dict(status='tracking_lost',reason=reason,segmentId=self.segment,
                               T_current_previous=None,frame=current))
            self.last=current;self.pending_new_segment=True
            return events
        transform=motion['T_current_previous']
        if self.pending_new_segment:
            self.segment+=1
            events.append(dict(status='new_segment',reason='tracking_recovered',segmentId=self.segment,
                               T_current_previous=None,frame=self.last))
            self.pending_new_segment=False
        events.append(dict(status='tracking_candidate',reason=None,segmentId=self.segment,
                           T_current_previous=transform,frame=current))
        self.last=current
        return events

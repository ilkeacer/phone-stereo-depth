import unittest
import numpy as np
from host.tracking import MotionTrackingState,boundary_reason,validate_frame


def frame(run,left,right):
    return dict(sourceRun=run,sourceTimestampsNs=[left,right])


def candidate(transform=None):
    return dict(status='candidate',reason=None,
                T_current_previous=(np.eye(4) if transform is None else transform).tolist())


class TrackingTests(unittest.TestCase):
    def test_start_track_loss_and_recovery_create_distinct_segments(self):
        tracker=MotionTrackingState()
        self.assertEqual(tracker.observe(frame('a',100,110))[0]['status'],'new_segment')
        tracked=tracker.observe(frame('a',200,210),candidate())
        self.assertEqual([x['status'] for x in tracked],['tracking_candidate'])
        lost=tracker.observe(frame('a',300,310),dict(status='rejected',reason='pnp_failed'))
        self.assertEqual(lost[0]['status'],'tracking_lost');self.assertIsNone(lost[0]['T_current_previous'])
        recovered=tracker.observe(frame('a',400,410),candidate())
        self.assertEqual([x['status'] for x in recovered],['new_segment','tracking_candidate'])
        self.assertEqual(recovered[0]['reason'],'tracking_recovered')
        self.assertEqual(recovered[0]['segmentId'],2)
        self.assertEqual(recovered[1]['segmentId'],2)

    def test_source_change_and_gap_emit_loss_then_reference(self):
        tracker=MotionTrackingState();tracker.observe(frame('a',100,110))
        changed=tracker.observe(frame('b',200,210),candidate())
        self.assertEqual([x['status'] for x in changed],['tracking_lost','new_segment'])
        self.assertEqual([x['reason'] for x in changed],['source_changed','source_changed'])
        self.assertTrue(all(x['T_current_previous'] is None for x in changed))
        gap=tracker.observe(frame('b',600_000_201,600_000_211),candidate())
        self.assertEqual(gap[0]['reason'],'temporal_gap')
        self.assertEqual(gap[1]['segmentId'],3)

    def test_non_increasing_and_malformed_transform_never_pass(self):
        tracker=MotionTrackingState();tracker.observe(frame('a',100,110))
        repeated=tracker.observe(frame('a',100,120),candidate())
        self.assertEqual(repeated[0]['reason'],'non_increasing_timestamp')
        bad=np.eye(4);bad[0,0]=2
        lost=tracker.observe(frame('a',200,220),candidate(bad))
        self.assertEqual(lost[0]['status'],'tracking_lost')
        self.assertEqual(lost[0]['reason'],'invalid_motion_candidate')
        self.assertIsNone(lost[0]['T_current_previous'])

    def test_missing_candidate_is_loss(self):
        tracker=MotionTrackingState();tracker.observe(frame('a',1,2))
        event=tracker.observe(frame('a',3,4))[0]
        self.assertEqual(event['reason'],'missing_motion_candidate')

    def test_reset_requires_new_reference_without_reusing_segment(self):
        tracker=MotionTrackingState();tracker.observe(frame('a',1,2));tracker.reset()
        event=tracker.observe(frame('a',3,4),candidate())[0]
        self.assertEqual(event['status'],'new_segment');self.assertEqual(event['segmentId'],2)
        self.assertIsNone(event['T_current_previous'])

    def test_validation(self):
        with self.assertRaises(ValueError):MotionTrackingState(0)
        with self.assertRaises(ValueError):validate_frame(frame('',1,2))
        with self.assertRaises(ValueError):validate_frame(frame('a',True,2))
        a=validate_frame(frame('a',1,2));b=validate_frame(frame('a',2,3))
        self.assertIsNone(boundary_reason(a,b))


if __name__=='__main__':unittest.main()

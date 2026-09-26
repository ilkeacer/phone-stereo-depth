import unittest
import numpy as np
from host.trajectory_check import assess,endpoint_metrics


class EndpointTests(unittest.TestCase):
    def test_nominal_scale_never_passes_centimeter_gate(self):
        a=np.array([[0,0,0,0,0,0,0,1],[1,.02,0,0,0,0,0,-1.]])
        result=assess(a,'nominal_display_estimate',True)
        self.assertEqual(result['endpointGate'],'not_assessable');self.assertIsNone(result['endpointTranslationCm'])
        self.assertAlmostEqual(result['endpointRotationDegrees'],0)
        self.assertEqual(assess(a,'measured',False)['endpointGate'],'not_assessable')
        self.assertEqual(assess(a,'measured',True)['endpointGate'],'pass')
        a[-1,1]=.11;self.assertEqual(assess(a,'measured',True)['endpointGate'],'fail')

    def test_rotation_and_invalid_time(self):
        a=np.array([[0,0,0,0,0,0,0,1],[1,0,0,0,0,0,np.sin(np.deg2rad(6)/2),np.cos(np.deg2rad(6)/2)]])
        self.assertEqual(assess(a,'measured',True)['endpointGate'],'fail')
        a[1,0]=0
        with self.assertRaises(ValueError):endpoint_metrics(a)

class TrackingValidityTests(unittest.TestCase):
    def test_identity_pose_requires_nonlost_same_stamp_status(self):
        from types import SimpleNamespace as N
        from host.trajectory_check import tracked_pose_values
        stamp=N(sec=1,nanosec=0)
        msg=N(header=N(stamp=stamp),pose=N(pose=N(position=N(x=0,y=0,z=0),orientation=N(x=0,y=0,z=0,w=1))))
        info=N(header=N(stamp=stamp),lost=True)
        with self.assertRaises(ValueError):tracked_pose_values(msg,info)
        info.lost=False
        self.assertEqual(tracked_pose_values(msg,info),[0,0,0,0,0,0,1])
        info.header=N(stamp=N(sec=2,nanosec=0))
        with self.assertRaises(ValueError):tracked_pose_values(msg,info)

class PredictedScaleTests(unittest.TestCase):
    def test_model_scale_cannot_pass_measured_gate(self):
        a=np.array([[0,0,0,0,0,0,0,1],[1,.01,0,0,0,0,0,1]])
        result=assess(a,'model_predicted_metres',True)
        self.assertEqual(result['endpointGate'],'not_assessable')
        self.assertIsNone(result['endpointTranslationCm'])
        self.assertEqual(result['endpointTranslationPredictedCm'],1)

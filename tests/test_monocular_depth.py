import unittest
import cv2
import numpy as np
from host.monocular_depth import comparison_stats,tracked_depth_change

class MonocularDiagnosticsTests(unittest.TestCase):
    def test_agreement_uses_only_valid_reference_and_is_not_accuracy(self):
        p=np.array([[2.,100],[6,np.nan]])
        z=np.array([[1.,np.nan],[3,0]])
        out=comparison_stats(p,z,np.ones((2,2),bool))
        self.assertEqual(out['pixels'],2)
        self.assertEqual(out['medianRatio'],2)
        self.assertEqual(out['medianAbsoluteRelativeDisagreementPercent'],100)
        self.assertIsNone(comparison_stats(p,z,np.zeros((2,2),bool))['medianRatio'])

    def test_flow_follows_shifted_texture(self):
        rng=np.random.default_rng(10)
        a=rng.integers(0,256,(160,240),dtype=np.uint8)
        b=cv2.warpAffine(a,np.float32([[1,0,2],[0,1,0]]),(240,160))
        first=np.full(a.shape,2.,np.float32);second=first*1.1
        out=tracked_depth_change(a,b,first,second)
        self.assertGreater(out['tracks'],20)
        self.assertAlmostEqual(out['medianChangePercent'],10,places=3)

    def test_blank_frame_cannot_claim_temporal_stability(self):
        a=np.zeros((80,100),np.uint8);d=np.ones(a.shape,np.float32)
        self.assertEqual(tracked_depth_change(a,a,d,d)['tracks'],0)
        self.assertIsNone(tracked_depth_change(a,a,d,d)['medianChangePercent'])

class ModelAlignmentTests(unittest.TestCase):
    def test_prediction_restores_asymmetric_input_coordinates_for_all_rotations(self):
        from contextlib import nullcontext
        from types import SimpleNamespace as N
        from host.monocular_depth import MetricDepthAnything
        class Tensor:
            def __init__(self,a):self.a=a
            def to(self,device):return self
            def __getitem__(self,key):return Tensor(self.a[key])
            def cpu(self):return self
            def numpy(self):return self.a
        class FakeModel:
            def image2tensor(self,image,size):return Tensor(image[:,:,0].astype(np.float32)[None]),image.shape[:2]
            def __call__(self,image):return image
        bgr=np.zeros((3,5,3),np.uint8);bgr[:,:,0]=np.arange(15).reshape(3,5)+1
        for rotation in (0,90,180,270):
            with self.subTest(rotation=rotation):
                model=MetricDepthAnything.__new__(MetricDepthAnything)
                model.device='cpu';model.rotation=rotation;model.input_size=518;model.model=FakeModel()
                model.torch=N(inference_mode=nullcontext,nn=N(functional=N(interpolate=lambda x,*a,**kw:x)))
                depth,_=model.predict(bgr)
                np.testing.assert_array_equal(depth,bgr[:,:,0])
                self.assertEqual(depth.shape,(3,5))

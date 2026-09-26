import unittest
import numpy as np
import cv2
from unittest.mock import patch
from host.stereo_benchmark import measure,matcher
from host.depth import StereoProcessor


class BenchmarkTests(unittest.TestCase):
    def test_host_profile_matches_actual_depth_matcher_calls(self):
        processor=StereoProcessor.__new__(StereoProcessor)
        processor.size=(512,128)
        processor.c={'P2':np.array([[100.,0,0,-10],[0,100,0,0],[0,0,1,0]])}
        rect=[np.zeros((128,512),np.uint8)]*2
        captured=[]
        real_create=cv2.StereoSGBM_create
        class StopAfterMatcher(Exception):pass
        def capture(**kwargs):
            captured.append(real_create(**kwargs))
            raise StopAfterMatcher()
        with patch('host.depth.cv2.StereoSGBM_create',side_effect=capture):
            with self.assertRaises(StopAfterMatcher):processor.compute(rect)
        _,settings=matcher('host_sgbm',0)
        for name,value in settings.items():
            self.assertEqual(getattr(captured[0],'get'+name)(),value,name)

    def test_known_shift_and_common_support_are_respected(self):
        rng=np.random.default_rng(5);left=rng.integers(0,256,(128,512),dtype=np.uint8);right=np.roll(left,-8,axis=1)
        roi=np.zeros(left.shape,bool);roi[25:100,155:355]=True
        regions=dict(full=np.ones_like(roi),commonSupport=roi,central200=roi)
        q=np.array([[1.,0,0,0],[0,1,0,0],[0,0,0,100],[0,0,1,0]])
        for profile in ('rtabmap_bm','controlled_bm','controlled_sgbm','host_sgbm'):
            with self.subTest(profile=profile):
                result,raw,valid=measure([left,right],{'Q':q},profile,regions)
                self.assertGreater(result['regions']['central200']['finalPercent'],90)
                self.assertFalse(valid[~roi].any())
                self.assertAlmostEqual(float(np.median(raw[0][valid]/16)),8,delta=.2)
                self.assertLessEqual(result['regions']['full']['finalPixels'],roi.sum())

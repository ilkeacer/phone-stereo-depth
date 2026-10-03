import unittest

import numpy as np

from host.arcore_detail import ConfirmedMeanVoxels
from host.arcore_gpu_map import confirmed_voxels_gpu


class GpuConfirmedMapTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        try:
            import torch
        except ModuleNotFoundError as error:
            if error.name != 'torch':raise
            raise unittest.SkipTest('Optional CUDA tests require PyTorch') from error
        if not torch.cuda.is_available():raise unittest.SkipTest('CUDA GPU unavailable')

    def test_pixels_from_one_frame_cannot_inflate_independent_support(self):
        points=np.array([[.001,0,0]]*50+[[.005,0,0],[.007,0,0]],np.float32)
        frame_ids=np.array([2]*50+[4,9],np.int64)
        result,support,stats=confirmed_voxels_gpu(points,frame_ids)
        np.testing.assert_allclose(result,[[.013/3,0,0]],atol=1e-8)
        np.testing.assert_array_equal(support,[3]);self.assertEqual(stats['perFrameCells'],3)
        empty,_,_=confirmed_voxels_gpu(points,frame_ids,min_frames=4)
        self.assertEqual(empty.shape,(0,3))

    def test_negative_coordinates_boundaries_and_order_match_cpu(self):
        rng=np.random.default_rng(73)
        chunks=[];ids=[];cpu=ConfirmedMeanVoxels(.01,3)
        for frame in range(5):
            points=np.vstack([rng.uniform(-.08,.08,(500,3)),
                [[-.01,.01,0],[0,0,0],[.02,0,0]]]).astype(np.float32)
            chunks.append(points);ids.append(np.full(len(points),frame*7,np.int64));cpu.add(points,frame*7)
        actual,_,_=confirmed_voxels_gpu(np.concatenate(chunks),np.concatenate(ids))
        np.testing.assert_array_equal(actual,cpu.array())

    def test_capacity_and_bad_inputs_are_rejected(self):
        points=np.array([[0,0,0],[1,0,0]],np.float32);ids=np.array([0,0])
        with self.assertRaisesRegex(ValueError,'Cell limit'):
            confirmed_voxels_gpu(points,ids,cell_limit=1)
        for bad_points,bad_ids in [(points.astype(np.float64),ids),(points,ids.astype(float)),
                                   (points,np.array([-1,0])),(points*np.nan,ids)]:
            with self.assertRaises(ValueError):confirmed_voxels_gpu(bad_points,bad_ids)


if __name__=='__main__':unittest.main()

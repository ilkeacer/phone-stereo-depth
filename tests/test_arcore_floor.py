import tempfile
import unittest
from pathlib import Path

import numpy as np

from host.arcore_floor import analyze, floor_candidate, write_cloud
from host.ros_map_view import read_pcl


class ArCoreFloorTests(unittest.TestCase):
    def test_floor_is_selected_over_larger_wall_and_near_table(self):
        rng = np.random.default_rng(5)
        floor = np.column_stack([rng.uniform(-1, 1, 1500), rng.uniform(.5, 3, 1500),
                                 rng.normal(-1, .01, 1500)])
        wall = np.column_stack([rng.uniform(-1, 1, 2500), np.full(2500, 3.),
                                rng.uniform(-1, 1, 2500)])
        table = np.column_stack([rng.uniform(-.3, .3, 600), rng.uniform(1, 2, 600),
                                 rng.normal(-.25, .01, 600)])
        xyz = np.concatenate([floor, wall, table])
        result = floor_candidate(xyz, np.zeros((10, 3)))
        self.assertIsNotNone(result)
        self.assertLess(abs(result['metrics']['planeHeightAtCameraM'] + 1), .05)
        self.assertGreater(result['mask'][:1500].mean(), .99)
        self.assertLess(result['mask'][1500:4000].mean(), .04)  # Wall intersects the floor plane.
        self.assertEqual(int(result['mask'][4000:].sum()), 0)

    def test_saved_candidate_is_separate_from_source(self):
        rng = np.random.default_rng(9)
        xyz = np.column_stack([rng.uniform(-1, 1, 1200), rng.uniform(0, 2, 1200),
                               rng.normal(-1, .01, 1200)]).astype(np.float32)
        with tempfile.TemporaryDirectory() as temp:
            source = Path(temp) / 'source.ply'
            poses = Path(temp) / 'poses.txt'
            output = Path(temp) / 'new'
            write_cloud(source, xyz)
            poses.write_text('0 0 0 0 0 0 0 1 1.0\n')
            before = source.read_bytes()
            metrics = analyze(source, poses, output)
            self.assertEqual(source.read_bytes(), before)
            self.assertEqual(metrics['status'], 'geometric_candidate')
            accepted, _ = read_pcl(output / 'surface_candidate.ply')
            self.assertGreater(len(accepted), 1100)
            self.assertTrue((output / 'classified_map.ply').exists())


if __name__ == '__main__':
    unittest.main()

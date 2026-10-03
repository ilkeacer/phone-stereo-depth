import unittest

import numpy as np

from host.arcore_observation_grid import observation_grid, summarize


class ObservationGridTests(unittest.TestCase):
    def test_depth_returns_do_not_turn_blank_cells_into_free_space(self):
        cameras = np.array([[0., 0., 1.], [1., 0., 1.]])
        points = np.array([[.1, .1, 0.], [.12, .12, .8], [.8, .1, .7]])
        grid, origin, outside = observation_grid(points, cameras,
                                                 np.array([True, False, False]),
                                                 resolution=.2, padding=.5)
        index = np.floor((points[:, :2] - origin) / .2 + 1e-8).astype(int)
        self.assertEqual(grid[index[0, 1], index[0, 0]], 3)
        self.assertEqual(grid[index[2, 1], index[2, 0]], 2)
        self.assertEqual(outside, 0)
        report = summarize(grid, origin, .2, points=len(points), poses=len(cameras),
                           outside=outside, floor_candidate_found=True)
        self.assertGreater(report['noReturnCells'], 0)
        self.assertFalse(report['noReturnIsFreeSpace'])
        self.assertFalse(report['noReturnProvesUnseen'])
        self.assertEqual(report['bothCells'], 1)

    def test_bad_geometry_is_rejected(self):
        with self.assertRaises(ValueError):
            observation_grid(np.array([[np.nan, 0., 0.]]), np.array([[0., 0., 0.]]),
                             np.array([True]))
        with self.assertRaises(ValueError):
            observation_grid(np.empty((0, 3)), np.empty((0, 3)), np.empty(0, bool))


if __name__ == '__main__':
    unittest.main()

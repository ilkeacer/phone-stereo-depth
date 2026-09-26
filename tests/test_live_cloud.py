import multiprocessing as mp
import unittest
from unittest.mock import Mock
import numpy as np
from host.live_cloud import CloudMailbox,renderable
from host.viewer import Viewer


class LiveCloudTests(unittest.TestCase):
    def test_latest_replaces_previous_and_expiry_is_not_refreshed(self):
        box=CloudMailbox(mp.get_context('spawn'),3)
        for n in range(10):self.assertTrue(box.publish(np.full((2,3),n,np.float32),20.,2,3))
        packet=box.read();self.assertEqual(packet['sequence'],10)
        np.testing.assert_array_equal(packet['xyz'],np.full((2,3),9))
        self.assertTrue(renderable(packet,19.));self.assertFalse(renderable(packet,20.))
        self.assertTrue(box.clear());self.assertFalse(renderable(box.read(),19.))

    def test_contention_never_blocks_or_partially_changes_packet(self):
        box=CloudMailbox(mp.get_context('spawn'),2)
        box.publish(np.ones((2,3)),20.,2,2)
        box.lock.acquire()
        try:
            self.assertIsNone(box.read())
            self.assertFalse(box.publish(np.zeros((1,3)),99.,1,1))
        finally:box.lock.release()
        self.assertEqual(box.read()['expires'],20.)
        np.testing.assert_array_equal(box.read()['xyz'],np.ones((2,3)))

    def test_limits_and_nonfinite_rejected(self):
        box=CloudMailbox(mp.get_context('spawn'),2)
        for points in [np.zeros((3,3)),np.full((1,3),np.nan),np.zeros((2,2))]:
            with self.assertRaises(ValueError):box.publish(points,20.,3,3)

    def test_recorded_mode_clears_live_output(self):
        v=Viewer.__new__(Viewer);v.live_cloud=Mock();v.live_cloud.alive.return_value=True
        v.mode='recorded';v.current_row=None;v.pipeline=None;v.root=Mock();v.closing=False
        v.poll_live_cloud();v.live_cloud.mailbox.clear.assert_called_once()
        v.live_cloud.mailbox.publish.assert_not_called()

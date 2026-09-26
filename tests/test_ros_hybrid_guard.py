import unittest
from host.ros_session import HybridTrackingGuard


class HybridGuardTests(unittest.TestCase):
    def test_loss_is_latched_even_if_tracking_recovers(self):
        guard=HybridTrackingGuard()
        guard.observe(1,False);guard.check()
        guard.observe(2,True)
        guard.observe(3,False)
        self.assertEqual(guard.observed,3)
        self.assertEqual(guard.lost,1)
        with self.assertRaisesRegex(RuntimeError,'tracking lost'):guard.check()

    def test_timestamp_restart_fails_and_progress_is_allowed(self):
        guard=HybridTrackingGuard()
        guard.observe(10,False);guard.observe(20,False);guard.check()
        guard.observe(1,False)
        with self.assertRaisesRegex(RuntimeError,'non-increasing'):guard.check()

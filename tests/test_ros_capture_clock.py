import unittest
from host.ros_capture_clock import MappingClock


class CaptureClockTests(unittest.TestCase):
    def test_preparation_does_not_consume_mapping_time(self):
        clock=MappingClock(90)
        self.assertFalse(clock.expired(999))
        self.assertEqual(clock.status(999)['remainingSeconds'],90)
        clock.admit(1000)
        self.assertEqual(clock.status(1000)['elapsedSeconds'],0)
        self.assertFalse(clock.expired(1089.99))
        self.assertTrue(clock.expired(1090))

    def test_subsequent_pairs_do_not_reset_start_or_extend_deadline(self):
        clock=MappingClock(90);clock.admit(100)
        clock.admit(130);clock.admit(185)
        self.assertEqual(clock.status(185),dict(durationSeconds=90,elapsedSeconds=85,remainingSeconds=5))
        self.assertEqual(clock.status(191)['remainingSeconds'],0)
        self.assertTrue(clock.expired(191))

    def test_gap_does_not_pause_or_restart_clock(self):
        clock=MappingClock(90);clock.admit(100)
        self.assertEqual(clock.status(115.2)['remainingSeconds'],75)
        self.assertEqual(clock.started,100)

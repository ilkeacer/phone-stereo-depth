"""Raw Android inertial messages preserve time and axes without a guessed pose."""
import sys
from types import SimpleNamespace
import unittest
from unittest.mock import patch

from host.ros_imu import gyro_message, nearest_acceleration


class FakeImu:
    def __init__(self):
        self.header = SimpleNamespace(stamp=SimpleNamespace(sec=0, nanosec=0), frame_id='')
        self.orientation = SimpleNamespace(w=0.)
        self.orientation_covariance = [0.] * 9
        self.angular_velocity = SimpleNamespace(x=0., y=0., z=0.)
        self.linear_acceleration = SimpleNamespace(x=0., y=0., z=0.)
        self.linear_acceleration_covariance = [0.] * 9


class RosImuTests(unittest.TestCase):
    def test_nearest_acceleration_has_bounded_time_delta(self):
        gyro = {'timestampNs': 100_000_000}
        samples = [{'timestampNs': 79_000_000}, {'timestampNs': 108_000_000}]
        self.assertEqual(nearest_acceleration(gyro, samples), samples[1])
        self.assertIsNone(nearest_acceleration(gyro, samples[:1]))

    def test_message_preserves_raw_axes_and_ros_timestamp(self):
        gyro = dict(timestampNs=1_200_000_003, x=.1, y=-.2, z=.3)
        accel = dict(timestampNs=1_201_000_000, x=1., y=2., z=9.7)
        with patch.dict(sys.modules, {'sensor_msgs.msg': SimpleNamespace(Imu=FakeImu)}):
            message = gyro_message(gyro, 2_000_000_004, accel)
            absent = gyro_message(gyro, 0)
        self.assertEqual((message.header.stamp.sec, message.header.stamp.nanosec), (3, 200_000_007))
        self.assertEqual(message.header.frame_id, 'phone_imu_android')
        self.assertEqual((message.angular_velocity.x, message.angular_velocity.y, message.angular_velocity.z), (.1, -.2, .3))
        self.assertEqual((message.linear_acceleration.x, message.linear_acceleration.y, message.linear_acceleration.z), (1., 2., 9.7))
        self.assertEqual(message.orientation_covariance[0], -1.)
        self.assertEqual(absent.linear_acceleration_covariance[0], -1.)


if __name__ == '__main__':
    unittest.main()

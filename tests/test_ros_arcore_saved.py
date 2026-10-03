from pathlib import Path
import tempfile
import unittest

import numpy as np

from host.arcore_floor import write_cloud
from host.ros_arcore_saved import load_saved, path_message
from host.ros_arcore_live import cloud_message


class SavedArCoreTests(unittest.TestCase):
    def test_load_saved_cloud_and_poses(self):
        with tempfile.TemporaryDirectory() as temp:
            session = Path(temp)
            write_cloud(session / 'map_cloud.ply', np.array([[1, 2, 3]], dtype=np.float32))
            (session / 'map_poses.txt').write_text('0 0 0 0 0 0 0 1 1.0\n1 1 0 0 0 0 0 1 2.0\n')
            xyz, rgb, poses = load_saved(session)
            np.testing.assert_allclose(xyz, [[1, 2, 3]])
            self.assertEqual((rgb.shape, poses.shape), ((1, 3), (2, 9)))
            with self.assertRaisesRegex(ValueError, 'timestamps'):
                (session / 'map_poses.txt').write_text('0 0 0 0 0 0 0 1 2.0\n1 1 0 0 0 0 0 1 1.0\n')
                load_saved(session)

    def test_ros_messages_keep_saved_colors_and_path(self):
        try:
            from builtin_interfaces.msg import Time
            from rclpy.serialization import deserialize_message, serialize_message
        except ImportError:
            self.skipTest('ROS Python environment required')
        stamp = Time(sec=1)
        cloud = cloud_message(np.asarray([[1, 2, 3]], dtype=np.float32), stamp,
                              np.asarray([[0, 1, 0]], dtype=np.float32))
        path = path_message(np.array([[0, 1, 2, 3, 0, 0, 0, 1, 4.0]]), stamp)
        cloud = deserialize_message(serialize_message(cloud), type(cloud))
        path = deserialize_message(serialize_message(path), type(path))
        self.assertEqual(int(np.frombuffer(cloud.data, dtype=[('x','<f4'),('y','<f4'),('z','<f4'),('rgb','<u4')])['rgb'][0]), 0x00ff00)
        self.assertEqual((path.header.frame_id, len(path.poses), path.poses[0].pose.position.z), ('arcore_map', 1, 3.0))


if __name__ == '__main__':
    unittest.main()

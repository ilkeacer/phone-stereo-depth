from pathlib import Path
import socket
import tempfile
import threading
import unittest

import numpy as np

from host.ros_arcore_live import VoxelMap, converted_packet, save_diagnostic, cloud_message, probe_lines
from host.ros_map_view import read_pcl


class ArCoreLiveTests(unittest.TestCase):
    def test_probe_lines_retries_empty_forward_connection(self):
        with socket.socket() as listener:
            listener.bind(('127.0.0.1', 0))
            listener.listen(2)
            port = listener.getsockname()[1]
            def serve():
                first, _ = listener.accept()
                first.close()  # ADB accepted locally; phone server was not open.
                second, _ = listener.accept()
                with second:
                    second.sendall(b'{"schemaVersion":1}\n')
            worker = threading.Thread(target=serve, daemon=True)
            worker.start()
            self.assertEqual(list(probe_lines(port, startup_seconds=3)), ['{"schemaVersion":1}\n'])
            worker.join(timeout=1)
            self.assertFalse(worker.is_alive())

    def test_packet_transform_voxel_map_and_saved_diagnostic(self):
        packet = dict(schemaVersion=1, timestampNs=1_000_000_000, trackingState='TRACKING',
                      translationM=[1, 2, 3], quaternion=[0, 0, 0, 1],
                      points=[[0, 0, -1], [.01, 0, -1]])
        timestamp, state, position, rotation, points = converted_packet(packet)
        self.assertEqual((timestamp, state), (1_000_000_000, 'TRACKING'))
        np.testing.assert_allclose(position, [1, -3, 2])
        np.testing.assert_allclose(points, [[0, 1, 0], [.01, 1, 0]])
        cells = VoxelMap()
        cells.add(points)
        self.assertEqual(len(cells.cells), 1)
        with tempfile.TemporaryDirectory() as temp:
            output = Path(temp) / 'new'
            summary = save_diagnostic(output, cells, [(timestamp, position, rotation)],
                                      dict(packets=1, trackedPackets=1, pausedPackets=0,
                                           inputPoints=2, droppedPackets=0))
            xyz, rgb = read_pcl(output / 'map_cloud.ply')
            self.assertEqual(len(xyz), 1)
            self.assertEqual(rgb.shape, (1, 3))
            np.testing.assert_allclose(np.loadtxt(output / 'map_poses.txt', ndmin=2)[0, 1:4], position)
            self.assertEqual(summary['points'], 1)
            self.assertEqual(summary['status'], 'diagnostic')
            with self.assertRaises(FileExistsError):
                save_diagnostic(output, cells, [], {})

    def test_bad_packet_rejected(self):
        packet = dict(schemaVersion=1, timestampNs=1, trackingState='PAUSED',
                      translationM=[0, 0, 0], quaternion=[0, 0, 0, 1], points=[[0, 1, 2]])
        with self.assertRaisesRegex(ValueError, 'require tracking'):
            converted_packet(packet)
        packet['trackingState'] = 'TRACKING'
        packet['points'] = [[float('nan'), 1, 2]]
        with self.assertRaisesRegex(ValueError, 'Invalid ARCore points'):
            converted_packet(packet)

    def test_ros_message_schema(self):
        try:
            from rclpy.serialization import serialize_message, deserialize_message
        except ImportError:
            self.skipTest('ROS Python environment required')
        from builtin_interfaces.msg import Time
        msg = cloud_message(np.asarray([[1, 2, 3]], dtype=np.float32), Time(sec=1))
        rebuilt = deserialize_message(serialize_message(msg), type(msg))
        self.assertEqual((rebuilt.header.frame_id, rebuilt.width, rebuilt.point_step), ('arcore_map', 1, 16))


if __name__ == '__main__':
    unittest.main()

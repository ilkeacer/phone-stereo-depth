import json
from pathlib import Path
import tempfile
import unittest

import numpy as np

from host.arcore_probe_import import convert, ros_pose
from host.ros_map_view import read_pcl
from host.ros_scale import saved_map_scale, scale_label


class ArCoreProbeImportTests(unittest.TestCase):
    def test_metric_world_rotation_and_unchanged_sources(self):
        with tempfile.TemporaryDirectory() as temp:
            root = Path(temp)
            cloud = root / 'phone.ply'
            cloud.write_text('ply\nformat ascii 1.0\ncomment local probe\nelement vertex 2\n'
                             'property float x\nproperty float y\nproperty float z\nend_header\n'
                             '1 2 3\n-1 0 4\n')
            report = root / 'phone.jsonl'
            events = [
                dict(event='session_created', depthSupported=True),
                dict(event='frame', trackingState='TRACKING', timestampNs=1_000_000_000,
                     translationM=[1, 2, 3], quaternion=[0, 0, 0, 1]),
                dict(event='depth', sampleCount=2),
                dict(event='result', reason='finished', tracking=1, paused=0, depthFrames=1),
                dict(event='map_export', points=2),
            ]
            report.write_text(''.join(json.dumps(event) + '\n' for event in events))
            originals = cloud.read_bytes(), report.read_bytes()
            output = root / 'converted'
            metadata = convert(report, cloud, output)
            xyz, rgb = read_pcl(output / 'map_cloud.ply')
            np.testing.assert_allclose(xyz, [[1, -3, 2], [-1, -4, 0]])
            self.assertEqual(rgb.shape, (2, 3))
            pose = np.loadtxt(output / 'map_poses.txt', ndmin=2)
            np.testing.assert_allclose(pose[0, 1:4], [1, -3, 2])
            np.testing.assert_allclose(pose[0, 4:8], [2**-.5, 0, 0, 2**-.5])
            self.assertEqual(metadata['status'], 'diagnostic')
            self.assertEqual(saved_map_scale(output / 'map_cloud.ply')['scaleSource'], 'arcore_depth_api')
            self.assertIn('henüz doğrulanmadı', scale_label(saved_map_scale(output / 'map_cloud.ply')))
            self.assertEqual((cloud.read_bytes(), report.read_bytes()), originals)
            with self.assertRaises(FileExistsError):
                convert(report, cloud, output)

    def test_empty_depth_rejected_before_output(self):
        with tempfile.TemporaryDirectory() as temp:
            root = Path(temp)
            cloud = root / 'phone.ply'
            cloud.write_text('ply\nformat ascii 1.0\nelement vertex 1\n'
                             'property float x\nproperty float y\nproperty float z\nend_header\n0 0 1\n')
            report = root / 'phone.jsonl'
            report.write_text('\n'.join(map(json.dumps, [
                dict(event='session_created'),
                dict(event='frame', trackingState='TRACKING', timestampNs=1,
                     translationM=[0, 0, 0], quaternion=[0, 0, 0, 1]),
                dict(event='depth', sampleCount=0),
                dict(event='result', reason='finished', tracking=1, paused=0, depthFrames=1),
                dict(event='map_export', points=1),
            ])) + '\n')
            with self.assertRaisesRegex(ValueError, 'No nonzero'):
                convert(report, cloud, root / 'converted')
            self.assertFalse((root / 'converted').exists())


if __name__ == '__main__':
    unittest.main()

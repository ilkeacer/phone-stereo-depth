import base64
import json
from pathlib import Path
import tempfile
import unittest

import numpy as np

from host.arcore_confirmed_map import build, confirmed_voxels
from host.ros_map_view import read_pcl


def depth_packet(sequence, timestamp):
    depth = np.full((2, 3), 1000, dtype='<u2')
    confidence = np.full((2, 3), 255, dtype='u1')
    encode = lambda array: base64.b64encode(array.tobytes()).decode('ascii')
    return dict(schemaVersion=2, type='depth', sequence=sequence,
                timestampNs=timestamp, trackingState='TRACKING',
                translationM=[0, 0, 0], quaternion=[0, 0, 0, 1],
                depthTimestampNs=timestamp, confidenceTimestampNs=timestamp,
                width=3, height=2, intrinsics=[2, 2, 1, .5],
                rawDepthU16LE=encode(depth), confidenceU8=encode(confidence))


class ConfirmedMapTests(unittest.TestCase):
    def test_two_frames_not_two_pixels_provide_support(self):
        points = np.array([[.001, 0, 0], [.003, 0, 0],
                           [.005, 0, 0], [.011, 0, 0]], dtype=np.float32)
        cloud, support, stats = confirmed_voxels(points, [1, 1, 2, 2], .01, 2)
        self.assertEqual((len(cloud), support.tolist(), stats['perFrameCells']), (1, [2], 3))
        np.testing.assert_allclose(cloud[0], [.0035, 0, 0], atol=1e-7)

    def test_complete_archive_builds_new_map_without_changing_source(self):
        with tempfile.TemporaryDirectory() as temp:
            source = Path(temp) / 'record.jsonl'
            rows = [depth_packet(1, 1_000_000_000),
                    depth_packet(2, 1_200_000_000),
                    dict(schemaVersion=2, type='end', sequence=3, reason='finished')]
            source.write_text(''.join(json.dumps(row) + '\n' for row in rows))
            original = source.read_bytes()
            output = Path(temp) / 'candidate'
            result = build(source, output, resolution=.01, min_frames=2)
            self.assertEqual((result['depthFrames'], result['points'], result['acceptedRawPixels']),
                             (2, 6, 12))
            self.assertEqual(result['sourceSha256Before'], result['sourceSha256After'])
            self.assertEqual(source.read_bytes(), original)
            xyz, _ = read_pcl(output / 'map_cloud.ply')
            self.assertEqual(len(xyz), 6)
            with self.assertRaises(FileExistsError):
                build(source, output)

    def test_incomplete_archive_does_not_create_a_map(self):
        with tempfile.TemporaryDirectory() as temp:
            source = Path(temp) / 'partial.jsonl'
            source.write_text(json.dumps(depth_packet(1, 1_000_000_000)) + '\n')
            output = Path(temp) / 'candidate'
            with self.assertRaises(ValueError):
                build(source, output, min_frames=1)
            self.assertFalse(output.exists())

    def test_deliberate_save_is_complete_but_cancel_and_error_remain_rejected(self):
        with tempfile.TemporaryDirectory() as temp:
            for reason in ('saved_by_user','stopped_by_user','interrupted','update_error'):
                with self.subTest(reason=reason):
                    source=Path(temp)/f'{reason}.jsonl';output=Path(temp)/reason
                    rows=[depth_packet(i+1,(i+1)*100_000_000) for i in range(3)]
                    rows.append(dict(schemaVersion=2,type='end',sequence=4,reason=reason))
                    source.write_text(''.join(json.dumps(row)+'\n' for row in rows));before=source.read_bytes()
                    if reason=='saved_by_user':
                        result=build(source,output)
                        self.assertTrue(result['captureComplete'])
                        self.assertEqual(result['captureEndReason'],reason)
                    else:
                        with self.assertRaises(ValueError):build(source,output)
                        self.assertFalse(output.exists())
                    self.assertEqual(source.read_bytes(),before)


if __name__ == '__main__':
    unittest.main()

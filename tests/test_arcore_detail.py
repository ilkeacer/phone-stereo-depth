import base64
import copy
import json
from pathlib import Path
import tempfile
import unittest

import numpy as np

from host.arcore_detail import (ConfirmedMeanVoxels, DetailSession, ObservedVoxels,
                                decode_packet, paired_metrics, replay, unproject)
from host.arcore_confirmed_map import confirmed_voxels
from host.ros_map_view import read_pcl


def packet(sequence=1, timestamp=1_000_000_000, depth_timestamp=None):
    raw = np.array([[1000, 1000, 2000], [1000, 1000, 2000]], dtype='<u2')
    # Full depth turns the sharp transition into a ramp on this synthetic input.
    smooth = np.array([[1000, 1400, 1800], [1000, 1400, 1800]], dtype='<u2')
    confidence = np.full((2, 3), 255, dtype='u1')
    encode = lambda a: base64.b64encode(a.tobytes()).decode('ascii')
    return dict(schemaVersion=2, type='depth', sequence=sequence, timestampNs=timestamp,
                trackingState='TRACKING', translationM=[0, 0, 0], quaternion=[0, 0, 0, 1],
                depthTimestampNs=depth_timestamp or timestamp-300_000,
                confidenceTimestampNs=timestamp-300_000, smoothTimestampNs=timestamp-200_000,
                width=3, height=2, intrinsics=[2, 2, 1, .5],
                rawDepthU16LE=encode(raw), confidenceU8=encode(confidence), smoothDepthU16LE=encode(smooth))


class DetailTests(unittest.TestCase):
    def test_projection_uses_pose_and_native_depth_axes(self):
        d = np.array([[1000]], dtype='<u2')
        # ARCore +90 around Y turns camera forward toward -X.
        q = np.array([0, np.sqrt(.5), 0, np.sqrt(.5)])
        xyz = unproject(d, np.ones((1, 1), bool), [1, 1, 0, 0], [2, 3, 4], q)
        np.testing.assert_allclose(xyz, [[1, -4, 3]], atol=1e-6)

    def test_confidence_and_distinct_frames_control_support(self):
        first = packet()
        confidence = np.array([[127, 128, 255], [0, 128, 255]], dtype='u1')
        first['confidenceU8'] = base64.b64encode(confidence.tobytes()).decode('ascii')
        session = DetailSession()
        self.assertTrue(session.add(first))
        self.assertEqual(session.counts['rawAcceptedPixels'], 4)
        self.assertEqual(len(session.raw.array()), 0)
        duplicate = copy.deepcopy(first)
        duplicate.update(sequence=2, timestampNs=first['timestampNs']+100_000_000)
        self.assertFalse(session.add(duplicate))
        self.assertEqual(len(session.raw.array()), 0)
        fresh = copy.deepcopy(duplicate)
        fresh.update(sequence=3, timestampNs=first['timestampNs']+200_000_000,
                     depthTimestampNs=first['depthTimestampNs']+200_000_000)
        self.assertTrue(session.add(fresh))
        self.assertEqual(len(session.raw.array()), 4)

    def test_raw_holes_are_not_filled_with_smooth_reference(self):
        p = packet()
        p['confidenceU8'] = base64.b64encode(bytes(6)).decode('ascii')
        session = DetailSession(min_observations=1)
        session.add(p)
        self.assertEqual(len(session.raw.array()), 0)
        self.assertEqual(len(session.smooth.array()), 6)

    def test_missing_smooth_frame_is_not_in_paired_comparison(self):
        p = packet(); p['smoothDepthU16LE'] = None
        session = DetailSession(min_observations=1)
        session.add(p)
        self.assertEqual(len(session.raw.array()), 6)
        self.assertEqual(len(session.paired_raw.array()), 0)
        self.assertEqual(len(session.smooth.array()), 0)

    def test_bad_payload_and_paused_depth_rejected(self):
        for key, value in [('width', 5), ('rawDepthU16LE', 'no'), ('trackingState', 'PAUSED'),
                           ('intrinsics', [0, 1, 0, 0]), ('quaternion', [0, 0, 0, 0])]:
            with self.subTest(key=key):
                p = packet(); p[key] = value
                with self.assertRaises(ValueError): decode_packet(p)

    def test_voxel_requires_separate_frames_without_averaging(self):
        m = ObservedVoxels()
        a = np.array([[.001, 0, 0], [.015, 0, 0]], dtype=np.float32)
        m.add(a, 1)
        self.assertEqual(len(m.array()), 0)
        m.add(a[:1], 2)
        np.testing.assert_allclose(m.array(), a[:1])

    def test_confirmed_online_matches_batch_and_counts_frames_once(self):
        frames = [np.array([[.001, 0, 0], [.003, 0, 0], [.021, 0, 0]], np.float32),
                  np.array([[.005, 0, 0], [.023, 0, 0]], np.float32),
                  np.array([[.007, 0, 0], [.025, 0, 0]], np.float32)]
        online = ConfirmedMeanVoxels(resolution=.01, min_observations=3)
        for index, points in enumerate(frames):
            online.add(points, index)
            online.add(points, index)  # Repeated writes cannot create support.
        cloud, support, _ = confirmed_voxels(
            np.concatenate(frames), np.concatenate([np.full(len(p), i) for i, p in enumerate(frames)]),
            resolution=.01, min_frames=3)
        np.testing.assert_allclose(online.array(), cloud, atol=1e-7)
        np.testing.assert_array_equal(support, [3, 3])

    def test_confirmed_map_reports_capacity_loss(self):
        grid = ConfirmedMeanVoxels(resolution=.01, min_observations=1, limit=1)
        grid.add(np.array([[0, 0, 0], [.02, 0, 0]], np.float32), 1)
        self.assertEqual(len(grid.array()), 1)
        self.assertEqual(grid.capacity_rejections, 1)

        session = DetailSession(resolution=.01, min_observations=1, raw_fusion='confirmed_mean')
        session.raw.limit = 1
        session.add(packet())
        with tempfile.TemporaryDirectory() as temp:
            self.assertTrue(session.save(temp)['mapTruncated'])

    def test_confirmed_session_keeps_reference_at_two_centimeters(self):
        session = DetailSession(resolution=.01, min_observations=3, raw_fusion='confirmed_mean')
        for i in range(3):
            self.assertTrue(session.add(packet(i + 1, 1_000_000_000 + i * 100_000_000)))
        self.assertEqual(session.raw.resolution, .01)
        self.assertEqual(session.smooth.resolution, .02)
        self.assertEqual(len(session.raw.array()), 6)
        self.assertEqual(len(session.paired_raw.array()), 6)

    def test_replay_preserves_sources_and_separates_outputs(self):
        with tempfile.TemporaryDirectory() as temp:
            source = Path(temp) / 'source.jsonl'
            rows = [packet(), packet(2, 1_200_000_000), dict(schemaVersion=2,type='end',sequence=3,reason='finished')]
            source.write_text(''.join(json.dumps(p)+'\n' for p in rows))
            original = source.read_bytes()
            output = Path(temp) / 'out'
            result = replay(source, output)
            self.assertTrue(result['captureComplete'])
            self.assertEqual((result['depthFrames'], result['pairedDepthFrames'], result['points']), (2, 2, 6))
            raw, _ = read_pcl(output / 'map_cloud.ply')
            smooth, _ = read_pcl(output / 'smooth_reference.ply')
            self.assertFalse(np.array_equal(raw, smooth))
            np.testing.assert_allclose(np.unique(raw[:, 1]), [1, 2])
            np.testing.assert_allclose(np.unique(smooth[:, 1]), [1, 1.4, 1.8])
            self.assertEqual(source.read_bytes(), original)
            self.assertEqual((output/'stream.jsonl').read_bytes(), original)
            with self.assertRaises(FileExistsError): replay(source, output)

    def test_no_end_marker_is_incomplete(self):
        session = DetailSession();session.add(packet())
        with tempfile.TemporaryDirectory() as temp:
            result = session.save(temp)
            self.assertFalse(result['captureComplete'])
        with self.assertRaises(ValueError): session.add(packet())
        with tempfile.TemporaryDirectory() as temp:
            source = Path(temp)/'incomplete.jsonl'
            source.write_text(json.dumps(packet())+'\n')
            result = replay(source,Path(temp)/'out')
            self.assertEqual(result['captureEndReason'],'eof_without_end_marker')

    def test_manual_save_is_complete_in_diagnostic_replay_but_errors_are_not(self):
        with tempfile.TemporaryDirectory() as temp:
            for reason in ('saved_by_user','stopped_by_user','interrupted','update_error'):
                with self.subTest(reason=reason):
                    source=Path(temp)/f'{reason}.jsonl'
                    rows=[packet(),packet(2,1_200_000_000),dict(schemaVersion=2,type='end',sequence=3,reason=reason)]
                    source.write_text(''.join(json.dumps(p)+'\n' for p in rows));before=source.read_bytes()
                    result=replay(source,Path(temp)/reason)
                    self.assertEqual(result['captureComplete'],reason=='saved_by_user')
                    self.assertEqual(result['captureEndReason'],reason)
                    self.assertEqual(source.read_bytes(),before)


if __name__ == '__main__':
    unittest.main()

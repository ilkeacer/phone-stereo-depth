import unittest
import json
from pathlib import Path
import tempfile
import cv2
import numpy as np
from host.depth_consistency import (BODY_FROM_OPTICAL, fit_scale, optical_relative,
                                    pose_at, pose_matrix, read_poses, reproject, sample, tracks)


class DepthConsistencyTests(unittest.TestCase):
    def test_pose_timestamp_rounding_and_loss_guard(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            (root/'replay-input.json').write_text('{"provenance":{}}')
            (root/'capture.json').write_text('{"odometryScaleSource":"nominal_display_estimate"}')
            (root/'lifecycle.json').write_text('{"odometryFrames":[["odom","phone_link"]]}')
            (root/'odometry-raw.txt').write_text('1.140426066 0 0 0 0 0 0 1\n')
            (root/'odometry-status.jsonl').write_text('{"stamp":1.1404260659999999,"lost":false}\n')
            self.assertEqual(list(read_poses(root)[0]), [1140426066])
            with (root/'odometry-status.jsonl').open('a') as f:
                f.write(json.dumps(dict(stamp=1.2, lost=True))+'\n')
            with self.assertRaisesRegex(ValueError, 'uninterrupted'):
                read_poses(root)

    def test_pose_interpolation_is_bounded_and_rotates(self):
        a = np.eye(4)
        b = pose_matrix([2, 0, 0, 0, 0, np.sin(np.pi/4), np.cos(np.pi/4)])
        poses = {100: a, 300: b}
        mid, info = pose_at(poses, 200, 200)
        np.testing.assert_allclose(mid[:3, 3], [1, 0, 0])
        np.testing.assert_allclose(mid[:3, :3] @ [1, 0, 0], [2**-.5, 2**-.5, 0])
        self.assertEqual(info['weight'], .5)
        for stamp, gap in [(99, 200), (301, 200), (200, 199)]:
            self.assertIsNone(pose_at(poses, stamp, gap)[0])
        np.testing.assert_array_equal(pose_at(poses, 100)[0], a)

    def test_ros_optical_axes_and_forward_camera_motion(self):
        # Optical right/down/forward = body -left/-up/forward.
        np.testing.assert_allclose(BODY_FROM_OPTICAL[:3, :3], [[0, 0, 1], [-1, 0, 0], [0, -1, 0]])
        a = np.eye(4)
        b = np.eye(4)
        b[0, 3] = .2
        relative = optical_relative(a, b)
        np.testing.assert_allclose(relative[:3, 3], [0, 0, -.2])
        uv, z, valid = reproject(np.array([[0., 0.]]), np.array([2.]), np.eye(3), relative)
        np.testing.assert_allclose(uv, [[0, 0]])
        np.testing.assert_allclose(z, [1.8])
        self.assertTrue(valid[0])

    def test_known_lateral_translation_requires_correct_depth_scale(self):
        k = np.array([[100., 0, 50], [0, 100, 40], [0, 0, 1]])
        t = np.eye(4)
        t[0, 3] = -.1
        p = np.array([[50., 40.]])
        correct, _, _ = reproject(p, np.array([2.]), k, t)
        doubled, _, _ = reproject(p, np.array([4.]), k, t)
        np.testing.assert_allclose(correct, [[45, 40]])
        np.testing.assert_allclose(doubled, [[47.5, 40]])

    def test_relative_roundtrip_with_rotated_and_translated_body(self):
        a = pose_matrix([1, 2, 3, 0, 0, np.sin(.2), np.cos(.2)])
        b = pose_matrix([2, 3, 1, np.sin(.1), 0, 0, np.cos(.1)])
        np.testing.assert_allclose(optical_relative(b, a) @ optical_relative(a, b), np.eye(4), atol=1e-12)

    def test_invalid_and_behind_camera_depth_cannot_count(self):
        t = np.eye(4)
        t[2, 3] = -2
        with np.errstate(invalid='ignore'):
            uv, _, valid = reproject(np.zeros((2, 2)), np.array([1., np.nan]), np.eye(3), t)
        self.assertFalse(valid.any())
        self.assertTrue(np.isnan(uv).all())

    def test_sampling_outside_does_not_use_border_or_nan(self):
        d = np.array([[2., np.nan], [0., 3.]])
        values = sample(d, np.array([[-1., 0], [0, 0], [1, 0], [0, 1], [1, 1], [np.nan, 0]]))
        np.testing.assert_allclose(values, [np.nan, 2, np.nan, np.nan, 3, np.nan], equal_nan=True)

    def test_fit_does_not_weight_one_textured_frame_more(self):
        self.assertEqual(fit_scale([np.full(1000, 2.), np.full(10, 4.)]), 3.)
        with self.assertRaises(ValueError):
            fit_scale([])

    def test_tracks_recover_known_image_translation(self):
        first = np.random.default_rng(12).integers(0, 256, (120, 160), dtype=np.uint8)
        second = cv2.warpAffine(first, np.float32([[1, 0, 3], [0, 1, -2]]), (160, 120))
        a, b, _ = tracks(first, second)
        self.assertGreater(len(a), 20)
        np.testing.assert_allclose(np.median(b-a, axis=0), [3, -2], atol=.05)
        self.assertEqual(len(tracks(np.zeros_like(first), np.zeros_like(first))[0]), 0)

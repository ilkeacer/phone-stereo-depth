import base64
import json
from pathlib import Path
import tempfile
import unittest

import cv2
import numpy as np

from host.arcore_ai_map import (build, decode_rgb, edge_safe_mask, reference_scale,
                                rgb_focal_pixels, texture_samples, upright_rotation)
from host.ros_map_view import read_pcl


class AiMapTests(unittest.TestCase):
    def test_gravity_rotation_for_portrait_and_ambiguous_overhead(self):
        self.assertEqual(upright_rotation([0, 0, 0, 1]), 0)
        self.assertEqual(upright_rotation([0, 0, -np.sqrt(.5), np.sqrt(.5)]), 90)
        self.assertEqual(upright_rotation([0, 0, np.sqrt(.5), np.sqrt(.5)]), 270)
        with self.assertRaises(ValueError):
            upright_rotation([np.sqrt(.5), 0, 0, np.sqrt(.5)])

    def test_pixel_center_mapping_handles_crop_and_rotation(self):
        image = np.arange(24, dtype=np.float32).reshape(4, 6)
        np.testing.assert_array_equal(texture_samples(image, [0, 1, 6, 1, 0, 3, 6, 3], (6, 2)), image[1:3])
        np.testing.assert_array_equal(texture_samples(image, [6, 4, 0, 4, 6, 0, 0, 0], (6, 4)), image[::-1, ::-1])
        with self.assertRaises(ValueError):
            texture_samples(image, [-10, 0, 6, 0, -10, 4, 6, 4], (6, 4))

    def test_scale_uses_reference_and_rejects_missing_support(self):
        raw = np.ones((30, 30), np.float32)
        prediction = np.full_like(raw, 2)
        prediction[:3] = 9  # An outlying patch must not set the scalar.
        scale, metrics = reference_scale(prediction, raw, np.full(raw.shape, 255, np.uint8))
        self.assertAlmostEqual(scale, .5)
        self.assertGreater(metrics['scaleReferencePixels'], 500)
        with self.assertRaises(ValueError):
            reference_scale(prediction, raw, np.zeros(raw.shape, np.uint8))

    def test_discontinuity_filter_preserves_surfaces_without_blurring(self):
        depth = np.array([[1, 1, 1, 2, 2, 2]], np.float32)
        mask = edge_safe_mask(depth, np.ones(depth.shape, bool))
        np.testing.assert_array_equal(mask, [[True, True, False, False, True, True]])
        np.testing.assert_array_equal(depth, [[1, 1, 1, 2, 2, 2]])

    def test_recorded_focal_uses_crop_and_upright_axis_and_rejects_shear(self):
        packet = dict(textureToRgbCornersPx=[0,60,640,60,0,420,640,420])
        decoded = dict(raw=np.zeros((90,160)), intrinsics=np.array([120,119,80,45]))
        self.assertEqual(rgb_focal_pixels(packet, decoded, 0), 480)
        self.assertEqual(rgb_focal_pixels(packet, decoded, 90), 476)
        packet['textureToRgbCornersPx'][-1] = 450
        with self.assertRaises(ValueError):
            rgb_focal_pixels(packet, decoded, 90)
        packet['textureToRgbCornersPx'] = [640,60,0,60,640,420,0,420]
        with self.assertRaises(ValueError):
            rgb_focal_pixels(packet, decoded, 90)

    def test_complete_synthetic_archive_is_new_and_unmodified(self):
        encode = lambda a: base64.b64encode(a.tobytes()).decode()
        raw = np.full((30, 30), 1000, '<u2')
        confidence = np.full(raw.shape, 255, 'u1')
        _, jpeg = cv2.imencode('.jpg', np.zeros((30, 30, 3), np.uint8))
        packets = []
        for i in range(3):
            packets.append(dict(schemaVersion=2, type='depth', sequence=i+1,
                                timestampNs=(i+1)*100_000_000, depthTimestampNs=(i+1)*100_000_000,
                                confidenceTimestampNs=(i+1)*100_000_000, trackingState='TRACKING',
                                translationM=[0, 0, 0], quaternion=[0, 0, 0, 1], width=30, height=30,
                                intrinsics=[30, 30, 15, 15], rawDepthU16LE=encode(raw),
                                confidenceU8=encode(confidence), rgbTimestampNs=(i+1)*100_000_000,
                                rgbWidth=30, rgbHeight=30, rgbEncoding='jpeg', rgbJpegBase64=encode(jpeg),
                                textureToRgbCornersPx=[0, 0, 30, 0, 0, 30, 30, 30]))
        packets.append(dict(schemaVersion=2, type='end', sequence=4, reason='finished'))

        class Model:
            metadata = dict(model='synthetic test double')
            def predict(self, image):
                return np.full(image.shape[:2], 2, np.float32), 1.

        with tempfile.TemporaryDirectory() as temp:
            source, output = Path(temp)/'source.jsonl', Path(temp)/'output'
            source.write_text(''.join(json.dumps(p)+'\n' for p in packets))
            before = source.read_bytes()
            result = build(source, output, Model(), output_factor=1)
            self.assertEqual(result['predictedFrames'], 3)
            self.assertEqual(result['skippedFrames'], 0)
            self.assertFalse(result['metricAccuracyValidated'])
            xyz, _ = read_pcl(output/'map_cloud.ply')
            np.testing.assert_allclose(xyz[:, 1], 1, atol=1e-6)
            self.assertEqual(source.read_bytes(), before)
            for reason in ('saved_by_user','stopped_by_user','interrupted','update_error'):
                with self.subTest(reason=reason):
                    ended=[*packets[:-1],dict(packets[-1],reason=reason)]
                    reason_source=Path(temp)/f'{reason}.jsonl'
                    reason_source.write_text(''.join(json.dumps(p)+'\n' for p in ended))
                    reason_output=Path(temp)/reason
                    if reason=='saved_by_user':
                        manual=build(reason_source,reason_output,Model())
                        self.assertEqual(manual['captureEndReason'],reason)
                        self.assertEqual(manual['predictedFrames'],3)
                    else:
                        with self.assertRaises(ValueError):build(reason_source,reason_output,Model())
                        self.assertFalse(reason_output.exists())
            with self.assertRaises(FileExistsError):
                build(source, output, Model())

            class FocalModel(Model):
                def predict(self, image, *, focal_px):
                    self.last_focal = focal_px
                    return super().predict(image)

            focal_model = FocalModel()
            second = build(source, Path(temp)/'heldout', focal_model, use_recorded_focal=True,
                           exclude_frames=[1], save_depths=True)
            self.assertEqual(second['predictedFrames'], 2)
            self.assertEqual(second['excludedFrames'], 1)
            self.assertEqual(focal_model.last_focal, 30)
            self.assertFalse((Path(temp)/'heldout/predictions/0001.npz').exists())
            with np.load(Path(temp)/'heldout/predictions/0000.npz') as cached:
                self.assertEqual(int(cached['timestampNs']), packets[0]['timestampNs'])
                self.assertEqual(int(cached['depthTimestampNs']), packets[0]['depthTimestampNs'])
            self.assertEqual(source.read_bytes(), before)
            oversized = dict(packets[0], rgbWidth=100_000)
            with self.assertRaises(ValueError):
                decode_rgb(oversized)
            with self.assertRaises(ValueError):
                decode_rgb(dict(packets[0], rgbWidth=1, rgbHeight=1))

            class FailedModel(Model):
                def predict(self, image):
                    raise RuntimeError('synthetic GPU failure')

            failed = Path(temp)/'failed'
            with self.assertRaises(RuntimeError):
                build(source, failed, FailedModel())
            self.assertEqual(json.loads((failed/'result.json').read_text())['status'], 'failed')
            self.assertEqual(source.read_bytes(), before)
            packets[0]['rgbTimestampNs'] += 20_000_000
            with self.assertRaises(ValueError):
                decode_rgb(packets[0])


if __name__ == '__main__':
    unittest.main()

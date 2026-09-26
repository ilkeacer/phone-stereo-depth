import unittest

from host.scene_quality import brightness_signal


class SceneQualityTests(unittest.TestCase):
    def test_dark_and_bright_mono8_with_row_padding(self):
        dark = bytes([30] * 16 + [255, 255]) * 16
        bright = bytes([160] * 16 + [0, 0]) * 16
        self.assertTrue(brightness_signal(dark, 16, 16, 18)['lowLight'])
        result = brightness_signal(bright, 16, 16, 18)
        self.assertFalse(result['lowLight'])
        self.assertEqual(result['medianGray'], 160)
        self.assertEqual(result['pixelsBelow50Percent'], 0)

    def test_invalid_layout_is_rejected(self):
        with self.assertRaises(ValueError):brightness_signal(bytes(8), 8, 2, 8)

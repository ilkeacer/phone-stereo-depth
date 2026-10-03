import unittest
from host.ros_ai_depth import checked_left
from tests.test_ros_sgbm import inputs


class AIDepthInputTests(unittest.TestCase):
    def test_left_image_and_projection_must_match_source(self):
        images,infos,projections=inputs()
        frame,stamp=checked_left(images[0],infos[0],projections[0],(6,4),None)
        self.assertEqual(frame.shape,(4,6))
        self.assertEqual(stamp,1_000_000_000)
        with self.assertRaisesRegex(ValueError,'Reused'):
            checked_left(images[0],infos[0],projections[0],(6,4),stamp)
        infos[0].p=infos[0].p.copy();infos[0].p[2]+=1
        with self.assertRaisesRegex(ValueError,'calibration differs'):
            checked_left(images[0],infos[0],projections[0],(6,4),None)

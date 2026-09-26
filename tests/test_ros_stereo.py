import unittest
import numpy as np
from host.ros_stereo import metric_projection,inspect_recording
import json
from pathlib import Path
import tempfile


class ProjectionTests(unittest.TestCase):
    def test_metric_baseline_does_not_rescale_intrinsics_or_mutate_source(self):
        p=np.array([[100.,0,30,-200],[0,100,20,0],[0,0,1,0]])
        result=metric_projection(p,25.)
        np.testing.assert_array_equal(result[:,:3],p[:,:3])
        self.assertEqual(result[0,3],-5.)
        self.assertAlmostEqual(-result[0,3]/result[0,0],.05)
        self.assertEqual(p[0,3],-200.)

    def test_invalid_physical_scale_rejected(self):
        for size in [0,-1,np.nan,np.inf]:
            with self.assertRaises(ValueError):metric_projection(np.eye(3,4),size)

    def test_invalid_projection_rejected(self):
        for p in [np.eye(3),np.zeros((3,4)),np.full((3,4),np.nan)]:
            with self.assertRaises(ValueError):metric_projection(p,25.)

    def test_derived_recording_must_disclose_unrecovered_loss(self):
        with tempfile.TemporaryDirectory() as temp:
            root=Path(temp);cal=root/'calibration.npz'
            cal.write_bytes(b'fixture')
            cal.with_suffix('.json').write_text('{}')
            (root/'manifest.json').write_text(json.dumps(dict(status='complete',derivedSubset=True,
                originalCaptureStatus='failed',sourceTrackingLossNotRecovered=False,
                separateMapOriginRequired=True,originalFrameIndexRange=[521,598],frames=78)))
            with self.assertRaisesRegex(ValueError,'provenance incomplete'):
                inspect_recording(root,cal)


if __name__=='__main__':unittest.main()

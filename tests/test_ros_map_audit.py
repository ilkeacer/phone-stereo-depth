"""A rejected one-pose capture should remain auditable."""
import json
from pathlib import Path
import tempfile
import unittest

from host.ros_map_audit import audit


class MapAuditTests(unittest.TestCase):
    def test_zero_or_one_pose_reports_unavailable_trajectory(self):
        with tempfile.TemporaryDirectory() as folder:
            root=Path(folder)
            for name,data in [('summary.json',{'graph':{},'scaleSource':'measured'}),
                              ('capture.json',{'recordingPairs':1}),
                              ('export-result.json',{'status':'skipped'})]:
                (root/name).write_text(json.dumps(data))
            path=root/'odometry-raw.txt'
            for count in (0,1):
                path.write_text('# odometry\n'+('1 0 0 0 0 0 0 1\n' if count else ''))
                result=audit(root)
                self.assertEqual(result['trajectory']['rawOdometryPoseCount'],count)
                self.assertIsNone(result['trajectory']['rawOdometry'])
                self.assertEqual(result['exportStatus'],'skipped')


if __name__=='__main__':unittest.main()

import struct
from pathlib import Path
import tempfile
import unittest
import numpy as np
from host.ros_map_view import read_pcl


class PclTests(unittest.TestCase):
    def test_pcl_with_trailing_camera_and_truncated_vertex(self):
        header=b'ply\nformat binary_little_endian 1.0\nelement vertex 1\nproperty float x\nproperty float y\nproperty float z\nproperty uchar red\nproperty uchar green\nproperty uchar blue\nelement face 0\nelement camera 1\nproperty float view_px\nend_header\n'
        with tempfile.TemporaryDirectory() as temp:
            path=Path(temp)/'cloud.ply';path.write_bytes(header+struct.pack('<fffBBBf',1,2,3,255,0,0,9))
            xyz,rgb=read_pcl(path)
            np.testing.assert_array_equal(xyz,[[1,2,3]])
            np.testing.assert_array_equal(rgb,[[1,0,0]])
            path.write_bytes(header+b'bad')
            with self.assertRaisesRegex(ValueError,'Truncated'):read_pcl(path)

    def test_nonfinite_cloud_rejected(self):
        with tempfile.TemporaryDirectory() as temp:
            path=Path(temp)/'cloud.ply'
            header='ply\nformat binary_little_endian 1.0\nelement vertex 1\n'+''.join('property '+kind+' '+name+'\n' for kind,name in [('float','x'),('float','y'),('float','z'),('uchar','red'),('uchar','green'),('uchar','blue')])+'end_header\n'
            path.write_bytes(header.encode()+struct.pack('<fffBBB',float('nan'),2,3,0,0,0))
            with self.assertRaisesRegex(ValueError,'Nonfinite'):read_pcl(path)

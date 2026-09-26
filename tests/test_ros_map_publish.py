import struct
from pathlib import Path
import tempfile
import unittest
import numpy as np
from host.ros_map_publish import map_messages,saved_last_transform


class SavedMapMessageTests(unittest.TestCase):
    def test_saved_map_uses_separate_frame_and_preserves_geometry(self):
        try:from rclpy.serialization import serialize_message,deserialize_message
        except ImportError: self.skipTest('ROS Python environment required')
        with tempfile.TemporaryDirectory() as temp:
            root=Path(temp)
            header='ply\nformat binary_little_endian 1.0\nelement vertex 1\n'+''.join('property '+kind+' '+name+'\n' for kind,name in [('float','x'),('float','y'),('float','z'),('uchar','red'),('uchar','green'),('uchar','blue')])+'end_header\n'
            cloud=root/'cloud.ply';cloud.write_bytes(header.encode()+struct.pack('<fffBBB',1,2,3,255,128,0))
            poses=root/'poses.txt';poses.write_text('0 1 2 3 0 0 0 1 1\n')
            msg,path=map_messages(cloud,poses)
            rebuilt=deserialize_message(serialize_message(msg),type(msg))
            self.assertEqual(rebuilt.header.frame_id,'saved_map');self.assertEqual(path.header.frame_id,'saved_map')
            self.assertEqual(struct.unpack('<fffI',bytes(rebuilt.data)),(1.,2.,3.,0xff8000))
            self.assertEqual(path.poses[0].pose.position.z,3.)
            marker=saved_last_transform(path,msg.header.stamp)
            self.assertEqual(marker.header.frame_id,'saved_map')
            self.assertEqual(marker.child_frame_id,'saved_phone_last')
            self.assertEqual(marker.transform.translation.x,1.)
            self.assertEqual(marker.transform.translation.y,2.)
            self.assertEqual(marker.transform.translation.z,3.)
            self.assertEqual(marker.transform.rotation.w,1.)
            poses.write_text('0 1 2 3 0 0 0 0 1\n')
            with self.assertRaisesRegex(ValueError,'dönüş'):map_messages(cloud,poses)

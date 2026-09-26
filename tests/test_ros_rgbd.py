import unittest
import numpy as np
from host.ros_rgbd import validate_frame,rgbd_messages
try:
    from sensor_msgs.msg import Image,CameraInfo
    from rclpy.serialization import serialize_message,deserialize_message
    ROS=True
except ImportError:ROS=False

class RGBDValidationTests(unittest.TestCase):
    def test_reject_wrong_shape_units_and_nonpositive_depth(self):
        image=np.zeros((3,4),np.uint8);depth=np.ones((3,4),np.float32)
        validate_frame(image,depth,(4,3))
        for invalid in [depth.astype(np.uint16),depth[:2],depth*0,depth*np.inf]:
            with self.assertRaises(ValueError):validate_frame(image,invalid,(4,3))
        depth[0,0]=np.nan;validate_frame(image,depth,(4,3))
        depth[:]=np.nan
        with self.assertRaises(ValueError):validate_frame(image,depth,(4,3))

@unittest.skipUnless(ROS,'ROS message runtime required')
class RGBDMessageTests(unittest.TestCase):
    def test_exact_stamp_aligned_float_metres_no_baseline(self):
        image=np.arange(12,dtype=np.uint8).reshape(3,4)
        depth=np.arange(1,13,dtype=np.float32).reshape(3,4)/10
        p=np.array([[100.,0,2,0],[0,100,1,0],[0,0,1,0]])
        messages=list(rgbd_messages(image,depth,p,1_123456789))
        self.assertEqual(len(messages),3)
        for topic,msg in messages:
            restored=deserialize_message(serialize_message(msg),type(msg))
            self.assertEqual(restored.header.stamp.sec*10**9+restored.header.stamp.nanosec,1_123456789)
            self.assertEqual(restored.header.frame_id,'phone_left_optical')
        d=messages[1][1];self.assertEqual(d.encoding,'32FC1');self.assertEqual(d.step,16)
        np.testing.assert_array_equal(np.frombuffer(bytes(d.data),dtype='<f4').reshape(3,4),depth)
        np.testing.assert_array_equal(messages[2][1].k,p[:,:3].ravel())
        p[0,3]=-5
        with self.assertRaises(ValueError):list(rgbd_messages(image,depth,p,1_123456789))

@unittest.skipUnless(ROS,'ROS message runtime required')
class RGBDQuantizedMessagesTests(unittest.TestCase):
    def test_uint16_transport_roundtrip_keeps_metre_scale_and_invalids(self):
        image=np.zeros((2,3),np.uint8);z=np.array([[.1234,1.9998,np.nan],[2.,5.,19.999]],np.float32)
        p=np.array([[100.,0,1,0],[0,100,1,0],[0,0,1,0]])
        depth=list(rgbd_messages(image,z,p,123,'16UC1'))[1][1]
        self.assertEqual(depth.encoding,'16UC1');self.assertEqual(depth.step,6)
        restored=np.frombuffer(bytes(depth.data),dtype='<u2').reshape(z.shape)/1000
        self.assertEqual(restored[0,2],0)
        self.assertLessEqual(np.max(abs(restored[np.isfinite(z)]-z[np.isfinite(z)])),.000501)

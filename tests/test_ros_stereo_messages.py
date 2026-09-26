"""Run under /opt/ros/humble/setup.bash with system Python."""
import unittest
import numpy as np
from host.ros_stereo import stereo_messages
try:
    from sensor_msgs.msg import Image,CameraInfo
    from rclpy.serialization import serialize_message,deserialize_message
    ROS=True
except ImportError:
    ROS=False


@unittest.skipUnless(ROS,'ROS message runtime required')
class MessageTests(unittest.TestCase):
    def test_serialized_images_info_and_exposure_offsets(self):
        rect=[np.arange(24,dtype=np.uint8).reshape(4,6)]*2
        p=np.array([[100.,0,3,0],[0,100,2,0],[0,0,1,0]])
        right=p.copy();right[0,3]=-5
        messages=list(stereo_messages(rect,[p,right],[1_000_000_001,1_018_000_001]))
        for topic,msg,stamp in messages:
            restored=deserialize_message(serialize_message(msg),type(msg))
            self.assertEqual(restored.header.stamp.sec*10**9+restored.header.stamp.nanosec,stamp)
            if isinstance(msg,Image):
                self.assertEqual(bytes(restored.data),rect[0].tobytes())
                self.assertEqual(restored.step,6)
            else:
                self.assertEqual(list(restored.d),[0.]*5)
                np.testing.assert_array_equal(restored.r,np.eye(3).ravel())
        self.assertEqual(messages[2][2]-messages[0][2],18_000_000)
        self.assertEqual(messages[3][1].p[3],-5)
        self.assertEqual(messages[0][1].header.frame_id,'phone_left_optical')
        self.assertEqual(messages[2][1].header.frame_id,'phone_right_optical')


if __name__=='__main__':unittest.main()

import copy
from types import SimpleNamespace as N
import unittest
import numpy as np
from host.ros_sgbm import CheckedStereoDepth,checked_pair,depth_message,local_cloud_message
from host.ros_session import online_depth_requested
try:
    from std_msgs.msg import Header
    ROS=True
except ImportError:ROS=False


def inputs():
    projections=[np.array([[100.,0,3,0],[0,100,2,0],[0,0,1,0]]),np.array([[100.,0,3,-10],[0,100,2,0],[0,0,1,0]])]
    images=[];infos=[]
    for side,stamp,p in zip(('left','right'),(0,20_000_000),projections):
        header=N(stamp=N(sec=1,nanosec=stamp),frame_id='phone_'+side+'_optical')
        images.append(N(header=header,width=6,height=4,encoding='mono8',step=8,data=bytes(range(32))))
        infos.append(N(header=copy.deepcopy(header),width=6,height=4,distortion_model='plumb_bob',d=[0.]*5,k=p[:,:3].ravel(),r=np.eye(3).ravel(),p=p.ravel()))
    return images,infos,projections


class SGBMContractTests(unittest.TestCase):
    def test_source_padding_and_twenty_ms_boundary(self):
        images,infos,projections=inputs()
        arrays,stamps=checked_pair(images,infos,projections,(6,4))
        np.testing.assert_array_equal(arrays[0],np.arange(32).reshape(4,8)[:,:6])
        self.assertEqual(stamps,(1_000_000_000,1_020_000_000))
        images[1].header.stamp.nanosec+=1
        with self.assertRaisesRegex(ValueError,'20 ms'):checked_pair(images,infos,projections,(6,4))

    def test_wrong_camera_info_timestamp_baseline_or_frame_rejected(self):
        for change in ('stamp','baseline','frame','reuse'):
            images,infos,projections=inputs()
            if change=='stamp':infos[0].header.stamp.nanosec+=1
            if change=='baseline':infos[1].p=infos[1].p.copy();infos[1].p[3]=-20
            if change=='frame':images[0].header.frame_id='other'
            previous=(1_000_000_000,1_020_000_000) if change=='reuse' else None
            with self.subTest(change=change),self.assertRaises(ValueError):
                checked_pair(images,infos,projections,(6,4),previous)

    def test_known_plane_stereo_depth_and_support(self):
        rng=np.random.default_rng(4);left=rng.integers(0,256,(160,320),dtype=np.uint8)
        right=np.zeros_like(left);right[:,:-16]=left[:,16:]
        engine=CheckedStereoDepth.__new__(CheckedStereoDepth)
        q=np.array([[1.,0,0,-160],[0,1,0,-80],[0,0,0,100],[0,0,.2,0]])
        engine.processor=N(size=(320,160),c={'Q':q});engine.scale=.02
        region=np.zeros_like(left,bool);region[30:130,150:260]=True
        engine.regions=dict(full=np.ones_like(region),commonSupport=region)
        depth,stats=engine.compute([left,right])
        self.assertGreater(np.isfinite(depth).sum(),1000)
        self.assertAlmostEqual(float(np.nanmedian(depth)),.625,places=3)
        self.assertFalse(np.isfinite(depth[~region]).any())
        self.assertEqual(stats['regions']['full']['finalPixels'],np.isfinite(depth).sum())

    def test_engine_routing_does_not_double_publish_cached_depth(self):
        self.assertFalse(online_depth_requested('live',False,'auto','stereo'))
        self.assertTrue(online_depth_requested('live',False,'sgbm','stereo'))
        self.assertFalse(online_depth_requested('live',True,'auto','stereo'))
        self.assertFalse(online_depth_requested('replay',False,'auto','hybrid'))
        self.assertTrue(online_depth_requested('replay',False,'sgbm','stereo'))
        for kind in ('hybrid','rgbd'):
            with self.assertRaises(ValueError):online_depth_requested('replay',False,'sgbm',kind)


@unittest.skipUnless(ROS,'ROS runtime required')
class DepthTransportTests(unittest.TestCase):
    def test_millimetres_invalid_empty_and_unchanged_left_stamp(self):
        h=Header();h.stamp.sec=8;h.stamp.nanosec=123;h.frame_id='phone_left_optical'
        d=np.array([[.625,np.nan],[1.1234,65.535]],np.float32)
        msg=depth_message(d,h)
        self.assertEqual(msg.header,h)
        np.testing.assert_array_equal(np.frombuffer(bytes(msg.data),'<u2').reshape(2,2),[[625,0],[1123,65535]])
        self.assertEqual(bytes(depth_message(np.full((2,2),np.nan,np.float32),h).data),bytes(8))
        for invalid in (0.,.0005,-1.,np.inf,66.):
            with self.assertRaises(ValueError):depth_message(np.array([[invalid]],np.float32),h)

    def test_local_cloud_has_metric_camera_points_and_omits_invalid_depth(self):
        h=Header();h.stamp.sec=8;h.frame_id='phone_left_optical'
        depth=np.full((4,4),np.nan,np.float32);depth[0,0]=2.;depth[0,2]=3.;depth[2,0]=9.;depth[2,2]=1.
        left=np.zeros((4,4),np.uint8);left[0,0]=20;left[0,2]=100;left[2,2]=255
        projection=np.array([[2.,0,1,0],[0,2.,1,0],[0,0,1,0]])
        cloud=local_cloud_message(depth,left,projection,h,stride=2)
        self.assertEqual(cloud.header,h)
        self.assertEqual((cloud.height,cloud.width,cloud.point_step,cloud.row_step),(1,3,16,48))
        points=np.frombuffer(bytes(cloud.data),dtype=[('x','<f4'),('y','<f4'),('z','<f4'),('rgb','<u4')])
        np.testing.assert_allclose(np.column_stack([points['x'],points['y'],points['z']]),
                                   [[-1.,-1.,2.],[1.5,-1.5,3.],[.5,.5,1.]])
        self.assertEqual(points['rgb'].tolist(),[0x141414,0x646464,0xffffff])

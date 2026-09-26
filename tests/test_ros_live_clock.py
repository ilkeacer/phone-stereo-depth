"""Exercise the publisher loop with a fake camera/ROS clock; no device commands."""
from contextlib import ExitStack
import json
from pathlib import Path
from types import SimpleNamespace
import tempfile
import unittest
from unittest.mock import Mock,patch
import numpy as np
from host import ros_live


class PublisherClockTests(unittest.TestCase):
    def run_publisher(self,slow_at=None,no_packets=False,seconds=90,measured_config=False):
        now=[0.];published=[];statuses=[]
        fake_time=SimpleNamespace(monotonic=lambda:now[0],sleep=lambda _:None)
        node=Mock()
        node.get_clock.return_value.now.return_value.nanoseconds=0
        node.create_publisher.return_value.publish.side_effect=lambda msg:published.append(now[0]) if msg=='image' else None
        ros=Mock();ros.ok.return_value=True;ros.create_node.return_value=node
        ros.shutdown.side_effect=lambda:setattr(ros.ok,'return_value',False)
        qos=SimpleNamespace(QoSProfile=Mock(),DurabilityPolicy=SimpleNamespace(TRANSIENT_LOCAL=1),ReliabilityPolicy=SimpleNamespace(RELIABLE=1))
        modules={'rclpy':ros,'rclpy.qos':qos,'sensor_msgs.msg':SimpleNamespace(Image=object,CameraInfo=object,Imu=object),'tf2_msgs.msg':SimpleNamespace(TFMessage=object)}
        def read():
            now[0]+=1
            return None if no_packets else 'packet'
        def rectify(_):
            if slow_at is not None and now[0]==slow_at:now[0]+=2
            return [np.zeros((2,2),np.uint8)]*2
        pair=SimpleNamespace(source='test',header={k:{'image':{'timestampSource':1}} for k in ('20','21')},
                             timestamps=(1,2),blobs=(b'',b''),age_upper=lambda _:0.)
        pair.header['deviceElapsedNs']=0
        with tempfile.TemporaryDirectory() as folder,ExitStack() as stack:
            root=Path(folder);cal=root/'calibration.npz'
            p1=np.array([[100.,0,3,0],[0,100,2,0],[0,0,1,0]])
            p2=p1.copy();p2[0,3]=-5
            np.savez(cal,P1=p1,P2=p2,lengthUnit='checker_square')
            cal.with_suffix('.json').write_text(json.dumps(dict(ids=['20','21'],lengthUnit='checker_square')))
            scale_args=['--estimated-square-mm','22']
            if measured_config:
                from host.ros_scale import explicit_scale
                settings=explicit_scale(21.44,'measured',cal)
                settings['scaleMeasurement']=dict(approximate=True,samePhysicalDisplayConfirmed=True,
                    reportedApproximateSpansMm=dict(AB=107.2,CD=107.2),squaresPerSpan=5)
                config=root/'scale.json';config.write_text(json.dumps(settings))
                scale_args=['--scale-config',str(config)]
            processor=Mock(c={'P1':p1,'P2':p2});processor.rectify.side_effect=rectify
            stack.enter_context(patch.dict('sys.modules',modules))
            stack.enter_context(patch('sys.argv',['ros_live','--calibration',str(cal),*scale_args,'--seconds',str(seconds),'--report',str(root/'capture.json')]))
            stack.enter_context(patch.object(ros_live,'__file__',str(root/'host/ros_live.py')))
            for name,value in dict(time=fake_time,StereoProcessor=Mock(return_value=processor),metric_projection=lambda p,s:p,
                    run_command=Mock(return_value=SimpleNamespace(stdout='')),authorized_devices=lambda _:['test'],
                    read_pair=read,packet_reason=lambda *a:None,ReceivedPair=SimpleNamespace(create=lambda *a:pair),
                    camera_transforms=lambda _:None,
                    stereo_messages=lambda *a:[('/phone/left/image_rect','image',0)],
                    atomic_json=lambda path,data:statuses.append(data)).items():
                stack.enter_context(patch.object(ros_live,name,value))
            stack.enter_context(patch.object(ros_live.cv2,'imdecode',return_value=np.zeros((2,2),np.uint8)))
            if no_packets:
                with self.assertRaisesRegex(RuntimeError,'20 seconds'):ros_live.main()
            else:ros_live.main()
            stats=json.loads((root/'capture.json').read_text())
        return published,statuses,stats

    def test_real_loop_records_ninety_seconds_after_preparation(self):
        times,statuses,stats=self.run_publisher()
        self.assertEqual(times[0],5)
        self.assertEqual(times[-1],94)
        self.assertEqual(len(times),90)
        active=[s for s in statuses if s['stage']=='mapping']
        self.assertEqual(active[0]['elapsedSeconds'],0)
        self.assertEqual(active[0]['remainingSeconds'],90)
        self.assertEqual(stats['mappingElapsedSeconds'],90)
        self.assertEqual(stats['warmupSeconds'],5)
        self.assertEqual(stats['rosStampOffsetNs'],0)
        self.assertEqual(stats['rosStampFormula'],
                         'ros_image_stamp_ns = source_image_timestamp_ns + rosStampOffsetNs')
        self.assertEqual(stats['adbSerial'],'test')
        self.assertEqual(stats['adbTransport'],'usb')

    def test_live_report_preserves_approximate_measured_scale(self):
        times,_,stats=self.run_publisher(measured_config=True)
        self.assertEqual(len(times),90)
        self.assertEqual(stats['squareMm'],21.44)
        self.assertEqual(stats['scaleSource'],'measured')
        self.assertTrue(stats['scaleMeasurement']['approximate'])
        self.assertFalse(stats['metricAccuracyValidated'])

    def test_pair_finishing_rectification_after_deadline_is_not_published(self):
        times,_,_=self.run_publisher(slow_at=94)
        self.assertEqual(times[-1],93)
        self.assertNotIn(96,times)

    def test_no_first_pair_still_has_bounded_startup_failure(self):
        times,statuses,stats=self.run_publisher(no_packets=True)
        self.assertEqual(times,[])
        self.assertIn('20 seconds',stats['error'])
        self.assertEqual(statuses[-1]['stage'],'finished')
        self.assertIsNotNone(statuses[-1]['error'])

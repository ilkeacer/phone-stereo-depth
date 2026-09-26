"""Bounded live phone stereo publisher for experimental ROS mapping.

Owns the camera session; no motor commands or invented base_link extrinsics.
"""
import argparse
import collections
import fcntl
import json
import math
import os
from pathlib import Path
import threading
import time
import cv2
import numpy as np
from host.contracts import ReceivedPair
from host.depth import StereoProcessor
from host.device import run_command,timing_args,select_adb_serial
from host.motion_capture import authorized_devices,packet_reason,PACKAGE,CaptureWriter
from host.ros_stereo import metric_projection,stereo_messages,camera_transforms,sha
from host.transport import read_pair
from host.ros_diagnostic import DiagnosticWriter
from host.motion_recording import atomic_json
from host.ros_capture_clock import MappingClock
from host.ros_scale import add_scale_options,scale_from_args
from host.ros_imu import gyro_message,nearest_acceleration


def main():
    import rclpy
    from rclpy.qos import QoSProfile,DurabilityPolicy,ReliabilityPolicy
    from sensor_msgs.msg import Image,CameraInfo,Imu
    from tf2_msgs.msg import TFMessage
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--calibration',type=Path,required=True)
    add_scale_options(parser)
    parser.add_argument('--seconds',type=int,default=120)
    parser.add_argument('--report',type=Path,required=True)
    parser.add_argument('--diagnostic-dir',type=Path,help='Save bounded 30-second raw motion diagnostic')
    parser.add_argument('--recording-dir',type=Path,help='Preserve the complete admitted mapping stream for offline replay')
    args=parser.parse_args()
    if not 5<=args.seconds<=600:parser.error('--seconds must be 5..600')
    if args.report.exists():raise FileExistsError('Report already exists')
    if args.diagnostic_dir is not None and args.diagnostic_dir.exists():
        raise FileExistsError('Diagnostic directory already exists')
    if args.recording_dir is not None and args.diagnostic_dir is not None:parser.error('Choose diagnostic or full recording')
    if args.recording_dir is not None and args.recording_dir.exists():raise FileExistsError('Recording directory already exists')
    report=json.loads(args.calibration.with_suffix('.json').read_text())
    with np.load(args.calibration) as source:cal={k:source[k] for k in source.files}
    if report['ids']!=['20','21'] or report['lengthUnit']!='checker_square' or str(cal['lengthUnit'])!='checker_square':
        raise ValueError('Wrong calibration camera IDs or unit')
    if not np.allclose(cal['P1'][:,3],0) or cal['P2'][0,3]>=0 or not np.allclose(cal['P2'][1:,3],0):
        raise ValueError('Expected positive-baseline horizontal stereo')
    processor=StereoProcessor(cal,.5);cv2.setNumThreads(2)
    scale=scale_from_args(args);size=scale['squareMm']
    projections=[metric_projection(processor.c[f'P{i}'],size) for i in (1,2)]
    project=Path(__file__).resolve().parents[1];(project/'work').mkdir(exist_ok=True)
    lock=(project/'work/assistant.lock').open('w')
    fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
    stop=threading.Event();configured=False;node=None;count=0;rejects=collections.Counter();error=None;adb_prefix=None
    imu_stream=None;imu_counts=collections.Counter();imu_dropped_on_phone=0
    diagnostic=None;diagnostic_complete=False;recording=None;recording_complete=False
    clock=MappingClock(args.seconds)
    args.report.parent.mkdir(parents=True,exist_ok=True)
    status_path=args.report.parent/'live-status.json'
    atomic_json(status_path,dict(stage='warming_up',updatedMonotonic=time.monotonic()))
    stats=dict(scale,robotExtrinsicsAvailable=False,
        timePolicy='ROS clock anchored to Android REALTIME snapshot once; exposure offsets preserved; host transport timing uncertainty remains')
    try:
        if args.recording_dir is not None:
            recording=CaptureWriter(args.recording_dir,report,dict(calibrationSha256=sha(args.calibration),calibrationReportSha256=sha(args.calibration.with_suffix('.json'))),max_frames=12000,max_bytes=1024*1024*1024)
            recording.finish('in_progress')
        if args.diagnostic_dir is not None:
            diagnostic=DiagnosticWriter(args.diagnostic_dir,report,dict(
                calibrationSha256=sha(args.calibration),
                calibrationReportSha256=sha(args.calibration.with_suffix('.json'))))
        devices=authorized_devices(run_command(['adb','devices','-l'],stop).stdout)
        serial=select_adb_serial(devices,os.environ.get('PHONE_ADB_SERIAL'))
        adb_prefix=['adb','-s',serial]
        stats.update(adbSerial=serial,adbTransport='wifi' if ':' in serial else 'usb',
                     transportScope='ADB port-forwarded stereo; Android sensor timestamps retained')
        run_command([*adb_prefix,'shell','am','force-stop',PACKAGE],stop);configured=True
        run_command([*adb_prefix,'forward','tcp:8765','tcp:8765'],stop)
        run_command([*adb_prefix,'shell','am','start','-n',f'{PACKAGE}/.MainActivity','--es','ids','20,21',
            '--ei','seconds',str(args.seconds+30),'--ei','width','1280','--ei','height','960',
            '--ez','fixed','true','--ez','live','true','--ef','focus','1.4','--ei','iso','400',*timing_args(30)],stop)
        rclpy.init();node=rclpy.create_node('phone_stereo_live')
        publishers={f'/phone/{side}/{suffix}':node.create_publisher(kind,f'/phone/{side}/{suffix}',2)
            for side in ('left','right') for suffix,kind in [('image_rect',Image),('camera_info',CameraInfo)]}
        imu_publisher=node.create_publisher(Imu,'/phone/imu_raw',64)
        imu_stream=(args.report.parent/'imu.jsonl').open('x')
        tf=node.create_publisher(TFMessage,'/tf_static',QoSProfile(depth=1,
            reliability=ReliabilityPolicy.RELIABLE,durability=DurabilityPolicy.TRANSIENT_LOCAL))
        tf.publish(camera_transforms(projections))
        begin=time.monotonic();last_good=begin;source=None;previous=None;offset=None
        last_imu_sequence=None;recent_accelerations=collections.deque(maxlen=64)
        warmup_source=None;settling_started=begin;last_status=0.
        def publish_inertial(pair):
            nonlocal last_imu_sequence,imu_dropped_on_phone
            header=pair.header
            imu_dropped_on_phone=max(imu_dropped_on_phone,header.get('imuDroppedTotal',0))
            if 'imuSensors' in header:stats['imuSensors']=header['imuSensors']
            samples=header.get('imuSamples',[])
            candidates=list(recent_accelerations)+[s for s in samples if s['kind']=='accel']
            for sample in samples:
                sequence=sample['sequence']
                if last_imu_sequence is not None and sequence<=last_imu_sequence:
                    imu_counts['duplicateSequence']+=1;continue
                if last_imu_sequence is not None and sequence>last_imu_sequence+1:
                    imu_counts['sequenceGaps']+=sequence-last_imu_sequence-1
                last_imu_sequence=sequence
                row=dict(sample,sourceRun=pair.source,rosStampNs=sample['timestampNs']+offset)
                if sample['kind']=='gyro':
                    acceleration=nearest_acceleration(sample,candidates)
                    if acceleration is not None:
                        row['matchedAccelerationTimestampNs']=acceleration['timestampNs']
                        row['accelerationTimeDeltaNs']=acceleration['timestampNs']-sample['timestampNs']
                        imu_counts['gyroWithAcceleration']+=1
                    imu_publisher.publish(gyro_message(sample,offset,acceleration))
                    imu_counts['gyroPublished']+=1
                else:
                    imu_counts['accelerometerRecorded']+=1
                imu_stream.write(json.dumps(row,allow_nan=False)+'\n')
                imu_counts['samplesRecorded']+=1
            recent_accelerations.extend(s for s in samples if s['kind']=='accel')
        def expired(now):
            return now-begin>=args.seconds if diagnostic is not None else clock.expired(now)
        while rclpy.ok() and not expired(time.monotonic()):
            if time.monotonic()-last_good>20:raise RuntimeError('No usable fresh camera pair for 20 seconds')
            started=time.monotonic()
            try:packet=read_pair()
            except (OSError,ConnectionError):time.sleep(.05);continue
            received=time.monotonic()
            if expired(received):break
            if packet is None:continue
            pair=ReceivedPair.create(packet,started,received)
            if source is None and pair.source!=warmup_source:
                warmup_source=pair.source;settling_started=received
            reason=packet_reason(pair,report,processor.input_size,source,previous,received)
            if source is not None and reason in ('source_changed','geometry_mismatch'):
                raise RuntimeError(reason)
            if any(pair.header[c]['image'].get('timestampSource')!=1 for c in ('20','21')):
                raise RuntimeError('REALTIME sensor clock required')
            if reason or received-settling_started<4:
                if offset is not None and pair.source==source:publish_inertial(pair)
                rejects[reason or 'warmup']+=1;time.sleep(.02);continue
            if offset is None:
                offset=node.get_clock().now().nanoseconds-pair.header['deviceElapsedNs']
                source=pair.source;stats['sourceRun']=source
                # Preserve the exact conversion used for published image stamps.
                # It lets a later audit locate a lost OdomInfo stamp in the raw
                # capture without guessing from subscriber image counts.
                stats['rosStampOffsetNs']=int(offset)
                stats['rosStampFormula']='ros_image_stamp_ns = source_image_timestamp_ns + rosStampOffsetNs'
            publish_inertial(pair)
            images=[cv2.imdecode(np.frombuffer(blob,np.uint8),0) for blob in pair.blobs]
            rect=processor.rectify(images)
            if pair.age_upper(time.monotonic())>1:rejects['stale_after_rectification']+=1;continue
            if expired(time.monotonic()):break
            stamps=[t+offset for t in pair.timestamps]
            for topic,msg,_ in stereo_messages(rect,projections,stamps):publishers[topic].publish(msg)
            previous=pair.timestamps;last_good=time.monotonic();count+=1
            clock.admit(last_good)
            if recording is not None:recording.save(pair,'room_mapping',time.monotonic())
            if last_good-last_status>=.5:
                atomic_json(status_path,dict(stage='diagnostic' if diagnostic else 'mapping',
                    updatedMonotonic=last_good,**(dict(remainingSeconds=max(0,math.ceil(args.seconds-(last_good-begin))))
                                                  if diagnostic else clock.status(last_good)),
                    publishedPairs=count))
                imu_stream.flush()
                last_status=last_good
            if diagnostic is not None and diagnostic.observe(pair,last_good):
                diagnostic_complete=True;break
            rclpy.spin_once(node,timeout_sec=0);time.sleep(.01)
        recording_complete=bool(rclpy.ok() and clock.expired(time.monotonic()) and count>0)
        stats.update(mappingElapsedSeconds=clock.elapsed(time.monotonic()),
                     warmupSeconds=None if clock.started is None else clock.started-begin)
    except KeyboardInterrupt:
        pass
    except Exception as exc:
        error=str(exc);raise
    finally:
        if imu_stream is not None:imu_stream.close()
        if node is not None:node.destroy_node()
        if rclpy.ok():rclpy.shutdown()
        if configured and adb_prefix:
            for command in (['adb','shell','am','force-stop',PACKAGE],['adb','forward','--remove','tcp:8765']):
                try:run_command([*adb_prefix,*command[1:]],threading.Event(),timeout=3)
                except Exception:pass
        lock.close()
        stats.update(publishedPairs=count,rejections=dict(rejects),error=error,
                     imuCounts=dict(imu_counts),imuDroppedOnPhone=imu_dropped_on_phone,
                     imuLogPath=str(args.report.parent/'imu.jsonl') if imu_stream is not None else None,
                     imuScope='Raw Android device axes; no camera-to-IMU TF or SLAM fusion')
        if diagnostic is not None:
            diagnostic.finish('failed' if error else 'complete' if diagnostic_complete else 'cancelled',error)
            stats.update(diagnosticDirectory=str(args.diagnostic_dir),
                         diagnosticPairs=diagnostic.writer.frames,diagnosticComplete=diagnostic_complete)
        if recording is not None:
            recording.finish('failed' if error else 'complete' if recording_complete else 'cancelled',error)
            path=recording.directory/'manifest.json';manifest=json.loads(path.read_text())
            manifest.update(guideMode='full mapping stream after camera warm-up; timer starts on first admitted pair',scope='all admitted mapping pairs; physical trajectory not ground truth',requestedDurationSeconds=args.seconds)
            atomic_json(path,manifest)
            stats.update(recordingDirectory=str(recording.directory),recordingPairs=recording.frames,recordingComplete=recording_complete)
        args.report.write_text(json.dumps(stats,indent=2))
        atomic_json(status_path,dict(stage='finished',updatedMonotonic=time.monotonic(),error=error))


if __name__=='__main__':main()

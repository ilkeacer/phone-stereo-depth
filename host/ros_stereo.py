"""Validated recorded stereo -> ROS 2 bag. Run with Humble's system Python.

Requires an explicitly measured or explicitly estimated calibration square.
Source exposure offsets survive replay unchanged. Estimated scale is for demos.
"""
import argparse
import hashlib
import json
from pathlib import Path
import cv2
import numpy as np
from host.contracts import check_geometry
from host.depth import StereoProcessor
from host.motion_recording import committed_pairs
from host.ros_scale import add_scale_options,scale_from_args,explicit_scale


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def inspect_recording(trial,calibration):
    report=json.loads(calibration.with_suffix('.json').read_text())
    manifest=json.loads((trial/'manifest.json').read_text())
    if manifest.get('status')!='complete':
        raise ValueError('Only completed recordings may be exported')
    if manifest.get('derivedSubset'):
        lo,hi=manifest.get('originalFrameIndexRange',[None,None])
        if (manifest.get('originalCaptureStatus') not in ('complete','failed','cancelled') or
                manifest.get('sourceTrackingLossNotRecovered') is not True or
                manifest.get('separateMapOriginRequired') is not True or
                type(lo) is not int or type(hi) is not int or hi-lo+1!=manifest.get('frames')):
            raise ValueError('Derived recording provenance incomplete')
    if report.get('ids')!=['20','21'] or report.get('lengthUnit')!='checker_square':
        raise ValueError('Expected camera 20/21 checker-square calibration')
    expected_provenance=dict(calibrationSha256=sha(calibration),
                             calibrationReportSha256=sha(calibration.with_suffix('.json')))
    if manifest.get('provenance')!=expected_provenance:
        raise ValueError('Recording calibration hashes differ')
    provenance=dict(expected_provenance)
    if manifest.get('derivedSubset'):
        provenance.update(derivedSubset=True,
            originalCaptureStatus=manifest['originalCaptureStatus'],
            originalCaptureError=manifest.get('originalCaptureError'),
            originalSourceRun=manifest.get('originalSourceRun'),
            originalFrameIndexRange=manifest['originalFrameIndexRange'],
            trackingLossRecovered=False,mapContinuityVerified=False,
            separateMapOriginRequired=True,
            sourceManifestSha256=sha(trial/'manifest.json'))
    with np.load(calibration) as source:
        cal={key:source[key] for key in source.files}
    if str(cal['lengthUnit'])!='checker_square':
        raise ValueError('Calibration unit differs')
    for name in ('P1','P2'):
        if cal[name].shape!=(3,4) or not np.isfinite(cal[name]).all():
            raise ValueError('Invalid projection matrix')
    if (not np.allclose(cal['P1'][:,3],0) or cal['P2'][0,3]>=0
            or not np.allclose(cal['P2'][1:,3],0)):
        raise ValueError('Expected positive-baseline horizontal left/right stereo')
    processor=StereoProcessor(cal,.5)
    pairs=list(committed_pairs(trial))
    if not pairs or len(pairs)!=manifest.get('frames'):
        raise ValueError('Recording pair count differs')
    previous=None
    for a,b,_ in pairs:
        header={camera:dict(image=item,capture=item['capture'])
                for camera,item in zip(('20','21'),(a,b))}
        check_geometry(header,report,processor.input_size)
        timestamps=tuple(item['imageTimestampNs'] for item in (a,b))
        if any(type(t) is not int or t<0 for t in timestamps):
            raise ValueError('Invalid sensor timestamp')
        if any(item.get('timestampSource')!=1 for item in (a,b)):
            raise ValueError('Shared REALTIME sensor clock required')
        if previous and any(t<=old for t,old in zip(timestamps,previous)):
            raise ValueError('Non-increasing sensor timestamps')
        previous=timestamps
        for item in (a,b):
            image=cv2.imread(str(trial/item['sampleFile']),0)
            if image is None or image.shape[::-1]!=processor.input_size:
                raise ValueError('Missing or incorrectly sized image')
    return processor,pairs,dict(**provenance,pairs=len(pairs),
        maxStereoDeltaMs=max(p[2] for p in pairs)/1e6,
        sourceSpanSeconds=(pairs[-1][0]['imageTimestampNs']-pairs[0][0]['imageTimestampNs'])/1e9)


def metric_projection(projection,square_mm):
    if not np.isfinite(square_mm) or square_mm<=0:
        raise ValueError('A positive measured square size in mm is required')
    result=np.array(projection,dtype=float,copy=True)
    if (result.shape!=(3,4) or not np.isfinite(result).all() or result[0,0]<=0
            or result[1,1]<=0 or not np.allclose(result[2],[0,0,1,0])):
        raise ValueError('Invalid rectified projection')
    result[:,3]*=square_mm/1000.
    return result


def stereo_messages(rect,projections,timestamps):
    from sensor_msgs.msg import Image,CameraInfo
    for side,im,p,stamp in zip(('left','right'),rect,projections,timestamps):
        image=Image()
        image.header.stamp.sec,image.header.stamp.nanosec=divmod(int(stamp),1_000_000_000)
        image.header.frame_id=f'phone_{side}_optical'
        image.height,image.width=im.shape
        image.encoding='mono8';image.is_bigendian=0;image.step=image.width
        image.data=np.ascontiguousarray(im).tobytes()
        info=CameraInfo();info.header=image.header
        info.height=image.height;info.width=image.width
        # The published images are already rectified, so K/R/D describe those images.
        info.distortion_model='plumb_bob';info.d=[0.]*5
        info.k=p[:,:3].ravel().tolist();info.r=np.eye(3).ravel().tolist()
        info.p=p.ravel().tolist()
        yield f'/phone/{side}/image_rect',image,stamp
        yield f'/phone/{side}/camera_info',info,stamp


def camera_transforms(projections):
    from geometry_msgs.msg import TransformStamped
    from tf2_msgs.msg import TFMessage
    body=TransformStamped();body.header.frame_id='phone_link';body.child_frame_id='phone_left_optical'
    body.transform.rotation.x=-.5;body.transform.rotation.y=.5
    body.transform.rotation.z=-.5;body.transform.rotation.w=.5
    right=TransformStamped();right.header.frame_id='phone_left_optical';right.child_frame_id='phone_right_optical'
    right.transform.rotation.w=1.
    right.transform.translation.x=float(-projections[1][0,3]/projections[1][0,0])
    return TFMessage(transforms=[body,right])


def export_bag(trial,calibration,output,square_mm,scale_source='measured',scale_settings=None):
    import rosbag2_py
    from rclpy.serialization import serialize_message
    if scale_source not in ('measured','nominal_display_estimate'):
        raise ValueError('Unknown scale source')
    settings=explicit_scale(square_mm,scale_source,calibration)
    if scale_settings is not None:
        for key in ('squareMm','scaleSource','calibrationSha256','calibrationReportSha256'):
            if scale_settings.get(key)!=settings[key]:raise ValueError('Export scale settings differ')
        settings.update(scale_settings)
    processor,pairs,evidence=inspect_recording(trial,calibration)
    projections=[metric_projection(processor.c[f'P{i}'],square_mm) for i in (1,2)]
    if output.exists():raise FileExistsError('Output already exists')
    output.parent.mkdir(parents=True,exist_ok=True)
    writer=rosbag2_py.SequentialWriter()
    writer.open(rosbag2_py.StorageOptions(uri=str(output),storage_id='sqlite3'),
                rosbag2_py.ConverterOptions('',''))
    for side in ('left','right'):
        for suffix,kind in [('image_rect','Image'),('camera_info','CameraInfo')]:
            writer.create_topic(rosbag2_py.TopicMetadata(name=f'/phone/{side}/{suffix}',
                type=f'sensor_msgs/msg/{kind}',serialization_format='cdr'))
    writer.create_topic(rosbag2_py.TopicMetadata(name='/tf_static',type='tf2_msgs/msg/TFMessage',
        serialization_format='cdr',offered_qos_profiles=json.dumps([dict(
            history=1,depth=1,reliability=1,durability=1,liveliness=1,
            deadline=dict(sec=9223372036,nsec=854775807),
            lifespan=dict(sec=9223372036,nsec=854775807),
            liveliness_lease_duration=dict(sec=9223372036,nsec=854775807),
            avoid_ros_namespace_conventions=False)])))
    writer.write('/tf_static',serialize_message(camera_transforms(projections)),1_000_000_000)
    origin=min(item['imageTimestampNs'] for item in pairs[0][:2])
    count=0
    try:
        for a,b,_ in pairs:
            rect=processor.rectify([cv2.imread(str(trial/item['sampleFile']),0) for item in (a,b)])
            stamps=[item['imageTimestampNs']-origin+1_000_000_000 for item in (a,b)]
            for topic,msg,stamp in sorted(stereo_messages(rect,projections,stamps),key=lambda x:x[2]):
                writer.write(topic,serialize_message(msg),int(stamp));count+=1
    finally:
        del writer
    evidence.update(settings)
    evidence.update(metricAccuracyValidated=False,
        sourceOriginNs=origin,rosOriginNs=1_000_000_000,messageCount=count+1,
        timePolicy='source sensor offsets preserved; replay with --clock',
        robotExtrinsicsAvailable=False,scope='recorded stereo input, not a verified map')
    (output/'phone-provenance.json').write_text(json.dumps(evidence,indent=2))
    return evidence


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--trial',type=Path,required=True)
    parser.add_argument('--calibration',type=Path,required=True)
    parser.add_argument('--check-only',action='store_true')
    add_scale_options(parser)
    parser.add_argument('--output',type=Path)
    args=parser.parse_args();cv2.setNumThreads(2)
    if args.check_only:
        result=inspect_recording(args.trial,args.calibration)[2]
    else:
        if args.output is None:parser.error('--output is required')
        settings=scale_from_args(args)
        result=export_bag(args.trial,args.calibration,args.output,settings['squareMm'],
            settings['scaleSource'],settings)
    print(json.dumps(result,indent=2))


if __name__=='__main__':main()

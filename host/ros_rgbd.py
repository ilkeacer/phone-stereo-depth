"""Recorded model predictions -> experimental ROS RGB-D bag, no phone capture.

Preparation uses the optional inference venv; bag export uses ROS system Python.
All outputs are new. Predicted metres never become measured scale by serialization.
"""
import argparse
import json
from pathlib import Path
import cv2
import numpy as np
from host.monocular_depth import file_sha
from host.motion_recording import atomic_json
from host.ros_stereo import inspect_recording


def prepare(trial,calibration,repository,checkpoint,output,rotation):
    from host.monocular_depth import MetricDepthAnything
    import torch
    processor,pairs,evidence=inspect_recording(trial,calibration)
    paths=[calibration,calibration.with_suffix('.json'),trial/'manifest.json',trial/'committed-pairs.json',Path(__file__),Path(__file__).with_name('monocular_depth.py'),Path(__file__).with_name('depth.py')]
    paths += [trial/item['sampleFile'] for pair in pairs for item in pair[:2]]
    before={str(f.resolve()):file_sha(f) for f in paths}
    output.mkdir(parents=True,exist_ok=False);atomic_json(output/'input-before.json',before)
    atomic_json(output/'manifest.json',dict(status='preparing',frames=0))
    cv2.setNumThreads(2);torch.set_num_threads(4)
    model=MetricDepthAnything(repository,checkpoint,rotation=rotation)
    mx,my=processor.maps[0]
    support=(mx>=0)&(my>=0)&(mx<processor.input_size[0]-1)&(my<processor.input_size[1]-1)
    origin=pairs[0][0]['imageTimestampNs'];records=[]
    for i,(left,right,_) in enumerate(pairs):
        rect=processor.rectify([cv2.imread(str(trial/item['sampleFile']),0) for item in (left,right)])[0]
        depth,ms=model.predict(cv2.cvtColor(rect,cv2.COLOR_GRAY2BGR));depth[~support]=np.nan
        name=f'{i:06d}.npz';np.savez_compressed(output/name,image=rect,depth=depth)
        records.append(dict(file=name,sha256=file_sha(output/name),stampNs=left['imageTimestampNs']-origin+1_000_000_000,inferenceMs=ms))
        if i%25==0 or i==len(pairs)-1:print(f'Derinlik kaydı: {i+1}/{len(pairs)}',flush=True)
    after={f:file_sha(f) for f in before};atomic_json(output/'input-after.json',after)
    if before!=after:raise RuntimeError('Input changed during prediction')
    manifest=dict(status='complete',inputKind='rgbd',frames=len(records),records=records,projection=processor.c['P1'].tolist(),
                  imageSize=list(processor.size),model=model.metadata,scaleSource='model_predicted_metres',metricAccuracyValidated=False,
                  sourceSpanSeconds=(records[-1]['stampNs']-records[0]['stampNs'])/1e9,sourceOriginNs=origin,
                  sourceEvidence=evidence,inputUnchanged=True,scope='Predicted depth aligned with recorded rectified telephoto; not measured geometry')
    atomic_json(output/'manifest.json',manifest);return manifest


def validate_frame(image,depth,size):
    w,h=size
    if image.dtype!=np.uint8 or image.shape!=(h,w):raise ValueError('Invalid RGB image geometry/encoding')
    if depth.dtype!=np.float32 or depth.shape!=(h,w):raise ValueError('Invalid depth geometry/encoding')
    if np.isinf(depth).any() or np.any(np.isfinite(depth)&(depth<=0)):raise ValueError('Depth must be positive predicted metres or NaN')
    if not np.isfinite(depth).any():raise ValueError('No finite depth')


def rgbd_messages(image,depth,projection,stamp,encoding='32FC1'):
    from sensor_msgs.msg import Image,CameraInfo
    validate_frame(image,depth,(image.shape[1],image.shape[0]))
    p=np.asarray(projection,dtype=float)
    if p.shape!=(3,4) or not np.isfinite(p).all() or p[0,0]<=0 or p[1,1]<=0 or not np.allclose(p[:,3],0) or not np.allclose(p[2],[0,0,1,0]):
        raise ValueError('Invalid monocular rectified projection')
    if encoding=='16UC1':
        if np.any(np.isfinite(depth)&((depth<.0005)|(depth>65.535))):raise ValueError('Depth outside millimetre transport range')
        transport=np.where(np.isfinite(depth),np.rint(depth*1000),0).astype('<u2');stride=2
    elif encoding=='32FC1':transport=depth;stride=4
    else:raise ValueError('Unsupported depth encoding')
    for topic,array,encoding,bytes_per_pixel in [('/phone/left/image_rect',image,'mono8',1),('/phone/depth/image_rect',transport,encoding,stride)]:
        msg=Image();msg.header.stamp.sec,msg.header.stamp.nanosec=divmod(int(stamp),10**9)
        msg.header.frame_id='phone_left_optical';msg.height,msg.width=image.shape
        msg.encoding=encoding;msg.step=msg.width*bytes_per_pixel;msg.is_bigendian=0
        msg.data=np.ascontiguousarray(array,dtype={4:'<f4',2:'<u2',1:'u1'}[bytes_per_pixel]).tobytes()
        yield topic,msg
    info=CameraInfo();info.header=msg.header;info.height,info.width=image.shape
    info.distortion_model='plumb_bob';info.d=[0.]*5;info.k=p[:,:3].ravel().tolist();info.r=np.eye(3).ravel().tolist();info.p=p.ravel().tolist()
    yield '/phone/left/camera_info',info


def export_bag(cache,output,depth_encoding='16UC1'):
    import rosbag2_py
    from rclpy.serialization import serialize_message
    from host.ros_stereo import camera_transforms
    m=json.loads((cache/'manifest.json').read_text());records=m.get('records',[])
    if m.get('status')!='complete' or m.get('inputKind')!='rgbd' or m.get('scaleSource')!='model_predicted_metres' or len(records)!=m.get('frames') or not records:raise ValueError('Complete predicted RGB-D cache required')
    previous=-1
    for record in records:
        name=record['file']
        if Path(name).name!=name:raise ValueError('Invalid frame filename')
        if record['stampNs']<=previous:raise ValueError('Non-increasing frame timestamp')
        previous=record['stampNs']
        if file_sha(cache/name)!=record['sha256']:raise ValueError('Cached frame hash differs')
        with np.load(cache/name) as frame:validate_frame(frame['image'],frame['depth'],m['imageSize'])
    if output.exists():raise FileExistsError(output)
    output.parent.mkdir(parents=True,exist_ok=True)
    writer=rosbag2_py.SequentialWriter();writer.open(rosbag2_py.StorageOptions(uri=str(output),storage_id='sqlite3'),rosbag2_py.ConverterOptions('',''))
    for topic,kind in [('/phone/left/image_rect','Image'),('/phone/depth/image_rect','Image'),('/phone/left/camera_info','CameraInfo')]:
        writer.create_topic(rosbag2_py.TopicMetadata(name=topic,type='sensor_msgs/msg/'+kind,serialization_format='cdr'))
    writer.create_topic(rosbag2_py.TopicMetadata(name='/tf_static',type='tf2_msgs/msg/TFMessage',serialization_format='cdr',offered_qos_profiles=json.dumps([dict(history=1,depth=1,reliability=1,durability=1,liveliness=1,deadline=dict(sec=9223372036,nsec=854775807),lifespan=dict(sec=9223372036,nsec=854775807),liveliness_lease_duration=dict(sec=9223372036,nsec=854775807),avoid_ros_namespace_conventions=False)])))
    projection=np.asarray(m['projection']);tf=camera_transforms([projection,projection]);tf.transforms=[tf.transforms[0]]
    writer.write('/tf_static',serialize_message(tf),10**9)
    try:
        for record in records:
            with np.load(cache/record['file']) as frame:
                for topic,msg in rgbd_messages(frame['image'],frame['depth'],projection,record['stampNs'],depth_encoding):writer.write(topic,serialize_message(msg),record['stampNs'])
    finally:del writer
    evidence=dict(inputKind='rgbd',pairs=len(records),sourceSpanSeconds=m['sourceSpanSeconds'],scaleSource='model_predicted_metres',
                  metricAccuracyValidated=False,model=m['model'],cacheManifestSha256=file_sha(cache/'manifest.json'),depthEncoding=depth_encoding,
                  depthQuantizationMaxM=.0005 if depth_encoding=='16UC1' else 0.,
                  messageCount=len(records)*3+1,scope='Experimental learned-depth RGB-D input; no measured metric scale',
                  calibrationSha256=m['sourceEvidence']['calibrationSha256'],sourceOriginNs=m['sourceOriginNs'],rosOriginNs=10**9)
    atomic_json(output/'phone-provenance.json',evidence);return evidence


def export_hybrid(cache,stereo_bag,output):
    import rosbag2_py
    from rclpy.serialization import serialize_message,deserialize_message
    from sensor_msgs.msg import Image
    m=json.loads((cache/'manifest.json').read_text())
    stereo=json.loads((stereo_bag/'phone-provenance.json').read_text())
    if m.get('status')!='complete' or m['sourceEvidence']['calibrationSha256']!=stereo['calibrationSha256']:
        raise ValueError('Complete prediction cache and matching stereo calibration required')
    by_stamp={}
    for record in m['records']:
        if Path(record['file']).name!=record['file'] or file_sha(cache/record['file'])!=record['sha256']:raise ValueError('Invalid cached frame')
        stamp=record['stampNs']+m['sourceOriginNs']-10**9-stereo['sourceOriginNs']+stereo['rosOriginNs']
        if stamp in by_stamp:raise ValueError('Duplicate source timestamp')
        by_stamp[stamp]=record
    if len(by_stamp)!=stereo['pairs']:raise ValueError('Source pair counts differ')
    if output.exists():raise FileExistsError(output)
    reader=rosbag2_py.SequentialReader();reader.open(rosbag2_py.StorageOptions(uri=str(stereo_bag),storage_id='sqlite3'),rosbag2_py.ConverterOptions('',''))
    writer=rosbag2_py.SequentialWriter();writer.open(rosbag2_py.StorageOptions(uri=str(output),storage_id='sqlite3'),rosbag2_py.ConverterOptions('',''))
    for topic in reader.get_all_topics_and_types():writer.create_topic(topic)
    writer.create_topic(rosbag2_py.TopicMetadata(name='/phone/depth/image_rect',type='sensor_msgs/msg/Image',serialization_format='cdr'))
    seen=set();count=0
    try:
        while reader.has_next():
            topic,data,stamp=reader.read_next();writer.write(topic,data,stamp);count+=1
            if topic=='/phone/left/image_rect':
                msg=deserialize_message(data,Image)
                sensor_stamp=msg.header.stamp.sec*10**9+msg.header.stamp.nanosec
                if sensor_stamp not in by_stamp or sensor_stamp in seen:raise ValueError('Left frame timestamp differs')
                record=by_stamp[sensor_stamp]
                with np.load(cache/record['file']) as frame:
                    image=frame['image'];depth=frame['depth']
                    validate_frame(image,depth,m['imageSize'])
                    if msg.encoding!='mono8' or msg.step!=image.shape[1] or bytes(msg.data)!=image.tobytes():raise ValueError('Cached and stereo left pixels differ')
                    depth_msg=list(rgbd_messages(image,depth,m['projection'],sensor_stamp,'16UC1'))[1][1]
                writer.write('/phone/depth/image_rect',serialize_message(depth_msg),stamp);count+=1;seen.add(sensor_stamp)
    finally:del writer
    if len(seen)!=len(by_stamp):raise ValueError('Not all predictions paired with stereo inputs')
    result=dict(stereo,inputKind='hybrid',scaleSource=m['scaleSource'],odometryScaleSource=stereo['scaleSource'],
                model=m['model'],messageCount=count,depthEncoding='16UC1',depthQuantizationMaxM=.0005,
                cacheManifestSha256=file_sha(cache/'manifest.json'),sourceStereoManifestSha256=file_sha(stereo_bag/'phone-provenance.json'),
                scope='Stereo motion with precomputed depth surfaces; source scale provenance is preserved')
    atomic_json(output/'phone-provenance.json',result);return result


def main():
    p=argparse.ArgumentParser(description=__doc__);sub=p.add_subparsers(dest='command',required=True)
    prepare_parser=sub.add_parser('prepare')
    for name in ('trial','calibration','repository','checkpoint','output'):prepare_parser.add_argument('--'+name,type=Path,required=True)
    prepare_parser.add_argument('--rotation',type=int,choices=[0,90,180,270],default=0)
    bag=sub.add_parser('bag');bag.add_argument('--cache',type=Path,required=True);bag.add_argument('--output',type=Path,required=True)
    bag.add_argument('--depth-encoding',choices=['16UC1','32FC1'],default='16UC1')
    hybrid=sub.add_parser('hybrid-bag')
    for name in ('cache','stereo-bag','output'):hybrid.add_argument('--'+name,type=Path,required=True)
    a=p.parse_args()
    if a.command=='prepare':result=prepare(a.trial,a.calibration,a.repository,a.checkpoint,a.output,a.rotation);result={k:v for k,v in result.items() if k!='records'}
    elif a.command=='hybrid-bag':result=export_hybrid(a.cache,a.stereo_bag,a.output)
    else:result=export_bag(a.cache,a.output,a.depth_encoding)
    print(json.dumps(result,indent=2),flush=True)

if __name__=='__main__':main()

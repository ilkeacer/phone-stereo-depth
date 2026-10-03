"""Publish local Depth Anything V2 depth for live stereo odometry + RGB-D mapping.

The model supplies surfaces only. Stereo odometry remains responsible for pose.
Predicted metres are not certified physical distances. Camera images and output
logs stay in the local ignored work directory.
"""
import argparse
import collections
from pathlib import Path
import queue
import threading
import time

import cv2
import numpy as np

from host.monocular_depth import MetricDepthAnything
from host.motion_recording import atomic_json
from host.ros_scale import load_scale
from host.ros_sgbm import CheckedStereoDepth, depth_message, local_cloud_message, stamp_ns

ROOT=Path(__file__).resolve().parents[1]
DEFAULT_REPOSITORY=ROOT/'work/Depth-Anything-V2'
DEFAULT_CHECKPOINT=ROOT/'data/models/depth-anything-v2/hypersim-small.pth'


def checked_left(image,info,projection,size,previous_stamp):
    w,h=size;stamp=stamp_ns(image)
    if stamp<=0 or (previous_stamp is not None and stamp<=previous_stamp):
        raise ValueError('Reused or non-increasing left source image')
    if image.header.frame_id!='phone_left_optical' or info.header.frame_id!=image.header.frame_id or stamp_ns(info)!=stamp:
        raise ValueError('Left CameraInfo frame or timestamp differs')
    if image.encoding!='mono8' or image.width!=w or image.height!=h or image.step<w or len(image.data)!=image.step*h:
        raise ValueError('Left image geometry or encoding differs')
    p=np.asarray(projection)
    if info.width!=w or info.height!=h or info.distortion_model!='plumb_bob' or len(info.d)!=5:
        raise ValueError('Left CameraInfo geometry or distortion differs')
    for observed,expected in ((info.p,p.ravel()),(info.k,p[:,:3].ravel()),(info.r,np.eye(3).ravel()),(info.d,np.zeros(5))):
        if not np.allclose(observed,expected,rtol=1e-7,atol=1e-7):
            raise ValueError('Left CameraInfo calibration differs')
    return np.frombuffer(bytes(image.data),np.uint8).reshape(h,image.step)[:,:w].copy(),stamp


def main():
    import torch
    import rclpy
    from rclpy.signals import SignalHandlerOptions
    from sensor_msgs.msg import Image,CameraInfo,PointCloud2
    from message_filters import Subscriber,TimeSynchronizer
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--calibration',type=Path,required=True)
    p.add_argument('--scale-config',type=Path,required=True)
    p.add_argument('--repository',type=Path,default=DEFAULT_REPOSITORY)
    p.add_argument('--checkpoint',type=Path,default=DEFAULT_CHECKPOINT)
    p.add_argument('--rotation',type=int,choices=[0,90,180,270],default=90)
    p.add_argument('--output',type=Path,required=True)
    p.add_argument('--use-sim-time',action='store_true')
    args=p.parse_args()
    args.output.mkdir(parents=True,exist_ok=False)
    cv2.setNumThreads(2);torch.set_num_threads(4)
    settings=load_scale(args.scale_config,args.calibration)
    stereo=CheckedStereoDepth(args.calibration,settings['squareMm'])
    model=MetricDepthAnything(args.repository,args.checkpoint,rotation=args.rotation)
    mx,my=stereo.processor.maps[0]
    support=(mx>=0)&(my>=0)&(mx<stereo.processor.input_size[0]-1)&(my<stereo.processor.input_size[1]-1)
    metadata=dict(engine='online_depth_anything_v2_hypersim_small',model=model.metadata,
                  odometryScaleSource=settings['scaleSource'],depthScaleSource='model_predicted_metres',
                  calibrationSha256=settings['calibrationSha256'],imageSize=list(stereo.processor.size),
                  finiteLeftSupportPixels=int(support.sum()),maxAgeSeconds=1.,pendingFrames=1,
                  metricAccuracyValidated=False)
    atomic_json(args.output/'settings.json',metadata)
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node=rclpy.create_node('phone_ai_depth')
    node.set_parameters([rclpy.parameter.Parameter('use_sim_time',value=args.use_sim_time)])
    depth_pub=node.create_publisher(Image,'/phone/depth/image_rect',10)
    cloud_pub=node.create_publisher(PointCloud2,'/phone/local_cloud',10)
    pending=queue.Queue(maxsize=1);stop=threading.Event();lock=threading.Lock()
    counts=collections.Counter();times=[];ages=[];errors=[];previous=None
    def status(state):
        with lock:
            result=dict(state=state,updatedMonotonic=time.monotonic(),counts=dict(counts),
                computeMs=dict(median=float(np.median(times)),p95=float(np.percentile(times,95)),max=float(max(times))) if times else None,
                publishAgeMs=dict(median=float(np.median(ages)),p95=float(np.percentile(ages,95)),max=float(max(ages))) if ages else None,
                error=errors[0] if errors else None)
        atomic_json(args.output/'status.json',result)
    def work():
        try:
            while not stop.is_set():
                try:left,stamp,header=pending.get(timeout=.1)
                except queue.Empty:continue
                age=(node.get_clock().now().nanoseconds-stamp)/1e9
                if age>1 or age<-.1:
                    with lock:counts['staleBefore']+=1
                    continue
                depth,ms=model.predict(cv2.cvtColor(left,cv2.COLOR_GRAY2BGR))
                if depth.shape!=left.shape:raise ValueError('Model depth shape differs from left image')
                depth[~support]=np.nan
                depth[~np.isfinite(depth)|(depth<=.0005)|(depth>20)]=np.nan
                age=(node.get_clock().now().nanoseconds-stamp)/1e9
                with lock:times.append(ms)
                if age>1:
                    with lock:counts['staleAfter']+=1
                    continue
                msg=depth_message(depth,header)
                cloud=local_cloud_message(depth,left,stereo.projections[0],header)
                depth_pub.publish(msg);cloud_pub.publish(cloud)
                with lock:
                    counts['publishedDepth']+=1;counts['finiteDepthPixels']+=int(np.isfinite(depth).sum())
                    counts['publishedLocalCloud']+=1;counts['localCloudLatestPoints']=cloud.width
                    ages.append(age*1000)
        except Exception as exc:
            with lock:errors.append(str(exc))
    def receive(image,info):
        nonlocal previous
        left,stamp=checked_left(image,info,stereo.projections[0],stereo.processor.size,previous)
        previous=stamp
        with lock:counts['synchronizedFrames']+=1
        try:pending.put_nowait((left,stamp,image.header))
        except queue.Full:
            try:pending.get_nowait()
            except queue.Empty:pass
            with lock:counts['supersededFrames']+=1
            pending.put_nowait((left,stamp,image.header))
    image_sub=Subscriber(node,Image,'/phone/left/image_rect',qos_profile=10)
    info_sub=Subscriber(node,CameraInfo,'/phone/left/camera_info',qos_profile=10)
    sync=TimeSynchronizer([image_sub,info_sub],10);sync.registerCallback(receive)
    node.create_timer(.5,lambda:status('running'))
    status('ready');worker=threading.Thread(target=work,name='ai_depth');worker.start()
    try:
        while rclpy.ok():
            rclpy.spin_once(node,timeout_sec=.05)
            if errors:raise RuntimeError(errors[0])
    except KeyboardInterrupt:pass
    finally:
        stop.set();worker.join();status('failed' if errors else 'stopped')
        node.destroy_node()
        if rclpy.ok():rclpy.shutdown()
    if errors:raise SystemExit(1)


if __name__=='__main__':main()

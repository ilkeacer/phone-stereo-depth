"""Compute checked depth from rectified ROS stereo. Never controls the phone.

Same controlled_sgbm profile, external LR <=1 px and common-support mask as the
recorded benchmark. ROS system OpenCV is recorded explicitly in the evidence.
"""
import argparse
import collections
import hashlib
import json
from pathlib import Path
import queue
import threading
import time
import cv2
import numpy as np
from host.depth import StereoProcessor
from host.ros_stereo import metric_projection, sha
from host.stereo_benchmark import measure
from host.motion_recording import atomic_json
from host.ros_scale import load_scale,explicit_scale


def stamp_ns(msg):
    return msg.header.stamp.sec*10**9+msg.header.stamp.nanosec


class CheckedStereoDepth:
    def __init__(self, calibration, square_mm):
        self.calibration=Path(calibration)
        report=json.loads(self.calibration.with_suffix('.json').read_text())
        with np.load(self.calibration) as source:cal={k:source[k] for k in source.files}
        if report['ids']!=['20','21'] or report['lengthUnit']!='checker_square' or str(cal['lengthUnit'])!='checker_square':
            raise ValueError('Expected camera 20/21 checker-square calibration')
        if not np.allclose(cal['P1'][:,3],0) or cal['P2'][0,3]>=0 or not np.allclose(cal['P2'][1:,3],0):
            raise ValueError('Expected positive horizontal baseline')
        self.processor=StereoProcessor(cal,.5)
        self.scale=square_mm/1000
        self.projections=[metric_projection(self.processor.c[f'P{i}'],square_mm) for i in (1,2)]
        w,h=self.processor.size
        common=np.zeros((h,w),bool)
        x,y,rw,rh=cv2.getValidDisparityROI(tuple(self.processor.c['roi1']),tuple(self.processor.c['roi2']),0,128,15)
        common[y:y+rh,x:x+rw]=True
        for mx,my in self.processor.maps:
            common&=(mx>=0)&(my>=0)&(mx<self.processor.input_size[0]-1)&(my<self.processor.input_size[1]-1)
        if not common.any():raise ValueError('Empty common stereo support')
        self.regions=dict(full=np.ones_like(common),commonSupport=common)
        self.metadata=dict(method='StereoSGBM',profile='controlled_sgbm',opencv=cv2.__version__,numpy=np.__version__,
            squareMm=square_mm,calibrationSha256=sha(self.calibration),calibrationReportSha256=sha(self.calibration.with_suffix('.json')),
            externalLRMaxDifferencePx=1.,stereoMaxDeltaNs=20_000_000,imageSize=[w,h],
            commonSupportPixels=int(common.sum()),depthEncoding='16UC1',depthQuantizationMaxM=.0005,
            metricAccuracyValidated=False)
        self.metadata['sourceSha256']={p.name:sha(p) for p in [Path(__file__),Path(__file__).with_name('stereo_benchmark.py'),Path(__file__).with_name('depth.py')]}

    def compute(self,rect):
        w,h=self.processor.size
        if len(rect)!=2 or any(v.dtype!=np.uint8 or v.shape!=(h,w) for v in rect):
            raise ValueError('Rectified mono8 stereo geometry differs')
        stats,raw,mask=measure(rect,self.processor.c,'controlled_sgbm',self.regions)
        depth=cv2.reprojectImageTo3D(raw[0].astype(np.float32)/16,self.processor.c['Q'])[:,:,2]*self.scale
        depth[~mask]=np.nan
        return depth.astype(np.float32),stats


def checked_pair(images,infos,expected_projections,size,previous=None):
    w,h=size;stamps=tuple(stamp_ns(m) for m in images)
    if any(t<0 for t in stamps) or abs(stamps[0]-stamps[1])>20_000_000:
        raise ValueError('Stereo exposure delta exceeds 20 ms')
    if previous and any(t<=old for t,old in zip(stamps,previous)):
        raise ValueError('Reused or non-increasing stereo source image')
    arrays=[]
    for side,image,info,p in zip(('left','right'),images,infos,expected_projections):
        frame='phone_'+side+'_optical'
        if image.header.frame_id!=frame or info.header.frame_id!=frame or stamp_ns(info)!=stamp_ns(image):
            raise ValueError('CameraInfo frame or timestamp does not match its image')
        if image.encoding!='mono8' or image.width!=w or image.height!=h or image.step<w or len(image.data)!=image.step*h:
            raise ValueError('Image geometry or encoding differs')
        if info.width!=w or info.height!=h or info.distortion_model!='plumb_bob' or len(info.d)!=5:
            raise ValueError('CameraInfo geometry or distortion model differs')
        for observed,expected in ((info.p,p.ravel()),(info.k,p[:,:3].ravel()),(info.r,np.eye(3).ravel()),(info.d,np.zeros(5))):
            if not np.allclose(observed,expected,rtol=1e-7,atol=1e-7):
                raise ValueError('CameraInfo projection/calibration differs')
        arrays.append(np.frombuffer(bytes(image.data),np.uint8).reshape(h,image.step)[:,:w].copy())
    return arrays,stamps


def depth_message(depth,header):
    from sensor_msgs.msg import Image
    if depth.dtype!=np.float32 or depth.ndim!=2 or np.isinf(depth).any():
        raise ValueError('Expected finite-positive or NaN float32 depth')
    finite=np.isfinite(depth)
    if np.any(finite&((depth<=.0005)|(depth>65.535))):
        raise ValueError('Depth outside positive millimetre transport range')
    transport=np.where(finite,np.rint(depth*1000),0).astype('<u2')
    msg=Image();msg.header=header;msg.height,msg.width=depth.shape
    msg.encoding='16UC1';msg.is_bigendian=0;msg.step=msg.width*2;msg.data=transport.tobytes()
    return msg


def local_cloud_message(depth,left,projection,header,stride=4,max_range_m=8.):
    """Current-frame metric points in the left optical frame; no pose is assumed."""
    from sensor_msgs.msg import PointCloud2,PointField
    if depth.dtype!=np.float32 or depth.ndim!=2 or left.dtype!=np.uint8 or left.shape!=depth.shape:
        raise ValueError('Expected aligned float32 depth and mono8 left image')
    if stride<1 or not np.isfinite(max_range_m) or max_range_m<=0:
        raise ValueError('Invalid cloud sampling')
    p=np.asarray(projection)
    if p.shape!=(3,4) or p[0,0]<=0 or p[1,1]<=0:
        raise ValueError('Invalid left projection')
    sampled=depth[::stride,::stride]
    valid=np.isfinite(sampled)&(sampled>0)&(sampled<=max_range_m)
    rows,cols=np.nonzero(valid)
    z=sampled[valid]
    u=cols*stride;v=rows*stride
    points=np.empty(len(z),dtype=[('x','<f4'),('y','<f4'),('z','<f4'),('rgb','<u4')])
    points['x']=(u-p[0,2])*z/p[0,0]
    points['y']=(v-p[1,2])*z/p[1,1]
    points['z']=z
    gray=left[v,u].astype(np.uint32)
    points['rgb']=(gray<<16)|(gray<<8)|gray
    msg=PointCloud2();msg.header=header;msg.height=1;msg.width=len(points)
    msg.fields=[PointField(name=name,offset=offset,datatype=datatype,count=1) for name,offset,datatype in
                [('x',0,PointField.FLOAT32),('y',4,PointField.FLOAT32),
                 ('z',8,PointField.FLOAT32),('rgb',12,PointField.UINT32)]]
    msg.is_bigendian=False;msg.point_step=16;msg.row_step=16*len(points)
    msg.is_dense=True;msg.data=points.tobytes()
    return msg


def main():
    import rclpy
    from rclpy.signals import SignalHandlerOptions
    from sensor_msgs.msg import Image,CameraInfo
    from message_filters import Subscriber,ApproximateTimeSynchronizer
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--calibration',type=Path,required=True)
    sizes=p.add_mutually_exclusive_group(required=True)
    sizes.add_argument('--square-mm',type=float)
    sizes.add_argument('--scale-config',type=Path)
    p.add_argument('--scale-source',choices=['measured','nominal_display_estimate'])
    p.add_argument('--output',type=Path,required=True)
    p.add_argument('--use-sim-time',action='store_true')
    args=p.parse_args()
    if args.scale_config is not None:
        if args.scale_source is not None:p.error('Scale source is already in --scale-config')
        settings=load_scale(args.scale_config,args.calibration)
    else:
        if args.scale_source is None:p.error('--square-mm requires --scale-source')
        settings=explicit_scale(args.square_mm,args.scale_source,args.calibration)
    args.output.mkdir(parents=True,exist_ok=False)
    cv2.setNumThreads(2);cv2.setRNGSeed(0)
    engine=CheckedStereoDepth(args.calibration,settings['squareMm'])
    engine.metadata.update(settings)
    engine.metadata.update(maxAgeSeconds=1.,syncQueueSize=10,pendingComputePairs=1,
        ageScope='ROS clock minus left exposure; replay clock sampling is not physical USB latency')
    atomic_json(args.output/'settings.json',engine.metadata)
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node=rclpy.create_node('phone_sgbm_depth')
    node.set_parameters([rclpy.parameter.Parameter('use_sim_time',value=args.use_sim_time)])
    publisher=node.create_publisher(Image,'/phone/depth/image_rect',10)
    from sensor_msgs.msg import PointCloud2
    cloud_publisher=node.create_publisher(PointCloud2,'/phone/local_cloud',10)
    counts=collections.Counter();times=[];ages=[];previous=None;error=None
    pending=queue.Queue(maxsize=1);stop=threading.Event();lock=threading.Lock();failure=[]
    log=(args.output/'frames.jsonl').open('x')
    def status(state):
        def stats(values):
            return dict(median=float(np.median(values)),p95=float(np.percentile(values,95)),max=float(max(values))) if values else None
        with lock:
            snapshot=dict(state=state,updatedMonotonic=time.monotonic(),counts=dict(counts),
                computeMs=stats(times),publishAgeMs=stats(ages),error=error)
        atomic_json(args.output/'status.json',snapshot)
    def process(rect,stamps,header):
        age=(node.get_clock().now().nanoseconds-stamps[0])/1e9
        row=dict(leftStampNs=stamps[0],rightStampNs=stamps[1],stereoDeltaMs=abs(stamps[0]-stamps[1])/1e6,inputAgeMs=age*1000)
        if age>1 or age<-.1:
            with lock:counts['staleBefore']+=1
            row['result']='stale_before'
        else:
            start=time.perf_counter();depth,metrics=engine.compute(rect);elapsed=(time.perf_counter()-start)*1000
            age=(node.get_clock().now().nanoseconds-stamps[0])/1e9
            row.update(computeMs=elapsed,publishAgeMs=age*1000,**metrics)
            with lock:times.append(elapsed)
            if age>1:
                with lock:counts['staleAfter']+=1
                row['result']='stale_after'
            else:
                # A fully invalid frame is zero depth, never hallucinated fill.
                msg=depth_message(depth,header)
                cloud=local_cloud_message(depth,rect[0],engine.projections[0],header)
                row.update(depthSha256=hashlib.sha256(bytes(msg.data)).hexdigest(),
                    leftImageSha256=hashlib.sha256(rect[0].tobytes()).hexdigest(),rightImageSha256=hashlib.sha256(rect[1].tobytes()).hexdigest(),
                    localCloudPoints=cloud.width)
                publisher.publish(msg);cloud_publisher.publish(cloud);row['result']='published'
                with lock:
                    counts['publishedDepth']+=1;ages.append(age*1000)
                    counts['finiteDepthPixels']+=int(np.isfinite(depth).sum())
                    counts['publishedLocalCloud']+=1
                    counts['localCloudLatestPoints']=cloud.width
        with lock:log.write(json.dumps(row,allow_nan=False)+'\n');log.flush()
    def work():
        try:
            while not stop.is_set():
                try:item=pending.get(timeout=.1)
                except queue.Empty:continue
                process(*item)
        except Exception as exc:failure.append(str(exc))
    def receive(left,right,left_info,right_info):
        nonlocal previous
        rect,stamps=checked_pair([left,right],[left_info,right_info],engine.projections,engine.processor.size,previous)
        previous=stamps
        with lock:counts['synchronizedPairs']+=1
        try:pending.put_nowait((rect,stamps,left.header))
        except queue.Full:
            try:old=pending.get_nowait()
            except queue.Empty:old=None
            if old is not None:
                with lock:
                    counts['supersededPairs']+=1
                    log.write(json.dumps(dict(leftStampNs=old[1][0],rightStampNs=old[1][1],result='superseded'))+'\n');log.flush()
            pending.put_nowait((rect,stamps,left.header))
    subscribers=[Subscriber(node,kind,topic,qos_profile=10) for topic,kind in [
        ('/phone/left/image_rect',Image),('/phone/right/image_rect',Image),
        ('/phone/left/camera_info',CameraInfo),('/phone/right/camera_info',CameraInfo)]]
    sync=ApproximateTimeSynchronizer(subscribers,10,.020000001)
    sync.registerCallback(receive)
    timer=node.create_timer(.5,lambda:status('running'))
    status('ready')
    worker=threading.Thread(target=work,name='checked_sgbm');worker.start()
    try:
        # Computation has its own single worker, keeping ROS /clock and source
        # subscriptions responsive. At most one newer pair may wait for it.
        while rclpy.ok():
            rclpy.spin_once(node,timeout_sec=.05)
            if failure:raise RuntimeError(failure[0])
    except KeyboardInterrupt:pass
    except Exception as exc:error=str(exc);raise
    finally:
        stop.set();worker.join()
        if failure and error is None:error=failure[0]
        status('failed' if error else 'stopped');log.close();node.destroy_node()
        if rclpy.ok():rclpy.shutdown()
    if error:raise SystemExit(1)


if __name__=='__main__':main()

"""One owner for live/replay mapping, shutdown, evidence and export. No camera on replay."""
import argparse
import hashlib
import json
import os
from pathlib import Path
import signal
import sys
import tempfile
import time
from host.motion_recording import atomic_json
from host.ros_process import ProcessGroup,session_lock
from host.ros_scale import default_scale,explicit_scale

ROOT=Path(__file__).resolve().parents[1]
DEFAULT_BAG=ROOT/'data/ros/diagnostic-VV2Vb5Xl'
DEFAULT_CALIBRATION=ROOT/'data/calibration/screen_20260910/calibration.npz'


def online_depth_requested(mode,diagnostic,engine,source_kind):
    if engine not in ('auto','stereo','sgbm','ai'):raise ValueError('Unknown depth engine')
    online=engine in ('sgbm','ai')
    if online and (diagnostic or source_kind!='stereo'):
        raise ValueError('Online depth requires a stereo source without precomputed depth; diagnostic remains stereo')
    if engine=='stereo' and source_kind!='stereo':raise ValueError('Requested stereo engine but bag contains depth')
    return online


def expected_subscribers(input_kind,depth_engine,topic,first_topic):
    if input_kind!='hybrid':
        return 3 if topic==first_topic else 2
    base=3 if topic==first_topic else 2 if topic=='/phone/left/camera_info' else 1
    if depth_engine=='ai' and topic in ('/phone/left/image_rect','/phone/left/camera_info'):
        return base+1
    if depth_engine=='sgbm' and topic!='/phone/depth/image_rect':
        return base+1
    return base


class HybridTrackingGuard:
    """Latch any observed RGB-D or hybrid tracking loss for export acceptance.

    This is an export/session guard, not an atomic per-image TF filter. A mapper
    may consume queued input or continue provisionally, so its DB is never accepted.
    """
    def __init__(self,source_kind='hybrid'):
        if source_kind not in ('hybrid','rgbd'):
            raise ValueError('Tracking guard requires RGB-D or hybrid input')
        self.source_kind=source_kind
        self.previous=None
        self.failure=None
        self.observed=0
        self.lost=0

    def observe(self,stamp_ns,lost):
        self.observed+=1
        self.lost+=int(lost)
        if self.failure is None:
            if lost:self.failure=f'{self.source_kind} mapping invalidated: tracking lost; reset bridging unsupported.'
            elif self.previous is not None and stamp_ns<=self.previous:
                self.failure=f'{self.source_kind} mapping invalidated: non-increasing tracking timestamp.'
        self.previous=stamp_ns

    def check(self):
        if self.failure is not None:raise RuntimeError(self.failure)


def run_session(mode,seconds=120,diagnostic=False,bag=DEFAULT_BAG,rate=1.,depth_engine='auto',calibration=DEFAULT_CALIBRATION,tracking_loss_policy=None,scale_config=None,localize_map_db=None):
    import rclpy
    from rclpy.qos import qos_profile_sensor_data,QoSProfile,ReliabilityPolicy,DurabilityPolicy
    from rclpy.signals import SignalHandlerOptions
    from sensor_msgs.msg import Image,PointCloud2
    from nav_msgs.msg import Odometry,Path as RosPath
    from geometry_msgs.msg import PoseWithCovarianceStamped,PoseStamped
    from rtabmap_msgs.msg import OdomInfo,Info
    from message_filters import Subscriber,TimeSynchronizer
    from host.trajectory_check import tracked_pose_values
    from host.ros_session_summary import summarize
    from host.ros_export_session import export
    # Live capture is valuable even when its provisional map becomes invalid.
    # Replay keeps its previous fail-fast policy unless explicitly overridden.
    tracking_loss_policy=tracking_loss_policy or ('keep-capture' if mode=='live' else 'stop')
    if tracking_loss_policy not in ('keep-capture','stop','continue-provisional'):raise ValueError('Unknown tracking loss policy')
    stop_requested=False;capture_only=False;diagnostic_notified=False
    def request_stop(*_):
        nonlocal stop_requested
        stop_requested=True
    old_handlers={sig:signal.signal(sig,request_stop) for sig in (signal.SIGINT,signal.SIGTERM)}
    children=[];logs=[];session=None;cleanup=[];error=None;input_code=None;observed=0;node=None;cloud_count=0;cloud_points=0;odom_count=0;invalid_odom=0;odom_frames=set();lost_odom=0;input_kind='stereo';shutdown_log_bytes=None;guard=None;depth_process=None;online_depth=False;scale=None;recovery_plan=None;recovery_plan_error=None;base_map=None;localization_count=0;last_localization=None;reference_matches=[];localization_active=False;last_tracking_loss_stamp_ns=None
    try:
        session=Path(tempfile.mkdtemp(prefix=f'ros-{mode}-',dir=ROOT/'work'))
        env=dict(os.environ,ROS_DOMAIN_ID=os.environ.get('ROS_DOMAIN_ID','72'),ROS_LOCALHOST_ONLY='1',
            ROS_LOG_DIR=str(session/'log'),PHONE_MAP_DIR=str(session/'map'),
            PHONE_USE_SIM_TIME='true' if mode=='replay' else 'false',PHONE_RTABMAP_VIZ='false')
        # This owner and its readiness observer must use the same ROS domain.
        os.environ.update({k:env[k] for k in ('ROS_DOMAIN_ID','ROS_LOCALHOST_ONLY','ROS_LOG_DIR')})
        print(f'Oturum kaydı: {session}',flush=True)
        print('Kayıtlı görüntülerden harita hazırlanıyor; telefon kullanılmıyor.' if mode=='replay'
              else 'Kameralar hazırlanıyor; aşama yönergesini bekleyin.',flush=True)
        replay_started=None
        provenance={}
        if mode=='replay':
            if scale_config is not None:raise ValueError('Replay preserves bag scale; re-export raw input to change scale')
            bag=bag.resolve();provenance=json.loads((bag/'phone-provenance.json').read_text())
            if not (bag/'metadata.yaml').is_file() or not list(bag.glob('*.db3')):raise ValueError('ROS kaydı eksik')
            if provenance.get('pairs',0)<1:raise ValueError('ROS kaydı boş')
            input_kind=provenance.get('inputKind','stereo')
            if input_kind not in ('stereo','rgbd','hybrid'):raise ValueError('Bilinmeyen kayıt türü')
            atomic_json(session/'replay-input.json',dict(bag=str(bag),rate=rate,provenance=provenance,
                sha256={p.name:hashlib.sha256(p.read_bytes()).hexdigest() for p in sorted(bag.iterdir()) if p.is_file()}))
        else:
            scale=default_scale(calibration,scale_config)
            atomic_json(session/'scale.json',scale)
        online_depth=online_depth_requested(mode,diagnostic,depth_engine,input_kind)
        if online_depth:
            from host.ros_stereo import sha
            if mode=='replay' and (sha(calibration)!=provenance['calibrationSha256'] or sha(calibration.with_suffix('.json'))!=provenance['calibrationReportSha256']):
                raise ValueError('Online depth calibration differs from source bag')
            if mode=='replay':
                scale=explicit_scale(provenance['squareMm'],provenance['scaleSource'],calibration)
                if 'scaleMeasurement' in provenance:scale['scaleMeasurement']=provenance['scaleMeasurement']
                atomic_json(session/'scale.json',scale)
            if depth_engine=='sgbm':
                from host.ros_sgbm import CheckedStereoDepth
                probe=CheckedStereoDepth(calibration,scale['squareMm'])
                atomic_json(session/'depth-engine.json',dict(engine='online_controlled_sgbm',**{**probe.metadata,**scale}))
            else:
                ai_settings=dict(scale)
                ai_settings.update(engine='online_depth_anything_v2_hypersim_small',
                    depthScaleSource='model_predicted_metres',odometryScaleSource=scale['scaleSource'],
                    metricAccuracyValidated=False)
                atomic_json(session/'depth-engine.json',ai_settings)
            input_kind='hybrid'
        env['PHONE_INPUT_KIND']=input_kind
        if localize_map_db is not None:
            from host.ros_localization import prepare_reference_map,accepted_reference_ids
            if mode=='replay' and not (session/'scale.json').exists():
                scale=explicit_scale(provenance['squareMm'],provenance['scaleSource'],calibration)
                atomic_json(session/'scale.json',scale)
            base_map=prepare_reference_map(localize_map_db,session/'map'/'map.db',json.loads((session/'scale.json').read_text()))
            atomic_json(session/'reference-map.json',base_map)
            env['PHONE_LOCALIZATION_ONLY']='1'
            print('Kayitli harita kopyasi acildi; yeni konum eslesmesi beklenecek.',flush=True)
        rclpy.init(signal_handler_options=SignalHandlerOptions.NO);node=rclpy.create_node('phone_session_observer')
        if base_map is not None:
            localization_log=(session/'localization-poses.jsonl').open('x');logs.append(localization_log)
            match_log=(session/'reference-matches.jsonl').open('x');logs.append(match_log)
            reference_ids=set(base_map['referenceNodeIds']);match_keys=set()
            localization_path=RosPath();localization_path.header.frame_id='map'
            path_publisher=node.create_publisher(RosPath,'/phone/localization_path',QoSProfile(depth=1,
                reliability=ReliabilityPolicy.RELIABLE,durability=DurabilityPolicy.TRANSIENT_LOCAL))
            def localization_seen(msg):
                nonlocal localization_count,last_localization
                p=msg.pose.pose.position;q=msg.pose.pose.orientation
                if msg.header.frame_id!='map':return
                last_localization=dict(stampNs=msg.header.stamp.sec*10**9+msg.header.stamp.nanosec,
                    frameId=msg.header.frame_id,position=[p.x,p.y,p.z],orientation=[q.x,q.y,q.z,q.w])
                localization_log.write(json.dumps(dict(last_localization,referenceMatchActive=localization_active))+'\n');localization_log.flush()
                if localization_active:
                    sample=PoseStamped();sample.header=msg.header;sample.pose=msg.pose.pose
                    localization_path.header.stamp=msg.header.stamp
                    localization_path.poses.append(sample)
                    path_publisher.publish(localization_path)
                localization_count+=1
            node.create_subscription(PoseWithCovarianceStamped,'/rtabmap/localization_pose',localization_seen,10)
            def info_seen(msg):
                nonlocal localization_active
                for node_id,kinds in accepted_reference_ids(msg,reference_ids):
                    stamp=msg.header.stamp.sec*10**9+msg.header.stamp.nanosec
                    key=(stamp,msg.ref_id,node_id)
                    if key in match_keys:continue
                    match_keys.add(key)
                    if last_tracking_loss_stamp_ns is None or stamp>=last_tracking_loss_stamp_ns:
                        localization_active=True
                    row=dict(stampNs=stamp,currentNodeId=msg.ref_id,referenceNodeId=node_id,kinds=kinds)
                    reference_matches.append(row);match_log.write(json.dumps(row)+'\n');match_log.flush()
            node.create_subscription(Info,'/rtabmap/info',info_seen,10)
        guard=HybridTrackingGuard(input_kind) if input_kind in ('hybrid','rgbd') else None
        if guard is not None:
            guard_log=(session/'tracking-status-observed.jsonl').open('x');logs.append(guard_log)
            # Observe OdomInfo independently: a lost sample need not have a
            # matching Odometry message for the trajectory logger callback.
            def tracking_seen(info):
                nonlocal localization_active,last_tracking_loss_stamp_ns
                first_failure=guard.failure is None
                stamp=info.header.stamp.sec*10**9+info.header.stamp.nanosec
                if base_map is not None and info.lost:
                    localization_active=False;last_tracking_loss_stamp_ns=stamp
                    localization_path.poses.clear();localization_path.header.stamp=info.header.stamp
                    path_publisher.publish(localization_path)
                guard_log.write(json.dumps(dict(stampNs=stamp,lost=bool(info.lost),inliers=int(info.inliers)))+'\n');guard_log.flush()
                guard.observe(stamp,bool(info.lost))
                if first_failure and guard.failure is not None:
                    atomic_json(session/'hybrid-tracking-failure.json',dict(stampNs=stamp,lost=bool(info.lost),error=guard.failure))
        def image_seen(_):
            nonlocal observed
            observed+=1
        node.create_subscription(Image,'/phone/left/image_rect',image_seen,qos_profile_sensor_data)
        def cloud_seen(msg):
            nonlocal cloud_count,cloud_points
            cloud_count+=1;cloud_points=msg.width*msg.height
        node.create_subscription(PointCloud2,'/rtabmap/cloud_map',cloud_seen,1)
        odom_log=(session/'odometry-raw.txt').open('x');logs.append(odom_log)
        status_log=(session/'odometry-status.jsonl').open('x');logs.append(status_log)
        odom_log.write('# stamp x y z qx qy qz qw; raw odometry, exact-stamp OdomInfo lost=false; before map optimization\n')
        def odometry_seen(msg,info):
            nonlocal odom_count,invalid_odom,lost_odom
            stamp=msg.header.stamp.sec+msg.header.stamp.nanosec/1e9
            status_log.write(json.dumps(dict(stamp=stamp,lost=bool(info.lost),inliers=int(info.inliers)))+'\n');status_log.flush()
            if info.lost:lost_odom+=1;return
            try:pose=tracked_pose_values(msg,info)
            except ValueError:
                invalid_odom+=1;return
            odom_frames.add((msg.header.frame_id,msg.child_frame_id))
            odom_log.write(f'{stamp:.9f} '+' '.join(f'{v:.12g}' for v in pose)+'\n');odom_log.flush();odom_count+=1
        odom_sub=Subscriber(node,Odometry,'/rtabmap/odom',qos_profile=100)
        info_sub=Subscriber(node,OdomInfo,'/rtabmap/odom_info',qos_profile=100)
        # Inspect each status before exact Odom/Info pairing; share its DDS
        # subscription rather than doubling observer traffic with another one.
        if guard is not None:info_sub.registerCallback(tracking_seen)
        odom_sync=TimeSynchronizer([odom_sub,info_sub],100)
        odom_sync.registerCallback(odometry_seen)
        log=(session/'mapping.log').open('w');logs.append(log)
        mapping=ProcessGroup([str(ROOT/'scripts/start_ros_mapping.sh')],env=env,cwd=ROOT,stdout=log);children.append(mapping)
        if online_depth:
            depth_command=[str(ROOT/'.venv/bin/python') if depth_engine=='ai' else sys.executable,
                '-m','host.ros_ai_depth' if depth_engine=='ai' else 'host.ros_sgbm',
                '--calibration',str(calibration),'--scale-config',str(session/'scale.json'),
                '--output',str(session/'depth')]
            if mode=='replay':depth_command+=['--use-sim-time']
            depth_log=(session/'depth.log').open('x');logs.append(depth_log)
            depth_process=ProcessGroup(depth_command,env=env,cwd=ROOT,stdout=depth_log);children.append(depth_process)
        def check_tracking():
            nonlocal capture_only,error,shutdown_log_bytes,diagnostic_notified
            if guard is None or guard.failure is None:return
            if tracking_loss_policy=='stop':guard.check()
            if tracking_loss_policy=='continue-provisional':
                if not diagnostic_notified:
                    diagnostic_notified=True
                    atomic_json(session/'mapping-diagnostic.json',dict(reason=guard.failure,
                        startedMonotonic=time.monotonic(),policy='continue-provisional',
                        acceptedAsContinuousMap=False))
                    print('Takip kesintisi kaydedildi. Deneysel ROS haritası ve kayıt sürüyor; bu oturum tek kesintisiz harita olarak kabul edilmeyecek.',flush=True)
                return
            if capture_only:return
            error=guard.failure;capture_only=True
            shutdown_log_bytes=(session/'mapping.log').stat().st_size
            fallback=dict(reason=error,startedMonotonic=time.monotonic(),
                          observedImagesAtLoss=observed,mappingStopped=False,
                          policy='keep-capture',mapResumeSupported=False)
            atomic_json(session/'mapping-fallback.json',fallback)
            print('Takip kayboldu: harita birikimi durduruluyor. '+
                  ('Görüntü kaydı süre bitene kadar devam edecek; testi yeniden başlatmayın.' if mode=='live' else 'Kayıt oynatımı devam ediyor; telefon kullanılmıyor.'),flush=True)
            def drain_observer():
                # Stop waits must not starve image/status subscriptions while
                # the independent source is still running and recording.
                for _ in range(32):rclpy.spin_once(node,timeout_sec=0)
            result=mapping.stop(wait_hook=drain_observer)
            cleanup.append(result);children.remove(mapping)
            if result['remaining'] or result['forcedKill']:
                raise RuntimeError('Harita işlemi temiz durmadı; görüntü kaydı güvenli kapanış için sonlandırıldı.')
            fallback.update(mappingStopped=True,stoppedMonotonic=time.monotonic())
            atomic_json(session/'mapping-fallback.json',fallback)
            print('Harita birikimi durdu; '+('görüntü kaydı ve anlık 3B derinlik sürüyor. Yavaşça devam edebilir veya Durdur ve kaydet ile bitirebilirsiniz.'
                                           if mode=='live' and depth_process is not None else
                                           'yalnız görüntü kaydı sürüyor. Yavaşça devam edebilir veya Durdur ve kaydet ile bitirebilirsiniz.'
                                           if mode=='live' else 'kayıt oynatımı sürüyor; telefon kullanılmıyor.'),flush=True)
        # Wait for actual subscribers, not a blind fixed sleep before replay.
        deadline=time.monotonic()+25
        topics=[f'/phone/{side}/{suffix}' for side in ('left','right') for suffix in ('image_rect','camera_info')]
        if input_kind=='rgbd':topics=['/phone/left/image_rect','/phone/depth/image_rect','/phone/left/camera_info']
        if input_kind=='hybrid':topics+=['/phone/depth/image_rect']
        while True:
            rclpy.spin_once(node,timeout_sec=.1)
            if stop_requested:break
            if mapping.process.poll() is not None:raise RuntimeError('ROS haritalama başlatılamadı; mapping.log dosyasına bakın.')
            if depth_process and depth_process.process.poll() is not None:raise RuntimeError('Derinlik işlemi başlatılamadı; depth.log dosyasına bakın.')
            if all(node.count_subscribers(topic)>=expected_subscribers(input_kind,depth_engine,topic,topics[0]) for topic in topics):break
            if time.monotonic()>deadline:raise RuntimeError('ROS görüntü aboneleri 25 saniyede hazır olmadı.')
        if not stop_requested:
            if mode=='replay':
                # rosbag2 takes milliseconds here, not seconds. Give queued
                # image payloads a bounded chance to arrive before DDS closes.
                command=['ros2','bag','play',str(bag),'--clock','--rate',str(rate),'--delay','1','--wait-for-all-acked','3000','--disable-keyboard-controls']
                replay_started=time.monotonic()+1
            else:
                command=[sys.executable,'-m','host.ros_live','--calibration',str(calibration),
                    '--scale-config',str(session/'scale.json'),'--seconds',str(seconds),'--report',str(session/'capture.json')]
                if diagnostic:command+=['--diagnostic-dir',str(session/'raw')]
                else:command+=['--recording-dir',str(session/'raw')]
            log=(session/'input.log').open('w');logs.append(log)
            source=ProcessGroup(command,env=env,cwd=ROOT,stdout=log);children.append(source)
            end=time.monotonic()+(provenance.get('sourceSpanSeconds',0)/rate+30 if mode=='replay' else seconds+40)
            position=0;input_position=0;last_status=0
            while source.process.poll() is None and not stop_requested:
                rclpy.spin_once(node,timeout_sec=.1)
                check_tracking()
                with (session/'input.log').open(errors='replace') as monitor:
                    monitor.seek(input_position);lines=monitor.read();input_position=monitor.tell()
                for line in lines.splitlines():
                    if 'Toplam:' in line or 'TEST TAMAMLANDI' in line:print(line,flush=True)
                with (session/'mapping.log').open(errors='replace') as monitor:
                    monitor.seek(position);new=monitor.read();position=monitor.tell()
                if not capture_only and (mapping.process.poll() is not None or 'process has died' in new or 'process has finished cleanly' in new):
                    raise RuntimeError('ROS haritalama işlemi kapandı; kaynak durduruldu.')
                if depth_process and depth_process.process.poll() is not None:
                    raise RuntimeError('Derinlik işlemi kapandı; kaynak durduruldu. depth.log dosyasına bakın.')
                if time.monotonic()>end:raise RuntimeError('Görüntü işlemi süre sınırında kapanmadı.')
                if time.monotonic()-last_status>.5:
                    last_status=time.monotonic()
                    atomic_json(session/'mapping-status.json',dict(updatedMonotonic=last_status,cloudMessages=cloud_count,cloudPoints=cloud_points))
                    if mode=='replay':
                        atomic_json(session/'live-status.json',dict(stage='replay',updatedMonotonic=last_status,
                            remainingSeconds=max(0,provenance['sourceSpanSeconds']/rate-(last_status-replay_started)),publishedPairs=observed))
            input_code=source.process.poll()
            if input_code not in (None,0) and not stop_requested:raise RuntimeError('Görüntü kaynağı hata verdi; input.log dosyasına bakın.')
            if not stop_requested:
                # Drain queued input and let the mapper finish its last update.
                deadline=time.monotonic()+2
                while time.monotonic()<deadline and not stop_requested:
                    rclpy.spin_once(node,timeout_sec=.1)
                    check_tracking()
                    if depth_process and depth_process.process.poll() is not None:raise RuntimeError('Derinlik işlemi kapandı.')
    except Exception as exc:
        error=str(exc);print('HATA: '+error,flush=True)
    finally:
        # Keep the exclusivity lock for cleanup/export too (outer lock in main).
        if shutdown_log_bytes is None and session is not None and (session/'mapping.log').exists():shutdown_log_bytes=(session/'mapping.log').stat().st_size
        for child in reversed(children):cleanup.append(child.stop())
        if online_depth and session is not None and (session/'depth/status.json').exists():
            depth_status=json.loads((session/'depth/status.json').read_text())
            if depth_status.get('error') and error is None:error=depth_engine+': '+depth_status['error']
        for log in logs:log.close()
        if node is not None:node.destroy_node()
        if rclpy.ok():rclpy.shutdown()
        for sig,handler in old_handlers.items():signal.signal(sig,handler)
        if session is not None:
            if mode=='live' and (session/'mapping-fallback.json').exists():
                try:
                    from host.ros_recovery_segments import plan
                    recovery_plan=plan(session)
                    atomic_json(session/'recovery-plan.json',recovery_plan)
                except (OSError,ValueError,KeyError,TypeError) as exc:
                    recovery_plan_error=str(exc)
                    atomic_json(session/'recovery-plan-error.json',dict(error=recovery_plan_error,
                        meaning='No automatic split; original recording is preserved'))
            atomic_json(session/'lifecycle.json',dict(mode=mode,inputKind=input_kind,error=error,stopRequested=stop_requested,
                localizationMode=base_map is not None,localizationPoseCount=localization_count,
                acceptedReferenceMatches=len(reference_matches),localizationActiveAtEnd=localization_active,
                localizationPathPoses=len(localization_path.poses) if base_map is not None else 0,
                trackingLossPolicy=tracking_loss_policy,captureContinuedAfterTrackingLoss=capture_only,
                recoveryPlanAvailable=bool(recovery_plan and any(s['available'] for s in recovery_plan['sections'])),
                recoveryPlanError=recovery_plan_error,
                depthEngine=('online_depth_anything_v2_hypersim_small' if depth_engine=='ai' else 'online_controlled_sgbm') if online_depth else 'source_default',
                depthScaleSource='model_predicted_metres' if online_depth and depth_engine=='ai' else None,
                odometryScaleSource=scale['scaleSource'] if scale is not None and online_depth and depth_engine=='ai' else None,
                inputReturnCode=input_code,cleanup=cleanup,mappingLogBytesBeforeShutdown=shutdown_log_bytes,observerImageCount=observed,cloudMessages=cloud_count,cloudPoints=cloud_points,rawOdometryPoses=odom_count,invalidOdometryPoses=invalid_odom,lostOdometryPosesExcluded=lost_odom,odometryFrames=sorted(odom_frames),
                hybridTrackingGuard=dict(observedStatuses=guard.observed,observedLostStatuses=guard.lost,failure=guard.failure) if guard is not None else None))
            if mode=='replay':
                atomic_json(session/'capture.json',dict(publishedPairs=observed,inputPairs=provenance.get('pairs'),
                    squareMm=provenance.get('squareMm'),scaleMeasurement=provenance.get('scaleMeasurement'),
                    scaleSource=provenance.get('scaleSource'),odometryScaleSource=provenance.get('odometryScaleSource',provenance.get('scaleSource')),error=error,source='recorded_rosbag',
                    countMeaning='left images received by observer; not a publisher acknowledgement'))
                atomic_json(session/'live-status.json',dict(stage='finished',updatedMonotonic=time.monotonic(),error=error))
            result=summarize(session)
            if base_map is not None:
                result['mapState']='reference_loaded'
                result['accumulatedGraphPresent']=False
                result['localization']=dict(referenceMap=base_map['referenceMap'],poseMessages=localization_count,
                    acceptedReferenceMatches=len(reference_matches),firstAcceptedReferenceMatch=reference_matches[0] if reference_matches else None,
                    publishedPathPoses=len(localization_path.poses),lastTrackingLossStampNs=last_tracking_loss_stamp_ns,
                    lastPublishedPose=last_localization,everMatched=bool(reference_matches),localized=localization_active,
                    scope='Only accepted Info loop/proximity IDs in source DB count as reference matches. Tracking loss clears the active path until another reference match. Pose messages alone may use last saved correction; not independent position accuracy.')
                result['conclusion']='Reference map loaded; '+('active localization at end.' if localization_active else
                    'reference matched earlier, but localization was not active at end.' if reference_matches else
                    'no accepted reference-map match was observed.')
            atomic_json(session/'summary.json',result)
            exported=(dict(status='skipped_localization',reason='Reference graph is not a new map export',
                           referenceMap=base_map['referenceMap']) if base_map is not None else export(session))
            atomic_json(session/'export-result.json',exported)
            diagnostic_export=None
            if base_map is None and tracking_loss_policy=='continue-provisional' and guard is not None and guard.failure is not None:
                from host.ros_export_session import export_diagnostic_fragment
                diagnostic_export=export_diagnostic_fragment(session)
                atomic_json(session/'diagnostic-export-result.json',diagnostic_export)
            print(f"Takip: {result['trackingResults']}/{result['odometryResults']} · bağlantılı poz: {result['graph'].get('largestComponentNodes',0)}",flush=True)
            if base_map is not None:print(f'Konum mesajlari: {localization_count} · kabul edilen eski harita eslesmesi: {len(reference_matches)}',flush=True)
            if capture_only:print(('Görüntü kaydı saklandı.' if mode=='live' else 'Kaynak kayıt korundu.')+' Takip kaybı nedeniyle bu oturumun haritası kabul edilmedi.',flush=True)
            elif diagnostic_export is not None:print('Deneysel harita parçası: '+str(diagnostic_export.get('poses',0))+' poz / '+str(diagnostic_export.get('points',0))+' nokta. Kesintisiz oda haritası olarak kabul edilmedi.',flush=True)
            elif base_map is not None:print('Konum bulma oturumu tamamlandi; kaynak harita degistirilmedi.',flush=True)
            else:print('Harita dosyası hazır.' if exported['status']=='complete' else 'Haritanın bağlantılı bir parçası kaydedildi.' if exported['status']=='partial' else 'Harita dışa aktarımı: '+exported['status'],flush=True)
    return 1 if error or any(c['remaining'] or c['forcedKill'] for c in cleanup) else 0


def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('mode',choices=['live','replay'])
    p.add_argument('--seconds',type=int,default=120);p.add_argument('--diagnostic',action='store_true')
    p.add_argument('--bag',type=Path,default=DEFAULT_BAG);p.add_argument('--rate',type=float,default=1.)
    p.add_argument('--depth-engine',choices=['auto','stereo','sgbm','ai'],default='auto',help='AI or SGBM depth is explicit opt-in; auto preserves the existing source pipeline')
    p.add_argument('--calibration',type=Path,default=DEFAULT_CALIBRATION)
    p.add_argument('--scale-config',type=Path,help='Live scale override; replay always preserves recorded scale')
    p.add_argument('--localize-map-db',type=Path,help='Load an accepted map.db into an isolated localization-only working copy')
    p.add_argument('--tracking-loss-policy',choices=['stop','keep-capture','continue-provisional'],help='Default: preserve live capture after map tracking fails; replay stops. Provisional mode keeps RTAB-Map running but never accepts a continuous map after loss.')
    args=p.parse_args()
    if not 5<=args.seconds<=600 or not .1<=args.rate<=2:p.error('Süre 5–600 sn, hız 0.1–2 olmalı.')
    # Separate outer lock survives all child shutdown and export paths.
    with session_lock(ROOT/'work/ros-session.lock'):
        raise SystemExit(run_session(args.mode,args.seconds,args.diagnostic,args.bag,args.rate,args.depth_engine,args.calibration,args.tracking_loss_policy,args.scale_config,args.localize_map_db))


if __name__=='__main__':main()

"""Optional local AI map, path and pose in ROS2. Listening never starts a camera.

Use --replay for camera-free verification. Live capture must be started by the
user on the phone. Existing raw-depth listener remains a separate option.
"""
import json
from contextlib import nullcontext
from pathlib import Path
import shutil
import socket
import subprocess
import tempfile
import threading
import time

from host.arcore_ai_stream import AiStream, argument_parser, load_model, paced_packets, recording_info
from host.arcore_scene_export import private_output
from host.arcore_confirmed_map import file_sha256
from host.ros_arcore_live import adb_serial, cloud_message, rviz_config
from host.ros_arcore_saved import path_message


def ai_rviz_config(path):
    import yaml
    rviz_config(path)
    config=yaml.safe_load(Path(path).read_text())
    topics={'/phone/arcore_map':'/phone/arcore_ai_map',
            '/phone/arcore_path':'/phone/arcore_ai_path',
            '/phone/arcore_pose':'/phone/arcore_ai_pose'}
    for display in config['Visualization Manager']['Displays']:
        topic=display.get('Topic',{}).get('Value')
        if topic in topics:
            display['Topic']['Value']=topics[topic]
            display['Topic']['Durability Policy']='Transient Local'
            if topic=='/phone/arcore_map':display['Name']='Deneysel AI derinligi · 1cm / 3 ayri kare'
    config['Visualization Manager']['Displays'].append(dict(
        Class='rviz_default_plugins/Pose',Name='Telefonun kayitli anlik pozu',Enabled=True,
        Shape='Axes',**{'Axes Length':.25,'Axes Radius':.015},
        Topic={'Value':'/phone/arcore_ai_pose','Depth':1,'History Policy':'Keep Last',
               'Reliability Policy':'Reliable','Durability Policy':'Transient Local'}))
    Path(path).write_text(yaml.safe_dump(config,sort_keys=False))


def cancellable_probe_lines(port, stop, startup_seconds=None):
    """Wait for the user's start; cancellation and active-stream idle stay bounded."""
    if startup_seconds is not None and startup_seconds <= 0:
        raise ValueError('Startup timeout must be positive or None')
    deadline=None if startup_seconds is None else time.monotonic()+startup_seconds
    started=False
    while not stop.is_set() and (deadline is None or time.monotonic()<deadline):
        try:
            with socket.create_connection(('127.0.0.1',port),timeout=.5) as client:
                client.settimeout(.5)
                buffer=b''
                last_data=time.monotonic()
                while not stop.is_set():
                    try:chunk=client.recv(65536)
                    except socket.timeout:
                        if time.monotonic()-last_data>(20 if started else 2):
                            if started:raise TimeoutError('Phone stream idle for 20 seconds')
                            break
                        continue
                    if not chunk:
                        if started:
                            if buffer:yield buffer.decode('utf-8')
                            return
                        break
                    started=True;last_data=time.monotonic();buffer+=chunk
                    if len(buffer)>16_000_000:raise ValueError('Oversized ARCore stream line')
                    while b'\n' in buffer:
                        line,buffer=buffer.split(b'\n',1)
                        yield (line+b'\n').decode('utf-8')
        except (ConnectionRefusedError,OSError):
            if started:raise
        stop.wait(.25)
    if not stop.is_set():raise TimeoutError('Phone ARCore stream did not start within the listener timeout')


def main():
    parser=argument_parser()
    parser.add_argument('--serial')
    parser.add_argument('--rviz',action='store_true')
    parser.add_argument('--exit-after-replay',action='store_true')
    args=parser.parse_args()
    if not args.replay and (args.queue_policy!='latest' or args.exclude_frames or args.exit_after_replay):
        parser.error('FIFO, exclusions and exit-after-replay are recorded-only diagnostics')
    if args.replay and args.serial:parser.error('Replay does not contact a phone; omit --serial')
    private_output(args.output)
    digest=recording_info(args.replay) if args.replay else None
    model=load_model(args)
    import rclpy
    from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
    from sensor_msgs.msg import PointCloud2
    from nav_msgs.msg import Path as RosPath
    from geometry_msgs.msg import PoseStamped
    from std_msgs.msg import String
    engine=AiStream(model,args.output,policy=args.queue_policy,use_recorded_focal=args.model=='depth-pro',
        exclude_frames=args.exclude_frames,save_depths=args.save_depths,cell_limit=args.cell_limit)
    serial=port=node=viewer=feeder=None
    stop=threading.Event();input_done=threading.Event()
    feed_result={'error':None}
    error=None
    ros_counts=dict(cloudMessages=0,pathMessages=0,poseMessages=0,statusMessages=0)
    def finalize():
        archived=engine.output/'stream.jsonl'
        save_source=args.replay if args.replay else archived if archived.exists() else None
        save_digest=digest if args.replay else file_sha256(archived) if archived.exists() else None
        return engine.save(source=save_source,source_sha=save_digest,
            runtime_mode='recorded_ros_replay' if args.replay else 'user_started_phone_stream')
    try:
        rclpy.init()
        node=rclpy.create_node('phone_arcore_ai')
        qos=QoSProfile(depth=1,reliability=ReliabilityPolicy.RELIABLE,
                       durability=DurabilityPolicy.TRANSIENT_LOCAL)
        cloud_pub=node.create_publisher(PointCloud2,'/phone/arcore_ai_map',qos)
        path_pub=node.create_publisher(RosPath,'/phone/arcore_ai_path',qos)
        pose_pub=node.create_publisher(PoseStamped,'/phone/arcore_ai_pose',qos)
        status_pub=node.create_publisher(String,'/phone/arcore_ai_status',qos)
        last_pose=last_map=-1
        last_path_time=last_map_time=last_status_time=0.

        def publish(force=False):
            nonlocal last_pose,last_map,last_path_time,last_map_time,last_status_time
            now=time.monotonic()
            cloud,poses,status=engine.snapshot()
            stamp=node.get_clock().now().to_msg()
            if len(poses) and len(poses)!=last_pose and (force or now-last_path_time>=.1):
                path=path_message(poses,stamp)
                path_pub.publish(path)
                pose_pub.publish(path.poses[-1])
                ros_counts['pathMessages']+=1
                ros_counts['poseMessages']+=1
                last_pose=len(poses);last_path_time=now
            if status['mapVersion']!=last_map and (force or now-last_map_time>=.5):
                cloud_pub.publish(cloud_message(cloud,stamp))
                ros_counts['cloudMessages']+=1
                last_map=status['mapVersion'];last_map_time=now
            if force or now-last_status_time>=.2:
                message=String();message.data=json.dumps(status)
                status_pub.publish(message);ros_counts['statusMessages']+=1
                last_status_time=now
        node.create_timer(.05,publish)

        with tempfile.TemporaryDirectory(prefix='arcore-ai-rviz-') as temp:
            if args.rviz:
                config=Path(temp)/'ai.rviz';ai_rviz_config(config)
                viewer=subprocess.Popen(['rviz2','-d',str(config)])
            if args.replay:
                shutil.copyfile(args.replay,engine.output/'stream.jsonl')
                packets=paced_packets(args.replay,args.replay_speed,stop)
                print('Kayıtlı AI replay; telefon/kamera bağlantısı yok.',flush=True)
            else:
                serial=adb_serial(args.serial)
                forwarded=subprocess.run(['adb','-s',serial,'forward','tcp:0','tcp:8766'],
                    capture_output=True,text=True,check=True)
                port=int(forwarded.stdout.strip())
                packets=cancellable_probe_lines(port,stop)
                print('AI dinleyicisi hazır. Süreyi telefonda seçip taramayı kendiniz başlatın.',flush=True)
            def receive():
                try:
                    archive_context=(engine.output/'stream.jsonl').open('x') if not args.replay else nullcontext()
                    with archive_context as archive, (engine.output/'rejections.jsonl').open('x') as rejected:
                        for item in packets:
                            if stop.is_set():break
                            if archive is not None:
                                archive.write(item);archive.flush()
                            try:
                                packet=item if args.replay else json.loads(item)
                                engine.add(packet)
                            except (ValueError,KeyError,TypeError) as exc:
                                engine.reject()
                                rejected.write(json.dumps(dict(reason=str(exc)))+'\n')
                            if engine.end_reason is not None:break
                    if engine.end_reason is None:
                        feed_result['error']='host_interrupted' if stop.is_set() else 'eof_without_end_marker'
                except Exception as exc:feed_result['error']=f'{type(exc).__name__}: {exc}'
                finally:
                    engine.close(feed_result['error'],wait=False)
                    input_done.set()
            feeder=threading.Thread(target=receive,name='arcore-ai-input',daemon=True)
            feeder.start()
            try:
                while not input_done.is_set():rclpy.spin_once(node,timeout_sec=.05)
            except KeyboardInterrupt:
                stop.set();feed_result['error']='host_interrupted'
                feeder.join(3)
                if feeder.is_alive():raise TimeoutError('Input worker has not stopped; final map refused')
            error=feed_result['error']
            engine.close(error,wait=False)
            deadline=time.monotonic()+(300 if args.queue_policy=='fifo' else 30)
            while engine.worker.is_alive() and time.monotonic()<deadline:
                rclpy.spin_once(node,timeout_sec=.05)
            engine.close(error,timeout=0)
            result=finalize()
            publish(force=True)
            (engine.output/'ros-publications.json').write_text(json.dumps(ros_counts,indent=2)+'\n')
            print(json.dumps(result,indent=2),flush=True)
            deadline=time.monotonic()+1
            while time.monotonic()<deadline:rclpy.spin_once(node,timeout_sec=.05)
            if not args.exit_after_replay:
                if result['captureComplete']:
                    print('Son AI haritası kaydedildi; RViz/ROS açık. Çıkış: Ctrl+C.',flush=True)
                else:
                    print('Kayıt tamamlanmadı; yalnız kısmi tanı dosyaları saklandı. '
                          'RViz/ROS açık. Çıkış: Ctrl+C.',flush=True)
                while rclpy.ok() and (viewer is None or viewer.poll() is None):
                    rclpy.spin_once(node,timeout_sec=.2)
    except KeyboardInterrupt:
        engine.close('host_interrupted',wait=False)
    except Exception as exc:
        engine.close(f'{type(exc).__name__}: {exc}',wait=False)
        raise
    finally:
        stop.set()
        try:
            if feeder and feeder.is_alive():feeder.join(3)
            if engine.worker.is_alive():engine.close(engine.error or 'host_shutdown',timeout=30)
            if not engine.closed:engine.close(engine.error or 'host_shutdown')
            if (not (engine.output/'result.json').exists() and not engine.worker.is_alive()
                    and not (feeder and feeder.is_alive())):
                finalize()
        finally:
            if viewer and viewer.poll() is None:viewer.terminate()
            if node:node.destroy_node()
            if rclpy.ok():rclpy.shutdown()
            if port is not None and serial is not None:
                subprocess.run(['adb','-s',serial,'forward','--remove',f'tcp:{port}'],capture_output=True)


if __name__=='__main__':main()

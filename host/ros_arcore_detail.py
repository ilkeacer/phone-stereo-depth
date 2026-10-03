"""Receive the phone's manually started ARCore detailed scan in ROS 2/RViz."""
import argparse
import copy
import json
from pathlib import Path
import subprocess
import tempfile
import time

import numpy as np

from host.arcore_detail import DetailSession
from host.ros_arcore_live import adb_serial, cloud_message, probe_lines, rviz_config
from host.ros_arcore_saved import path_message


def detail_rviz_config(path, confirmed_map=False):
    import yaml
    rviz_config(path)
    config = yaml.safe_load(Path(path).read_text())
    displays = config['Visualization Manager']['Displays']
    cloud = next(d for d in displays if d.get('Topic', {}).get('Value') == '/phone/arcore_map')
    cloud['Name'] = ('Ham derinlik · 1 cm / 3 ayri kare / ortalama'
                     if confirmed_map else 'Ham derinlik · tekrar gorulen noktalar')
    reference = copy.deepcopy(cloud)
    reference['Name'] = 'Karsilastirma · yumusatilmis derinlik'
    reference['Enabled'] = False
    reference['Topic']['Value'] = '/phone/arcore_smooth_reference'
    displays.append(reference)
    paired = copy.deepcopy(reference)
    paired['Name'] = 'Karsilastirma · ayni karelerin ham derinligi'
    paired['Topic']['Value'] = '/phone/arcore_paired_raw'
    displays.append(paired)
    Path(path).write_text(yaml.safe_dump(config, sort_keys=False))


def main():
    import rclpy
    from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
    from sensor_msgs.msg import PointCloud2
    from nav_msgs.msg import Path as RosPath
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--serial')
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--rviz', action='store_true')
    parser.add_argument('--confirmed-map', action='store_true',
                        help='Experimental raw-depth map: 1 cm cells, three distinct frames, per-cell mean')
    args = parser.parse_args()
    if args.output.exists():
        parser.error('Output must be a new local directory')
    serial = adb_serial(args.serial)
    forwarded = subprocess.run(['adb', '-s', serial, 'forward', 'tcp:0', 'tcp:8766'],
                               capture_output=True, text=True, check=True)
    port = int(forwarded.stdout.strip())
    session = (DetailSession(.01, 3, raw_fusion='confirmed_mean') if args.confirmed_map
               else DetailSession())
    viewer = node = None
    args.output.mkdir(parents=True, exist_ok=False)
    rclpy.init()
    try:
        node = rclpy.create_node('phone_arcore_detail')
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.TRANSIENT_LOCAL)
        cloud_pub = node.create_publisher(PointCloud2, '/phone/arcore_map', qos)
        reference_pub = node.create_publisher(PointCloud2, '/phone/arcore_smooth_reference', qos)
        paired_pub = node.create_publisher(PointCloud2, '/phone/arcore_paired_raw', qos)
        path_pub = node.create_publisher(RosPath, '/phone/arcore_path', qos)

        def publish():
            stamp = node.get_clock().now().to_msg()
            cloud_pub.publish(cloud_message(session.raw.array(), stamp))
            reference_pub.publish(cloud_message(session.smooth.array(), stamp))
            paired_pub.publish(cloud_message(session.paired_raw.array(), stamp))
            if session.trajectory:
                poses = np.array([(i, *pos, *q, ts / 1e9) for i, (ts, pos, q) in enumerate(session.trajectory)])
                path_pub.publish(path_message(poses, stamp))
            rclpy.spin_once(node, timeout_sec=0)

        with tempfile.TemporaryDirectory(prefix='arcore-detail-rviz-') as temp:
            if args.rviz:
                config = Path(temp) / 'detail.rviz'
                detail_rviz_config(config, confirmed_map=args.confirmed_map)
                viewer = subprocess.Popen(['rviz2', '-d', str(config)])
            print('Ayrıntılı harita dinleyicisi hazır. Telefonda “ARCore 60 sn ayrıntılı harita” düğmesine basın.', flush=True)
            last_publish = 0.
            error = None
            try:
                with (args.output / 'stream.jsonl').open('w') as archive, (args.output / 'rejections.jsonl').open('w') as rejected:
                    for line in probe_lines(port):
                        archive.write(line)
                        archive.flush()
                        try:
                            packet = json.loads(line)
                            if isinstance(packet, dict) and packet.get('schemaVersion') == 1:
                                raise RuntimeError('Eski tarama modu seçildi. Ayrıntılı harita düğmesini kullanın.')
                            changed = session.add(packet)
                        except (ValueError, KeyError, TypeError) as exc:
                            session.counts['rejectedPackets'] += 1
                            rejected.write(json.dumps(dict(reason=str(exc))) + '\n')
                            continue
                        if changed and time.monotonic() - last_publish > .5:
                            publish()
                            last_publish = time.monotonic()
                            print(f"Ham derinlik {session.counts['depthFrames']} kare · "
                                  f"tekrar görülen hücre {len(session.raw.array())} · "
                                  f"referans {len(session.smooth.array())}", flush=True)
                        if session.end_reason is not None:
                            break
            except (OSError, RuntimeError) as exc:
                error = str(exc)
            except KeyboardInterrupt:
                error = 'host_interrupted'
            if session.end_reason is None and error is None:
                error = 'eof_without_end_marker'
            summary = session.save(args.output)
            summary['streamError'] = error
            (args.output / 'result.json').write_text(json.dumps(summary, indent=2) + '\n')
            print(json.dumps(summary, ensure_ascii=False), flush=True)
            if summary['mapTruncated']:
                print('UYARI: Harita hücre sınırına ulaştı; kayıt tam olsa da canlı bulutta nokta eksik.',
                      flush=True)
            if rclpy.ok():
                publish()
            if viewer and viewer.poll() is None and session.trajectory and rclpy.ok():
                print('Harita kaydedildi. RViz son haritayı gösteriyor.', flush=True)
                while viewer.poll() is None:
                    rclpy.spin_once(node, timeout_sec=.2)
    except KeyboardInterrupt:
        pass
    finally:
        if viewer and viewer.poll() is None:
            viewer.terminate()
        if node:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        subprocess.run(['adb', '-s', serial, 'forward', '--remove', f'tcp:{port}'], capture_output=True)


if __name__ == '__main__':
    main()

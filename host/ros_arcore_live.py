"""Display one ARCore probe's pose and accumulated Depth API points in ROS 2.

Run this before tapping the phone's 12-second ARCore button. Local ADB forwarding
is used; no camera image is recorded or sent. This is a diagnostic local map,
not cross-session relocalization or a validated robot navigation map.
"""
import argparse
import json
import os
from pathlib import Path
import socket
import subprocess
import tempfile
import time

import numpy as np

from host.arcore_probe_import import PLY_HEADER, ros_pose
from host.device import select_adb_serial


def converted_packet(packet):
    if packet.get('schemaVersion') != 1:
        raise ValueError('Unknown ARCore stream schema')
    timestamp = packet.get('timestampNs')
    if not isinstance(timestamp, int) or timestamp <= 0:
        raise ValueError('Invalid ARCore timestamp')
    state = packet.get('trackingState')
    if state not in ('TRACKING', 'PAUSED', 'STOPPED'):
        raise ValueError('Invalid ARCore tracking state')
    raw_points = np.asarray(packet.get('points'), dtype=np.float32)
    if raw_points.size == 0:
        raw_points = raw_points.reshape(0, 3)
    if raw_points.ndim != 2 or raw_points.shape[1] != 3 or len(raw_points) > 10_000 or not np.isfinite(raw_points).all():
        raise ValueError('Invalid ARCore points')
    if state != 'TRACKING' and len(raw_points):
        raise ValueError('Depth points require tracking')
    translation = np.asarray(packet.get('translationM'), dtype=np.float64)
    quaternion = np.asarray(packet.get('quaternion'), dtype=np.float64)
    if translation.shape != (3,) or quaternion.shape != (4,) or not np.isfinite(translation).all() or not np.isfinite(quaternion).all():
        raise ValueError('Invalid ARCore pose')
    norm = float(np.linalg.norm(quaternion))
    if not .99 <= norm <= 1.01:
        raise ValueError('Invalid ARCore rotation')
    position, rotation = ros_pose(translation, quaternion / norm)
    points = raw_points[:, [0, 2, 1]].copy()
    points[:, 1] *= -1
    return timestamp, state, position, rotation, points


class VoxelMap:
    def __init__(self, resolution=.05, limit=150_000):
        self.resolution = resolution
        self.limit = limit
        self.cells = {}

    def add(self, points):
        for point in points:
            key = tuple(np.floor(point / self.resolution).astype(np.int32))
            if key in self.cells or len(self.cells) < self.limit:
                self.cells[key] = tuple(map(float, point))

    def array(self):
        return np.asarray(list(self.cells.values()), dtype=np.float32).reshape(-1, 3)


def cloud_message(xyz, stamp, rgb=None):
    from sensor_msgs.msg import PointCloud2, PointField
    points = np.empty(len(xyz), dtype=[('x', '<f4'), ('y', '<f4'), ('z', '<f4'), ('rgb', '<u4')])
    for index, name in enumerate('xyz'):
        points[name] = xyz[:, index]
    if rgb is None:
        points['rgb'] = 0xb9c9d7
    else:
        rgb = np.asarray(rgb)
        if rgb.shape != (len(xyz), 3) or not np.isfinite(rgb).all() or np.any(rgb < 0) or np.any(rgb > 1):
            raise ValueError('Expected RGB values in [0, 1] for each point')
        channels = np.rint(rgb * 255).astype(np.uint32)
        points['rgb'] = (channels[:, 0] << 16) | (channels[:, 1] << 8) | channels[:, 2]
    msg = PointCloud2()
    msg.header.frame_id = 'arcore_map'
    msg.header.stamp = stamp
    msg.height = 1
    msg.width = len(points)
    msg.fields = [PointField(name=name, offset=index * 4,
                             datatype=PointField.FLOAT32 if index < 3 else PointField.UINT32, count=1)
                  for index, name in enumerate(('x', 'y', 'z', 'rgb'))]
    msg.point_step = 16
    msg.row_step = len(points) * 16
    msg.is_dense = True
    msg.data = points.tobytes()
    return msg


def save_diagnostic(output, voxel_map, trajectory, counts):
    output = Path(output)
    output.mkdir(parents=True, exist_ok=False)
    xyz = voxel_map.array()
    summary = dict(status='diagnostic', source='arcore_depth_api', scaleSource='arcore_depth_api',
                   metricAccuracyValidated=False, points=len(xyz), poses=len(trajectory), **counts,
                   note='Local single-session ARCore accumulation; no loop closure or relocalization; no images')
    (output / 'result.json').write_text(json.dumps(summary, indent=2, ensure_ascii=False) + '\n')
    if len(xyz) and trajectory:
        vertices = np.empty(len(xyz), dtype=[(name, '<f4') for name in 'xyz'] + [(name, 'u1') for name in ('red', 'green', 'blue')])
        for index, name in enumerate('xyz'):
            vertices[name] = xyz[:, index]
        vertices['red'], vertices['green'], vertices['blue'] = 185, 201, 215
        (output / 'map_cloud.ply').write_bytes(PLY_HEADER.format(count=len(xyz)).encode('ascii') + vertices.tobytes())
        with (output / 'map_poses.txt').open('w') as stream:
            for index, (stamp, position, rotation) in enumerate(trajectory):
                stream.write(' '.join(map(str, (index, *position, *rotation, stamp / 1e9))) + '\n')
    return summary


def rviz_config(path):
    import yaml
    root = Path(__file__).resolve().parents[1]
    config = yaml.safe_load((root / 'configs/phone-map.rviz').read_text())
    manager = config['Visualization Manager']
    manager['Global Options']['Fixed Frame'] = 'arcore_map'
    for display in manager['Displays']:
        name = display.get('Name')
        display['Enabled'] = name in ('Zemin', 'Canli 3B harita', 'Canli kamera yolu')
        if name == 'Canli 3B harita':
            display['Name'] = 'ARCore tanı 3B noktaları'
            display['Topic']['Value'] = '/phone/arcore_map'
        if name == 'Canli kamera yolu':
            display['Name'] = 'ARCore anlık kamera yolu'
            display['Topic']['Value'] = '/phone/arcore_path'
    Path(path).write_text(yaml.safe_dump(config, sort_keys=False))


def adb_serial(requested):
    result = subprocess.run(['adb', 'devices', '-l'], capture_output=True, text=True, check=True)
    devices = [fields[0] for line in result.stdout.splitlines()[1:]
               if (fields := line.split()) and len(fields) >= 2 and fields[1] == 'device']
    return select_adb_serial(devices, requested or os.environ.get('PHONE_ADB_SERIAL'))


def probe_lines(port, startup_seconds=300):
    """ADB forward accepts locally even when the phone endpoint is not open yet."""
    deadline = time.monotonic() + startup_seconds
    started = False
    while time.monotonic() < deadline:
        try:
            with socket.create_connection(('127.0.0.1', port), timeout=.5) as client:
                client.settimeout(2)
                with client.makefile('r', encoding='utf-8') as stream:
                    first = stream.readline()
                    if not first:
                        time.sleep(.25)
                        continue
                    client.settimeout(20)
                    started = True
                    yield first
                    yield from stream
                    return
        except (ConnectionRefusedError, TimeoutError, OSError):
            if started:
                raise
            time.sleep(.25)
    raise TimeoutError('Telefonun ARCore nokta akışı 5 dakikada başlamadı')


def main():
    import rclpy
    from nav_msgs.msg import Path as RosPath
    from geometry_msgs.msg import PoseStamped
    from sensor_msgs.msg import PointCloud2
    from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True, help='New ignored work/ directory')
    parser.add_argument('--serial', help='ADB transport, or PHONE_ADB_SERIAL')
    parser.add_argument('--rviz', action='store_true')
    args = parser.parse_args()
    if args.output.exists():
        parser.error('Output directory already exists')
    serial = adb_serial(args.serial)
    forwarded = subprocess.run(['adb', '-s', serial, 'forward', 'tcp:0', 'tcp:8766'],
                               capture_output=True, text=True, check=True)
    port = int(forwarded.stdout.strip())
    viewer = None
    node = None
    rclpy.init()
    try:
        node = rclpy.create_node('phone_arcore_diagnostic')
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.TRANSIENT_LOCAL)
        cloud_pub = node.create_publisher(PointCloud2, '/phone/arcore_map', qos)
        path_pub = node.create_publisher(RosPath, '/phone/arcore_path', qos)
        path = RosPath()
        path.header.frame_id = 'arcore_map'
        voxels = VoxelMap()
        trajectory = []
        counts = dict(packets=0, trackedPackets=0, pausedPackets=0, inputPoints=0, droppedPackets=0)
        last_timestamp = 0
        last_cloud_publish = 0.
        with tempfile.TemporaryDirectory(prefix='arcore-rviz-') as temp:
            if args.rviz:
                config = Path(temp) / 'arcore.rviz'
                rviz_config(config)
                viewer = subprocess.Popen(['rviz2', '-d', str(config)])
            print('ARCore dinleyici hazır. Telefonda seçtiğiniz ARCore deneme düğmesine basın.', flush=True)
            counts['streamError'] = None
            try:
                for line in probe_lines(port):
                    try:
                        packet=json.loads(line)
                        if packet.get('schemaVersion') == 2:
                            raise RuntimeError('Ayrıntı modu için python3 -m host.ros_arcore_detail kullanın')
                        timestamp, state, position, rotation, points = converted_packet(packet)
                    except (ValueError, KeyError, TypeError):
                        counts['droppedPackets'] += 1
                        continue
                    if timestamp <= last_timestamp:
                        counts['droppedPackets'] += 1
                        continue
                    last_timestamp = timestamp
                    counts['packets'] += 1
                    if state != 'TRACKING':
                        counts['pausedPackets'] += 1
                        continue
                    counts['trackedPackets'] += 1
                    counts['inputPoints'] += len(points)
                    voxels.add(points)
                    trajectory.append((timestamp, position, rotation))
                    stamp = node.get_clock().now().to_msg()
                    pose = PoseStamped()
                    pose.header.frame_id = 'arcore_map'
                    pose.header.stamp = stamp
                    pose.pose.position.x, pose.pose.position.y, pose.pose.position.z = position
                    pose.pose.orientation.x, pose.pose.orientation.y, pose.pose.orientation.z, pose.pose.orientation.w = rotation
                    path.poses.append(pose)
                    path.header.stamp = stamp
                    path_pub.publish(path)
                    if len(voxels.cells) and time.monotonic() - last_cloud_publish >= .5:
                        cloud_pub.publish(cloud_message(voxels.array(), stamp))
                        last_cloud_publish = time.monotonic()
                    rclpy.spin_once(node, timeout_sec=0)
                    if counts['packets'] % 10 == 0:
                        print(f"Takip {counts['trackedPackets']}/{counts['packets']} · 3B hücre {len(voxels.cells)}", flush=True)
            except (OSError, TimeoutError, RuntimeError) as error:
                counts['streamError'] = str(error)
            summary = save_diagnostic(args.output, voxels, trajectory, counts)
            print(json.dumps(summary, ensure_ascii=False), flush=True)
            if viewer and viewer.poll() is None and counts['packets']:
                print('Kayıt sona erdi. RViz son haritayı gösteriyor; pencereyi kapatınca çıkılacak.', flush=True)
                while viewer.poll() is None:
                    rclpy.spin_once(node, timeout_sec=.2)
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

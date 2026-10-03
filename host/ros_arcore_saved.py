"""Reopen a locally saved ARCore diagnostic cloud and trajectory in ROS 2/RViz.

The saved pose is local to its capture session. Displaying it does not perform
cross-session relocalization or validate the map for robot navigation.
"""
import argparse
from pathlib import Path
import subprocess
import tempfile

import numpy as np

from host.ros_arcore_live import cloud_message, rviz_config
from host.ros_map_view import read_pcl


def load_saved(session, classified=None):
    session = Path(session)
    cloud = Path(classified) if classified else session / 'map_cloud.ply'
    xyz, rgb = read_pcl(cloud)
    poses = np.loadtxt(session / 'map_poses.txt', ndmin=2)
    if poses.shape[1] != 9 or not np.isfinite(poses).all() or len(poses) < 1:
        raise ValueError('Expected finite 9-column ARCore poses')
    if not np.all(np.diff(poses[:, -1]) > 0):
        raise ValueError('ARCore pose timestamps must increase')
    return xyz, rgb, poses


def path_message(poses, stamp):
    from geometry_msgs.msg import PoseStamped
    from nav_msgs.msg import Path as RosPath
    path = RosPath()
    path.header.frame_id = 'arcore_map'
    path.header.stamp = stamp
    for row in poses:
        pose = PoseStamped()
        pose.header = path.header
        pose.pose.position.x, pose.pose.position.y, pose.pose.position.z = map(float, row[1:4])
        pose.pose.orientation.x, pose.pose.orientation.y, pose.pose.orientation.z, pose.pose.orientation.w = map(float, row[4:8])
        path.poses.append(pose)
    return path


def main():
    import rclpy
    from nav_msgs.msg import Path as RosPath
    from sensor_msgs.msg import PointCloud2
    from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--session', type=Path, required=True)
    parser.add_argument('--classified', type=Path, help='Optional separately derived colored cloud')
    parser.add_argument('--rviz', action='store_true')
    args = parser.parse_args()
    xyz, rgb, poses = load_saved(args.session, args.classified)
    rclpy.init()
    node = rclpy.create_node('phone_arcore_saved_map')
    viewer = None
    try:
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.TRANSIENT_LOCAL)
        cloud_pub = node.create_publisher(PointCloud2, '/phone/arcore_map', qos)
        path_pub = node.create_publisher(RosPath, '/phone/arcore_path', qos)
        with tempfile.TemporaryDirectory(prefix='arcore-saved-rviz-') as temp:
            if args.rviz:
                config = Path(temp) / 'saved.rviz'
                rviz_config(config)
                viewer = subprocess.Popen(['rviz2', '-d', str(config)])
            stamp = node.get_clock().now().to_msg()
            cloud_pub.publish(cloud_message(xyz, stamp, rgb))
            path_pub.publish(path_message(poses, stamp))
            print(f'Kayıtlı ARCore haritası ROS\'ta açık: {len(xyz)} nokta, {len(poses)} poz. '
                  'Telefon kamerası kapalı. Çıkmak için RViz penceresini kapatın veya Ctrl+C.', flush=True)
            while viewer is None or viewer.poll() is None:
                rclpy.spin_once(node, timeout_sec=.2)
    except KeyboardInterrupt:
        pass
    finally:
        if viewer and viewer.poll() is None:
            viewer.terminate()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()

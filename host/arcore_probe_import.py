"""Convert one local ARCore depth probe into a viewable ROS map diagnostic.

The probe's map is a single-session point accumulation. This does not perform
loop closure or certify the depth/pose scale for robot navigation.
"""
import argparse
import json
import math
from pathlib import Path
import struct

import numpy as np


MAX_POINTS = 2_000_000
PLY_HEADER = ("ply\nformat binary_little_endian 1.0\n"
              "element vertex {count}\nproperty float x\nproperty float y\n"
              "property float z\nproperty uchar red\nproperty uchar green\n"
              "property uchar blue\nend_header\n")


def read_probe_cloud(source):
    """Read only the XYZ ASCII PLY produced by ArCoreProbeActivity."""
    with Path(source).open('r', encoding='ascii') as stream:
        if stream.readline().strip() != 'ply':
            raise ValueError('Expected PLY')
        if stream.readline().strip() != 'format ascii 1.0':
            raise ValueError('Expected ARCore ASCII PLY')
        count = None
        properties = []
        for _ in range(32):
            line = stream.readline()
            if not line:
                raise ValueError('Truncated PLY header')
            words = line.split()
            if words[:2] == ['element', 'vertex']:
                count = int(words[2])
            elif words[:1] == ['property']:
                properties.append(words)
            elif words == ['end_header']:
                break
        else:
            raise ValueError('PLY header too long')
        if count is None or not 0 < count <= MAX_POINTS:
            raise ValueError('Empty or oversized ARCore cloud')
        if properties != [['property', 'float', key] for key in 'xyz']:
            raise ValueError('Expected XYZ-only ARCore cloud')
        xyz = np.empty((count, 3), dtype=np.float32)
        for i in range(count):
            words = stream.readline().split()
            if len(words) != 3:
                raise ValueError('Truncated or malformed PLY vertex')
            xyz[i] = [float(word) for word in words]
        if stream.read().strip():
            raise ValueError('Unexpected data after PLY vertices')
    if not np.isfinite(xyz).all():
        raise ValueError('Nonfinite ARCore point')
    return xyz


def read_probe_report(source):
    events = [json.loads(line) for line in Path(source).read_text().splitlines() if line.strip()]
    result = next((event for event in reversed(events) if event.get('event') == 'result'), None)
    exported = next((event for event in reversed(events) if event.get('event') == 'map_export'), None)
    if result is None or exported is None or result.get('reason') not in ('finished', 'stopped_by_user'):
        raise ValueError('ARCore probe did not finish or stop cleanly')
    if not any(event.get('event') == 'session_created' for event in events):
        raise ValueError('ARCore session evidence missing')
    poses = []
    last_stamp = -1
    for event in events:
        if event.get('event') != 'frame' or event.get('trackingState') != 'TRACKING':
            continue
        timestamp = event['timestampNs']
        translation = np.asarray(event['translationM'], dtype=np.float64)
        quaternion = np.asarray(event['quaternion'], dtype=np.float64)
        if not isinstance(timestamp, int) or timestamp <= last_stamp:
            raise ValueError('Nonmonotonic ARCore frame timestamps')
        if translation.shape != (3,) or quaternion.shape != (4,) or not np.isfinite(translation).all() or not np.isfinite(quaternion).all():
            raise ValueError('Invalid ARCore pose')
        length = float(np.linalg.norm(quaternion))
        if not .99 <= length <= 1.01:
            raise ValueError('Invalid ARCore pose rotation')
        poses.append((timestamp / 1e9, translation, quaternion / length))
        last_stamp = timestamp
    if not poses:
        raise ValueError('No tracked ARCore poses')
    return result, exported, poses


def ros_pose(translation, quaternion):
    """Rotate ARCore (+Y up, -Z forward) into ROS (+Z up, +Y forward)."""
    x, y, z = map(float, translation)
    qx, qy, qz, qw = map(float, quaternion)
    s = math.sqrt(.5)  # +90 degrees about X; left-multiply world orientation
    return (x, -z, y), (s * (qw + qx), s * (qy - qz), s * (qz + qy), s * (qw - qx))


def convert(report_path, cloud_path, output_dir):
    result, exported, poses = read_probe_report(report_path)
    xyz = read_probe_cloud(cloud_path)
    if exported.get('points') != len(xyz):
        raise ValueError('Probe log and cloud point counts differ')
    if not any(event.get('event') == 'depth' and event.get('sampleCount', 0) > 0
               for event in map(json.loads, Path(report_path).read_text().splitlines())):
        raise ValueError('No nonzero sampled depth in probe report')
    xyz = xyz[:, [0, 2, 1]].copy()
    xyz[:, 1] *= -1
    output_dir = Path(output_dir)
    output_dir.mkdir(parents=True, exist_ok=False)
    points = np.empty(len(xyz), dtype=[(key, '<f4') for key in 'xyz'] + [(key, 'u1') for key in ('red', 'green', 'blue')])
    for i, key in enumerate('xyz'):
        points[key] = xyz[:, i]
    points['red'], points['green'], points['blue'] = 185, 201, 215
    (output_dir / 'map_cloud.ply').write_bytes(PLY_HEADER.format(count=len(points)).encode('ascii') + points.tobytes())
    with (output_dir / 'map_poses.txt').open('w') as stream:
        for index, (stamp, translation, quaternion) in enumerate(poses):
            position, rotation = ros_pose(translation, quaternion)
            stream.write(' '.join(map(str, (index, *position, *rotation, stamp))) + '\n')
    metadata = dict(status='diagnostic', source='arcore_depth_api', scaleSource='arcore_depth_api',
                    metricAccuracyValidated=False, points=len(points), poses=len(poses),
                    captureEndReason=result['reason'],
                    trackingFrames=result['tracking'], pausedFrames=result['paused'],
                    depthFrames=result['depthFrames'], coordinateFrame='ROS Z-up from ARCore local session',
                    note='One ARCore session; no loop closure or cross-session localization; untextured cloud')
    (output_dir / 'result.json').write_text(json.dumps(metadata, indent=2, ensure_ascii=False) + '\n')
    return metadata


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('report', type=Path, help='ARCore probe JSONL pulled from phone')
    parser.add_argument('cloud', type=Path, help='ARCore probe ASCII PLY pulled from phone')
    parser.add_argument('output', type=Path, help='New output directory; must not exist')
    args = parser.parse_args()
    print(json.dumps(convert(args.report, args.cloud, args.output), indent=2, ensure_ascii=False))


if __name__ == '__main__':
    main()

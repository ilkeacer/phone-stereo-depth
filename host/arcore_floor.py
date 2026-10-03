"""Extract a gravity-aligned surface candidate from a saved ARCore ROS cloud.

This is a local geometry diagnostic, not free-space or robot navigation output.
The ARCore world frame supplies the up axis; the physical floor is not verified.
"""
import argparse
import json
from pathlib import Path

import numpy as np

from host.arcore_probe_import import PLY_HEADER
from host.ros_map_view import read_pcl


def floor_candidate(xyz, camera_positions, *, seed=20260926, trials=1000, tolerance=.05):
    xyz = np.asarray(xyz, dtype=np.float64)
    cameras = np.asarray(camera_positions, dtype=np.float64)
    if xyz.ndim != 2 or xyz.shape[1] != 3 or cameras.ndim != 2 or cameras.shape[1] != 3:
        raise ValueError('Expected XYZ points and camera positions')
    if not np.isfinite(xyz).all() or not np.isfinite(cameras).all():
        raise ValueError('Nonfinite geometry')
    if len(xyz) < 500 or not len(cameras):
        return None
    rng = np.random.default_rng(seed)
    sample = xyz[rng.choice(len(xyz), min(len(xyz), 10_000), replace=False)]
    camera = np.median(cameras, axis=0)
    best_count, best_plane = 0, None
    min_up = np.cos(np.deg2rad(10))
    for _ in range(trials):
        p = sample[rng.choice(len(sample), 3, replace=False)]
        normal = np.cross(p[1] - p[0], p[2] - p[0])
        length = np.linalg.norm(normal)
        if length < 1e-7:
            continue
        normal /= length
        if normal[2] < 0:
            normal = -normal
        if normal[2] < min_up:
            continue
        offset = -float(normal @ p[0])
        # A tabletop near camera height should not be labeled as floor.
        plane_at_camera = -(normal[0] * camera[0] + normal[1] * camera[1] + offset) / normal[2]
        if plane_at_camera > camera[2] - .5:
            continue
        count = int(np.count_nonzero(np.abs(sample @ normal + offset) <= tolerance))
        if count > best_count:
            best_count, best_plane = count, (normal, offset)
    if best_plane is None:
        return None
    normal, offset = best_plane
    # Least-squares refinement uses only provisional inliers and keeps gravity constraint.
    for _ in range(2):
        mask = np.abs(xyz @ normal + offset) <= tolerance
        if mask.sum() < 500:
            return None
        fitted, *_ = np.linalg.lstsq(np.column_stack([xyz[mask, :2], np.ones(mask.sum())]), xyz[mask, 2], rcond=None)
        normal = np.asarray([-fitted[0], -fitted[1], 1.], dtype=np.float64)
        normal /= np.linalg.norm(normal)
        offset = -float(fitted[2]) * normal[2]
        if normal[2] < min_up:
            return None
    distances = np.abs(xyz @ normal + offset)
    mask = distances <= tolerance
    if mask.sum() < 500:
        return None
    plane_at_camera = -(normal[0] * camera[0] + normal[1] * camera[1] + offset) / normal[2]
    if plane_at_camera > camera[2] - .5:
        return None
    floor_xy = xyz[mask, :2]
    cells = np.unique(np.floor(floor_xy / .1).astype(np.int32), axis=0)
    return dict(mask=mask, normal=normal, offset=offset,
                metrics=dict(status='geometric_candidate', source='arcore_depth_api',
                             physicalFloorValidated=False, robotFreeSpaceValidated=False,
                             inputPoints=len(xyz), candidatePoints=int(mask.sum()),
                             candidateFraction=round(float(mask.mean()), 4),
                             tiltDegrees=round(float(np.degrees(np.arccos(normal[2]))), 3),
                             planeHeightAtCameraM=round(float(plane_at_camera), 3),
                             cameraHeightM=round(float(camera[2]), 3),
                             distanceP95M=round(float(np.percentile(distances[mask], 95)), 4),
                             toleranceM=tolerance, floor10cmCells=len(cells),
                             occupied10cmCellAreaM2=round(len(cells) * .01, 3)))


def write_cloud(path, xyz, floor_mask=None):
    points = np.empty(len(xyz), dtype=[(name, '<f4') for name in 'xyz'] +
                      [(name, 'u1') for name in ('red', 'green', 'blue')])
    for index, name in enumerate('xyz'):
        points[name] = xyz[:, index]
    points['red'], points['green'], points['blue'] = 185, 201, 215
    if floor_mask is not None:
        points['red'][floor_mask], points['green'][floor_mask], points['blue'][floor_mask] = 65, 225, 130
    Path(path).write_bytes(PLY_HEADER.format(count=len(points)).encode('ascii') + points.tobytes())


def analyze(cloud, poses, output):
    xyz, _ = read_pcl(cloud)
    trajectory = np.loadtxt(poses, ndmin=2)
    if trajectory.shape[1] != 9 or not np.isfinite(trajectory).all():
        raise ValueError('Expected 9-column finite ARCore trajectory')
    result = floor_candidate(xyz, trajectory[:, 1:4])
    output = Path(output)
    output.mkdir(parents=True, exist_ok=False)
    if result is None:
        metrics = dict(status='no_candidate', inputPoints=len(xyz))
    else:
        metrics = result['metrics']
        mask = result['mask']
        write_cloud(output / 'surface_candidate.ply', xyz[mask])
        write_cloud(output / 'classified_map.ply', xyz, mask)
    (output / 'floor_metrics.json').write_text(json.dumps(metrics, indent=2) + '\n')
    return metrics


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--cloud', type=Path, required=True)
    parser.add_argument('--poses', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True, help='New local directory')
    args = parser.parse_args()
    print(json.dumps(analyze(args.cloud, args.poses, args.output), indent=2))


if __name__ == '__main__':
    main()

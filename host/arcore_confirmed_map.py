"""Build an experimental confirmed-point map from a saved ARCore detail stream.

The phone is never contacted. Raw depth, confidence, pose and frame identity come
from the archive; RGB and smoothed depth are not used. Output is private scene data
and belongs under ignored ``work/``. ARCore units are not physical ground truth.
"""
import argparse
import hashlib
import json
from pathlib import Path

import numpy as np

from host.arcore_detail import COMPLETE_CAPTURE_REASONS, decode_packet, unproject
from host.arcore_floor import write_cloud


def file_sha256(path):
    digest = hashlib.sha256()
    with Path(path).open('rb') as source:
        for chunk in iter(lambda: source.read(1024 * 1024), b''):
            digest.update(chunk)
    return digest.hexdigest()


def confirmed_voxels(points, frame_ids, resolution=.01, min_frames=3):
    """One mean point per voxel; each independent depth frame has equal weight."""
    points = np.asarray(points, dtype=np.float32)
    frame_ids = np.asarray(frame_ids)
    if (points.ndim != 2 or points.shape[1] != 3 or frame_ids.shape != (len(points),)
            or not len(points) or not np.isfinite(points).all()):
        raise ValueError('Finite XYZ points and one frame ID per point are required')
    if not .005 <= resolution <= .1 or type(min_frames) is not int or min_frames < 1:
        raise ValueError('Invalid voxel resolution or frame support')
    key = np.floor(points / resolution).astype(np.int32)
    order = np.lexsort((frame_ids, key[:, 2], key[:, 1], key[:, 0]))
    key, frame_ids, points = key[order], frame_ids[order], points[order]
    new_frame_cell = np.r_[True, np.any(key[1:] != key[:-1], axis=1) |
                           (frame_ids[1:] != frame_ids[:-1])]
    first = np.flatnonzero(new_frame_cell)
    counts = np.diff(np.r_[first, len(points)])
    frame_key = key[first]
    frame_mean = np.add.reduceat(points, first, axis=0) / counts[:, None]
    order = np.lexsort((frame_key[:, 2], frame_key[:, 1], frame_key[:, 0]))
    frame_key, frame_mean = frame_key[order], frame_mean[order]
    new_cell = np.r_[True, np.any(frame_key[1:] != frame_key[:-1], axis=1)]
    first = np.flatnonzero(new_cell)
    support = np.diff(np.r_[first, len(frame_mean)])
    representative = np.add.reduceat(frame_mean, first, axis=0) / support[:, None]
    keep = support >= min_frames
    stats = dict(inputPoints=len(points), perFrameCells=len(frame_mean),
                 allCells=len(first), confirmedCells=int(keep.sum()))
    return representative[keep].astype(np.float32), support[keep], stats


def build(source, output, resolution=.01, min_frames=3, confidence_min=128,
          max_points=5_000_000):
    source, output = Path(source), Path(output)
    if output.exists():
        raise FileExistsError(output)
    if not source.is_file():
        raise FileNotFoundError(source)
    if type(confidence_min) is not int or not 0 <= confidence_min <= 255:
        raise ValueError('Invalid confidence threshold')
    if type(max_points) is not int or max_points < 1:
        raise ValueError('Invalid point budget')
    before = file_sha256(source)
    point_parts, id_parts, trajectory = [], [], []
    last_sequence = last_pose_stamp = last_depth_stamp = 0
    frames = accepted_pixels = 0
    end_reason = None
    with source.open() as archive:
        for line in archive:
            if not line.strip():
                continue
            if end_reason is not None:
                raise ValueError('Packets after end marker')
            row = decode_packet(json.loads(line))
            if row['sequence'] <= last_sequence:
                raise ValueError('Nonincreasing packet sequence')
            last_sequence = row['sequence']
            if row['kind'] == 'end':
                end_reason = row['reason']
                continue
            if row['timestamp'] < last_pose_stamp:
                raise ValueError('Decreasing camera timestamp')
            if row['state'] == 'TRACKING' and row['timestamp'] > last_pose_stamp:
                trajectory.append((row['timestamp'], row['position'], row['rotation']))
                last_pose_stamp = row['timestamp']
            if row['kind'] != 'depth':
                continue
            if row['depth_timestamp'] <= last_depth_stamp:
                raise ValueError('Duplicate or decreasing depth timestamp')
            last_depth_stamp = row['depth_timestamp']
            raw, confidence = row['raw'], row['confidence']
            trusted = (raw >= 500) & (raw <= 5000) & (confidence >= confidence_min)
            points = unproject(raw, trusted, row['intrinsics'], row['translation'], row['quaternion'])
            accepted_pixels += len(points)
            if accepted_pixels > max_points:
                raise ValueError('Point budget exceeded; use a shorter recording')
            point_parts.append(points)
            id_parts.append(np.full(len(points), frames, dtype=np.int32))
            frames += 1
    if end_reason not in COMPLETE_CAPTURE_REASONS or frames < min_frames or not accepted_pixels:
        raise ValueError('Complete capture with enough trusted depth frames required')
    after = file_sha256(source)
    if before != after:
        raise RuntimeError('Source changed during replay')
    cloud, support, stats = confirmed_voxels(
        np.concatenate(point_parts), np.concatenate(id_parts), resolution, min_frames)
    output.mkdir(parents=True, exist_ok=False)
    if len(cloud):
        write_cloud(output / 'map_cloud.ply', cloud)
    with (output / 'map_poses.txt').open('w') as poses:
        for index, (stamp, position, rotation) in enumerate(trajectory):
            poses.write(' '.join(map(str, (index, *position, *rotation, stamp / 1e9))) + '\n')
    result = dict(status='experimental', source='arcore_raw_depth_confidence',
                  sourceFile=str(source.resolve()), sourceSha256Before=before,
                  sourceSha256After=after, sourceUnchanged=True,
                  captureComplete=True, captureEndReason=end_reason,
                  depthFrames=frames, poses=len(trajectory), acceptedRawPixels=accepted_pixels,
                  points=len(cloud), **stats,
                  supportP50=float(np.median(support)) if len(support) else None,
                  supportP95=float(np.percentile(support, 95)) if len(support) else None,
                  settings=dict(voxelM=resolution, minIndependentDepthFrames=min_frames,
                                confidenceMinimum=confidence_min, depthMm=[500, 5000],
                                fusion='mean of one measured point per frame per cell; no hole filling'),
                  metricAccuracyValidated=False,
                  note='Recorded-session map candidate; no loop closure, relocalization, or physical accuracy claim')
    (output / 'result.json').write_text(json.dumps(result, indent=2) + '\n')
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--input', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--voxel-m', type=float, default=.01)
    parser.add_argument('--min-frames', type=int, default=3)
    parser.add_argument('--confidence-min', type=int, default=128)
    args = parser.parse_args()
    print(json.dumps(build(args.input, args.output, args.voxel_m,
                           args.min_frames, args.confidence_min), indent=2))


if __name__ == '__main__':
    main()

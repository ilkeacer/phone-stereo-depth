"""Replay paired ARCore raw/confidence and smoothed depth without a phone or ROS.

Numeric depth archives remain private scene data. No RGB image is required.
Point count, confidence and edge contrast do not establish physical accuracy.
"""
import argparse
import base64
import binascii
import hashlib
import json
from pathlib import Path
import shutil

import numpy as np

# Intentional user save is a complete capture; interruption/error/cancel remain incomplete.
COMPLETE_CAPTURE_REASONS = frozenset(('finished', 'saved_by_user'))

from host.arcore_floor import write_cloud
from host.arcore_probe_import import ros_pose

MAX_PIXELS = 640 * 480


def integer(value, name, low=1):
    if type(value) is not int or value < low:
        raise ValueError(f'Invalid {name}')
    return value


def plane(value, dtype, shape):
    count = int(np.prod(shape))
    size = count * np.dtype(dtype).itemsize
    if not isinstance(value, str) or len(value) != 4 * ((size + 2) // 3):
        raise ValueError('Invalid encoded depth plane length')
    try:
        data = base64.b64decode(value, validate=True)
    except (ValueError, binascii.Error) as error:
        raise ValueError('Invalid encoded depth plane') from error
    if len(data) != size:
        raise ValueError('Depth plane dimensions do not match payload')
    return np.frombuffer(data, dtype=dtype).reshape(shape)


def decode_packet(packet):
    if not isinstance(packet, dict) or packet.get('schemaVersion') != 2 or packet.get('type') not in ('pose', 'depth', 'end'):
        raise ValueError('Expected ARCore detail protocol v2')
    sequence = integer(packet.get('sequence'), 'sequence')
    if packet['type'] == 'end':
        if not isinstance(packet.get('reason'), str):
            raise ValueError('Missing capture end reason')
        return dict(kind='end', sequence=sequence, reason=packet['reason'])
    timestamp = integer(packet.get('timestampNs'), 'timestamp')
    state = packet.get('trackingState')
    if state not in ('TRACKING', 'PAUSED', 'STOPPED'):
        raise ValueError('Invalid tracking state')
    translation = np.asarray(packet.get('translationM'), dtype=np.float64)
    quaternion = np.asarray(packet.get('quaternion'), dtype=np.float64)
    if translation.shape != (3,) or quaternion.shape != (4,) or not np.isfinite(translation).all() or not np.isfinite(quaternion).all():
        raise ValueError('Invalid pose')
    length = np.linalg.norm(quaternion)
    if not .99 <= length <= 1.01:
        raise ValueError('Invalid rotation')
    quaternion /= length
    position, rotation = ros_pose(translation, quaternion)
    result = dict(kind=packet['type'], sequence=sequence, timestamp=timestamp,
                  state=state, translation=translation, quaternion=quaternion,
                  position=position, rotation=rotation)
    if packet['type'] == 'pose':
        return result
    if state != 'TRACKING':
        raise ValueError('Depth requires tracking')
    width, height = integer(packet.get('width'), 'width'), integer(packet.get('height'), 'height')
    if width * height > MAX_PIXELS:
        raise ValueError('Depth image too large')
    intrinsics = np.asarray(packet.get('intrinsics'), dtype=np.float64)
    if intrinsics.shape != (4,) or not np.isfinite(intrinsics).all() or np.any(intrinsics[:2] <= 0):
        raise ValueError('Invalid scaled texture intrinsics')
    shape = height, width
    raw = plane(packet.get('rawDepthU16LE'), '<u2', shape)
    confidence = plane(packet.get('confidenceU8'), 'u1', shape)
    smooth = None
    if packet.get('smoothDepthU16LE') is not None:
        smooth = plane(packet['smoothDepthU16LE'], '<u2', shape)
        integer(packet.get('smoothTimestampNs'), 'smooth timestamp')
    depth_timestamp = integer(packet.get('depthTimestampNs'), 'depth timestamp')
    integer(packet.get('confidenceTimestampNs'), 'confidence timestamp')
    result.update(raw=raw, confidence=confidence, smooth=smooth, intrinsics=intrinsics,
                  depth_timestamp=depth_timestamp, confidence_timestamp=packet['confidenceTimestampNs'],
                  smooth_timestamp=packet.get('smoothTimestampNs'))
    return result


def unproject(depth, mask, intrinsics, translation, quaternion):
    """Native depth pixels -> ARCore camera (-Z forward) -> ROS world (+Z up)."""
    y, x = np.nonzero(mask)
    z = depth[mask].astype(np.float64) / 1000.
    fx, fy, cx, cy = intrinsics
    camera = np.column_stack([z * (x - cx) / fx, z * (cy - y) / fy, -z])
    qx, qy, qz, qw = quaternion
    qv = np.array([qx, qy, qz])
    cross = 2 * np.cross(qv, camera)
    world = camera + qw * cross + np.cross(qv, cross) + translation
    world = world[:, [0, 2, 1]]
    world[:, 1] *= -1
    return world.astype(np.float32)


class ObservedVoxels:
    """Keep measured points; count a cell at most once per independent depth frame."""
    def __init__(self, resolution=.02, min_observations=2, limit=500_000):
        if not .005 <= resolution <= .1 or min_observations < 1:
            raise ValueError('Invalid map settings')
        self.resolution, self.min_observations, self.limit = resolution, min_observations, limit
        self.cells = {}
        self.capacity_rejections = 0

    def add(self, points, frame_id):
        for point in points:
            key = tuple(np.floor(point / self.resolution).astype(np.int32))
            previous = self.cells.get(key)
            if previous is None:
                if len(self.cells) >= self.limit:
                    self.capacity_rejections += 1
                    continue
                self.cells[key] = (point.copy(), 1, frame_id)
            else:
                # No spatial average or hole filling: retain an observed sample.
                self.cells[key] = (point.copy(), previous[1] + int(previous[2] != frame_id), frame_id)

    def array(self, confirmed=True):
        minimum = self.min_observations if confirmed else 1
        return np.asarray([p for p, count, _ in self.cells.values() if count >= minimum],
                          dtype=np.float32).reshape(-1, 3)


class ConfirmedMeanVoxels:
    """Online mean of one measured point per independent frame and voxel."""
    def __init__(self, resolution=.01, min_observations=3, limit=2_000_000):
        if not .005 <= resolution <= .1 or min_observations < 1 or limit < 1:
            raise ValueError('Invalid confirmed map settings')
        self.resolution, self.min_observations, self.limit = resolution, min_observations, limit
        self.cells = {}
        self.capacity_rejections = 0

    def add(self, points, frame_id):
        if not len(points):
            return
        keys = np.floor(points / self.resolution).astype(np.int32)
        unique, inverse = np.unique(keys, axis=0, return_inverse=True)
        sums = np.zeros((len(unique), 3), dtype=np.float64)
        np.add.at(sums, inverse, points)
        counts = np.bincount(inverse)
        means = sums / counts[:, None]
        for key, point in zip(unique, means):
            address = tuple(int(value) for value in key)
            previous = self.cells.get(address)
            if previous is None:
                if len(self.cells) >= self.limit:
                    self.capacity_rejections += 1
                    continue
                self.cells[address] = [point, 1, frame_id]
            elif previous[2] != frame_id:
                previous[0] += point
                previous[1] += 1
                previous[2] = frame_id

    def array(self, confirmed=True):
        minimum = self.min_observations if confirmed else 1
        return np.asarray([total / count for total, count, _ in self.cells.values()
                           if count >= minimum], dtype=np.float32).reshape(-1, 3)


def paired_metrics(raw, smooth, trusted):
    if smooth is None:
        return dict(pairedPixels=0, pairedAdjacentPixels=0)
    common = trusted & (smooth >= 500) & (smooth <= 5000)
    delta = np.abs(raw.astype(np.int32) - smooth.astype(np.int32))[common]
    raw_jumps, smooth_jumps = [], []
    for axis in (0, 1):
        pair = (common[:-1, :] & common[1:, :]) if axis == 0 else (common[:, :-1] & common[:, 1:])
        raw_jumps.extend(np.abs(np.diff(raw.astype(np.int32), axis=axis))[pair].tolist())
        smooth_jumps.extend(np.abs(np.diff(smooth.astype(np.int32), axis=axis))[pair].tolist())
    return dict(pairedPixels=int(common.sum()),
                rawSmoothDifferenceP95Mm=float(np.percentile(delta, 95)) if len(delta) else None,
                pairedAdjacentPixels=len(raw_jumps),
                rawAdjacentJumps100mm=int(np.count_nonzero(np.asarray(raw_jumps) >= 100)),
                smoothAdjacentJumps100mm=int(np.count_nonzero(np.asarray(smooth_jumps) >= 100)))


class DetailSession:
    def __init__(self, resolution=.02, min_observations=2, raw_fusion='last'):
        if raw_fusion not in ('last', 'confirmed_mean'):
            raise ValueError('Invalid raw fusion mode')
        self.raw_fusion = raw_fusion
        self.raw = (ConfirmedMeanVoxels(resolution, min_observations)
                    if raw_fusion == 'confirmed_mean'
                    else ObservedVoxels(resolution, min_observations))
        # Keep diagnostic references at their existing 2 cm settings when
        # displaying an experimental 1 cm raw map beside them.
        reference_resolution = .02 if raw_fusion == 'confirmed_mean' else resolution
        reference_minimum = 2 if raw_fusion == 'confirmed_mean' else min_observations
        self.paired_raw = ObservedVoxels(reference_resolution, reference_minimum)
        self.smooth = ObservedVoxels(reference_resolution, reference_minimum)
        self.trajectory, self.frame_metrics = [], []
        self.last_sequence = self.last_pose_timestamp = self.last_depth_timestamp = 0
        self.end_reason = None
        self.counts = dict(packets=0, trackedPackets=0, pausedPackets=0, rejectedPackets=0,
                           sequenceGaps=0, duplicateDepthFrames=0, depthFrames=0, pairedDepthFrames=0,
                           rawPixels=0, rawNonZeroPixels=0, rawRangePixels=0, rawAcceptedPixels=0,
                           smoothAcceptedPixels=0)

    def add(self, packet):
        decoded = decode_packet(packet)
        seq = decoded['sequence']
        if seq <= self.last_sequence:
            raise ValueError('Nonincreasing packet sequence')
        if self.end_reason is not None:
            raise ValueError('Packet after capture end')
        if decoded['kind'] != 'end' and decoded['timestamp'] < self.last_pose_timestamp:
            raise ValueError('Decreasing camera timestamp')
        self.counts['sequenceGaps'] += seq - self.last_sequence - 1
        self.last_sequence = seq
        self.counts['packets'] += 1
        if decoded['kind'] == 'end':
            self.end_reason = decoded['reason']
            return False
        if decoded['state'] != 'TRACKING':
            self.counts['pausedPackets'] += 1
            return False
        self.counts['trackedPackets'] += 1
        timestamp = decoded['timestamp']
        if timestamp > self.last_pose_timestamp:
            self.trajectory.append((timestamp, decoded['position'], decoded['rotation']))
            self.last_pose_timestamp = timestamp
        if decoded['kind'] != 'depth':
            return False
        depth_timestamp = decoded['depth_timestamp']
        if depth_timestamp <= self.last_depth_timestamp:
            self.counts['duplicateDepthFrames'] += 1
            return False
        self.last_depth_timestamp = depth_timestamp
        raw, smooth, confidence = decoded['raw'], decoded['smooth'], decoded['confidence']
        in_range = (raw >= 500) & (raw <= 5000)
        trusted = in_range & (confidence >= 128)
        args = decoded['intrinsics'], decoded['translation'], decoded['quaternion']
        raw_points = unproject(raw, trusted, *args)
        self.raw.add(raw_points, depth_timestamp)
        self.counts['depthFrames'] += 1
        self.counts['rawPixels'] += raw.size
        self.counts['rawNonZeroPixels'] += int((raw > 0).sum())
        self.counts['rawRangePixels'] += int(in_range.sum())
        self.counts['rawAcceptedPixels'] += int(trusted.sum())
        smooth_count = 0
        if smooth is not None:
            self.paired_raw.add(raw_points, depth_timestamp)
            smooth_valid = (smooth >= 500) & (smooth <= 5000)
            smooth_count = int(smooth_valid.sum())
            self.smooth.add(unproject(smooth, smooth_valid, *args), depth_timestamp)
            self.counts['pairedDepthFrames'] += 1
            self.counts['smoothAcceptedPixels'] += smooth_count
        self.frame_metrics.append(dict(timestampNs=timestamp, depthTimestampNs=depth_timestamp,
            confidenceTimestampNs=decoded['confidence_timestamp'], smoothTimestampNs=decoded['smooth_timestamp'],
            width=raw.shape[1], height=raw.shape[0], acceptedRawPixels=int(trusted.sum()),
            acceptedSmoothPixels=smooth_count, **paired_metrics(raw, smooth, trusted)))
        return True

    def save(self, output):
        output = Path(output)
        output.mkdir(parents=True, exist_ok=True)
        raw, observed, smooth = self.raw.array(), self.raw.array(False), self.smooth.array()
        summary = dict(status='diagnostic', source='arcore_raw_depth_confidence', scaleSource='arcore_depth_api',
            metricAccuracyValidated=False, captureEndReason=self.end_reason,
            captureComplete=self.end_reason in COMPLETE_CAPTURE_REASONS, points=len(raw), observedPoints=len(observed),
            mapTruncated=self.raw.capacity_rejections > 0,
            smoothReferencePoints=len(smooth), pairedRawReferencePoints=len(self.paired_raw.array()),
            poses=len(self.trajectory), **self.counts,
            settings=dict(depthMm=[500, 5000], confidenceMinimum=128,
                          voxelM=self.raw.resolution, minIndependentDepthFrames=self.raw.min_observations,
                          fusion=('mean of one measured point per frame per cell; no hole filling'
                                  if self.raw_fusion == 'confirmed_mean'
                                  else 'last measured sample per cell; no hole filling'),
                          referenceVoxelM=self.smooth.resolution,
                          referenceMinIndependentDepthFrames=self.smooth.min_observations),
            rawCapacityRejections=self.raw.capacity_rejections,
            pairedRawCapacityRejections=self.paired_raw.capacity_rejections,
            smoothCapacityRejections=self.smooth.capacity_rejections,
            note='Paired same-view depth diagnostic; 3D export omits RGB (private stream may contain JPEG); no physical accuracy or relocalization claim')
        for name, array in [('map_cloud.ply', raw), ('raw_observed.ply', observed),
                            ('paired_raw_reference.ply', self.paired_raw.array()), ('smooth_reference.ply', smooth)]:
            if len(array):
                write_cloud(output / name, array)
        with (output / 'map_poses.txt').open('w') as stream:
            for index, (stamp, position, rotation) in enumerate(self.trajectory):
                stream.write(' '.join(map(str, (index, *position, *rotation, stamp / 1e9))) + '\n')
        (output / 'result.json').write_text(json.dumps(summary, indent=2) + '\n')
        (output / 'depth_comparison.json').write_text(json.dumps(self.frame_metrics, indent=2) + '\n')
        return summary


def replay(source, output, resolution=.02, min_observations=2):
    output = Path(output)
    output.mkdir(parents=True, exist_ok=False)
    session = DetailSession(resolution, min_observations)
    with Path(source).open() as stream:
        for line in stream:
            if line.strip():
                session.add(json.loads(line))
    if session.end_reason is None:
        session.end_reason = 'eof_without_end_marker'
    summary = session.save(output)
    shutil.copyfile(source, output / 'stream.jsonl')
    digest = hashlib.sha256()
    with Path(source).open('rb') as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b''):
            digest.update(chunk)
    (output / 'input-sha256.txt').write_text(digest.hexdigest() + '\n')
    return summary


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--replay', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--voxel-m', type=float, default=.02)
    parser.add_argument('--min-observations', type=int, default=2)
    args = parser.parse_args()
    print(json.dumps(replay(args.replay, args.output, args.voxel_m, args.min_observations), indent=2))


if __name__ == '__main__':
    main()

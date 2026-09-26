"""Offline depth/optical-track agreement with raw stereo poses, never ground truth.

Fits ONE scalar on alternating 30-frame blocks; evaluates only other blocks.
No camera, calibration edits, per-frame fit, or automatic production selection.
"""
import argparse
import bisect
from decimal import Decimal
import json
from pathlib import Path
import sys
import cv2
import numpy as np
from host.monocular_depth import file_sha
from host.motion_recording import atomic_json
from host.ros_rgbd import validate_frame


def pose_matrix(values):
    p = np.asarray(values, dtype=float)
    if p.shape != (7,) or not np.isfinite(p).all():
        raise ValueError('Finite translation and quaternion required')
    x, y, z, w = p[3:]
    if abs(np.linalg.norm(p[3:]) - 1) > 1e-3:
        raise ValueError('Unit quaternion required')
    x, y, z, w = p[3:] / np.linalg.norm(p[3:])
    t = np.eye(4)
    t[:3, :3] = [[1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
                 [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
                 [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]]
    t[:3, 3] = p[:3]
    return t


BODY_FROM_OPTICAL = pose_matrix([0, 0, 0, -.5, .5, -.5, .5])


def optical_relative(world_from_body_a, world_from_body_b):
    """Target optical from source optical; ROS poses are world-from-body."""
    return np.linalg.inv(world_from_body_b @ BODY_FROM_OPTICAL) @ (world_from_body_a @ BODY_FROM_OPTICAL)


def pose_at(poses, stamp, max_gap_ns=250_000_000):
    """Interpolate inside tracked pose support only; no nearest/extrapolated pose."""
    if stamp in poses:
        return poses[stamp], dict(stampNs=stamp, bracketsNs=[stamp, stamp], weight=0.)
    stamps = sorted(poses)
    i = bisect.bisect_left(stamps, stamp)
    if i == 0 or i == len(stamps):
        return None, None
    low, high = stamps[i-1:i+1]
    if high-low > max_gap_ns:
        return None, None
    weight = (stamp-low)/(high-low)
    a, b = poses[low], poses[high]
    result = np.eye(4)
    delta = cv2.Rodrigues(a[:3, :3].T @ b[:3, :3])[0]
    result[:3, :3] = a[:3, :3] @ cv2.Rodrigues(delta*weight)[0]
    result[:3, 3] = a[:3, 3]*(1-weight)+b[:3, 3]*weight
    return result, dict(stampNs=stamp, bracketsNs=[low, high], weight=weight)


def reproject(points, depth, k, target_from_source):
    rays = np.column_stack((points, np.ones(len(points)))) @ np.linalg.inv(k).T
    xyz = (rays * depth[:, None]) @ target_from_source[:3, :3].T + target_from_source[:3, 3]
    projected = xyz @ k.T
    uv = np.full((len(points), 2), np.nan)
    valid = np.isfinite(xyz).all(axis=1) & (xyz[:, 2] > 0)
    uv[valid] = projected[valid, :2] / projected[valid, 2:]
    return uv, xyz[:, 2], valid


def sample(depth, points):
    """Nearest pixel; invalid/outside samples remain NaN, never clipped to edge."""
    result = np.full(len(points), np.nan)
    finite = np.isfinite(points).all(axis=1)
    xy = np.zeros((len(points), 2), dtype=int)
    xy[finite] = np.rint(points[finite]).astype(int)
    h, w = depth.shape
    valid = finite & (xy[:, 0] >= 0) & (xy[:, 0] < w) & (xy[:, 1] >= 0) & (xy[:, 1] < h)
    result[valid] = depth[xy[valid, 1], xy[valid, 0]]
    result[result <= 0] = np.nan
    return result


def tracks(first, second):
    points = cv2.goodFeaturesToTrack(first, 800, .01, 8)
    empty = np.empty((0, 2), dtype=np.float32)
    if points is None:
        return empty, empty, 0
    nxt, ok, _ = cv2.calcOpticalFlowPyrLK(first, second, points, None, winSize=(21, 21), maxLevel=3)
    if nxt is None:
        return empty, empty, len(points)
    back, ok_back, _ = cv2.calcOpticalFlowPyrLK(second, first, nxt, None, winSize=(21, 21), maxLevel=3)
    if back is None:
        return empty, empty, len(points)
    a, b = points[:, 0], nxt[:, 0]
    h, w = first.shape
    valid = ok[:, 0].astype(bool) & ok_back[:, 0].astype(bool)
    valid &= np.isfinite(b).all(axis=1) & (np.linalg.norm(back[:, 0]-a, axis=1) <= 1.)
    valid &= (b[:, 0] >= 0) & (b[:, 0] < w-1) & (b[:, 1] >= 0) & (b[:, 1] < h-1)
    return a[valid], b[valid], len(points)


def distribution(values):
    values = np.asarray(values)
    values = values[np.isfinite(values)]
    return dict(count=len(values), median=float(np.median(values)) if len(values) else None,
                p95=float(np.percentile(values, 95)) if len(values) else None)


def fit_scale(frame_ratios):
    """Equal weight per training frame; scalar maps prediction to stereo estimate."""
    medians = [float(np.median(x)) for x in frame_ratios if len(x)]
    if not medians or not np.isfinite(medians).all() or min(medians) <= 0:
        raise ValueError('Positive finite training ratios required')
    return float(np.median(medians))


def read_cache(path):
    manifest = json.loads((path/'manifest.json').read_text())
    records = manifest.get('records', [])
    if manifest.get('status') != 'complete' or not records or len(records) != manifest.get('frames'):
        raise ValueError('Complete cache required')
    paths = [path/'manifest.json']
    previous = -1
    for r in records:
        if Path(r['file']).name != r['file'] or r['stampNs'] <= previous:
            raise ValueError('Invalid cache record')
        previous = r['stampNs']
        frame_path = path/r['file']
        if file_sha(frame_path) != r['sha256']:
            raise ValueError('Cache hash differs')
        paths.append(frame_path)
    return manifest, paths


def load_frame(cache, manifest, index):
    with np.load(cache/manifest['records'][index]['file']) as frame:
        image, depth = frame['image'], frame['depth']
    validate_frame(image, depth, manifest['imageSize'])
    return image, depth


def read_poses(session):
    provenance = json.loads((session/'replay-input.json').read_text())['provenance']
    capture = json.loads((session/'capture.json').read_text())
    lifecycle = json.loads((session/'lifecycle.json').read_text())
    if lifecycle.get('odometryFrames') != [['odom', 'phone_link']]:
        raise ValueError('Expected odom-from-phone_link pose frame evidence')
    if capture.get('odometryScaleSource', capture.get('scaleSource')) != 'nominal_display_estimate':
        raise ValueError('This comparison expects nominal-display stereo odometry')
    status = {}
    for line in (session/'odometry-status.jsonl').read_text().splitlines():
        r = json.loads(line, parse_float=Decimal)
        # JSON stores seconds as a float; round back to its nearest sensor ns.
        stamp = int(round(r['stamp'] * 10**9))
        if stamp in status:
            raise ValueError('Duplicate odometry status stamp')
        status[stamp] = not r['lost']
    if not all(status.values()):
        raise ValueError('Audit requires an uninterrupted non-lost odometry segment')
    poses = {}
    previous = -1
    for line in (session/'odometry-raw.txt').read_text().splitlines():
        if not line or line.startswith('#'):
            continue
        fields = line.split()
        stamp = int(Decimal(fields[0])*10**9)
        if stamp <= previous or not status.get(stamp, False):
            raise ValueError('Raw pose lacks exact non-lost status or monotonic time')
        previous = stamp
        poses[stamp] = pose_matrix([float(x) for x in fields[1:]])
    return poses, provenance


def run(stereo, model, session, output):
    sm, sp = read_cache(stereo)
    mm, mp = read_cache(model)
    if sm.get('scaleSource') != 'nominal_display_estimate' or mm.get('scaleSource') != 'model_predicted_metres':
        raise ValueError('Explicit stereo estimate and unscaled model cache required')
    for key in ('frames', 'projection', 'imageSize', 'sourceOriginNs'):
        if sm[key] != mm[key]:
            raise ValueError('Cache geometry or source mismatch: '+key)
    if sm['sourceEvidence']['calibrationSha256'] != mm['sourceEvidence']['calibrationSha256']:
        raise ValueError('Calibration differs')
    poses, provenance = read_poses(session)
    if provenance['calibrationSha256'] != sm['sourceEvidence']['calibrationSha256']:
        raise ValueError('Pose calibration differs')
    # Confirm identical images for ALL frames before using a shared track set.
    for i in range(sm['frames']):
        if sm['records'][i]['stampNs'] != mm['records'][i]['stampNs']:
            raise ValueError('Source timestamps differ')
        if not np.array_equal(load_frame(stereo, sm, i)[0], load_frame(model, mm, i)[0]):
            raise ValueError('Source pixels differ')
    paths = sp+mp+[session/name for name in ('replay-input.json', 'capture.json', 'lifecycle.json', 'odometry-raw.txt', 'odometry-status.jsonl')]
    paths += [Path(__file__), Path(__file__).with_name('ros_rgbd.py'), Path(__file__).with_name('monocular_depth.py')]
    before = {str(p.resolve()): file_sha(p) for p in paths}
    output.mkdir(parents=True, exist_ok=False)
    atomic_json(output/'input-before.json', before)
    config = dict(blockFrames=30, trainingBlocks='even', evaluationBlocks='odd', trainingStride=10,
                  evaluationAnchorStride=10, anchorOffset=5, targetOffsets=[-4, 4],
                  maxCorners=800, qualityLevel=.01, minDistance=8, forwardBackwardMaxPx=1.,
                  sampling='nearest pixel', noReprojectionOutlierRejection=True,
                  posePolicy='exact or bracketed linear translation / SO(3) rotation; no extrapolation',
                  maxPoseBracketGapNs=250_000_000,
                  scaleFit='median of per-frame medians of stereo/model; one global scalar',
                  command=[sys.executable, *sys.argv], opencv=cv2.__version__, numpy=np.__version__)
    atomic_json(output/'protocol.json', config)
    ratios, training = [], []
    n = sm['frames']
    for i in range(0, n, 10):
        if (i//30) % 2:
            continue
        sd = load_frame(stereo, sm, i)[1]
        md = load_frame(model, mm, i)[1]
        valid = np.isfinite(sd) & np.isfinite(md) & (sd > 0) & (md > 0)
        ratio = sd[valid]/md[valid]
        ratios.append(ratio)
        training.append(dict(frame=i, ratio=distribution(ratio)))
    scale = fit_scale(ratios)
    atomic_json(output/'scale-fit.json', dict(factor=scale, training=training, scaleSource='nominal_display_estimate', measured=False))
    rows, arrays = [], []
    names = ['sgbm', 'model', 'model_global_scale']
    k = np.asarray(sm['projection'])[:, :3]
    def pose_for(index):
        stamp = sm['records'][index]['stampNs']+sm['sourceOriginNs']-10**9-provenance['sourceOriginNs']+provenance['rosOriginNs']
        return pose_at(poses, stamp, config['maxPoseBracketGapNs'])
    for i in range(5, n, 10):
        if (i//30) % 2 != 1:
            continue
        for delta in config['targetOffsets']:
            j = i+delta
            if j < 0 or j >= n or j//30 != i//30:
                continue
            (pa, pa_meta), (pb, pb_meta) = pose_for(i), pose_for(j)
            if pa is None or pb is None:
                rows.append(dict(source=i, target=j, skipped='no bounded tracked pose support'))
                continue
            ia, sa = load_frame(stereo, sm, i)
            ib, sb = load_frame(stereo, sm, j)
            ma = load_frame(model, mm, i)[1]
            mb = load_frame(model, mm, j)[1]
            a, b, detected = tracks(ia, ib)
            transform = optical_relative(pa, pb)
            rot = float(np.degrees(np.arccos(np.clip((np.trace(transform[:3, :3])-1)/2, -1, 1))))
            entry = dict(source=i, target=j, sourcePose=pa_meta, targetPose=pb_meta, detected=detected, tracks=len(a),
                         translationEstimatedM=float(np.linalg.norm(transform[:3, 3])), rotationDegrees=rot, methods={})
            values = []
            for da, db in ((sa, sb), (ma, mb), (ma*scale, mb*scale)):
                za, zb = sample(da, a), sample(db, b)
                uv, z, valid = reproject(a, za, k, transform)
                error = np.linalg.norm(uv-b, axis=1)
                error[~valid] = np.nan
                relative = np.abs(z-zb)/zb*100
                relative[~valid | ~np.isfinite(zb)] = np.nan
                values.append((error, relative))
            reproj = np.asarray([x[0] for x in values])
            depth_error = np.asarray([x[1] for x in values])
            common = np.isfinite(reproj).all(axis=0)
            common_depth = np.isfinite(depth_error).all(axis=0)
            for m, name in enumerate(names):
                entry['methods'][name] = dict(allReprojectionPx=distribution(reproj[m]),
                    commonReprojectionPx=distribution(reproj[m, common]),
                    commonDepthDisagreementPercent=distribution(depth_error[m, common_depth]))
            file = f'tracks-{i:06d}-{j:06d}.npz'
            np.savez_compressed(output/file, sourcePoints=a, targetPoints=b, targetFromSource=transform,
                                reprojectionPx=reproj, depthDisagreementPercent=depth_error,
                                common=common, commonDepth=common_depth)
            entry['arrayFile'] = file
            rows.append(entry)
            arrays.append((reproj, depth_error, common, common_depth, len(a)))
    with (output/'pairs.jsonl').open('x') as f:
        for row in rows:
            f.write(json.dumps(row, allow_nan=False)+'\n')
    summary = dict(scaleFactor=scale, trainingFrames=[r['frame'] for r in training],
                   evaluatedPairs=len(arrays), skippedPairs=len(rows)-len(arrays),
                   opticalTracks=sum(x[4] for x in arrays), methods={},
                   metricAccuracyValidated=False, automaticProductionSelection=False,
                   limitations=['Same short scene; temporal blocks are not independent scene validation',
                                'Poses use stereo, so the reference is not independent of SGBM',
                                'Model metres and estimated odometry may have inconsistent scale',
                                'Optical-flow errors, occlusion, dynamic objects and pose errors contribute',
                                'Raw pose reprojection is consistency, not physical geometry accuracy'])
    if not arrays:
        raise ValueError('No evaluation pairs with tracked pose support')
    for m, name in enumerate(names):
        summary['methods'][name] = dict(
            allReprojectionPx=distribution(np.concatenate([v[0][m] for v in arrays])),
            commonReprojectionPx=distribution(np.concatenate([v[0][m, v[2]] for v in arrays])),
            commonDepthDisagreementPercent=distribution(np.concatenate([v[1][m, v[3]] for v in arrays])))
    after = {p: file_sha(p) for p in before}
    atomic_json(output/'input-after.json', after)
    if before != after:
        raise RuntimeError('An input changed during evaluation')
    summary['inputFiles'] = len(before)
    summary['inputUnchanged'] = True
    atomic_json(output/'summary.json', summary)
    print(json.dumps(summary, indent=2, allow_nan=False), flush=True)
    return summary


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    for name in ('stereo', 'model', 'session', 'output'):
        parser.add_argument('--'+name, type=Path, required=True)
    args = parser.parse_args()
    cv2.setNumThreads(2)
    cv2.setRNGSeed(0)
    run(args.stereo, args.model, args.session, args.output)

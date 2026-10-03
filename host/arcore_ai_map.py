"""Optional recorded ARCore pose + upright pretrained depth map experiment.

No phone is contacted. Model predictions are scaled to unvalidated ARCore depth;
they are not measured surfaces. Source RGB, depth caches and maps remain private.
"""
import argparse
import base64
from io import BytesIO
import json
from pathlib import Path
import time

import cv2
import numpy as np

from host.arcore_confirmed_map import file_sha256
from host.arcore_detail import COMPLETE_CAPTURE_REASONS, ConfirmedMeanVoxels, decode_packet, unproject
from host.arcore_floor import write_cloud


def upright_rotation(quaternion):
    """Nearest clockwise quarter turn putting ARCore world up above the image."""
    x, y, z, w = quaternion
    # R.T @ world-up is the second row of the camera-to-world rotation.
    gravity = np.array([2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)])
    image_up = np.array([gravity[0], -gravity[1]])
    if np.linalg.norm(image_up) < .1:
        raise ValueError('Camera points too close to gravity direction to infer upright rotation')
    # Clockwise image rotation: (x, y) -> (-y, x).
    directions = [image_up]
    for _ in range(3):
        a, b = directions[-1]
        directions.append(np.array([-b, a]))
    return int(np.argmin([v[1] for v in directions])) * 90


def decode_rgb(packet):
    from PIL import Image
    if packet.get('rgbEncoding') != 'jpeg':
        raise ValueError('Recorded matching JPEG required')
    width, height = packet.get('rgbWidth'), packet.get('rgbHeight')
    if (type(width) is not int or type(height) is not int or min(width, height) < 1
            or max(width, height) > 4096 or width*height > 8_388_608):
        raise ValueError('Invalid or oversized recorded RGB dimensions')
    stamp = packet.get('rgbTimestampNs')
    if type(stamp) is not int or abs(stamp-packet['timestampNs']) > 10_000_000:
        raise ValueError('RGB and camera timestamps differ by more than 10 ms')
    encoded = packet.get('rgbJpegBase64')
    if not isinstance(encoded, str) or len(encoded) > 8_000_000:
        raise ValueError('Invalid JPEG payload')
    payload = base64.b64decode(encoded, validate=True)
    # Inspect dimensions without expanding pixels before calling the OpenCV decoder.
    try:
        with Image.open(BytesIO(payload)) as header:
            if header.format != 'JPEG' or header.size != (width, height):
                raise ValueError('JPEG header differs from recorded RGB geometry')
    except (OSError, Image.DecompressionBombError) as error:
        raise ValueError('Invalid JPEG header') from error
    rgb = cv2.imdecode(np.frombuffer(payload, np.uint8), cv2.IMREAD_COLOR)
    if rgb is None or rgb.shape[:2] != (packet.get('rgbHeight'), packet.get('rgbWidth')):
        raise ValueError('JPEG dimensions differ from recorded RGB geometry')
    return rgb


def texture_samples(array, corners, size):
    """Sample native RGB/prediction at texture pixel centers, including crop/rotation."""
    width, height = size
    p = np.asarray(corners, dtype=np.float64).reshape(4, 2)
    if width < 1 or height < 1 or not np.isfinite(p).all():
        raise ValueError('Invalid texture/RGB mapping')
    u, v = np.meshgrid((np.arange(width)+.5)/width, (np.arange(height)+.5)/height)
    xy = ((1-u)[..., None]*(1-v)[..., None]*p[0] + u[..., None]*(1-v)[..., None]*p[1]
          + (1-u)[..., None]*v[..., None]*p[2] + u[..., None]*v[..., None]*p[3]) - .5
    # Mapping must stay within pixel-edge bounds; do not invent pixels outside RGB.
    if (xy[..., 0].min() < -.5 or xy[..., 1].min() < -.5
            or xy[..., 0].max() > array.shape[1]-.5 or xy[..., 1].max() > array.shape[0]-.5):
        raise ValueError('Texture samples outside recorded RGB')
    return cv2.remap(array, xy[..., 0].astype(np.float32), xy[..., 1].astype(np.float32),
                     cv2.INTER_LINEAR, borderMode=cv2.BORDER_REPLICATE)


def edge_safe_mask(depth, valid, absolute_m=.05, relative=.03):
    """Reject both sides of depth jumps; never blur or extend a surface."""
    safe = np.asarray(valid, dtype=bool).copy()
    for axis in (0, 1):
        a = (slice(None, -1), slice(None)) if axis == 0 else (slice(None), slice(None, -1))
        b = (slice(1, None), slice(None)) if axis == 0 else (slice(None), slice(1, None))
        jump = (valid[a] & valid[b]
                & (np.abs(depth[a]-depth[b]) > np.maximum(absolute_m, relative*np.minimum(depth[a], depth[b]))))
        safe[a] &= ~jump
        safe[b] &= ~jump
    return safe


def reference_scale(predicted, raw_m, confidence, minimum_pixels=500):
    """Robust scalar anchoring only; ARCore reference is explicitly unvalidated."""
    valid = (np.isfinite(predicted) & (predicted > 0) & (raw_m >= .5)
             & (raw_m <= 5) & (confidence >= 192))
    valid &= edge_safe_mask(raw_m, valid)
    valid &= edge_safe_mask(predicted, valid)
    if int(valid.sum()) < minimum_pixels:
        raise ValueError('Not enough stable high-confidence ARCore scale pixels')
    ratios = raw_m[valid]/predicted[valid]
    scale = float(np.median(ratios))
    if not np.isfinite(scale) or not .1 <= scale <= 10:
        raise ValueError('Invalid model/reference scale')
    return scale, dict(scaleReferencePixels=int(valid.sum()), scale=scale,
                       scaleRatioP10=float(np.percentile(ratios, 10)),
                       scaleRatioP90=float(np.percentile(ratios, 90)))


def rgb_focal_pixels(packet, decoded, rotation):
    """Recorded pinhole focal length for the upright CPU RGB; reject shear."""
    p = np.asarray(packet['textureToRgbCornersPx'], dtype=np.float64).reshape(4, 2)
    x, y = p[1]-p[0], p[2]-p[0]
    if (not np.isfinite(p).all() or min(np.linalg.norm(x), np.linalg.norm(y)) <= 0
            or not np.allclose(p[3], p[1]+p[2]-p[0], atol=.01, rtol=0)
            or abs(x@y) > .01*np.linalg.norm(x)*np.linalg.norm(y)):
        raise ValueError('Recorded mapping is not a rectangular pinhole crop')
    # Gravity/rotation is expressed in native camera axes; CPU RGB must match.
    if abs(x[1]) > .01 or abs(y[0]) > .01 or x[0] <= 0 or y[1] <= 0:
        raise ValueError('Rotated/reflected CPU RGB requires explicit camera axes')
    height, width = decoded['raw'].shape
    fx = decoded['intrinsics'][0]*np.linalg.norm(x)/width
    fy = decoded['intrinsics'][1]*np.linalg.norm(y)/height
    return float(fy if rotation in (90, 270) else fx)


def predict_frame(packet, decoded, model, output_factor=1, use_recorded_focal=False):
    if output_factor not in (1, 2, 4):
        raise ValueError('Output factor must be 1, 2 or 4')
    rgb = decode_rgb(packet)
    rotation = upright_rotation(decoded['quaternion'])
    model.rotation = rotation
    focal = rgb_focal_pixels(packet, decoded, rotation) if use_recorded_focal else None
    predicted, elapsed_ms = model.predict(rgb, focal_px=focal) if use_recorded_focal else model.predict(rgb)
    width, height = decoded['raw'].shape[1], decoded['raw'].shape[0]
    corners = packet['textureToRgbCornersPx']
    native = texture_samples(predicted, corners, (width, height))
    scale, metrics = reference_scale(native, decoded['raw'].astype(np.float32)/1000,
                                     decoded['confidence'])
    depth = texture_samples(predicted, corners, (width*output_factor, height*output_factor))*scale
    valid = np.isfinite(depth) & (depth >= .5) & (depth <= 5)
    safe = edge_safe_mask(depth, valid)
    metrics.update(inputRotationClockwise=rotation, inferenceMs=elapsed_ms,
                   predictedRangePixels=int(valid.sum()), acceptedPixels=int(safe.sum()),
                   edgeRejectedPixels=int((valid & ~safe).sum()),
                   outputWidth=depth.shape[1], outputHeight=depth.shape[0])
    if focal is not None:
        metrics['recordedFocalPx'] = focal
    k = decoded['intrinsics']*output_factor
    k[2:] += .5*(output_factor-1)  # Preserve pixel centers after changing image size.
    return depth, safe, k, metrics


def build(source, output, model, *, output_factor=1, save_depths=False,
          use_recorded_focal=False, exclude_frames=(), cell_limit=2_000_000):
    source, output = Path(source), Path(output)
    if output.exists():
        raise FileExistsError(output)
    excluded = set(exclude_frames)
    if any(type(i) is not int or i < 0 for i in excluded):
        raise ValueError('Excluded depth frame indices must be nonnegative integers')
    if type(cell_limit) is not int or cell_limit < 1:
        raise ValueError('Positive cell limit required')
    before = file_sha256(source)
    # Validate a complete stream before running inference or writing any scene output.
    rows, trajectory = [], []
    last_sequence = last_pose = last_depth = 0
    end_reason = None
    with source.open() as archive:
        for line in archive:
            if not line.strip():
                continue
            if end_reason is not None:
                raise ValueError('Packet after capture end')
            packet = json.loads(line)
            d = decode_packet(packet)
            if d['sequence'] <= last_sequence:
                raise ValueError('Nonincreasing packet sequence')
            last_sequence = d['sequence']
            if d['kind'] == 'end':
                end_reason = d['reason']
                continue
            if d['timestamp'] < last_pose:
                raise ValueError('Decreasing camera timestamp')
            if d['state'] == 'TRACKING' and d['timestamp'] > last_pose:
                trajectory.append((d['timestamp'], d['position'], d['rotation']))
                last_pose = d['timestamp']
            if d['kind'] == 'depth':
                if d['depth_timestamp'] <= last_depth:
                    raise ValueError('Duplicate or decreasing raw depth timestamp')
                last_depth = d['depth_timestamp']
                rows.append((packet, d))
    if end_reason not in COMPLETE_CAPTURE_REASONS or not rows:
        raise ValueError('Complete recorded capture required')
    output.mkdir(parents=True, exist_ok=False)
    if save_depths:
        (output/'predictions').mkdir()
    grid = ConfirmedMeanVoxels(.01, 3, limit=cell_limit)
    metrics = []
    start = time.monotonic()
    for index, (packet, d) in enumerate(rows):
        row = dict(depthFrame=index, timestampNs=d['timestamp'], depthTimestampNs=d['depth_timestamp'])
        if index in excluded:
            row.update(status='excluded', reason='Explicit comparison exclusion')
            metrics.append(row)
            continue
        try:
            depth, mask, k, info = predict_frame(packet, d, model, output_factor, use_recorded_focal)
        except ValueError as error:
            row.update(status='skipped', reason=str(error))
        except Exception as error:
            # Preserve partial experimental evidence and make failure explicit.
            row.update(status='failed', errorType=type(error).__name__)
            metrics.append(row)
            (output/'frame_metrics.json').write_text(json.dumps(metrics, indent=2)+'\n')
            after = file_sha256(source)
            failure = dict(status='failed', errorType=type(error).__name__, failedDepthFrame=index,
                           sourceSha256Before=before, sourceSha256After=after,
                           sourceUnchanged=before == after, metricAccuracyValidated=False,
                           note='Partial predictions retained; retry into a new output directory')
            (output/'result.json').write_text(json.dumps(failure, indent=2)+'\n')
            raise
        else:
            points = unproject(depth*1000, mask, k, d['translation'], d['quaternion'])
            grid.add(points, d['timestamp'])
            row.update(status='predicted', **info)
            if save_depths:
                np.savez_compressed(output/'predictions'/f'{index:04d}.npz', depthM=depth,
                                    valid=mask, intrinsics=k, timestampNs=d['timestamp'],
                                    depthTimestampNs=d['depth_timestamp'])
        metrics.append(row)
    cloud = grid.array()
    if len(cloud):
        write_cloud(output/'map_cloud.ply', cloud)
    with (output/'map_poses.txt').open('w') as poses:
        for i, (stamp, position, rotation) in enumerate(trajectory):
            poses.write(' '.join(map(str, (i, *position, *rotation, stamp/1e9)))+'\n')
    after = file_sha256(source)
    if after != before:
        raise RuntimeError('Recorded input changed')
    metadata = dict(model.metadata, inputRotationClockwise='per-frame ARCore gravity',
                    outputGeometry='Native CPU JPEG prediction remapped into depth texture pixel centers')
    result = dict(status='experimental', source='pretrained_depth_scaled_to_arcore',
                  metricAccuracyValidated=False, model=metadata,
                  sourceSha256Before=before, sourceSha256After=after, sourceUnchanged=True,
                  captureComplete=True, captureEndReason=end_reason, depthFrames=len(rows), poses=len(trajectory),
                  predictedFrames=sum(r['status']=='predicted' for r in metrics),
                  skippedFrames=sum(r['status']=='skipped' for r in metrics),
                  excludedFrames=sum(r['status']=='excluded' for r in metrics),
                  points=len(cloud), mapTruncated=grid.capacity_rejections > 0,
                  capacityRejections=grid.capacity_rejections, elapsedS=time.monotonic()-start,
                  settings=dict(voxelM=.01, minIndependentCameraFrames=3, outputFactor=output_factor,
                                excludedDepthFrames=sorted(excluded), cellLimit=cell_limit,
                                depthRangeM=[.5, 5], scaleConfidenceMinimum=192,
                                edgeJumpMinimumM=.05, edgeJumpRelative=.03),
                  note='Predicted shape candidate; no measured object distance, loop closure or relocalization claim')
    (output/'frame_metrics.json').write_text(json.dumps(metrics, indent=2)+'\n')
    (output/'result.json').write_text(json.dumps(result, indent=2)+'\n')
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    for name in ('input', 'output', 'repository', 'checkpoint'):
        parser.add_argument('--'+name, type=Path, required=True)
    parser.add_argument('--input-size', type=int, default=518)
    parser.add_argument('--model', choices=('metric-small', 'depth-pro'), default='metric-small')
    parser.add_argument('--output-factor', type=int, default=1)
    parser.add_argument('--save-depths', action='store_true')
    parser.add_argument('--exclude-frames', nargs='*', type=int, default=[])
    parser.add_argument('--cell-limit', type=int, default=2_000_000)
    args = parser.parse_args()
    if args.model == 'depth-pro':
        from host.depth_pro_model import DepthProModel
        model = DepthProModel(args.repository, args.checkpoint)
    else:
        from host.monocular_depth import MetricDepthAnything
        model = MetricDepthAnything(args.repository, args.checkpoint, input_size=args.input_size)
    print(json.dumps(build(args.input, args.output, model, output_factor=args.output_factor,
                           save_depths=args.save_depths, use_recorded_focal=args.model == 'depth-pro',
                           exclude_frames=args.exclude_frames, cell_limit=args.cell_limit), indent=2))


if __name__ == '__main__':
    main()

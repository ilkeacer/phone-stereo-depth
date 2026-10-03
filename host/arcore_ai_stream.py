"""Optional local AI stream: independent pose ingestion and a latest-frame slot.

The replay CLI never contacts a phone. Live transport is a separate ROS wrapper.
Predictions keep their original camera pose; no TSDF or geometry interpolation.
"""
import argparse
from collections import deque
import copy
from dataclasses import dataclass
import json
from pathlib import Path
import threading
import time

import numpy as np

from host.arcore_ai_map import predict_frame
from host.arcore_confirmed_map import file_sha256
from host.arcore_detail import COMPLETE_CAPTURE_REASONS, ConfirmedMeanVoxels, decode_packet, unproject
from host.arcore_floor import write_cloud
from host.arcore_scene_export import private_output


@dataclass(frozen=True)
class FrameJob:
    index: int
    packet: dict
    decoded: dict
    received: float


class AiStream:
    """One inference worker; poses never acquire the model or map locks.

    FIFO is a bounded recorded-only diagnostic control, never a live setting.
    Close drains the latest job only for a complete, error-free source capture.
    """
    def __init__(self, model, output, *, policy='latest', use_recorded_focal=False,
                 exclude_frames=(), save_depths=False, cell_limit=2_000_000,
                 predictor=None):
        if policy not in ('latest', 'fifo'):
            raise ValueError('Invalid queue policy')
        excluded = set(exclude_frames)
        if any(type(i) is not int or i < 0 for i in excluded):
            raise ValueError('Invalid excluded depth frame')
        self.grid = ConfirmedMeanVoxels(.01, 3, limit=cell_limit)
        self.output = private_output(output)
        self.output.mkdir(parents=True, exist_ok=False)
        self.save_depths = save_depths
        if save_depths:
            (self.output/'predictions').mkdir()
        self.model, self.policy, self.excluded = model, policy, excluded
        self.predictor = predictor or (lambda p, d: predict_frame(p, d, model, 1, use_recorded_focal))
        self.condition = threading.Condition()
        self.pending = deque()
        self.closed = self.cancelled = False
        self.error = self.worker_error = self.end_reason = None
        self.last_sequence = self.last_timestamp = self.last_depth = 0
        self.tracking_state = 'NOT_STARTED'
        self.trajectory, self.metrics = [], []
        self.input_metrics = []
        self.cloud = np.empty((0, 3), np.float32)
        self.map_version = 0
        self.cloud_timestamp = self.last_predicted_timestamp = 0
        self.inflight = False
        self.counts = dict(packets=0, poses=0, pausedPackets=0, sequenceGaps=0,
                           rejectedPackets=0, depthFrames=0, duplicateDepthFrames=0,
                           submittedFrames=0, predictedFrames=0, skippedFrames=0,
                           excludedFrames=0, replacedFrames=0, cancelledFrames=0,
                           unavailableFrames=0, maxPendingFrames=0)
        self.started = time.monotonic()
        self.worker = threading.Thread(target=self._work, name='arcore-ai-inference', daemon=True)
        self.worker.start()

    def add(self, packet):
        received = time.monotonic()
        d = decode_packet(packet)
        with self.condition:
            if self.closed or self.end_reason is not None:
                raise ValueError('Packet after capture end or close')
            if d['sequence'] <= self.last_sequence:
                raise ValueError('Nonincreasing packet sequence')
            if d['kind'] != 'end' and d['timestamp'] < self.last_timestamp:
                raise ValueError('Decreasing camera timestamp')
            self.counts['sequenceGaps'] += d['sequence']-self.last_sequence-1
            self.last_sequence = d['sequence']
            self.counts['packets'] += 1
            if d['kind'] == 'end':
                self.end_reason = d['reason']
                return
            self.last_timestamp = d['timestamp']
            self.tracking_state = d['state']
            if d['state'] != 'TRACKING':
                self.counts['pausedPackets'] += 1
            elif not self.trajectory or d['timestamp'] > self.trajectory[-1][0]:
                self.trajectory.append((d['timestamp'], d['position'], d['rotation']))
                self.counts['poses'] += 1
            if d['kind'] == 'depth':
                if d['depth_timestamp'] <= self.last_depth:
                    self.counts['duplicateDepthFrames'] += 1
                    return
                self.last_depth = d['depth_timestamp']
                index = self.counts['depthFrames']
                self.counts['depthFrames'] += 1
                row = dict(depthFrame=index, timestampNs=d['timestamp'],
                           depthTimestampNs=d['depth_timestamp'], status='pending')
                self.metrics.append(row)
                if index in self.excluded:
                    row['status'] = 'excluded'
                    self.counts['excludedFrames'] += 1
                elif self.worker_error:
                    row['status'] = 'unavailable'
                    self.counts['unavailableFrames'] += 1
                else:
                    if self.policy == 'latest' and self.pending:
                        previous = self.pending.pop()
                        self.metrics[previous.index]['status'] = 'replaced'
                        self.counts['replacedFrames'] += 1
                    if len(self.pending) >= 5000:
                        raise RuntimeError('Recorded FIFO diagnostic capacity exceeded')
                    self.pending.append(FrameJob(index, copy.deepcopy(packet), d, received))
                    self.counts['submittedFrames'] += 1
                    self.counts['maxPendingFrames'] = max(self.counts['maxPendingFrames'], len(self.pending))
                    self.condition.notify()
            self.input_metrics.append(dict(sequence=d['sequence'], timestampNs=d['timestamp'],
                                           ingestMs=(time.monotonic()-received)*1000))

    def reject(self):
        with self.condition:
            self.counts['rejectedPackets'] += 1

    def _cancel_pending(self):
        while self.pending:
            job = self.pending.popleft()
            self.metrics[job.index]['status'] = 'cancelled'
            self.counts['cancelledFrames'] += 1

    def _work(self):
        last_snapshot = 0.
        while True:
            with self.condition:
                self.condition.wait_for(lambda: self.pending or self.closed)
                if not self.pending:
                    break
                job = self.pending.popleft()
                self.inflight = True
                row = self.metrics[job.index]
                row.update(status='processing', queueWaitMs=(time.monotonic()-job.received)*1000)
            try:
                depth, valid, k, info = self.predictor(job.packet, job.decoded)
                with self.condition:
                    cancelled = self.cancelled
                if cancelled:
                    with self.condition:
                        row['status'] = 'cancelled'
                        self.counts['cancelledFrames'] += 1
                    continue
                points = unproject(depth*1000, valid, k, job.decoded['translation'], job.decoded['quaternion'])
                self.grid.add(points, job.decoded['timestamp'])
                if self.save_depths:
                    np.savez_compressed(self.output/'predictions'/f'{job.index:04d}.npz',
                        depthM=depth, valid=valid, intrinsics=k,
                        timestampNs=job.decoded['timestamp'], depthTimestampNs=job.decoded['depth_timestamp'])
                now = time.monotonic()
                cloud = self.grid.array() if now-last_snapshot >= .5 else None
                if cloud is not None:
                    last_snapshot = now
                with self.condition:
                    row.update(info, status='predicted', resultAgeMs=(time.monotonic()-job.received)*1000,
                               cameraLagAtResultMs=max(0, self.last_timestamp-job.decoded['timestamp'])/1e6)
                    self.counts['predictedFrames'] += 1
                    self.last_predicted_timestamp = job.decoded['timestamp']
                    if cloud is not None:
                        self.cloud = cloud
                        self.cloud_timestamp = job.decoded['timestamp']
                        self.map_version += 1
            except ValueError as exc:
                with self.condition:
                    row.update(status='skipped', reason=str(exc))
                    self.counts['skippedFrames'] += 1
            except Exception as exc:
                with self.condition:
                    row.update(status='failed', errorType=type(exc).__name__, reason=str(exc))
                    self.worker_error = dict(errorType=type(exc).__name__, reason=str(exc))
                    self._cancel_pending()
                break
            finally:
                with self.condition:
                    self.inflight = False
        try:
            cloud = self.grid.array()
        except Exception as exc:
            with self.condition:
                self.worker_error = dict(errorType=type(exc).__name__, reason=str(exc))
            return
        with self.condition:
            self.cloud = cloud
            self.cloud_timestamp = self.last_predicted_timestamp
            self.map_version += 1

    def snapshot(self):
        with self.condition:
            poses = np.asarray([(i, *p, *q, t/1e9) for i, (t, p, q) in enumerate(self.trajectory)],
                               dtype=np.float64).reshape(-1, 9)
            return self.cloud, poses, dict(self.counts, mapVersion=self.map_version,
                pendingFrames=len(self.pending), inferenceActive=self.inflight,
                trackingState=self.tracking_state, workerError=self.worker_error,
                captureEndReason=self.end_reason, streamError=self.error,
                cameraTimestampNs=self.last_timestamp,cloudTimestampNs=self.cloud_timestamp)

    def close(self, error=None, *, wait=True, timeout=30):
        with self.condition:
            if error is not None:
                self.error = str(error)
            self.closed = True
            if self.end_reason not in COMPLETE_CAPTURE_REASONS or self.error or self.worker_error:
                self.cancelled = True
                self._cancel_pending()
            self.condition.notify_all()
        if wait:
            self.worker.join(timeout)
            if self.worker.is_alive():
                with self.condition:
                    self.cancelled = True
                    self.error = 'inference_shutdown_timeout'
                    self._cancel_pending()
                raise TimeoutError('Inference worker has not stopped; final map refused')

    def save(self, *, source=None, source_sha=None, runtime_mode='recorded_replay'):
        if self.worker.is_alive() or not self.closed:
            raise RuntimeError('Worker must finish before saving the final map')
        if (self.output/'result.json').exists():
            raise FileExistsError('Final map already saved')
        cloud, poses, counts = self.snapshot()
        if len(cloud):
            write_cloud(self.output/'map_cloud.ply', cloud)
        np.savetxt(self.output/'map_poses.txt', poses, fmt='%.17g')
        after = file_sha256(source) if source is not None else None
        source_changed = source_sha is not None and after != source_sha
        source_complete = self.end_reason in COMPLETE_CAPTURE_REASONS
        complete = bool(source_complete and not (self.error or self.worker_error or source_changed
                                                or counts['rejectedPackets'] or counts['sequenceGaps']))
        predicted = [r for r in self.metrics if r['status']=='predicted']
        def percentiles(values):
            return dict(median=float(np.median(values)), p95=float(np.percentile(values,95)),
                        maximum=float(np.max(values))) if values else None
        result = dict(status='experimental' if complete else 'incomplete',
            source='pretrained_depth_scaled_to_arcore', runtimeMode=runtime_mode,
            cameraStarted=False, metricAccuracyValidated=False, relocalizationValidated=False,
            captureComplete=complete, sourceCaptureComplete=source_complete, captureEndReason=self.end_reason,
            streamError=self.error, workerError=self.worker_error,
            sourceSha256Before=source_sha, sourceSha256After=after,
            sourceUnchanged=not source_changed if source_sha is not None else None,
            points=len(cloud), poses=len(poses), mapTruncated=self.grid.capacity_rejections>0,
            capacityRejections=self.grid.capacity_rejections, **{k:v for k,v in self.counts.items() if k!='poses'},
            timingsMs=dict(queueWait=percentiles([r['queueWaitMs'] for r in predicted]),
                arrivalToMap=percentiles([r['resultAgeMs'] for r in predicted]),
                cameraLagAtResult=percentiles([r['cameraLagAtResultMs'] for r in predicted]),
                inference=percentiles([r['inferenceMs'] for r in predicted]),
                packetIngest=percentiles([r['ingestMs'] for r in self.input_metrics])),
            elapsedS=time.monotonic()-self.started, model=dict(self.model.metadata),
            settings=dict(queuePolicy=self.policy, pendingFrameLimit=1 if self.policy=='latest' else 5000,
                voxelM=.01, minIndependentCameraFrames=3, outputFactor=1, cellLimit=self.grid.limit,
                excludedDepthFrames=sorted(self.excluded), depthRangeM=[.5,5],
                scaleConfidenceMinimum=192, edgeJumpMinimumM=.05, edgeJumpRelative=.03),
            note='Experimental stream; replaced RGB/depth jobs are skipped, poses retained; own-frame poses only; physical shape and phone live throughput unverified')
        (self.output/'frame_metrics.json').write_text(json.dumps(self.metrics,indent=2)+'\n')
        (self.output/'input_metrics.json').write_text(json.dumps(self.input_metrics,indent=2)+'\n')
        (self.output/'result.json').write_text(json.dumps(result,indent=2)+'\n')
        return result


def recording_info(source):
    """Prevalidate an entire recorded source before model inference or pacing."""
    digest = file_sha256(source)
    sequence = timestamp = depth_stamp = 0
    end = None
    frames = 0
    with Path(source).open() as stream:
        for line in stream:
            if not line.strip():
                continue
            d = decode_packet(json.loads(line))
            if end is not None or d['sequence']<=sequence:
                raise ValueError('Nonincreasing sequence or packet after end')
            sequence = d['sequence']
            if d['kind']=='end':
                end=d['reason']
                continue
            if d['timestamp']<timestamp:
                raise ValueError('Decreasing camera timestamp')
            timestamp=d['timestamp']
            if d['kind']=='depth':
                if d['depth_timestamp']<=depth_stamp:
                    raise ValueError('Duplicate/decreasing raw depth timestamp')
                depth_stamp=d['depth_timestamp']
                frames+=1
    if end not in COMPLETE_CAPTURE_REASONS or not frames:
        raise ValueError('Complete recorded detail capture required')
    return digest


def paced_packets(source, speed=1., stop_event=None):
    if not np.isfinite(speed) or not .1<=speed<=10:
        raise ValueError('Replay speed must be between0.1 and10')
    started=time.monotonic()
    first=None
    with Path(source).open() as stream:
        for line in stream:
            if stop_event is not None and stop_event.is_set():return
            if not line.strip():
                continue
            packet=json.loads(line)
            if packet['type']!='end':
                stamp=packet['timestampNs']
                if first is None:first=stamp
                delay=started+(stamp-first)/1e9/speed-time.monotonic()
                if delay>0:
                    if stop_event is not None:
                        if stop_event.wait(delay):return
                    else:time.sleep(delay)
            yield packet


def argument_parser():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--replay',type=Path)
    for name in ('output','repository','checkpoint'):
        parser.add_argument('--'+name,type=Path,required=True)
    parser.add_argument('--model',choices=('depth-pro','metric-small'),default='depth-pro')
    parser.add_argument('--input-size',type=int,default=518)
    parser.add_argument('--queue-policy',choices=('latest','fifo'),default='latest')
    parser.add_argument('--replay-speed',type=float,default=1.)
    parser.add_argument('--exclude-frames',type=int,nargs='*',default=[])
    parser.add_argument('--cell-limit',type=int,default=2_000_000)
    parser.add_argument('--save-depths',action='store_true')
    return parser


def load_model(args):
    if args.model=='depth-pro':
        from host.depth_pro_model import DepthProModel
        return DepthProModel(args.repository,args.checkpoint)
    from host.monocular_depth import MetricDepthAnything
    return MetricDepthAnything(args.repository,args.checkpoint,input_size=args.input_size)


def main():
    parser=argument_parser()
    args=parser.parse_args()
    if args.replay is None:parser.error('Recorded CLI requires --replay; no phone is contacted')
    private_output(args.output)
    digest=recording_info(args.replay)
    model=load_model(args)
    engine=AiStream(model,args.output,policy=args.queue_policy,use_recorded_focal=args.model=='depth-pro',
        exclude_frames=args.exclude_frames,save_depths=args.save_depths,cell_limit=args.cell_limit)
    error=None
    try:
        for packet in paced_packets(args.replay,args.replay_speed):engine.add(packet)
    except KeyboardInterrupt:error='host_interrupted'
    except Exception as exc:error=f'{type(exc).__name__}: {exc}'
    finally:
        engine.close(error,timeout=300 if args.queue_policy=='fifo' else 30)
    print(json.dumps(engine.save(source=args.replay,source_sha=digest),indent=2),flush=True)


if __name__=='__main__':main()

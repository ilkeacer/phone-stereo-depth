"""Time an optional local YOLO detector without opening a phone camera.

This diagnostic never writes an input or annotated image. Model/framework terms
are separate from this repository; keep the checkpoint and report outside Git.
"""
import argparse
import hashlib
import json
from pathlib import Path
import statistics
import time


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--image', type=Path, required=True)
    parser.add_argument('--weights', type=Path, required=True)
    parser.add_argument('--report', type=Path, required=True)
    parser.add_argument('--device', default='0', help='Ultralytics device, e.g. 0 or cpu')
    parser.add_argument('--image-size', type=int, default=640)
    parser.add_argument('--source-width', type=int, default=1280)
    parser.add_argument('--source-height', type=int, default=960)
    parser.add_argument('--warmup', type=int, default=5)
    parser.add_argument('--runs', type=int, default=20)
    args = parser.parse_args()
    if not args.image.is_file() or not args.weights.is_file():
        parser.error('Existing input image and local model weights are required')
    if args.report.exists():
        parser.error('Report must be a new file')
    if not 1 <= args.runs <= 1000 or not 0 <= args.warmup <= 100 or not 64 <= args.image_size <= 2048:
        parser.error('Invalid benchmark size or repetition count')
    if not 64 <= args.source_width <= 4096 or not 64 <= args.source_height <= 4096:
        parser.error('Invalid source dimensions')

    try:
        import cv2
        import numpy as np
        import torch
        from ultralytics import YOLO
    except ImportError as error:
        parser.error(f'Optional local detector dependency missing: {error}')
    image = cv2.imread(str(args.image), cv2.IMREAD_COLOR)
    if image is None:
        parser.error('Input image could not be decoded')
    original_size = [int(image.shape[1]), int(image.shape[0])]
    frame = cv2.resize(image, (args.source_width, args.source_height), interpolation=cv2.INTER_AREA)
    model = YOLO(str(args.weights))
    sync = args.device != 'cpu' and torch.cuda.is_available()

    def infer():
        result = model.predict(frame, imgsz=args.image_size, device=args.device, verbose=False)[0]
        if sync:
            torch.cuda.synchronize()
        return result

    for _ in range(args.warmup):
        infer()
    latencies, detection_counts = [], []
    for _ in range(args.runs):
        start = time.perf_counter()
        result = infer()
        latencies.append((time.perf_counter() - start) * 1000)
        detection_counts.append(len(result.boxes) if result.boxes is not None else 0)
    report = {
        'scope': 'offline detector only; excludes phone capture, transport, depth, ROS and concurrent GPU work',
        'sourceDimensions': original_size,
        'resizedSourceDimensions': [args.source_width, args.source_height],
        'modelImageSize': args.image_size,
        'device': str(result.boxes.data.device) if result.boxes is not None else args.device,
        'warmupRuns': args.warmup,
        'timedRuns': args.runs,
        'medianMs': statistics.median(latencies),
        'p95Ms': float(np.percentile(latencies, 95)),
        'minMs': min(latencies),
        'maxMs': max(latencies),
        'detectionsPerRun': detection_counts,
        'weightsSha256': hashlib.sha256(args.weights.read_bytes()).hexdigest(),
        'framework': f'ultralytics / torch {torch.__version__}',
    }
    args.report.parent.mkdir(parents=True, exist_ok=True)
    with args.report.open('x') as stream:
        json.dump(report, stream, indent=2)
        stream.write('\n')
    print(json.dumps(report, ensure_ascii=False))


if __name__ == '__main__':
    main()

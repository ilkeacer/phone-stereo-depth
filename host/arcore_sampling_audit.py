"""Compare voxel sampling on the SAME saved smoothed ARCore point sequence.

This measures sample reduction, not depth accuracy or recovered object detail.
Raw-depth confidence cannot be reconstructed from an old smoothed XYZ cloud.
"""
import argparse
import hashlib
import json
from pathlib import Path

import numpy as np

from host.arcore_floor import write_cloud
from host.arcore_probe_import import read_probe_cloud


def audit(report, cloud, output):
    output = Path(output)
    if output.exists():
        raise FileExistsError(output)
    events = [json.loads(line) for line in Path(report).read_text().splitlines() if line.strip()]
    depths = [e for e in events if e.get('event') == 'depth']
    original = read_probe_cloud(cloud)
    if sum(e['sampleCount'] for e in depths) != len(original):
        raise ValueError('Per-frame accepted samples and saved PLY count differ')
    xyz = original[:, [0, 2, 1]].copy()
    xyz[:, 1] *= -1
    output.mkdir(parents=True, exist_ok=False)
    metrics = dict(sourcePoints=len(xyz), recordedDepthFrames=len(depths),
                   uniqueDepthTimestamps=len({e['timestampNs'] for e in depths}),
                   depthDimensions=sorted({(e['width'], e['height']) for e in depths}),
                   rawConfidenceAvailable=False, physicalAccuracyMeasured=False,
                   note='Representative displacement is voxel sample reduction error, not physical depth error',
                   inputSHA256={str(Path(p).name): hashlib.sha256(Path(p).read_bytes()).hexdigest()
                                for p in (report, cloud)}, variants=[])
    for resolution in (.05, .02):
        keys = np.floor(xyz / resolution).astype(np.int32)
        # Stable last observation per cell, exactly as the original live VoxelMap.
        unique, reversed_indices, inverse = np.unique(keys[::-1], axis=0, return_index=True, return_inverse=True)
        representatives = xyz[len(xyz) - 1 - reversed_indices]
        displacement = np.linalg.norm(xyz - representatives[inverse[::-1]], axis=1)
        filename = f'map_{int(resolution*100)}cm.ply'
        write_cloud(output / filename, representatives)
        metrics['variants'].append(dict(voxelM=resolution, points=len(representatives),
            representativeDisplacementP50M=float(np.percentile(displacement, 50)),
            representativeDisplacementP95M=float(np.percentile(displacement, 95)), output=filename))
    (output / 'metrics.json').write_text(json.dumps(metrics, indent=2) + '\n')
    return metrics


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--report', type=Path, required=True)
    parser.add_argument('--cloud', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    print(json.dumps(audit(args.report, args.cloud, args.output), indent=2))


if __name__ == '__main__':
    main()

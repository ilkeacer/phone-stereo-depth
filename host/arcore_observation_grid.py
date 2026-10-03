"""Show where a saved ARCore map has depth returns, without inventing free space.

Blank cells mean no saved depth return in this arbitrary path-centered window.
They are not a measured room boundary, obstacle clearance, or navigation map.
"""
import argparse
import json
from pathlib import Path

import numpy as np

from host.arcore_floor import floor_candidate
from host.ros_arcore_saved import load_saved


def observation_grid(points, camera_positions, floor_mask, *, resolution=.1, padding=3.):
    points = np.asarray(points, dtype=np.float64)
    cameras = np.asarray(camera_positions, dtype=np.float64)
    floor_mask = np.asarray(floor_mask, dtype=np.bool_)
    if points.ndim != 2 or points.shape[1] != 3 or cameras.ndim != 2 or cameras.shape[1] != 3:
        raise ValueError('Expected XYZ points and camera positions')
    if not len(cameras) or floor_mask.shape != (len(points),):
        raise ValueError('Missing camera positions or mismatched floor mask')
    if not np.isfinite(points).all() or not np.isfinite(cameras).all():
        raise ValueError('Nonfinite geometry')
    if not .02 <= resolution <= 1. or not .5 <= padding <= 10.:
        raise ValueError('Invalid observation window')
    minimum = np.floor((cameras[:, :2].min(axis=0) - padding) / resolution) * resolution
    maximum = np.ceil((cameras[:, :2].max(axis=0) + padding) / resolution) * resolution
    width, height = (np.rint((maximum - minimum) / resolution).astype(int) + 1).tolist()
    if width * height > 1_000_000:
        raise ValueError('Observation window exceeds one million cells')
    grid = np.zeros((height, width), dtype=np.uint8)
    indices = np.floor((points[:, :2] - minimum) / resolution + 1e-8).astype(np.int64)
    inside = ((indices[:, 0] >= 0) & (indices[:, 0] < width) &
              (indices[:, 1] >= 0) & (indices[:, 1] < height))
    accepted = indices[inside]
    if len(accepted):
        values = np.where(floor_mask[inside], 1, 2).astype(np.uint8)
        np.bitwise_or.at(grid, (accepted[:, 1], accepted[:, 0]), values)
    return grid, minimum, int((~inside).sum())


def summarize(grid, origin, resolution, *, points, poses, outside, floor_candidate_found):
    counts = np.bincount(grid.ravel(), minlength=4)
    return dict(status='saved_depth_return_observation_only',
                meaning='0=no saved depth return; 1=floor-plane candidate; 2=other depth return; 3=both',
                noReturnIsFreeSpace=False, noReturnProvesUnseen=False,
                physicalFloorValidated=False, robotNavigationValidated=False,
                floorCandidateFound=floor_candidate_found,
                inputPoints=points, inputPoses=poses, pointsOutsideWindow=outside,
                originXYM=origin.tolist(), resolutionM=resolution,
                width=int(grid.shape[1]), height=int(grid.shape[0]),
                noReturnCells=int(counts[0]), floorOnlyCells=int(counts[1]),
                otherOnlyCells=int(counts[2]), bothCells=int(counts[3]))


def render(grid, origin, resolution, poses, path):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    from matplotlib.colors import ListedColormap
    from matplotlib.patches import Patch
    from matplotlib.lines import Line2D

    height, width = grid.shape
    figure, axis = plt.subplots(figsize=(9, 7))
    axis.imshow(grid, origin='lower', interpolation='nearest',
                extent=(origin[0], origin[0] + width * resolution,
                        origin[1], origin[1] + height * resolution),
                cmap=ListedColormap(['#151a20', '#53b578', '#559acb', '#d6be63']),
                vmin=0, vmax=3)
    axis.plot(poses[:, 0], poses[:, 1], color='#ff7660', linewidth=1.5)
    axis.scatter(poses[0, 0], poses[0, 1], color='#ff7660', s=35, marker='o')
    axis.set(xlabel='ROS X (m)', ylabel='ROS Y (m)',
             title='Kaydedilmiş derinlik dönüşleri (üstten görünüm)')
    axis.set_aspect('equal')
    axis.legend(handles=[Patch(color='#151a20', label='Derinlik dönüşü yok'),
                         Patch(color='#53b578', label='Zemin düzlemi adayı'),
                         Patch(color='#559acb', label='Diğer 3B nokta'),
                         Patch(color='#d6be63', label='İkisi aynı XY hücresinde'),
                         Line2D([0], [0], color='#ff7660', label='ARCore kamera yolu')],
                loc='upper right', fontsize=8)
    figure.text(.5, .015, 'Koyu alan: serbest alan değil, bu kayıtta derinlik dönüşü yok',
                ha='center', fontsize=9)
    figure.tight_layout(rect=(0, .03, 1, 1))
    figure.savefig(path, dpi=140)
    plt.close(figure)


def analyze(session, output, *, resolution=.1, padding=3.):
    xyz, _, trajectory = load_saved(session)
    cameras = trajectory[:, 1:4]
    candidate = floor_candidate(xyz, cameras)
    mask = candidate['mask'] if candidate is not None else np.zeros(len(xyz), dtype=bool)
    grid, origin, outside = observation_grid(xyz, cameras, mask,
                                             resolution=resolution, padding=padding)
    summary = summarize(grid, origin, resolution, points=len(xyz), poses=len(cameras),
                        outside=outside, floor_candidate_found=candidate is not None)
    output = Path(output)
    output.mkdir(parents=True, exist_ok=False)
    np.save(output / 'return_grid.npy', grid)
    render(grid, origin, resolution, cameras, output / 'return_grid.png')
    (output / 'observation.json').write_text(json.dumps(summary, indent=2) + '\n')
    return summary


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--session', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True, help='New private output directory')
    parser.add_argument('--resolution', type=float, default=.1)
    parser.add_argument('--padding', type=float, default=3.)
    args = parser.parse_args()
    print(json.dumps(analyze(args.session, args.output,
                             resolution=args.resolution, padding=args.padding), indent=2))


if __name__ == '__main__':
    main()

"""Run each provenance-labelled post-loss raw section as a separate ROS map.

No phone access and no map merge. Failed runs retain their logs and index.
"""
import argparse
import json
import os
from pathlib import Path
import re
import shutil
import subprocess
import time

from host.motion_recording import atomic_json
from host.ros_scale import load_scale

ROOT = Path(__file__).resolve().parents[1]
ROS_SETUP = '/opt/ros/humble/setup.bash'


def validate_segments(segments):
    if not segments or len(segments) > 8:
        raise ValueError('One to eight independent raw sections are required')
    manifests = []
    for directory in segments:
        value = json.loads((Path(directory)/'manifest.json').read_text())
        lo, hi = value.get('originalFrameIndexRange', [None, None])
        if (value.get('status') != 'complete' or value.get('derivedSubset') is not True or
                value.get('sourceTrackingLossNotRecovered') is not True or
                value.get('separateMapOriginRequired') is not True or
                type(lo) is not int or type(hi) is not int or
                hi-lo+1 != value.get('frames') or lo < 0):
            raise ValueError('Section lacks explicit separate-origin tracking-loss provenance')
        manifests.append(value)
    identity = lambda m:(m.get('originalSourceRun'),m.get('sourceDirectory'),m.get('sourceFailureStampNs'))
    if len({identity(m) for m in manifests}) != 1 or any(None in identity(m) for m in manifests):
        raise ValueError('Sections do not share one original loss event')
    ordered = sorted(zip(segments, manifests), key=lambda x:x[1]['originalFrameIndexRange'][0])
    for (_,left),(_,right) in zip(ordered, ordered[1:]):
        if left['originalFrameIndexRange'][1] >= right['originalFrameIndexRange'][0]:
            raise ValueError('Section source frame ranges overlap')
    return ordered


def _ros_command(module, *args):
    return ['bash','-c',f'source {ROS_SETUP} && exec /usr/bin/python3 -m {module} "$@"',
            'bash',*map(str,args)]


def run(segments, output, calibration, scale_config, rate=.5):
    output, calibration, scale_config = map(Path, (output, calibration, scale_config))
    output = output.resolve()
    ordered = validate_segments([Path(segment).resolve() for segment in segments])
    if not .1 <= rate <= 2:
        raise ValueError('Replay rate must be 0.1 to 2')
    scale = load_scale(scale_config,calibration)
    if output.exists():
        raise FileExistsError('Output already exists')
    output.mkdir(parents=True)
    environment = dict(os.environ, PYTHONPATH=str(ROOT)+os.pathsep+os.environ.get('PYTHONPATH',''))
    index = dict(schemaVersion=1, sourceRun=ordered[0][1]['originalSourceRun'],
                 sourceTrackingLossNotRecovered=True, mapContinuityVerified=False,
                 mapsMerged=False, independentMapOrigins=True,
                 scale=scale, replayRate=rate, parts=[])
    atomic_json(output/'fragment-index.json',index)
    for number,(segment,manifest) in enumerate(ordered):
        part = output/f'part-{number:03d}'
        part.mkdir()
        row = dict(sourceRaw=str(segment),sourceFrameIndexRange=manifest['originalFrameIndexRange'],
                   originalCaptureStatus=manifest['originalCaptureStatus'],
                   status='bag_pending',mapContinuityVerified=False)
        index['parts'].append(row);atomic_json(output/'fragment-index.json',index)
        bag = part/'bag'
        export_command = _ros_command('host.ros_stereo','--trial',segment,'--calibration',calibration,
                                      '--scale-config',scale_config,'--output',bag)
        with (part/'bag-export.log').open('w') as log:
            bag_result = subprocess.run(export_command,cwd=ROOT,env=environment,stdout=log,
                                        stderr=subprocess.STDOUT,timeout=120)
        row['bagReturnCode'] = bag_result.returncode
        if bag_result.returncode != 0 or not (bag/'phone-provenance.json').exists():
            row.update(status='bag_failed',error='See bag-export.log')
            atomic_json(output/'fragment-index.json',index)
            break
        provenance = json.loads((bag/'phone-provenance.json').read_text())
        if provenance.get('mapContinuityVerified') is not False or provenance.get('pairs') != manifest['frames']:
            row.update(status='bag_failed',error='Bag provenance differs from raw section')
            atomic_json(output/'fragment-index.json',index)
            break
        row.update(status='replay_pending',bag=str(bag),pairs=provenance['pairs'])
        atomic_json(output/'fragment-index.json',index)
        print(f"Ayrı bölüm {number+1}/{len(ordered)}: {provenance['pairs']} çift ROS içinde işleniyor.",flush=True)
        replay_command = _ros_command('host.ros_session','replay','--bag',bag,'--rate',rate,
                                       '--depth-engine','sgbm','--calibration',calibration)
        with (part/'replay.log').open('w') as log:
            replay_result = subprocess.run(replay_command,cwd=ROOT,env=environment,stdout=log,
                                           stderr=subprocess.STDOUT)
        row['replayReturnCode'] = replay_result.returncode
        text = (part/'replay.log').read_text(errors='replace')
        match = re.search(r'^Oturum kaydı: (.+)$',text,re.MULTILINE)
        if not match:
            row.update(status='replay_failed',error='Replay session path missing; see replay.log')
            atomic_json(output/'fragment-index.json',index)
            break
        replay = Path(match.group(1))
        row['replaySession'] = str(replay)
        summary_path,export_path = replay/'summary.json',replay/'export/result.json'
        if replay_result.returncode != 0 or not summary_path.exists() or not export_path.exists():
            row.update(status='replay_failed',error='No accepted replay result; see replay.log')
            atomic_json(output/'fragment-index.json',index)
            break
        summary,export = json.loads(summary_path.read_text()),json.loads(export_path.read_text())
        if (summary.get('mapState') != 'connected' or summary.get('lostResults') != 0 or
                summary.get('sessionError') or not summary.get('shutdownClean') or
                export.get('status') != 'complete'):
            row.update(status='map_unaccepted',error='Tracking/graph/export did not pass existing checks')
            atomic_json(output/'fragment-index.json',index)
            break
        shutil.copy2(replay/'export/map_cloud.ply',part/'map_cloud.ply')
        shutil.copy2(replay/'export/map_poses.txt',part/'map_poses.txt')
        shutil.copy2(summary_path,part/'summary.json')
        shutil.copy2(export_path,part/'export-result.json')
        # The standalone map viewer reads result.json beside the cloud.
        shutil.copy2(export_path,part/'result.json')
        row.update(status='independent_map_complete',poses=export['poses'],points=export['points'],
                   cloud=str(part/'map_cloud.ply'),posesFile=str(part/'map_poses.txt'))
        atomic_json(output/'fragment-index.json',index)
    index['completedAtUnixSeconds'] = time.time()
    index['status'] = ('independent_maps_complete' if len(index['parts']) == len(ordered) and
                       all(part['status'] == 'independent_map_complete' for part in index['parts'])
                       else 'incomplete')
    atomic_json(output/'fragment-index.json',index)
    return index


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--segment',type=Path,action='append',required=True)
    parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--calibration',type=Path,default=ROOT/'data/calibration/screen_20260910/calibration.npz')
    parser.add_argument('--scale-config',type=Path,default=ROOT/'data/calibration/screen_20260910/scale.json')
    parser.add_argument('--rate',type=float,default=.5)
    args=parser.parse_args()
    result=run(args.segment,args.output,args.calibration,args.scale_config,args.rate)
    print(json.dumps(result,indent=2))
    if result['status'] != 'independent_maps_complete':raise SystemExit(1)


if __name__=='__main__':main()

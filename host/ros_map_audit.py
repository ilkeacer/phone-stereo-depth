"""Read-only, reproducible audit of one saved live ROS mapping session."""
import argparse
import hashlib
import json
from pathlib import Path
import sqlite3

import cv2
import numpy as np

from host.ros_map_view import read_pcl
from host.trajectory_check import assess, endpoint_metrics


def read_json(path):
    return json.loads(path.read_text()) if path.is_file() else None


def digest(path):
    h = hashlib.sha256()
    with path.open('rb') as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b''):
            h.update(chunk)
    return h.hexdigest()


def distribution(values):
    data = np.asarray(values, dtype=float)
    if data.size == 0:
        return None
    if not np.isfinite(data).all():
        raise ValueError('Nonfinite audit metric')
    return dict(count=int(data.size), median=float(np.median(data)), p95=float(np.percentile(data, 95)),
                maximum=float(data.max()))


def brightness_samples(raw, pairs, count=31):
    if not pairs:
        return None
    indices=np.unique(np.linspace(0,len(pairs)-1,min(count,len(pairs)),dtype=int))
    results=[]
    for index in indices:
        name=pairs[int(index)]['sampleFiles'][0]
        if Path(name).name!=name:
            raise ValueError('Unsafe raw image name')
        path=raw/name
        image=cv2.imread(str(path),cv2.IMREAD_GRAYSCALE)
        if image is None:
            raise ValueError(f'Cannot read {path}')
        results.append(dict(index=int(index),meanGray=float(image.mean()),medianGray=float(np.median(image)),
                            pixelsBelow50Percent=float(np.mean(image<50)*100),sha256=digest(path)))
    return dict(strategy='evenly spaced committed pairs; camera 20 raw JPEG; no rectification',
                samples=results,medianOfMeanGray=float(np.median([r['meanGray'] for r in results])),
                medianOfMedianGray=float(np.median([r['medianGray'] for r in results])),
                medianPixelsBelow50Percent=float(np.median([r['pixelsBelow50Percent'] for r in results])))


def audit(directory, sample_count=31):
    directory=Path(directory).resolve()
    summary=read_json(directory/'summary.json')
    capture=read_json(directory/'capture.json')
    export=read_json(directory/'export-result.json')
    if not summary or not capture or not export:
        raise ValueError('Completed session summary, capture and export reports required')
    source_files={name:digest(directory/name) for name in ('summary.json','capture.json','export-result.json')}
    result=dict(session=str(directory),sourceSha256=source_files,
                transport=capture.get('adbTransport'),recordingPairs=capture.get('recordingPairs'),
                recordingComplete=capture.get('recordingComplete'),captureError=capture.get('error'),
                stereoRejections=capture.get('rejections'),
                trackingResults=summary.get('trackingResults'),lostResults=summary.get('lostResults'),
                trackingGuard=summary.get('hybridTrackingGuard'),graph=summary.get('graph'),
                sessionError=summary.get('sessionError'),shutdownClean=summary.get('shutdownClean'),
                scaleSource=summary.get('scaleSource'),metricAccuracyValidated=summary.get('mapAccuracyValidated'),
                exportStatus=export.get('status'))
    raw=directory/'raw';pairs=read_json(raw/'committed-pairs.json')
    manifest=read_json(raw/'manifest.json')
    if manifest:
        result['rawManifest']=dict(status=manifest.get('status'),frames=manifest.get('frames'),
                                   jpegBytes=manifest.get('jpegBytes'),error=manifest.get('error'))
        result['sourceSha256']['raw/manifest.json']=digest(raw/'manifest.json')
    if pairs is not None:
        referenced=[name for pair in pairs for name in pair['sampleFiles']]
        if any(Path(name).name!=name for name in referenced):
            raise ValueError('Unsafe raw image name')
        missing=[name for name in referenced if not (raw/name).is_file()]
        result['rawFileCheck']=dict(committedPairs=len(pairs),referencedJpegs=len(referenced),
                                    distinctJpegs=len(set(referenced)),missingJpegs=missing,
                                    referencedBytes=sum((raw/name).stat().st_size for name in referenced if (raw/name).is_file()))
        result['timing']=dict(stereoDeltaMs=distribution([p['stereoDeltaMs'] for p in pairs]),
                              freshnessAgeUpperMs=distribution([p['freshnessAgeUpperMs'] for p in pairs]))
        result['brightness']=brightness_samples(raw,pairs,sample_count)
        result['sourceSha256']['raw/committed-pairs.json']=digest(raw/'committed-pairs.json')
    depth=read_json(directory/'depth/status.json')
    if depth:
        depth_frames=directory/'depth/frames.jsonl'
        fractions=[]
        if depth_frames.is_file():
            with depth_frames.open() as stream:
                for line in stream:
                    if line.strip():
                        frame=json.loads(line)
                        if 'regions' in frame:fractions.append(frame['regions']['full']['finalPercent'])
            result['sourceSha256']['depth/frames.jsonl']=digest(depth_frames)
        result['depth']=dict(counts=depth.get('counts'),computeMs=depth.get('computeMs'),
                             publishAgeMs=depth.get('publishAgeMs'),
                             finalValidPercentFullImage=distribution(fractions),error=depth.get('error'))
    db=directory/'map/map.db'
    if db.is_file():
        with sqlite3.connect(db.resolve().as_uri()+'?mode=ro',uri=True) as connection:
            result['databaseQuickCheck']=connection.execute('PRAGMA quick_check').fetchone()[0]
    odometry=directory/'odometry-raw.txt'
    if odometry.is_file():
        pose_lines=[line for line in odometry.read_text().splitlines()
                    if line.strip() and not line.lstrip().startswith('#')]
        if len(pose_lines)>=2:
            result['trajectory']=dict(rawOdometry=assess(np.loadtxt(odometry,ndmin=2),
                                                          summary.get('scaleSource','unknown'),False))
        else:
            result['trajectory']=dict(rawOdometry=None,rawOdometryPoseCount=len(pose_lines),
                                      rawOdometryUnavailableReason='At least two poses required for trajectory metrics')
        result['sourceSha256']['odometry-raw.txt']=digest(odometry)
    if export.get('status')=='complete':
        cloud=directory/'export/map_cloud.ply';poses=directory/'export/map_poses.txt'
        xyz,_=read_pcl(cloud)
        trajectory=np.loadtxt(poses,ndmin=2)
        if trajectory.shape[1]!=9:
            raise ValueError('Invalid optimized path')
        result.setdefault('trajectory',{})['optimizedInformationalOnly']=endpoint_metrics(trajectory)
        occupied=np.unique(np.floor(xyz/.10).astype(np.int32),axis=0)
        result['map']=dict(points=len(xyz),poses=len(trajectory),
                           boundsMin=xyz.min(0).tolist(),boundsMax=xyz.max(0).tolist(),
                           occupied10cmVoxelCount=int(len(occupied)),
                           voxelScope='spatial coverage only; not geometry accuracy or room completeness')
        result['sourceSha256']['export/map_cloud.ply']=digest(cloud)
        result['sourceSha256']['export/map_poses.txt']=digest(poses)
    result['acceptanceScope']='Transfer/tracking/graph evidence only; physical endpoint, room coverage and metric geometry require independent validation'
    return result


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('session',type=Path)
    parser.add_argument('--output',required=True,type=Path)
    parser.add_argument('--samples',type=int,default=31)
    args=parser.parse_args()
    if not 1<=args.samples<=200:
        parser.error('--samples must be 1..200')
    report=audit(args.session,args.samples)
    with args.output.open('x') as stream:
        json.dump(report,stream,indent=2,allow_nan=False)
        stream.write('\n')
    print(json.dumps({k:report.get(k) for k in ('session','transport','recordingPairs','trackingResults','lostResults',
                                              'exportStatus','map','databaseQuickCheck')},indent=2))


if __name__=='__main__':
    main()

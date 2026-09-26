"""Quarantine a tracking-loss boundary and copy independent raw stereo sections.

This does not estimate a transform or resume a map. ROS image timestamps use
the offset recorded by ros_live; older captures without that offset are not
automatically split from approximate subscriber image counts.
"""
import argparse
import hashlib
import json
from pathlib import Path
import shutil
import tempfile

from host.motion_recording import atomic_json

DEFAULT_MARGIN_MS = 200
FORMULA = 'ros_image_stamp_ns = source_image_timestamp_ns + rosStampOffsetNs'


def _sha(path):
    digest = hashlib.sha256()
    with Path(path).open('rb') as stream:
        for chunk in iter(lambda: stream.read(1024*1024), b''):
            digest.update(chunk)
    return digest.hexdigest()


def _read(path):
    return json.loads(path.read_text())


def plan(session, margin_ms=DEFAULT_MARGIN_MS):
    """Return explicit source-index sections; never infer lost poses from counts."""
    session = Path(session)
    if type(margin_ms) is not int or not 50 <= margin_ms <= 1000:
        raise ValueError('Boundary margin must be an integer from 50 to 1000 ms')
    raw = session/'raw'
    manifest_path = raw/'manifest.json'
    rows_path = raw/'committed-pairs.json'
    capture_path = session/'capture.json'
    failure_path = session/'hybrid-tracking-failure.json'
    fallback_path = session/'mapping-fallback.json'
    manifest, rows, capture, failure, fallback = map(_read,
        (manifest_path, rows_path, capture_path, failure_path, fallback_path))
    if (manifest.get('status') not in ('complete', 'failed', 'cancelled') or
            manifest.get('derivedSubset') or not isinstance(rows, list) or
            len(rows) != manifest.get('frames') or len(rows) < 3):
        raise ValueError('Expected a finalized original recording with committed stereo pairs')
    if (failure.get('lost') is not True or
            fallback.get('mappingStopped') is not True or
            fallback.get('policy') != 'keep-capture'):
        raise ValueError('No finalized observed tracking loss with retained capture')
    offset = capture.get('rosStampOffsetNs')
    if (type(offset) is not int or capture.get('rosStampFormula') != FORMULA or
            capture.get('sourceRun') != manifest.get('sourceRun')):
        raise ValueError('Exact ROS/source image timestamp offset unavailable; automatic split refused')
    loss_ns = failure.get('stampNs')
    if type(loss_ns) is not int or loss_ns <= 0:
        raise ValueError('Invalid observed loss timestamp')
    previous = None
    for index, row in enumerate(rows):
        timestamps = row.get('sourceTimestampsNs')
        if (row.get('frameIndex') != index or row.get('sourceRun') != manifest['sourceRun'] or
                not isinstance(timestamps, list) or len(timestamps) != 2 or
                any(type(t) is not int or t <= 0 for t in timestamps) or
                not isinstance(row.get('sampleFiles'), list) or len(row['sampleFiles']) != 2):
            raise ValueError('Source commit index is incomplete or non-contiguous')
        if previous is not None and any(t <= old for t, old in zip(timestamps, previous)):
            raise ValueError('Source image timestamps are not increasing')
        previous = timestamps
    margin_ns = margin_ms*1_000_000
    safe_before = [i for i, row in enumerate(rows)
                   if max(row['sourceTimestampsNs'])+offset <= loss_ns-margin_ns]
    safe_after = [i for i, row in enumerate(rows)
                  if min(row['sourceTimestampsNs'])+offset >= loss_ns+margin_ns]
    before_end = safe_before[-1] if safe_before else -1
    after_start = safe_after[0] if safe_after else len(rows)
    if safe_before != list(range(before_end+1)) or safe_after != list(range(after_start, len(rows))):
        raise ValueError('Timestamp boundary does not produce contiguous sections')
    sections = [dict(name='before-loss', start=0, end=before_end,
                     pairs=before_end+1, available=before_end+1 >= 2),
                dict(name='after-loss', start=after_start, end=len(rows)-1,
                     pairs=len(rows)-after_start, available=len(rows)-after_start >= 2)]
    return dict(schemaVersion=1, sourceSession=str(session.resolve()),
                sourceDirectory=str(raw.resolve()), sourceStatus=manifest['status'],
                sourceError=manifest.get('error'), sourceRun=manifest['sourceRun'],
                sourcePairs=len(rows), rosStampOffsetNs=offset, lossStampNs=loss_ns,
                excludedMarginMs=margin_ms, excludedFrameIndexRange=[before_end+1, after_start-1],
                sections=sections, trackingLossRecovered=False, mapContinuityVerified=False,
                separateMapOriginRequired=True,
                sourceSha256={p.name: _sha(p) for p in
                              (manifest_path, rows_path, capture_path, failure_path, fallback_path)})


def materialize(session, output, recovery_plan=None):
    """Copy source JPEGs into immutable-origin sections; never edit originals."""
    session, output = Path(session), Path(output)
    recovery_plan = recovery_plan or plan(session)
    if recovery_plan['sourceSession'] != str(session.resolve()):
        raise ValueError('Plan/source mismatch')
    if output.resolve().is_relative_to(session.resolve()):
        raise ValueError('Output must be outside the original session')
    if output.exists():
        raise FileExistsError('Output already exists')
    if not any(section['available'] for section in recovery_plan['sections']):
        raise ValueError('No section has at least two committed pairs')
    raw = session/'raw'
    rows_path = raw/'committed-pairs.json'
    manifest_path = raw/'manifest.json'
    if (_sha(rows_path) != recovery_plan['sourceSha256']['committed-pairs.json'] or
            _sha(manifest_path) != recovery_plan['sourceSha256']['manifest.json']):
        raise ValueError('Source recording changed after plan')
    rows, manifest = _read(rows_path), _read(manifest_path)
    output.parent.mkdir(parents=True, exist_ok=True)
    temporary = Path(tempfile.mkdtemp(prefix='recovery-pending-', dir=output.parent))
    try:
        atomic_json(temporary/'recovery-plan.json', recovery_plan)
        for section in recovery_plan['sections']:
            if not section['available']:
                continue
            directory = temporary/section['name']
            directory.mkdir()
            selected = []
            hashes = {}
            total_bytes = 0
            for source_index in range(section['start'], section['end']+1):
                row = rows[source_index]
                new_row = dict(row, frameIndex=len(selected), sourceFrameIndex=source_index)
                for name in row['sampleFiles']:
                    source = raw/name
                    if source.resolve().parent != raw.resolve() or not source.is_file():
                        raise ValueError('Source image path escapes raw capture or is missing')
                    target = directory/name
                    if target.exists():
                        raise ValueError('Duplicate source image name')
                    shutil.copy2(source, target)
                    source_hash = _sha(source)
                    if _sha(target) != source_hash:
                        raise ValueError('Copied source image hash differs')
                    hashes[name] = source_hash
                    total_bytes += target.stat().st_size
                selected.append(new_row)
            derived = dict(manifest, status='complete', error=None,
                frames=len(selected), jpegBytes=total_bytes,
                phaseCounts={'derived_'+section['name'].replace('-', '_'):len(selected)},
                derivedSubset=True, originalCaptureStatus=manifest['status'],
                originalCaptureError=manifest.get('error'),
                originalSourceRun=manifest['sourceRun'], sourceDirectory=str(raw.resolve()),
                originalFrameIndexRange=[section['start'], section['end']],
                sourceFailureStampNs=recovery_plan['lossStampNs'],
                excludedFrameIndexRange=recovery_plan['excludedFrameIndexRange'],
                sourceTrackingLossNotRecovered=True, separateMapOriginRequired=True,
                originalManifestSha256=recovery_plan['sourceSha256']['manifest.json'],
                originalCommitIndexSha256=recovery_plan['sourceSha256']['committed-pairs.json'])
            atomic_json(directory/'manifest.json', derived)
            atomic_json(directory/'committed-pairs.json', selected)
            atomic_json(directory/'source-images-sha256.json', hashes)
        # Verify all provenance inputs again before publishing a complete set.
        for name, digest in recovery_plan['sourceSha256'].items():
            path = (raw/name if name in ('manifest.json', 'committed-pairs.json') else session/name)
            if _sha(path) != digest:
                raise ValueError('Source evidence changed while copying')
        temporary.rename(output)
    except Exception:
        shutil.rmtree(temporary)
        raise
    return output


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('session', type=Path)
    parser.add_argument('--margin-ms', type=int, default=DEFAULT_MARGIN_MS)
    parser.add_argument('--output', type=Path, help='Copy independent raw sections here')
    args = parser.parse_args()
    try:
        result = plan(args.session, args.margin_ms)
        if args.output:
            materialize(args.session, args.output, result)
            result['output'] = str(args.output.resolve())
    except (OSError, ValueError, KeyError, TypeError) as exc:
        parser.exit(2, f'Kurtarma bölümleri hazırlanamadı: {exc}\n')
    print(json.dumps(result, indent=2))


if __name__ == '__main__':
    main()

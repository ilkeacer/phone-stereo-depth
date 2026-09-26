"""Offline source-timestamp feasibility audit. Never rewrites timestamps."""
import argparse
import bisect
import json
import statistics
from pathlib import Path


def read_complete_rows(path):
    raw=Path(path).read_bytes()
    lines=raw.splitlines()
    rows=[];truncated=0
    for index,line in enumerate(lines):
        if not line.strip():continue
        try:rows.append(json.loads(line))
        except (json.JSONDecodeError,UnicodeDecodeError):
            # Force-stopping the old Android recorder can truncate its final row.
            # Corruption anywhere else is an error, never silently removed.
            if index!=len(lines)-1 or raw.endswith(b'\n'):raise
            truncated=1
    return rows,truncated


def feasibility(left,right,threshold_ns=20_000_000):
    if threshold_ns<0:raise ValueError('Negative threshold')
    for stream in (left,right):
        if len(stream)<2 or any(type(t) is not int for t in stream):raise ValueError('Need integer source timestamps')
        if any(a>=b for a,b in zip(stream,stream[1:])):raise ValueError('Non-monotonic source timestamps')
    start=max(left[0],right[0]);end=min(left[-1],right[-1])
    if end<=start:raise ValueError('No overlapping source interval')
    eligible=[];nearest=[]
    for timestamp in left:
        if not start<=timestamp<=end:continue
        j=bisect.bisect_left(right,timestamp)
        delta=min(abs(timestamp-right[k]) for k in (j-1,j) if 0<=k<len(right))
        nearest.append(delta)
        if delta<=threshold_ns:eligible.append(timestamp)
    gaps=[(b-a,a,b) for a,b in zip(eligible,eligible[1:])]
    boundaries=[start,*eligible,end]
    all_gaps=[b-a for a,b in zip(boundaries,boundaries[1:])]
    details=[]
    for stream in (left,right):
        intervals=[b-a for a,b in zip(stream,stream[1:])]
        details.append(dict(frames=len(stream),fps=(len(stream)-1)*1e9/(stream[-1]-stream[0]),
                            medianPeriodNs=statistics.median(intervals),maxIntervalMs=max(intervals)/1e6))
    return dict(thresholdMs=threshold_ns/1e6,overlapSeconds=(end-start)/1e9,streams=details,
                eligibleLeftFrames=len(eligible),
                interpretation='Optimistic feasibility: each left frame may use any source right frame, including reuse. Not a unique pair count or a live schedule.',
                maxInternalEligibleGapSeconds=max((g[0] for g in gaps),default=0)/1e9,
                maxNoEligibleIntervalIncludingEdgesSeconds=max(all_gaps)/1e9,
                largestInternalGaps=[dict(seconds=d/1e9,startRelativeSeconds=(a-start)/1e9,endRelativeSeconds=(b-start)/1e9)
                                     for d,a,b in sorted(gaps,reverse=True)[:3]],
                nearestDeltaMedianMs=statistics.median(nearest)/1e6 if nearest else None,
                hardwareSynchronizationVerified=False)


def audit(directory):
    streams=[];joined=[];counts={};metadata=[]
    for camera in ('20','21'):
        images,tail=read_complete_rows(directory/f'camera_{camera}_images.jsonl')
        captures,mtail=read_complete_rows(directory/f'camera_{camera}_metadata.jsonl')
        ts=[row['imageTimestampNs'] for row in images]
        by_ts={row['sensorTimestampNs']:row for row in captures}
        if len(by_ts)!=len(captures):raise ValueError('Duplicate capture metadata')
        streams.append(ts);joined.append([t for t in ts if t in by_ts])
        counts[camera]=dict(truncatedImageTail=tail,truncatedMetadataTail=mtail,
                            imageRows=len(images),metadataRows=len(captures),exactJoinedFrames=len(joined[-1]))
        durations=[row['frameDurationNs'] for row in captures if row.get('frameDurationNs') is not None]
        metadata.append(dict(cameraId=camera,reportedFrameDurationMedianNs=statistics.median(durations) if durations else None))
    return dict(allImageTimestamps=feasibility(*streams),exactMetadataJoinedTimestamps=feasibility(*joined),
                inputCounts=counts,captureMetadata=metadata,
                timestampComparability='Requires the previously verified comparable sensor clock domains; does not establish exposure synchronization.')


def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('directory',type=Path);parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args();report=audit(args.directory)
    args.output.parent.mkdir(parents=True,exist_ok=True)
    args.output.write_text(json.dumps(report,indent=2));print(json.dumps(report,indent=2))


if __name__=='__main__':main()

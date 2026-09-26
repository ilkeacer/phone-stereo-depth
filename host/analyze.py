"""Offline evidence analysis. No camera ID or visual difference proves sensor identity."""
import argparse,bisect,csv,json,re,statistics
from pathlib import Path


def parse_dumpsys(text):
    out=[]
    pattern=r'^== Camera HAL device device@[^/]+/legacy/(\d+) \(v[^)]+\) static information: ==\n(.*?)(?=^== |\Z)'
    for camera,body in re.findall(pattern,text,re.M|re.S):
        def field(name):
            m=re.search(re.escape(name)+r'[^\n]*\n\s*\[([^\]]*)\]',body)
            return m.group(1).split() if m else None
        cost=re.search(r'Resource cost: (\d+)',body)
        out.append(dict(id=camera,resourceCost=int(cost.group(1)) if cost else None,
            facing=field('android.lens.facing'),focalLengths=field('android.lens.info.availableFocalLengths'),
            pixelArray=field('android.sensor.info.pixelArraySize'),activeArray=field('android.sensor.info.activeArraySize'),
            capabilities=field('android.request.availableCapabilities'),timestampSource=field('android.sensor.info.timestampSource')))
    return out


def nearest_unique(left,right,threshold_ns):
    """Order-preserving unique assignment: maximize count, then minimize total |dt|.

    Integer dynamic programming avoids timestamp precision loss. Backtrace bytes
    bound memory to O(n*m), score rows use O(m); suitable for offline recordings.
    """
    if threshold_ns<0: raise ValueError('negative threshold')
    if left!=sorted(left) or right!=sorted(right): raise ValueError('timestamps must be monotonic')
    if len(set(left))!=len(left) or len(set(right))!=len(right): raise ValueError('duplicate sensor timestamps')
    n,m=len(left),len(right)
    trace=bytearray((n+1)*(m+1));counts=[0]*(m+1);costs=[0]*(m+1)
    for i in range(1,n+1):
        nc=[0]*(m+1);ns=[0]*(m+1)
        for j in range(1,m+1):
            count,cost,action=counts[j],costs[j],1
            if nc[j-1]>count or (nc[j-1]==count and ns[j-1]<cost):
                count,cost,action=nc[j-1],ns[j-1],2
            delta=abs(left[i-1]-right[j-1])
            if delta<=threshold_ns:
                c,v=counts[j-1]+1,costs[j-1]+delta
                if c>count or (c==count and v<=cost):count,cost,action=c,v,3
            nc[j],ns[j]=count,cost;trace[i*(m+1)+j]=action
        counts,costs=nc,ns
    pairs=[];i,j=n,m
    while i and j:
        action=trace[i*(m+1)+j]
        if action==3:pairs.append((i-1,j-1,abs(left[i-1]-right[j-1])));i-=1;j-=1
        elif action==1:i-=1
        else:j-=1
    return list(reversed(pairs))


def continuous_evidence(measurements,overlap,seconds=60):
    return len(measurements)==2 and overlap>=seconds and all(
        m['spanSeconds']>=seconds and m['fps']>=7.5 and m['gapsOver200ms']==0
        and m['monotonic'] and m['duplicateImageTimestamps']==0
        and m['metadataMatches']/max(1,m['frames'])>=0.99 for m in measurements)


def read_jsonl(path):
    return [json.loads(line) for line in path.read_text().splitlines() if line.strip()] if path.exists() else []


def stats(values):
    if not values:return None
    v=sorted(values)
    return dict(median=statistics.median(v),p95=v[max(0,__import__('math').ceil(.95*len(v))-1)],maximum=max(v))


def analyze_trial(directory,threshold_ns=20_000_000):
    summary=json.loads((directory/'summary.json').read_text())
    streams={};details=[]
    for f in summary['feeds']:
        camera=f['cameraId']
        images=read_jsonl(directory/f'camera_{camera}_images.jsonl')
        meta=read_jsonl(directory/f'camera_{camera}_metadata.jsonl')
        by_ts={m['sensorTimestampNs']:m for m in meta if m['sensorTimestampNs'] is not None}
        ts=[im['imageTimestampNs'] for im in images]
        joined=[dict(im,capture=by_ts.get(im['imageTimestampNs'])) for im in images]
        with (directory/f'camera_{camera}_joined.jsonl').open('w') as out:
            for j in joined:out.write(json.dumps(j)+'\n')
        streams[camera]=ts
        intervals=[(b-a)/1e6 for a,b in zip(ts,ts[1:])]
        span=(ts[-1]-ts[0])/1e9 if len(ts)>1 else 0
        details.append(dict(cameraId=camera,frames=len(ts),spanSeconds=span,
            fps=(len(ts)-1)/span if span else 0,metadataMatches=sum(im['imageTimestampNs'] in by_ts for im in images),
            unmatchedMetadata=len(set(by_ts)-set(ts)),duplicateMetadataTimestamps=len(meta)-len(by_ts),
            duplicateImageTimestamps=len(ts)-len(set(ts)),monotonic=all(a<b for a,b in zip(ts,ts[1:])),
            frameIntervalMs=stats(intervals),gapsOver200ms=sum(t>200 for t in intervals)))
    result=dict(trial=directory.name,**summary,measurements=details)
    if len(streams)==2 and all(streams.values()):
        ids=list(streams);l,r=(streams[i] for i in ids)
        pairs=nearest_unique(l,r,threshold_ns)
        overlap=max(0,min(l[-1],r[-1])-max(l[0],r[0]))/1e9
        result['pairing']=dict(policy='order-preserving max-cardinality then minimum total absolute delta',thresholdMs=threshold_ns/1e6,
            pairs=len(pairs),droppedLeft=len(l)-len(pairs),droppedRight=len(r)-len(pairs),
            deltaMs=stats([p[2]/1e6 for p in pairs]),overlapSeconds=overlap,
            timestampComparability='Check discovery timestampSource and realtime offsets; equal source does not prove exposure sync')
        result['continuous60sEvidence']=continuous_evidence(details,overlap)
        with (directory/'pairs.csv').open('w') as out:
            w=csv.writer(out);w.writerow(['leftSequence','rightSequence','leftTimestampNs','rightTimestampNs','deltaNs'])
            w.writerows((i+1,j+1,l[i],r[j],d) for i,j,d in pairs)
    return result


def main():
    ap=argparse.ArgumentParser();ap.add_argument('run',type=Path);ap.add_argument('--threshold-ms',type=float,default=20)
    args=ap.parse_args()
    results=[analyze_trial(p.parent,int(args.threshold_ms*1e6)) for p in sorted(args.run.glob('*/summary.json'))]
    (args.run/'analysis.json').write_text(json.dumps(results,indent=2))
    for r in results:
        print(r['trial'],r['streamingCandidate'],[(m['cameraId'],m['frames'],round(m['fps'],2)) for m in r['measurements']])
    return results
if __name__=='__main__': main()

"""Summarize a finished local ROS trial without treating DB rows as a map."""
import argparse
import json
import math
from pathlib import Path
import re
import sqlite3


def summarize(directory):
    capture_path=directory/'capture.json'
    capture=json.loads(capture_path.read_text()) if capture_path.exists() else {}
    log=directory/'mapping.log'
    text=log.read_text(errors='replace') if log.exists() else ''
    qualities=[int(v) for v in re.findall(r'Odom: quality=(\d+)',text)]
    status_path=directory/'odometry-status.jsonl'
    statuses=[json.loads(line) for line in status_path.read_text().splitlines() if line.strip()] if status_path.exists() else None
    odometry_results=len(statuses) if statuses is not None else len(qualities)
    tracked=sum(not row['lost'] for row in statuses) if statuses is not None else sum(q>0 for q in qualities)
    lost=sum(row['lost'] for row in statuses) if statuses is not None else sum(q==0 for q in qualities)
    process_failures=re.findall(r'\[ERROR\].*?process has died.*',text)
    database=directory/'map'/'map.db'
    graph=dict(available=False,activeNodes=0,links=0)
    timeline=dict(available=False)
    if database.exists():
        try:
            with sqlite3.connect(database.resolve().as_uri()+'?mode=ro',uri=True) as connection:
                columns={r[1] for r in connection.execute('PRAGMA table_info(Node)')}
                key='id' if 'id' in columns else 'rowid'
                if 'stamp' in columns:
                    stamps=sorted(r[0] for r in connection.execute(f'SELECT stamp FROM Node WHERE {key}>0')
                                  if r[0] is not None and math.isfinite(r[0]) and r[0]>0)
                    timeline=dict(available=bool(stamps),storedNodeSamples=len(stamps),
                        firstStampSeconds=stamps[0] if stamps else None,lastStampSeconds=stamps[-1] if stamps else None,
                        maxGapSeconds=max((b-a for a,b in zip(stamps,stamps[1:])),default=None),
                        scope='All stored positive-ID nodes, including merged/rejected keyframes; not every input image or active pose')
                ids={r[0] for r in connection.execute(f'SELECT {key} FROM Node WHERE weight>=0')}
                edges=list(connection.execute('SELECT from_id,to_id FROM Link WHERE from_id!=to_id'))
                adjacency={i:set() for i in ids};valid_edges=[]
                for a,b in edges:
                    if a in ids and b in ids:
                        adjacency[a].add(b);adjacency[b].add(a);valid_edges.append((a,b))
                unseen=set(ids);components=[]
                while unseen:
                    todo=[unseen.pop()];component=[]
                    while todo:
                        current=todo.pop();component.append(current)
                        neighbors=adjacency[current]&unseen;unseen-=neighbors;todo.extend(neighbors)
                    components.append(sorted(component))
                components.sort(key=len,reverse=True)
                graph=dict(available=True,activeNodes=len(ids),links=len(valid_edges),
                    danglingLinks=len(edges)-len(valid_edges),components=components,
                    connectedComponents=len(components),largestComponentNodes=max(map(len,components),default=0),
                    allActiveNodesConnected=len(components)==1 and len(ids)>1)
        except sqlite3.Error as exc:graph['error']=str(exc)
    accumulated=graph.get('largestComponentNodes',0)>1
    lifecycle_path=directory/'lifecycle.json'
    lifecycle=json.loads(lifecycle_path.read_text()) if lifecycle_path.exists() else {}
    depth_path=directory/'depth/status.json'
    depth_status=json.loads(depth_path.read_text()) if depth_path.exists() else None
    input_count=capture.get('inputPairs') or capture.get('publishedPairs')
    guard_path=directory/'hybrid-tracking-failure.json'
    guard_failure=json.loads(guard_path.read_text()) if guard_path.exists() else None
    cutoff=lifecycle.get('mappingLogBytesBeforeShutdown')
    if cutoff is not None:
        raw=log.read_bytes() if log.exists() else b''
        before=raw[:cutoff].decode(errors='replace');after=raw[cutoff:].decode(errors='replace')
        process_failures=re.findall(r'\[ERROR\].*?process has died.*',before)
        process_failures += [line for line in re.findall(r'\[ERROR\].*?process has died.*',after) if not re.search(r'exit code -2[,\]]',line)]
    cleanup_ok=not any(c.get('forcedKill') or c.get('remaining') for c in lifecycle.get('cleanup',[]))
    map_state='none' if not accumulated else 'connected' if graph.get('allActiveNodesConnected') else 'partial'
    depth_tracking_invalid=lifecycle.get('inputKind') in ('rgbd','hybrid') and lost>0
    return dict(mapState=map_state,publishedPairs=capture.get('publishedPairs',0),inputKind=lifecycle.get('inputKind'),
        odometryResults=odometry_results,trackingResults=tracked,lostResults=lost,graph=graph,mappingTimeline=timeline,
        trackingSource='synchronized_OdomInfo' if statuses is not None else 'legacy_console_quality_proxy',
        trackingCountScope='Matched Odom/OdomInfo only; independent unpaired guard events reported separately' if statuses is not None else 'Console proxy only',
        hybridTrackingGuard=lifecycle.get('hybridTrackingGuard'),hybridTrackingFailure=guard_failure,
        captureContinuedAfterTrackingLoss=lifecycle.get('captureContinuedAfterTrackingLoss',False),
        recoveryPlanAvailable=lifecycle.get('recoveryPlanAvailable',False),
        recoveryPlanError=lifecycle.get('recoveryPlanError'),
        recordingComplete=capture.get('recordingComplete',False),recordingPairs=capture.get('recordingPairs',0),
        depthEngine=lifecycle.get('depthEngine','source_default'),onlineDepth=depth_status,
        onlineDepthInputCoverageRatio=depth_status.get('counts',{}).get('publishedDepth',0)/input_count if depth_status and input_count else None,
        consolePositiveQualityResults=sum(q>0 for q in qualities),consoleZeroQualityResults=sum(q==0 for q in qualities),
        inputPairs=capture.get('inputPairs'),leftImagesObserved=capture.get('publishedPairs',0),
        odometryInputCoverageRatio=odometry_results/(capture.get('inputPairs') or capture['publishedPairs']) if (capture.get('inputPairs') or capture.get('publishedPairs')) else None,
        accumulatedGraphPresent=accumulated,mapAccuracyValidated=False,
        depthTrackingInvalid=depth_tracking_invalid,
        squareMm=capture.get('squareMm'),scaleMeasurement=capture.get('scaleMeasurement'),
        scaleSource=lifecycle.get('depthScaleSource') or capture.get('scaleSource'),
        odometryScaleSource=lifecycle.get('odometryScaleSource') or capture.get('odometryScaleSource',capture.get('scaleSource')),
        captureError=capture.get('error'),
        captureReportPresent=capture_path.exists(),processFailures=process_failures,
        sessionError=lifecycle.get('error'),shutdownClean=cleanup_ok,mode=lifecycle.get('mode','live'),
        conclusion=('Session failed; any retained graph is diagnostic evidence, not an accepted map.' if
                    lifecycle.get('error') or capture.get('error') or process_failures or guard_failure or depth_tracking_invalid else
                    'One connected graph exists; trajectory/scale accuracy remains unverified.' if map_state=='connected' else
                    'Only disconnected map fragments exist; this is not one continuous map.' if map_state=='partial' else
                    'No accumulated map demonstrated by this session.'))


def main():
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('directory',type=Path)
    args=parser.parse_args();result=summarize(args.directory)
    with (args.directory/'summary.json').open('x') as stream:json.dump(result,stream,indent=2)
    print(f"\nTakip: {result['trackingResults']}/{result['odometryResults']} · kayıp: {result['lostResults']}")
    if result['processFailures']:print('HATA: ROS işlemi beklenmeden kapandı; günlük incelenmeli.')
    print('Tek bağlantılı harita verisi oluştu; doğruluğu henüz doğrulanmadı.' if result['mapState']=='connected'
          else 'Ayrı harita parçaları oluştu; kesintisiz tek harita değil.' if result['mapState']=='partial'
          else 'Bu oturumda birikimli harita oluştuğu doğrulanamadı.')
    print(f"Sonuç: {args.directory/'summary.json'}")


if __name__=='__main__':main()

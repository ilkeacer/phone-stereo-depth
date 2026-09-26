"""Export a closed session; retain failed attempts and distinguish map fragments."""
import argparse
import json
from pathlib import Path
import sqlite3
import subprocess
import tempfile
from host.motion_recording import atomic_json


def select_component_database(source,destination,component):
    """Copy the closed source DB, then retain only one linked graph component."""
    ids=sorted(set(component))
    if len(ids)<2 or any(type(i) is not int or i<=0 for i in ids):
        raise ValueError('At least two positive node IDs required for component export')
    with sqlite3.connect(source.resolve().as_uri()+'?mode=ro',uri=True) as original,sqlite3.connect(destination) as selected:
        original.backup(selected)
        selected.execute('CREATE TEMP TABLE selected_ids(id INTEGER PRIMARY KEY)')
        selected.executemany('INSERT INTO selected_ids VALUES (?)',[(i,) for i in ids])
        selected.execute('DELETE FROM Link WHERE from_id NOT IN (SELECT id FROM selected_ids) OR to_id NOT IN (SELECT id FROM selected_ids)')
        selected.execute('DELETE FROM Node WHERE id NOT IN (SELECT id FROM selected_ids)')
        actual=[row[0] for row in selected.execute('SELECT id FROM Node ORDER BY id')]
        if actual!=ids:
            raise ValueError('Selected component differs from source database nodes')
        selected.commit()


def export(directory,diagnostic=False):
    if not diagnostic and any((directory/name).exists() for name in ('hybrid-tracking-failure.json','mapping-fallback.json')):
        return dict(status='skipped',reason='Tracking invalidated this map; retained database is diagnostic evidence')
    summary=json.loads((directory/'summary.json').read_text())
    if (not summary['accumulatedGraphPresent'] or summary.get('processFailures')
            or not summary.get('shutdownClean',True) or summary.get('sessionError') or summary.get('captureError')
            or (not diagnostic and (summary.get('hybridTrackingFailure') or summary.get('captureContinuedAfterTrackingLoss')))):
        return dict(status='skipped',reason='No intact accumulated graph or session failed')
    if diagnostic and not (directory/'hybrid-tracking-failure.json').exists():
        return dict(status='skipped',reason='Diagnostic fragment requires recorded tracking failure')
    final=directory/('diagnostic-export' if diagnostic else 'export')
    if final.exists():return dict(status='skipped',reason='An export already exists; preserved unchanged')
    output=Path(tempfile.mkdtemp(prefix='export-attempt-',dir=directory))
    result={}
    with (output/'export.log').open('w') as log:
        try:
            source=directory/'map/map.db'
            component=None
            if diagnostic or summary.get('mapState')=='partial':
                components=summary.get('graph',{}).get('components') or []
                if not components:raise ValueError('Partial graph has no connected component list')
                component=max(components,key=len)
                source=output/'selected-component.db'
                select_component_database(directory/'map/map.db',source,component)
                atomic_json(output/'component-selection.json',dict(nodeIds=sorted(component),
                    sourceDatabase=str((directory/'map/map.db').resolve()),sourceComponentCount=len(components)))
            process=subprocess.run(['rtabmap-export','--cloud','--poses','--output','map',
                '--output_dir',str(output.resolve()),str(source.resolve())],
                stdout=log,stderr=subprocess.STDOUT,timeout=60)
            good=process.returncode==0 and all((output/name).is_file() for name in ('map_cloud.ply','map_poses.txt'))
            if not good:result=dict(status='failed',returnCode=process.returncode)
            else:
                from host.ros_map_view import read_pcl
                import numpy as np
                xyz,_=read_pcl(output/'map_cloud.ply')
                poses=np.loadtxt(output/'map_poses.txt',ndmin=2)
                if poses.shape[1]!=9 or not np.isfinite(poses).all() or len(poses)<2:
                    raise ValueError('Invalid or insufficient exported poses')
                exported_ids=poses[:,8].astype(int).tolist()
                if component is not None and not set(exported_ids)<=set(component):
                    raise ValueError('Exporter included nodes outside the selected component')
                result=dict(status='diagnostic_fragment' if diagnostic else 'partial' if summary.get('mapState')=='partial' else 'complete',
                    returnCode=process.returncode,points=len(xyz),poses=len(poses),
                    exportedNodeIds=exported_ids,
                    pointBounds=dict(min=xyz.min(0).tolist(),max=xyz.max(0).tolist()),
                    pathLengthEstimatedM=float(np.linalg.norm(np.diff(poses[:,1:4],axis=0),axis=1).sum()),
                    scaleSource=summary.get('scaleSource'),squareMm=summary.get('squareMm'),
                    scaleMeasurement=summary.get('scaleMeasurement'),metricAccuracyValidated=False,
                    sourceComponents=summary['graph'].get('connectedComponents'),
                    selectedComponentNodes=len(component) if component is not None else None,
                    scope=('One linked fragment from tracking-invalidated source; not an accepted continuous map.' if diagnostic else
                           'One connected component exported from a fragmented source graph.' if summary.get('mapState')=='partial' else
                           'Connected accumulated graph exported; geometric accuracy remains unverified.'))
        except subprocess.TimeoutExpired:result=dict(status='failed',reason='Export timed out')
        except (OSError,ValueError,sqlite3.Error) as exc:result=dict(status='failed',reason=str(exc))
    if result.get('status') in ('complete','partial','diagnostic_fragment') and (output/'selected-component.db').exists():
        (output/'selected-component.db').unlink()
    atomic_json(output/'result.json',result)
    if result['status'] in ('complete','partial','diagnostic_fragment'):
        output.rename(final);result['directory']=str(final.resolve())
    else:result['attemptDirectory']=str(output.resolve())
    return result


def export_diagnostic_fragment(directory):
    return export(directory,diagnostic=True)


def main():
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('directory',type=Path)
    args=parser.parse_args();result=export(args.directory)
    # Every failed attempt retains its own result/log; current status is atomic.
    if not (args.directory/'export-result.json').exists() or result.get('reason')!='An export already exists; preserved unchanged':
        atomic_json(args.directory/'export-result.json',result)
    print('Harita dosyası hazır.' if result['status']=='complete' else 'Harita dışa aktarımı: '+result['status'],flush=True)


if __name__=='__main__':main()

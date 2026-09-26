"""Export disconnected RTAB-Map components separately, never inventing alignment."""
import argparse
import hashlib
import json
from pathlib import Path
import sqlite3
from host.motion_recording import atomic_json
from host.ros_session_summary import summarize
from host.ros_export_session import export


def sha(path):return hashlib.sha256(path.read_bytes()).hexdigest()


def component_database(source,destination,node_ids):
    """Prune only a new SQLite snapshot; preserve all retained poses/constraints."""
    ids=sorted(set(node_ids))
    if len(ids)<2 or any(type(i) is not int or i<=0 for i in ids):raise ValueError('At least two positive node IDs required')
    if destination.exists():raise FileExistsError(destination)
    destination.parent.mkdir(parents=True,exist_ok=True)
    with sqlite3.connect(source.resolve().as_uri()+'?mode=ro',uri=True) as src,sqlite3.connect(destination) as dst:
        src.backup(dst)
        existing={r[0] for r in dst.execute('SELECT id FROM Node WHERE weight>=0')}
        if not set(ids)<=existing:raise ValueError('Requested node missing or deleted')
        dst.execute('CREATE TEMP TABLE selected_component(id INTEGER PRIMARY KEY)')
        dst.executemany('INSERT INTO selected_component VALUES (?)',[(i,) for i in ids])
        tables={r[0] for r in dst.execute("SELECT name FROM sqlite_master WHERE type='table'")}
        dst.execute('DELETE FROM Link WHERE from_id NOT IN (SELECT id FROM selected_component) OR to_id NOT IN (SELECT id FROM selected_component)')
        for table,key in [('Feature','node_id'),('GlobalDescriptor','node_id'),('Statistics','id'),('Data','id')]:
            if table in tables:dst.execute(f'DELETE FROM {table} WHERE {key} NOT IN (SELECT id FROM selected_component)')
        dst.execute('DELETE FROM Node WHERE id NOT IN (SELECT id FROM selected_component)')
        if 'Admin' in tables:
            columns={r[1] for r in dst.execute('PRAGMA table_info(Admin)')}
            for name in columns:
                if name.startswith('opt_') or name in ('preview_image','dictionary_index'):
                    # Column names come from SQLite but are still quoted safely.
                    dst.execute('UPDATE Admin SET "'+name.replace('"','""')+'"=NULL')
        if dst.execute('PRAGMA foreign_key_check').fetchall():raise ValueError('Derived database has dangling foreign keys')
        if dst.execute('PRAGMA quick_check').fetchone()[0]!='ok':raise ValueError('Derived database check failed')


def export_components(directory,output):
    source=directory/'map/map.db';before=sha(source);original=summarize(directory)
    if original.get('sessionError') or original.get('captureError') or original.get('processFailures') or not original['shutdownClean']:
        raise ValueError('Source session did not finish cleanly')
    output.mkdir(parents=True,exist_ok=False);rows=[]
    for number,ids in enumerate(original['graph'].get('components',[]),1):
        if len(ids)<2:
            rows.append(dict(component=number,nodeIds=ids,status='skipped',reason='Only one pose; no accumulated map'));continue
        folder=output/f'component-{number:02d}'
        component_database(source,folder/'map/map.db',ids)
        atomic_json(folder/'capture.json',dict(source='derived_database_component',publishedPairs=0,
            scaleSource=original.get('scaleSource'),error=None,rawFrameCountKnown=False))
        summary=summarize(folder)
        if not summary['graph']['allActiveNodesConnected']:raise ValueError('Selected component is not connected')
        summary.update(mapState='partial',componentSource=str(directory.resolve()),
            sourceConnectedComponents=original['graph']['connectedComponents'],
            conclusion='An individual component; alignment with the other components is unknown.')
        atomic_json(folder/'summary.json',summary)
        atomic_json(folder/'component-provenance.json',dict(sourceDatabase=str(source.resolve()),sourceSha256=before,
            nodeIds=ids,method='SQLite snapshot; other nodes removed; cached global products cleared; no new links or transforms',
            sharedFrameWithOtherComponents=False))
        result=export(folder)
        exported=result.get('exportedNodeIds',[])
        if result['status'] in ('complete','partial') and set(exported)!=set(ids):
            result.update(status='failed',reason='Export node IDs differ from selected component')
        result.update(sourceConnectedComponents=original['graph']['connectedComponents'])
        atomic_json(folder/'export-result.json',result)
        rows.append(dict(component=number,nodeIds=ids,**result))
        print(json.dumps(dict(component=number,status=result['status'],poses=result.get('poses'),points=result.get('points')),ensure_ascii=False),flush=True)
    after=sha(source)
    result=dict(sourceDatabase=str(source.resolve()),sourceSha256Before=before,sourceSha256After=after,
                sourceUnchanged=before==after,components=rows,alignmentBetweenComponentsKnown=False)
    atomic_json(output/'components.json',result)
    if before!=after:raise RuntimeError('Source changed during export')
    return result


def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('directory',type=Path);p.add_argument('--output',type=Path,required=True)
    a=p.parse_args();export_components(a.directory,a.output)


if __name__=='__main__':main()

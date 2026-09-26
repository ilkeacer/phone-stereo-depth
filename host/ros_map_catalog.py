"""Only successful complete/partial exports are offered as saved maps."""
import json
from pathlib import Path


def available(folder):
    try:
        diagnostic=folder.name=='diagnostic-export'
        result=json.loads((folder.parent/('diagnostic-export-result.json' if diagnostic else 'export-result.json')).read_text())
        expected=('diagnostic_fragment',) if diagnostic else ('complete','partial')
        return result.get('status') in expected and all((folder/f).is_file() for f in ('map_cloud.ply','map_poses.txt'))
    except (OSError,ValueError):return False


def saved_maps(work):
    folders=set(work.glob('ros-*/export'))|set(work.glob('ros-*/diagnostic-export'))|set(work.glob('ros-*/component-*/export'))
    return sorted((p for p in folders if available(p)),key=lambda p:p.stat().st_mtime,reverse=True)


def localization_maps(work):
    """Only accepted complete sessions with their original RTAB-Map DB."""
    result=[]
    for folder in saved_maps(work):
        if folder.name!='export' or folder.parent.name.startswith('component-'):continue
        try:
            summary=json.loads((folder.parent/'summary.json').read_text())
            exported=json.loads((folder.parent/'export-result.json').read_text())
            database=folder.parent/'map'/'map.db'
            if summary.get('mapState')=='connected' and exported.get('status')=='complete' and database.is_file() and database.stat().st_size>0:
                result.append((folder,database))
        except (OSError,ValueError):continue
    return sorted(result,key=lambda item:(item[0].parent.name.startswith('ros-live-'),item[0].stat().st_mtime),reverse=True)


def display_name(folder):
    if folder.name=='diagnostic-export':
        return 'DENEYSEL parça · '+folder.parent.name
    if folder.parent.name.startswith('component-'):
        return 'Harita parçası '+folder.parent.name.removeprefix('component-')+' · '+folder.parent.parent.name
    return folder.parent.name

"""Prepare an isolated copy of an accepted RTAB-Map database for localization."""
import hashlib
import json
from pathlib import Path
import sqlite3


def sha256(path):
    digest=hashlib.sha256()
    with Path(path).open('rb') as stream:
        for block in iter(lambda:stream.read(1024*1024),b''):digest.update(block)
    return digest.hexdigest()


def accepted_reference_ids(info,reference_ids):
    """Return unique accepted RTAB-Map links into the preloaded database."""
    candidates=(('global',info.loop_closure_id),('proximity',info.proximity_detection_id))
    return [(node_id,[kind for kind,candidate in candidates if candidate==node_id])
            for node_id in sorted({node_id for _,node_id in candidates if node_id in reference_ids})]


def prepare_reference_map(source,target,current_scale):
    source=Path(source).resolve();target=Path(target)
    if source.name!='map.db' or not source.is_file():raise ValueError('Tamamlanmis map.db gerekli')
    if target.exists() or target.parent.exists():raise FileExistsError(target)
    session=source.parent.parent
    summary=json.loads((session/'summary.json').read_text())
    exported=json.loads((session/'export-result.json').read_text())
    original_scale=json.loads((session/'scale.json').read_text())
    if summary.get('mapState')!='connected' or exported.get('status')!='complete':
        raise ValueError('Yalniz kabul edilmis, bagli harita konum bulmada kullanilabilir')
    if (original_scale.get('calibrationSha256')!=current_scale.get('calibrationSha256') or
            original_scale.get('squareMm')!=current_scale.get('squareMm') or
            original_scale.get('scaleSource')!=current_scale.get('scaleSource')):
        raise ValueError('Kayitli harita ile mevcut kalibrasyon/olcek farkli')
    before=sha256(source)
    with sqlite3.connect(source.as_uri()+'?mode=ro',uri=True) as old:
        if old.execute('PRAGMA quick_check').fetchone()[0]!='ok':raise ValueError('Kayitli veritabani bozuk')
        count=old.execute('SELECT count(*) FROM Node WHERE id>0 AND weight>=0').fetchone()[0]
        reference_ids=[row[0] for row in old.execute('SELECT id FROM Node WHERE id>0 AND weight>=0 ORDER BY id')]
        if count<2:raise ValueError('Kayitli haritada yeterli etkin poz yok')
        target.parent.mkdir(parents=True,exist_ok=False)
        with sqlite3.connect(target) as new:old.backup(new)
    if sha256(source)!=before:raise RuntimeError('Kaynak harita kopyalanirken degisti')
    with sqlite3.connect(target) as new:
        if new.execute('PRAGMA quick_check').fetchone()[0]!='ok':raise RuntimeError('Harita kopyasi bozuk')
    return dict(referenceMap=str(source),referenceSession=str(session),referenceSha256=before,
                workingCopy=str(target.resolve()),workingCopySha256=sha256(target),referenceActiveNodes=count,
                referenceNodeIds=reference_ids,
                calibrationSha256=original_scale['calibrationSha256'],squareMm=original_scale['squareMm'],
                scaleSource=original_scale['scaleSource'],scope='Existing accepted graph; localization requires a new observed pose match')

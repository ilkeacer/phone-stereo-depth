"""Commit index for motion recordings: partial writes are never input to an audit."""
import json
import os
from pathlib import Path


def atomic_json(path,value):
    path=Path(path);temporary=path.with_name(path.name+'.pending')
    with temporary.open('w') as stream:
        json.dump(value,stream,allow_nan=False)
        stream.flush();os.fsync(stream.fileno())
    temporary.replace(path)


def committed_pairs(directory):
    rows=json.loads((directory/'committed-pairs.json').read_text())
    if not isinstance(rows,list):raise ValueError('Invalid capture commit index')
    for row in rows:
        pair=[]
        for camera in ('20','21'):
            data=row['cameras'][camera]
            image=data['image'];capture=data['capture']
            if image['imageTimestampNs']!=capture['sensorTimestampNs']:
                raise ValueError('Committed metadata timestamp mismatch')
            path=(directory/image['sampleFile']).resolve()
            if not path.is_relative_to(directory.resolve()) or not path.is_file():
                raise ValueError('Missing or invalid committed image')
            pair.append(dict(image,capture=capture))
        delta=abs(pair[0]['imageTimestampNs']-pair[1]['imageTimestampNs'])
        if delta>20_000_000:raise ValueError('Committed stereo timestamps differ too much')
        yield pair[0],pair[1],delta

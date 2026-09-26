import json
from pathlib import Path
import sqlite3
import tempfile
import unittest
from types import SimpleNamespace

from host.ros_localization import prepare_reference_map,sha256,accepted_reference_ids


class LocalizationMapTests(unittest.TestCase):
    def test_reference_match_requires_old_id_and_deduplicates_kinds(self):
        info=SimpleNamespace(loop_closure_id=8,proximity_detection_id=8)
        self.assertEqual(accepted_reference_ids(info,{1,8}),[(8,['global','proximity'])])
        self.assertEqual(accepted_reference_ids(info,{1}),[])
        info.proximity_detection_id=1
        self.assertEqual(accepted_reference_ids(info,{1,8}),[(1,['proximity']),(8,['global'])])

    def test_copy_preserves_source_and_requires_matching_scale(self):
        with tempfile.TemporaryDirectory() as temporary:
            root=Path(temporary);source=root/'accepted'/'map'/'map.db'
            source.parent.mkdir(parents=True)
            with sqlite3.connect(source) as db:
                db.execute('CREATE TABLE Node (id INTEGER, weight INTEGER)')
                db.executemany('INSERT INTO Node VALUES (?,?)',[(1,0),(2,0),(3,-9)])
            session=source.parent.parent
            (session/'summary.json').write_text(json.dumps(dict(mapState='connected')))
            (session/'export-result.json').write_text(json.dumps(dict(status='complete')))
            scale=dict(calibrationSha256='same',squareMm=21.44,scaleSource='measured')
            (session/'scale.json').write_text(json.dumps(scale))
            before=sha256(source);target=root/'new'/'map'/'map.db'
            with self.assertRaisesRegex(ValueError,'kalibrasyon/olcek'):
                prepare_reference_map(source,target,dict(scale,squareMm=22.))
            self.assertFalse(target.exists())
            evidence=prepare_reference_map(source,target,scale)
            self.assertEqual(sha256(source),before)
            self.assertEqual(evidence['referenceActiveNodes'],2)
            self.assertEqual(evidence['referenceNodeIds'],[1,2])
            self.assertEqual(accepted_reference_ids(SimpleNamespace(loop_closure_id=3,proximity_detection_id=0),
                                                    set(evidence['referenceNodeIds'])),[])
            self.assertEqual(evidence['workingCopySha256'],sha256(target))
            with self.assertRaises(FileExistsError):prepare_reference_map(source,target,scale)


if __name__=='__main__':unittest.main()

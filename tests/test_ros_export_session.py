import json
from pathlib import Path
import sqlite3
import tempfile
import unittest
from unittest.mock import patch
from host.ros_export_session import export,export_diagnostic_fragment


class ExportTests(unittest.TestCase):
    def test_durable_failure_marker_wins_over_stale_summary(self):
        for name in ('hybrid-tracking-failure.json','mapping-fallback.json'):
            with self.subTest(name=name),tempfile.TemporaryDirectory() as tmp:
                root=Path(tmp);(root/name).write_text('{}')
                (root/'summary.json').write_text(json.dumps(dict(accumulatedGraphPresent=True)))
                with patch('host.ros_export_session.subprocess.run') as run:
                    self.assertEqual(export(root)['status'],'skipped');run.assert_not_called()

    def test_failed_capture_is_not_exported_as_a_map(self):
        for field in ('sessionError','captureError','hybridTrackingFailure','captureContinuedAfterTrackingLoss'):
            with self.subTest(field=field),tempfile.TemporaryDirectory() as tmp:
                root=Path(tmp);(root/'summary.json').write_text(json.dumps(dict(accumulatedGraphPresent=True,**{field:'failure'})))
                with patch('host.ros_export_session.subprocess.run') as run:
                    self.assertEqual(export(root)['status'],'skipped');run.assert_not_called()

    def test_failed_attempt_is_preserved_and_does_not_block_retry(self):
        with tempfile.TemporaryDirectory() as tmp:
            root=Path(tmp);(root/'summary.json').write_text(json.dumps(dict(accumulatedGraphPresent=True)))
            with patch('host.ros_export_session.subprocess.run',side_effect=FileNotFoundError('missing exporter')):
                a=export(root);b=export(root)
            self.assertEqual(a['status'],'failed');self.assertEqual(b['status'],'failed')
            self.assertNotEqual(a['attemptDirectory'],b['attemptDirectory'])
            self.assertTrue((Path(a['attemptDirectory'])/'result.json').exists())
            self.assertFalse((root/'export').exists())

    def test_fragment_export_is_labeled_partial(self):
        import struct
        from types import SimpleNamespace
        with tempfile.TemporaryDirectory() as tmp:
            root=Path(tmp);(root/'map').mkdir()
            with sqlite3.connect(root/'map/map.db') as db:
                db.execute('CREATE TABLE Node(id INTEGER PRIMARY KEY)')
                db.execute('CREATE TABLE Link(from_id INTEGER,to_id INTEGER)')
                db.executemany('INSERT INTO Node VALUES (?)',[(1,),(2,),(3,)])
                db.execute('INSERT INTO Link VALUES (1,2)')
            (root/'summary.json').write_text(json.dumps(dict(accumulatedGraphPresent=True,mapState='partial',
                graph=dict(connectedComponents=2,components=[[1,2],[3]]))))
            def exporter(command,**kwargs):
                out=Path(command[command.index('--output_dir')+1])
                with sqlite3.connect(command[-1]) as selected:
                    self.assertEqual([r[0] for r in selected.execute('SELECT id FROM Node ORDER BY id')],[1,2])
                    self.assertEqual(selected.execute('SELECT count(*) FROM Link').fetchone()[0],1)
                header='ply\nformat binary_little_endian 1.0\nelement vertex 1\n'+''.join('property '+kind+' '+name+'\n' for kind,name in [('float','x'),('float','y'),('float','z'),('uchar','red'),('uchar','green'),('uchar','blue')])+'end_header\n'
                (out/'map_cloud.ply').write_bytes(header.encode()+struct.pack('<fffBBB',1,2,3,255,0,0))
                (out/'map_poses.txt').write_text('0 0 0 0 0 0 0 1 1\n1 1 0 0 0 0 0 1 2\n')
                return SimpleNamespace(returncode=0)
            with patch('host.ros_export_session.subprocess.run',side_effect=exporter):result=export(root)
            self.assertEqual(result['status'],'partial');self.assertEqual(result['sourceComponents'],2)
            self.assertEqual(result['selectedComponentNodes'],2)
            with sqlite3.connect(root/'map/map.db') as original:
                self.assertEqual(original.execute('SELECT count(*) FROM Node').fetchone()[0],3)
            self.assertFalse((root/'export'/'selected-component.db').exists())
            self.assertTrue((root/'export'/'result.json').exists())

    def test_tracking_invalidated_session_exports_only_labeled_diagnostic_fragment(self):
        import struct
        from types import SimpleNamespace
        with tempfile.TemporaryDirectory() as tmp:
            root=Path(tmp);(root/'map').mkdir()
            with sqlite3.connect(root/'map/map.db') as db:
                db.execute('CREATE TABLE Node(id INTEGER PRIMARY KEY)')
                db.execute('CREATE TABLE Link(from_id INTEGER,to_id INTEGER)')
                db.executemany('INSERT INTO Node VALUES (?)',[(1,),(2,)])
                db.execute('INSERT INTO Link VALUES (1,2)')
            (root/'hybrid-tracking-failure.json').write_text('{}')
            (root/'summary.json').write_text(json.dumps(dict(accumulatedGraphPresent=True,mapState='connected',
                hybridTrackingFailure={'lost':True},graph=dict(connectedComponents=1,components=[[1,2]]))))
            self.assertEqual(export(root)['status'],'skipped')
            def exporter(command,**kwargs):
                out=Path(command[command.index('--output_dir')+1])
                header='ply\nformat binary_little_endian 1.0\nelement vertex 1\n'+''.join('property '+kind+' '+name+'\n' for kind,name in [('float','x'),('float','y'),('float','z'),('uchar','red'),('uchar','green'),('uchar','blue')])+'end_header\n'
                (out/'map_cloud.ply').write_bytes(header.encode()+struct.pack('<fffBBB',1,2,3,255,0,0))
                (out/'map_poses.txt').write_text('0 0 0 0 0 0 0 1 1\n1 1 0 0 0 0 0 1 2\n')
                return SimpleNamespace(returncode=0)
            with patch('host.ros_export_session.subprocess.run',side_effect=exporter):result=export_diagnostic_fragment(root)
            self.assertEqual(result['status'],'diagnostic_fragment')
            self.assertIn('tracking-invalidated',result['scope'])
            self.assertFalse((root/'export').exists())
            self.assertTrue((root/'diagnostic-export'/'component-selection.json').exists())

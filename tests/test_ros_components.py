import hashlib,json,sqlite3,tempfile,unittest
from pathlib import Path
from host.ros_components import component_database
from host.ros_map_catalog import saved_maps,localization_maps


class ComponentTests(unittest.TestCase):
    def test_subset_preserves_pose_and_link_blobs_and_source(self):
        with tempfile.TemporaryDirectory() as temp:
            root=Path(temp);source=root/'source.db'
            with sqlite3.connect(source) as c:
                c.execute('CREATE TABLE Node(id INTEGER PRIMARY KEY,weight INTEGER,pose BLOB)')
                c.execute('CREATE TABLE Link(from_id INTEGER,to_id INTEGER,transform BLOB, FOREIGN KEY(from_id) REFERENCES Node(id),FOREIGN KEY(to_id) REFERENCES Node(id))')
                c.execute('CREATE TABLE Data(id INTEGER PRIMARY KEY,image BLOB)')
                c.execute('CREATE TABLE Admin(version TEXT,opt_poses BLOB)')
                c.executemany('INSERT INTO Node VALUES (?,0,?)',[(1,b'pose1'),(2,b'pose2'),(3,b'pose3')])
                c.executemany('INSERT INTO Data VALUES (?,?)',[(1,b'image1'),(2,b'image2'),(3,b'image3')])
                c.execute('INSERT INTO Link VALUES (1,2,?)',(b'constraint12',))
                c.execute('INSERT INTO Admin VALUES ("v",?)',(b'old-global',))
            before=hashlib.sha256(source.read_bytes()).hexdigest();derived=root/'copy.db'
            component_database(source,derived,[1,2])
            self.assertEqual(hashlib.sha256(source.read_bytes()).hexdigest(),before)
            with sqlite3.connect(derived) as c:
                self.assertEqual(c.execute('SELECT id,pose FROM Node ORDER BY id').fetchall(),[(1,b'pose1'),(2,b'pose2')])
                self.assertEqual(c.execute('SELECT transform FROM Link').fetchone()[0],b'constraint12')
                self.assertEqual(c.execute('SELECT version,opt_poses FROM Admin').fetchone(),('v',None))
                self.assertEqual(c.execute('SELECT COUNT(*) FROM Data').fetchone()[0],2)
            with self.assertRaises(FileExistsError):component_database(source,derived,[1,2])

    def test_catalog_includes_nested_partial_but_excludes_failed_exports(self):
        with tempfile.TemporaryDirectory() as temp:
            root=Path(temp)
            for name,status in [('ros-old','complete'),('ros-components/component-02','partial'),('ros-bad','failed')]:
                folder=root/name/'export';folder.mkdir(parents=True)
                (folder/'map_cloud.ply').touch();(folder/'map_poses.txt').touch()
                (folder.parent/'export-result.json').write_text(json.dumps(dict(status=status)))
            self.assertEqual({p.parent.name for p in saved_maps(root)},{'ros-old','component-02'})

    def test_localization_catalog_requires_complete_connected_database(self):
        with tempfile.TemporaryDirectory() as temp:
            root=Path(temp)
            for name,state in [('ros-good','connected'),('ros-partial','partial')]:
                folder=root/name/'export';folder.mkdir(parents=True)
                (folder/'map_cloud.ply').touch();(folder/'map_poses.txt').touch()
                (folder.parent/'export-result.json').write_text(json.dumps(dict(status='complete')))
                (folder.parent/'summary.json').write_text(json.dumps(dict(mapState=state)))
                database=folder.parent/'map'/'map.db';database.parent.mkdir();database.write_bytes(b'valid-sized-placeholder')
            self.assertEqual([folder.parent.name for folder,_ in localization_maps(root)],['ros-good'])

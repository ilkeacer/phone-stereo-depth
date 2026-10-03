import tempfile,unittest,zipfile
from pathlib import Path
from scripts.export_public import public_files,write_archive

class PublicExportTests(unittest.TestCase):
 def test_only_empty_scene_template_is_allowed_and_private_html_is_excluded(self):
  with tempfile.TemporaryDirectory() as tmp:
   root=Path(tmp);(root/'host').mkdir()
   template=root/'host/arcore_scene_viewer.html';template.write_text('<script>__SCENE_PAYLOAD__</script>')
   (root/'host/viewer.html').write_text('<script>{"glb":"PRIVATE_BASE64_SCENE"}</script>')
   self.assertEqual(public_files(root),[template])
   template.write_text('<script>{"glb":"PRIVATE_BASE64_SCENE"}</script>')
   with self.assertRaisesRegex(ValueError,'private scene'):public_files(root)
 def test_archive_retains_executable_mode_and_never_overwrites(self):
  with tempfile.TemporaryDirectory() as tmp:
   root=Path(tmp);script=root/'start.sh';script.write_text('#!/bin/sh\nexit 0\n');script.chmod(0o755)
   output=root/'source.zip';write_archive(output,[script],root)
   with zipfile.ZipFile(output) as z:self.assertEqual((z.getinfo('start.sh').external_attr>>16)&0o777,0o755)
   before=output.read_bytes()
   with self.assertRaises(FileExistsError):write_archive(output,[script],root)
   self.assertEqual(output.read_bytes(),before)
 def test_ros_config_and_workflow_are_included(self):
  with tempfile.TemporaryDirectory() as tmp:
   root=Path(tmp)
   for name in ('configs/map.rviz','.github/workflows/tests.yml'):
    p=root/name;p.parent.mkdir(parents=True,exist_ok=True);p.write_text('name: example')
   self.assertEqual({str(p.relative_to(root)) for p in public_files(root)},{'configs/map.rviz','.github/workflows/tests.yml'})
 def test_capture_and_derived_data_are_excluded_even_under_source(self):
  with tempfile.TemporaryDirectory() as tmp:
   root=Path(tmp);(root/'host/data').mkdir(parents=True)
   (root/'README.md').write_text('Public project summary')
   (root/'host/main.py').write_text('print("example")')
   for name in ('photo.jpg','scene.npy','recording.mcap'):(root/'host'/name).write_bytes(b'PRIVATE TEST PAYLOAD')
   (root/'host/data/private.py').write_text('private fixture')
   self.assertEqual({str(p.relative_to(root)) for p in public_files(root)},{'README.md','host/main.py'})
 def test_local_identity_and_symlinks_stop_export(self):
  with tempfile.TemporaryDirectory() as tmp:
   root=Path(tmp);(root/'host').mkdir()
   (root/'README.md').write_text('/ho'+'me/'+'example-user/private-folder')
   with self.assertRaises(ValueError):public_files(root)
   (root/'README.md').write_text('Public summary')
   (root/'host/link.py').symlink_to(root/'README.md')
   with self.assertRaises(ValueError):public_files(root)

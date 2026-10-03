import base64
import json
from pathlib import Path
import struct
import tempfile
import unittest

import numpy as np

from host.arcore_confirmed_map import file_sha256
from host.arcore_detail import unproject
from host.arcore_floor import write_cloud
from host.arcore_scene_export import (ROOT, camera_frustum, compact_surface, export,
                                     glb_scene, pixel_faces, private_output, srgb_to_linear, supported_faces)


class SceneExportTests(unittest.TestCase):
    def test_triangle_support_requires_independent_frames_not_duplicate_pixels(self):
        faces=np.array([[0,1,2],[2,1,0],[0,1,2],[0,2,3]],np.uint32)
        selected,metrics=supported_faces(faces,np.array([10,10,11,10]),2)
        np.testing.assert_array_equal(selected,[[0,1,2]])
        self.assertEqual(metrics['independentFaceFrameHistogram'],{'1':1,'2':1})
        empty,_=supported_faces(faces,np.array([10,10,10,10]),2)
        self.assertEqual(len(empty),0)
        with self.assertRaises(ValueError):supported_faces(faces,np.array([10]),2)
    def test_faces_do_not_bridge_gaps_or_depth_steps_or_collapsed_voxels(self):
        ids=np.arange(6).reshape(2,3);depth=np.ones((2,3))
        self.assertEqual(len(pixel_faces(ids,depth)),4)
        depth[:,2]=2
        faces=pixel_faces(ids,depth)
        self.assertEqual(len(faces),2)
        self.assertFalse(np.isin(faces,[2,5]).any())
        ids[1,0]=-1
        self.assertEqual(len(pixel_faces(ids,depth)),0)
        self.assertEqual(len(pixel_faces(np.zeros((2,2),int),np.ones((2,2)))),0)

    def test_compaction_preserves_source_xyz_and_removes_duplicate_faces(self):
        xyz=np.array([[0,0,0],[1,0,0],[0,1,0],[9,9,9]],np.float32)
        rgb=np.ones_like(xyz)
        vertices,colors,faces,used=compact_surface(xyz,rgb,np.array([[0,1,2],[2,1,0]],np.uint32))
        np.testing.assert_array_equal(vertices,xyz[used])
        self.assertEqual((len(vertices),len(faces)),(3,1))

    def test_glb_binary_ranges_axis_and_linear_colors(self):
        xyz=np.array([[0,0,0],[1,0,0],[0,1,0]],np.float32)
        poses=np.array([[0,0,0,0,0,0,0,1,1],[1,1,0,0,0,0,0,1,2]],float)
        frustum=camera_frustum(poses[-1],[2,2,1,1],(3,3))
        self.assertEqual(frustum.shape,(16,3))
        np.testing.assert_allclose(frustum[0],poses[-1,1:4])
        glb,doc=glb_scene(xyz,np.full_like(xyz,.5),np.array([[0,1,2]],np.uint32),poses,frustum)
        magic,version,length=struct.unpack('<4sII',glb[:12])
        self.assertEqual((magic,version,length),(b'glTF',2,len(glb)))
        jsonlen,kind=struct.unpack('<II',glb[12:20]);self.assertEqual(kind,0x4E4F534A)
        self.assertEqual(json.loads(glb[20:20+jsonlen]),doc)
        binarylen,kind=struct.unpack('<II',glb[20+jsonlen:28+jsonlen]);self.assertEqual(kind,0x004E4942)
        self.assertEqual(binarylen,doc['buffers'][0]['byteLength'])
        for v in doc['bufferViews']:
            self.assertEqual(v['byteOffset']%4,0)
            self.assertLessEqual(v['byteOffset']+v['byteLength'],binarylen)
        self.assertFalse(doc['extras']['fullRoomCoverageValidated'])
        np.testing.assert_allclose(srgb_to_linear([.5]),[.21404114],atol=1e-7)
        self.assertEqual([m['primitives'][0]['mode'] for m in doc['meshes']],[4,3,1])

    def test_private_output_rejects_source_directories(self):
        with self.assertRaises(ValueError):private_output(ROOT/'host'/'private-scene-test')
        self.assertEqual(private_output(ROOT/'work'/'private-scene-test'),ROOT/'work'/'private-scene-test')

    def test_complete_synthetic_capture_exports_new_scene_and_checks_cache_identity(self):
        with tempfile.TemporaryDirectory() as temp:
            root=Path(temp);source=root/'stream.jsonl';model=root/'model';colored=root/'colored'
            model.mkdir();colored.mkdir();(model/'predictions').mkdir()
            depth=np.full((2,2),1000,'<u2');confidence=np.full((2,2),255,'u1');k=np.array([10,10,.5,.5])
            enc=lambda a:base64.b64encode(a.tobytes()).decode()
            packets=[];metrics=[]
            for i in range(3):
                stamp=(i+1)*100000000
                packets.append(dict(schemaVersion=2,type='depth',sequence=i+1,timestampNs=stamp,
                    depthTimestampNs=stamp,confidenceTimestampNs=stamp,trackingState='TRACKING',
                    translationM=[0,0,0],quaternion=[0,0,0,1],width=2,height=2,intrinsics=k.tolist(),
                    rawDepthU16LE=enc(depth),confidenceU8=enc(confidence)))
                metrics.append(dict(depthFrame=i,status='predicted',timestampNs=stamp))
                np.savez_compressed(model/'predictions'/f'{i:04d}.npz',depthM=depth.astype('f4')/1000,
                    valid=np.ones((2,2),bool),intrinsics=k,timestampNs=stamp,depthTimestampNs=stamp)
            packets.append(dict(schemaVersion=2,type='end',sequence=4,reason='saved_by_user'))
            source.write_text(''.join(json.dumps(p)+'\n' for p in packets))
            xyz=unproject(depth,np.ones((2,2),bool),k,np.zeros(3),[0,0,0,1])
            write_cloud(model/'map_cloud.ply',xyz);write_cloud(colored/'map_cloud.ply',xyz)
            pose_text=''.join(f'{i} 0 0 0 .707106781 0 0 .707106781 {(i+1)/10}\n' for i in range(3))
            (colored/'map_poses.txt').write_text(pose_text);(model/'map_poses.txt').write_text(pose_text)
            sha=file_sha256(source);geo=file_sha256(model/'map_cloud.ply')
            manifest=dict(status='experimental',captureComplete=True,mapTruncated=False,
                sourceSha256Before=sha,sourceSha256After=sha,
                settings=dict(voxelM=.01,outputFactor=1,minIndependentCameraFrames=3),
                model=dict(model='synthetic </script> test'))
            (model/'result.json').write_text(json.dumps(manifest))
            (model/'frame_metrics.json').write_text(json.dumps(metrics))
            (colored/'result.json').write_text(json.dumps(dict(sourceSha256Before=sha,sourceSha256After=sha,
                geometryFileSha256Before=geo,geometryFileSha256After=geo)))
            output=root/'new-scene';result=export(source,model,colored,output)
            self.assertEqual((result['meshVertices'],result['meshTriangles'],result['outputPoints']),(4,2,4))
            self.assertTrue(result['inputsUnchanged']);self.assertEqual(file_sha256(source),sha)
            self.assertTrue(result['pathMatchesRecordedSource'])
            self.assertEqual(result['captureEndReason'],'saved_by_user')
            self.assertEqual(result['lastMappedPoseTimestampS'],.3)
            supported_result=export(source,model,colored,root/'supported-scene',min_face_frames=3)
            self.assertEqual(supported_result['meshTriangles'],2)
            self.assertEqual(supported_result['independentFaceFrameHistogram'],{'3':2})
            no_surface=export(source,model,colored,root/'no-surface',min_face_frames=4)
            self.assertEqual((no_surface['meshVertices'],no_surface['meshTriangles'],no_surface['outputPoints']),(0,0,4))
            self.assertFalse(no_surface['surfaceAvailable'])
            self.assertEqual((root/'no-surface/map_cloud.ply').read_bytes(),(colored/'map_cloud.ply').read_bytes())
            point_glb=(root/'no-surface/observed_room.glb').read_bytes()
            point_jsonlen=struct.unpack_from('<I',point_glb,12)[0]
            point_doc=json.loads(point_glb[20:20+point_jsonlen])
            self.assertEqual(point_doc['meshes'][0]['primitives'][0]['mode'],0)
            html=(output/'viewer.html').read_text()
            self.assertNotIn('synthetic </script> test',html)
            self.assertIn(r'synthetic \u003c/script\u003e test',html)
            self.assertIn("connect-src 'none'",html)
            with self.assertRaises(FileExistsError):export(source,model,colored,output)
            (colored/'map_poses.txt').write_text(pose_text.replace('0 0 0 0','0 1 0 0',1))
            with self.assertRaisesRegex(ValueError,'camera path'):
                export(source,model,colored,root/'bad-path')
            (model/'map_poses.txt').write_text((colored/'map_poses.txt').read_text())
            with self.assertRaisesRegex(ValueError,'recorded source poses'):
                export(source,model,colored,root/'bad-source-path')
            (colored/'map_poses.txt').write_text(pose_text);(model/'map_poses.txt').write_text(pose_text)
            np.savez_compressed(model/'predictions/0000.npz',depthM=depth.astype('f4')/1000,
                valid=np.ones((2,2),bool),intrinsics=k,timestampNs=999,depthTimestampNs=100000000)
            with self.assertRaisesRegex(ValueError,'identity'):
                export(source,model,colored,root/'bad-scene')
            self.assertFalse((root/'bad-scene').exists())


if __name__=='__main__':unittest.main()

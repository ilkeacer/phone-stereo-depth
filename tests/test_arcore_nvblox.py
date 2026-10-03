import base64
from io import BytesIO
import json
from pathlib import Path
import tempfile
import unittest

import numpy as np
from PIL import Image

from host.arcore_confirmed_map import file_sha256
from host.arcore_detail import decode_packet, unproject
from host.arcore_nvblox import cached_depth, load_frames, optical_pose, padded_planes


class NvbloxRecordedTests(unittest.TestCase):
    def test_optical_pose_matches_existing_unprojection_with_rotation(self):
        depth=np.array([[1100,1400],[1700,2200]],np.float32)
        valid=np.ones((2,2),bool);k=np.array([5.,6.,.4,.7]);t=np.array([1.2,-.3,.8])
        for q in ([0,0,0,1],[.2,.3,-.1,np.sqrt(.86)],[0,np.sqrt(.5),0,np.sqrt(.5)]):
            y,x=np.nonzero(valid);z=depth[valid]/1000
            camera=np.column_stack([z*(x-k[2])/k[0],z*(y-k[3])/k[1],z])
            pose=optical_pose(t,q)
            actual=camera@pose[:3,:3].T+pose[:3,3]
            np.testing.assert_allclose(actual,unproject(depth,valid,k,t,np.asarray(q)),atol=5e-7)
            self.assertAlmostEqual(np.linalg.det(pose[:3,:3]),1.,places=6)

    def test_padding_never_adds_valid_pixels_or_changes_intrinsics(self):
        depth=np.full((90,160),2.,np.float32);rgb=np.full((90,160,3),127,np.uint8)
        valid=np.ones((90,160),bool)
        padded,color,mask=padded_planes(depth,rgb,valid)
        self.assertEqual(padded.shape,(92,160));self.assertEqual(mask.sum(),valid.sum())
        np.testing.assert_array_equal(padded[:90],depth)
        np.testing.assert_array_equal(color[:90],rgb)
        self.assertFalse(mask[90:].any());self.assertFalse(padded[90:].any())

    def fixture(self, root):
        model=root/'model';model.mkdir();(model/'predictions').mkdir()
        source=root/'stream.jsonl';k=np.array([5,5,.5,.5],float)
        image=BytesIO();Image.new('RGB',(2,2),(120,80,20)).save(image,format='JPEG')
        enc=lambda a:base64.b64encode(a.tobytes()).decode()
        packets=[];metrics=[]
        for i in range(3):
            stamp=(i+1)*100000000
            packet=dict(schemaVersion=2,type='depth',sequence=i+1,timestampNs=stamp,
                depthTimestampNs=stamp,confidenceTimestampNs=stamp,trackingState='TRACKING',
                translationM=[0,0,0],quaternion=[0,0,0,1],width=2,height=2,intrinsics=k.tolist(),
                rawDepthU16LE=enc(np.full((2,2),1000,'<u2')),confidenceU8=enc(np.full((2,2),255,'u1')),
                rgbEncoding='jpeg',rgbWidth=2,rgbHeight=2,rgbTimestampNs=stamp,
                rgbJpegBase64=base64.b64encode(image.getvalue()).decode(),
                textureToRgbCornersPx=[0,0,2,0,0,2,2,2])
            packets.append(packet);metrics.append(dict(depthFrame=i,status='predicted',timestampNs=stamp,depthTimestampNs=stamp))
            np.savez(model/'predictions'/f'{i:04d}.npz',depthM=np.ones((2,2),np.float32),valid=np.ones((2,2),bool),
                intrinsics=k,timestampNs=stamp,depthTimestampNs=stamp)
        packets.append(dict(schemaVersion=2,type='end',sequence=4,reason='saved_by_user'))
        source.write_text(''.join(json.dumps(p)+'\n' for p in packets))
        poses=np.array([(i,*decode_packet(p)['position'],*decode_packet(p)['rotation'],p['timestampNs']/1e9)
                        for i,p in enumerate(packets[:-1])])
        np.savetxt(model/'map_poses.txt',poses,fmt='%.17g')
        (model/'frame_metrics.json').write_text(json.dumps(metrics))
        sha=file_sha256(source)
        manifest=dict(status='experimental',captureComplete=True,mapTruncated=False,sourceUnchanged=True,
            sourceSha256Before=sha,sourceSha256After=sha,predictedFrames=3,depthFrames=3,poses=3,
            settings=dict(outputFactor=1),model=dict(model='synthetic'))
        (model/'result.json').write_text(json.dumps(manifest))
        return source,model,packets,metrics,manifest

    def test_complete_capture_and_original_inputs_unchanged(self):
        with tempfile.TemporaryDirectory() as temp:
            source,model,_,_,_=self.fixture(Path(temp))
            frames,poses,_,hashes,end=load_frames(source,model,'cache')
            self.assertEqual((len(frames),len(poses),end),(3,3,'saved_by_user'))
            self.assertTrue(all(file_sha256(p)==sha for p,sha in hashes.items()))
            self.assertEqual([f['index'] for f in frames],[0,1,2])
            raw=load_frames(source,model,'raw')[0]
            np.testing.assert_array_equal(raw[0]['depth'],frames[0]['depth'])
            # Different source SHA and changed saved path must be rejected.
            source.write_text(source.read_text()+'\n')
            with self.assertRaisesRegex(ValueError,'provenance'):
                load_frames(source,model,'cache')

    def test_cache_timestamp_intrinsics_and_mask_are_checked(self):
        with tempfile.TemporaryDirectory() as temp:
            _,model,packets,metrics,_=self.fixture(Path(temp))
            d=decode_packet(packets[0]);path=model/'predictions/0000.npz'
            for stamp,k,valid in [(123,d['intrinsics'],np.ones((2,2),bool)),
                                  (d['timestamp'],d['intrinsics']+1,np.ones((2,2),bool)),
                                  (d['timestamp'],d['intrinsics'],np.ones((2,2),np.uint8))]:
                np.savez(path,depthM=np.ones((2,2),np.float32),valid=valid,intrinsics=k,
                         timestampNs=stamp,depthTimestampNs=d['depth_timestamp'])
                with self.assertRaises(ValueError):cached_depth(path,d,metrics[0],1)

    def test_incomplete_capture_and_changed_path_are_rejected(self):
        with tempfile.TemporaryDirectory() as temp:
            source,model,packets,_,manifest=self.fixture(Path(temp))
            saved=model/'map_poses.txt';poses=np.loadtxt(saved);poses[0,1]+=1;np.savetxt(saved,poses)
            with self.assertRaisesRegex(ValueError,'path'):load_frames(source,model,'cache')
            packets[-1]['reason']='stopped_by_user'
            source.write_text(''.join(json.dumps(p)+'\n' for p in packets))
            manifest['sourceSha256Before']=manifest['sourceSha256After']=file_sha256(source)
            (model/'result.json').write_text(json.dumps(manifest))
            with self.assertRaisesRegex(ValueError,'Incomplete'):load_frames(source,model,'cache')


if __name__=='__main__':unittest.main()

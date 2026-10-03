import copy
import json
from pathlib import Path
import tempfile
import threading
import unittest

import numpy as np
import cv2
import base64

from host.arcore_ai_stream import AiStream, recording_info
from host.arcore_nvblox import load_frames
from tests.test_arcore_detail import packet


class Model:
    metadata=dict(model='synthetic deterministic test double')


def prediction(p,d):
    return np.ones(d['raw'].shape,np.float32),np.ones(d['raw'].shape,bool),d['intrinsics'],dict(inferenceMs=1.)


class AiStreamTests(unittest.TestCase):
    def test_latest_slot_keeps_poses_and_own_frame_geometry_while_model_is_blocked(self):
        started=threading.Event();release=threading.Event();seen=[]
        def predict(p,d):
            seen.append(p['sequence'])
            if len(seen)==1:
                started.set();self.assertTrue(release.wait(3))
            return prediction(p,d)
        with tempfile.TemporaryDirectory() as tmp:
            engine=AiStream(Model(),Path(tmp)/'new',predictor=predict,save_depths=True)
            try:
                first=packet();first['translationM']=[1,0,0];engine.add(first)
                self.assertTrue(started.wait(3))
                for i in range(2,8):
                    p=packet(i,i*1_000_000_000);p['translationM']=[i,0,0]
                    engine.add(p)
                    p['translationM'][0]=999  # caller mutation must not alter the job
                cloud,poses,counts=engine.snapshot()
                self.assertEqual((len(cloud),len(poses),counts['pendingFrames']),(0,7,1))
                self.assertEqual(counts['replacedFrames'],5)
                engine.add(dict(schemaVersion=2,type='end',sequence=8,reason='finished'))
                release.set();engine.close()
                self.assertEqual(seen,[1,7])
                self.assertEqual(len(engine.grid.array()),0)  # two views cannot pass min3
                observed=engine.grid.array(False)
                self.assertGreater(observed[:,0].max(),6)
                self.assertLess(observed[:,0].max(),8)
                result=engine.save()
                self.assertTrue(result['captureComplete'])
                self.assertEqual(result['predictedFrames'],2)
                with np.load(engine.output/'predictions/0006.npz') as cache:
                    self.assertEqual(int(cache['timestampNs']),7_000_000_000)
                with self.assertRaises(ValueError):engine.add(packet(9,9_000_000_000))
                with self.assertRaises(FileExistsError):engine.save()
            finally:release.set();engine.close()

    def test_fifo_control_counts_three_frames_and_drains_on_manual_save(self):
        with tempfile.TemporaryDirectory() as tmp:
            engine=AiStream(Model(),Path(tmp)/'new',policy='fifo',predictor=prediction)
            for i in range(3):engine.add(packet(i+1,(i+1)*1_000_000_000))
            engine.add(dict(schemaVersion=2,type='end',sequence=4,reason='saved_by_user'))
            engine.close();result=engine.save()
            self.assertEqual((result['predictedFrames'],result['points'],result['poses']),(3,6,3))
            self.assertTrue(result['captureComplete'])
            self.assertEqual(result['settings']['minIndependentCameraFrames'],3)

    def test_same_camera_frame_cannot_gain_support_from_new_depth_timestamps(self):
        with tempfile.TemporaryDirectory() as tmp:
            engine=AiStream(Model(),Path(tmp)/'new',policy='fifo',predictor=prediction)
            for i in range(3):engine.add(packet(i+1,1_000_000_000,depth_timestamp=500_000_000+i))
            engine.add(dict(schemaVersion=2,type='end',sequence=4,reason='finished'))
            engine.close();result=engine.save()
            self.assertEqual((result['predictedFrames'],result['poses'],result['points']),(3,1,0))

    def test_interrupt_discards_pending_and_inflight_result(self):
        started=threading.Event();release=threading.Event()
        def predict(p,d):
            started.set();self.assertTrue(release.wait(3));return prediction(p,d)
        with tempfile.TemporaryDirectory() as tmp:
            engine=AiStream(Model(),Path(tmp)/'new',predictor=predict)
            try:
                engine.add(packet());self.assertTrue(started.wait(3))
                engine.add(packet(2,2_000_000_000));engine.close('host_interrupted',wait=False)
                release.set();engine.close()
                result=engine.save()
                self.assertFalse(result['captureComplete'])
                self.assertEqual((result['predictedFrames'],result['cancelledFrames']),(0,2))
                self.assertEqual(result['points'],0)
            finally:release.set();engine.close()

    def test_worker_failure_keeps_new_pose_but_refuses_complete_map(self):
        failed=threading.Event()
        def predict(p,d):failed.set();raise RuntimeError('synthetic GPU failure')
        with tempfile.TemporaryDirectory() as tmp:
            engine=AiStream(Model(),Path(tmp)/'new',predictor=predict)
            engine.add(packet());self.assertTrue(failed.wait(3));engine.worker.join(3)
            engine.add(packet(2,2_000_000_000))
            engine.add(dict(schemaVersion=2,type='end',sequence=3,reason='finished'))
            engine.close();result=engine.save()
            self.assertEqual(result['poses'],2)
            self.assertFalse(result['captureComplete'])
            self.assertEqual(result['workerError']['errorType'],'RuntimeError')
            self.assertEqual(result['unavailableFrames'],1)

    def test_frame_skip_exclusion_duplicate_and_timestamp_guards(self):
        def skip(p,d):raise ValueError('synthetic insufficient scale anchors')
        with tempfile.TemporaryDirectory() as tmp:
            engine=AiStream(Model(),Path(tmp)/'new',policy='fifo',predictor=skip,exclude_frames=[1])
            engine.add(packet())
            duplicate=packet(2,2_000_000_000,depth_timestamp=packet()['depthTimestampNs'])
            engine.add(duplicate)
            with self.assertRaises(ValueError):engine.add(packet(3,500_000_000))
            engine.add(packet(3,3_000_000_000))
            engine.add(dict(schemaVersion=2,type='end',sequence=4,reason='finished'))
            engine.close();result=engine.save()
            self.assertEqual((result['skippedFrames'],result['excludedFrames'],result['duplicateDepthFrames']),(1,1,1))
            self.assertEqual(result['points'],0)

    def test_recording_prevalidation_rejects_incomplete_before_output(self):
        with tempfile.TemporaryDirectory() as tmp:
            path=Path(tmp)/'in.jsonl';p=packet()
            path.write_text(json.dumps(p)+'\n')
            with self.assertRaises(ValueError):recording_info(path)
            end=dict(schemaVersion=2,type='end',sequence=2,reason='finished')
            path.write_text(json.dumps(p)+'\n'+json.dumps(end)+'\n');before=path.read_bytes()
            self.assertEqual(len(recording_info(path)),64)
            self.assertEqual(path.read_bytes(),before)
            path.write_text(json.dumps(p)+'\n'+json.dumps(end)+'\n'+json.dumps(p)+'\n')
            with self.assertRaises(ValueError):recording_info(path)

    def test_archived_stream_hash_and_cache_are_accepted_by_scene_loader(self):
        with tempfile.TemporaryDirectory() as tmp:
            engine=AiStream(Model(),Path(tmp)/'new',policy='fifo',predictor=prediction,save_depths=True)
            rgb=np.full((30,30,3),150,np.uint8)
            _,jpeg=cv2.imencode('.jpg',rgb)
            rows=[]
            for i in range(3):
                p=packet(i+1,(i+1)*1_000_000_000)
                p.update(rgbWidth=30,rgbHeight=30,rgbEncoding='jpeg',
                         rgbTimestampNs=p['timestampNs'],rgbJpegBase64=base64.b64encode(jpeg).decode(),
                         textureToRgbCornersPx=[0,0,30,0,0,30,30,30])
                rows.append(p);engine.add(p)
            end=dict(schemaVersion=2,type='end',sequence=4,reason='finished')
            rows.append(end);engine.add(end);engine.close()
            source=engine.output/'stream.jsonl'
            source.write_text(''.join(json.dumps(p)+'\n' for p in rows))
            digest=recording_info(source)
            result=engine.save(source=source,source_sha=digest,runtime_mode='synthetic_phone_stream')
            self.assertTrue(result['sourceUnchanged'])
            frames,poses,manifest,hashes,end=load_frames(source,engine.output,'cache')
            self.assertEqual((len(frames),len(poses),manifest['predictedFrames']),(3,3,3))
            self.assertEqual(hashes[str(source)],digest)


if __name__=='__main__':unittest.main()

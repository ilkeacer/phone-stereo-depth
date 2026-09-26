"""Exercise the real capture worker with a virtual clock and synthetic camera packets."""
import io
import json
import queue
import tempfile
import unittest
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import patch
from PIL import Image
from host.motion_capture import MotionCaptureApp
from host.motion_guide import CaptureGuide


class CaptureSessionTests(unittest.TestCase):
    def run_session(self,mode):
        with tempfile.TemporaryDirectory() as tmp:
            root=Path(tmp);cal=root/'data/calibration/screen_20260910';cal.mkdir(parents=True)
            (cal/'calibration.npz').write_bytes(b'synthetic provenance fixture')
            (cal/'calibration.json').write_text(json.dumps({'geometrySignature':[['crop20',1.4],['crop21',0]]}))
            out=io.BytesIO();Image.new('RGB',(1280,960)).save(out,format='JPEG');blob=out.getvalue()
            clock=SimpleNamespace(now=0.,cancelled=False)
            class Stop:
                def wait(self,seconds):clock.now+=.25;return clock.cancelled
                def is_set(self):return clock.cancelled
            app=MotionCaptureApp.__new__(MotionCaptureApp)
            app.stop=Stop();app.commands=queue.Queue(maxsize=4);app.preview_queue=queue.Queue(maxsize=1)
            commands=[];states=[];first_ready=None;pause_start=None;paused=False
            def adb(*args):
                commands.append(args)
                return SimpleNamespace(stdout='List of devices attached\nabc      device product:test\n')
            app.adb=adb
            def packet():
                t=int(clock.now*1e9);source='old' if clock.now<1 else 'new'
                if mode=='source_changed_ready' and clock.now>6:source='third'
                header={'ok':True,'paired':mode!='unpaired','sourceRun':source,'deviceElapsedNs':t+10_000_000,'deltaNs':1}
                for camera,ts in [('20',t),('21',t+1)]:
                    header[camera]={'length':len(blob),'image':dict(sequence=t,imageTimestampNs=ts,
                        width=1280,height=960,arrivalElapsedNs=t,timestampSource=1),
                        'capture':dict(sensorTimestampNs=ts,crop='crop'+camera,
                                       focusDiopters=1.4 if camera=='20' else 0)}
                return header,[blob,blob]
            def post(kind,message,guide=None,frames=0):
                nonlocal first_ready,pause_start,paused
                state=guide.snapshot() if guide else None
                states.append((kind,message,state,frames,clock.now))
                if kind=='ready':
                    if first_ready is None:first_ready=clock.now
                    if mode=='cancel':clock.cancelled=True;return
                    if mode=='source_changed_ready':return
                    if mode=='pause' and state['active'] and frames>=2 and not paused:
                        app.commands.put(('pause',state['revision']));paused=True;pause_start=clock.now;return
                    if not state['active'] and (pause_start is None or clock.now-pause_start>=2):
                        app.commands.put(('start',state['revision']))
            app.post=post
            with patch('host.motion_capture.PROJECT',root),patch('host.motion_capture.read_pair',packet), \
                 patch('host.motion_capture.time.monotonic',lambda:clock.now), \
                 patch('host.motion_capture.CaptureGuide',lambda:CaptureGuide(duration=.75)), \
                 patch('host.motion_capture.subprocess.run') as cleanup:
                app.capture()
                self.assertEqual(cleanup.call_count,2)
            manifests=[json.loads(path.read_text()) for path in root.glob('data/motion/*/manifest.json')]
            return states,manifests,first_ready

    def test_startup_source_restart_recovers_and_all_steps_complete(self):
        states,manifests,ready=self.run_session('complete')
        self.assertGreaterEqual(ready,5.)
        self.assertEqual(states[-1][0],'done')
        self.assertEqual(manifests[0]['status'],'complete')
        self.assertEqual(len(manifests[0]['phaseCounts']),4)
        self.assertEqual(len([e for e in manifests[0]['guideEvents'] if e['action']=='step_complete']),4)

    def test_pause_resume_recorded_and_completion_waits(self):
        states,manifests,_=self.run_session('pause')
        self.assertEqual(states[-1][0],'done')
        self.assertIn('pause',[e['action'] for e in manifests[0]['guideEvents']])
        self.assertEqual(manifests[0]['status'],'complete')

    def test_unpaired_startup_times_out_without_creating_capture(self):
        states,manifests,_=self.run_session('unpaired')
        self.assertEqual(states[-1][0],'error');self.assertEqual(manifests,[])
        self.assertLess(states[-1][-1],22)

    def test_source_change_after_ready_requires_reconnect(self):
        states,manifests,_=self.run_session('source_changed_ready')
        self.assertEqual(states[-1][0],'error');self.assertIn('oturumu değişti',states[-1][1])
        self.assertEqual(manifests,[])

    def test_cancel_preview_never_creates_capture(self):
        states,manifests,_=self.run_session('cancel')
        self.assertEqual(states[-1][0],'cancelled');self.assertEqual(manifests,[])

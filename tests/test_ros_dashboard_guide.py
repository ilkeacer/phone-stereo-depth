"""Real Tk checks; no ROS node, subprocess or camera is started."""
import json
import os
from pathlib import Path
import tempfile
import time
import unittest
from unittest.mock import Mock, patch


@unittest.skipUnless(os.environ.get('DISPLAY'), 'Requires desktop Tk display')
class DashboardGuideTests(unittest.TestCase):
    def setUp(self):
        import tkinter as tk
        from host.ros_dashboard import Dashboard
        self.tk=tk;self.root=tk.Tk();self.errors=[]
        self.root.report_callback_exception=lambda *args:self.errors.append(str(args))
        self.receiver=patch.object(Dashboard,'receive_images');self.receiver.start()
        self.spawn=patch('host.ros_dashboard.subprocess.Popen');self.popen=self.spawn.start()
        self.dashboard=Dashboard(self.root);self.root.update()

    def tearDown(self):
        self.dashboard.process=None;self.dashboard.shutdown.set();self.dashboard.receiver.join()
        for callback in self.root.tk.call('after','info'):self.root.after_cancel(callback)
        self.root.destroy();self.receiver.stop();self.spawn.stop()
        self.assertEqual(self.errors,[]);self.popen.assert_not_called()

    def descendants(self,widget):
        for child in widget.winfo_children():
            yield child
            yield from self.descendants(child)

    def test_guide_open_close_does_not_start_and_controls_fit_minimum_size(self):
        d=self.dashboard
        d.buttons[0].invoke();self.root.update()
        guide=d.guide_dialog;guide.geometry('820x720');self.root.update()
        for w in self.descendants(guide):
            if isinstance(w,self.tk.Button):
                self.assertGreaterEqual(w.winfo_height(),w.winfo_reqheight(),w.cget('text'))
                self.assertLessEqual(w.winfo_rooty()+w.winfo_height(),guide.winfo_rooty()+guide.winfo_height())
        self.assertIsNone(d.process)
        guide.destroy();d.show_test_guide();self.root.update()
        start=next(w for w in self.descendants(d.guide_dialog) if isinstance(w,self.tk.Button) and w.cget('text').startswith('Hazırım'))
        with patch.object(d,'start') as launch:
            start.invoke();launch.assert_called_once_with(False,depth_engine='sgbm')

    def test_full_room_button_has_distinct_guide_and_180_second_capture(self):
        from host.ros_guidance import ROOM_SECONDS
        d=self.dashboard
        d.buttons[1].invoke();self.root.update()
        self.assertIn('3 dakikalık',d.guide_dialog.title())
        start=next(w for w in self.descendants(d.guide_dialog)
                   if isinstance(w,self.tk.Button) and w.cget('text').startswith('Hazırım'))
        with patch.object(d,'start') as launch:
            start.invoke()
            launch.assert_called_once_with(False,depth_engine='sgbm',seconds=ROOM_SECONDS)
        self.popen.return_value.stdout=[]
        d.start(False,depth_engine='sgbm',seconds=ROOM_SECONDS)
        self.assertEqual(self.popen.call_args.kwargs['env']['PHONE_CAPTURE_SECONDS'],'180')
        self.assertEqual(self.popen.call_args.kwargs['env']['PHONE_TRACKING_LOSS_POLICY'],'continue-provisional')
        self.assertEqual(d.progress['maximum'],180)
        self.assertIn('5–145',d.route.cget('text'))
        d.process=None
        self.popen.reset_mock()

    def test_all_movement_stages_fit_with_long_health_status(self):
        from host.ros_guidance import mapping_guidance,STEPS,ROOM_STEPS,ROOM_SECONDS
        d=self.dashboard;self.root.geometry('1040x1010')
        d.health.config(text='Takip var\nTakip: 500 · Kayıp: 0\nROS haritası: 21.228 nokta\nSGBM: 500 derinlik karesi · 61 ms\nGüncellik için atlanan: 30')
        for duration,steps in ((90,STEPS),(ROOM_SECONDS,ROOM_STEPS)):
            for start,*_ in steps:
                title,counter,detail=mapping_guidance(dict(stage='mapping',durationSeconds=duration,
                    elapsedSeconds=start,remainingSeconds=duration-start,updatedMonotonic=100),100)
                d.state.config(text=title);d.counter.config(text=counter);d.detail.config(text=detail)
                self.root.update()
                self.assertGreaterEqual(d.detail.winfo_height(),d.detail.winfo_reqheight())
                for w in self.descendants(self.root):
                    if isinstance(w,self.tk.Button):
                        self.assertGreaterEqual(w.winfo_height(),w.winfo_reqheight(),w.cget('text'))
                        self.assertLessEqual(w.winfo_rooty()+w.winfo_height(),d.footer.winfo_rooty(),w.cget('text'))

    def test_guard_failure_replaces_movement_in_actual_poll(self):
        d=self.dashboard
        with tempfile.TemporaryDirectory() as directory:
            d.directory=Path(directory);d.process=Mock();d.process.poll.return_value=None;d.diagnostic=False
            (d.directory/'live-status.json').write_text(json.dumps(dict(stage='mapping',durationSeconds=90,elapsedSeconds=40,remainingSeconds=50,updatedMonotonic=time.monotonic())))
            d.poll();self.assertIn('3/5',d.state.cget('text'))
            (d.directory/'hybrid-tracking-failure.json').write_text('{}')
            d.poll();self.assertIn('DUR',d.state.cget('text'))
            self.assertIn('Hareketi durdur',d.detail.cget('text'))
            d.process=None

    def test_camera_preview_does_not_start_mapper_or_record_raw_images(self):
        import shutil
        d=self.dashboard
        d.start_preview()
        command=self.popen.call_args.args[0]
        self.assertIn('host.ros_live',command)
        self.assertNotIn('host.ros_session',command)
        self.assertNotIn('--recording-dir',command)
        self.assertEqual(d.mode,'preview')
        self.assertEqual(d.stop_button.cget('text'),'Önizlemeyi durdur')
        preview_dir=d.directory
        d.process=None
        self.popen.reset_mock()
        shutil.rmtree(preview_dir)

    def test_preview_shows_received_dark_frame_without_blocking(self):
        d=self.dashboard
        with tempfile.TemporaryDirectory() as directory:
            d.directory=Path(directory);d.mode='preview';d.process=Mock();d.process.poll.return_value=None
            (d.directory/'live-status.json').write_text(json.dumps(dict(stage='mapping',remainingSeconds=20,publishedPairs=5)))
            d.images.put((bytes([30])*16*16,16,16,16,time.monotonic()))
            d.poll()
            self.assertTrue(d.preview_seen)
            self.assertIn('Karanlık sahne',d.health.cget('text'))
            self.assertIn('5 stereo çift',d.health.cget('text'))
            self.assertNotIn('DUR',d.state.cget('text'))
            d.process=None

    def test_display_rotation_changes_preview_only(self):
        d=self.dashboard
        pixels=bytes([10])*4*2
        d.images.put((pixels,4,2,4,time.monotonic()))
        d.poll()
        self.assertEqual((d.photo.width(),d.photo.height()),(2,4))
        d.rotate_button.invoke()
        self.assertEqual(d.rotate_button.cget('text'),'Dik göster')
        d.images.put((pixels,4,2,4,time.monotonic()))
        d.poll()
        self.assertEqual((d.photo.width(),d.photo.height()),(4,2))

    def test_intentional_mapper_shutdown_does_not_stop_capture(self):
        d=self.dashboard
        with tempfile.TemporaryDirectory() as directory:
            d.directory=Path(directory);d.process=Mock();d.process.poll.return_value=None;d.diagnostic=False
            (d.directory/'live-status.json').write_text(json.dumps(dict(stage='mapping',durationSeconds=90,elapsedSeconds=40,remainingSeconds=50,updatedMonotonic=time.monotonic())))
            (d.directory/'hybrid-tracking-failure.json').write_text('{}')
            (d.directory/'mapping-fallback.json').write_text('{}')
            (d.directory/'mapping.log').write_text('[ERROR] process has died [exit code -2, cmd mapper]\n')
            with patch.object(d,'stop') as stop:
                d.poll();stop.assert_not_called()
            self.assertIn('KAYIT SÜRÜYOR',d.state.cget('text'))
            self.assertIn('görüntü kaydı sürüyor',d.health.cget('text'))
            d.process=None

    def test_capture_error_after_tracking_loss_remains_prominent(self):
        d=self.dashboard
        with tempfile.TemporaryDirectory() as directory:
            d.directory=Path(directory);d.diagnostic=False;d.process=Mock()
            d.process.poll.return_value=1;d.process.returncode=1
            summary=dict(accumulatedGraphPresent=False,graph={},captureContinuedAfterTrackingLoss=True,
                recordingComplete=False,recordingPairs=24,captureReportPresent=True,shutdownClean=True,
                hybridTrackingFailure=dict(error='tracking lost'),sessionError='source failed',captureError='USB disconnected')
            (d.directory/'summary.json').write_text(json.dumps(summary))
            d.poll()
            self.assertIn('KAYIT HATASI',d.state.cget('text'))
            self.assertIn('USB disconnected',d.detail.cget('text'))
            self.assertEqual(d.counter.cget('text'),'Kayıt tamamlanamadı')

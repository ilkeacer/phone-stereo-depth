import unittest
import json
import tempfile
from pathlib import Path
from unittest.mock import patch
from types import SimpleNamespace
from host.motion_capture import CaptureWriter,authorized_devices,packet_reason
from host.motion_guide import CaptureGuide
from host.motion_recording import committed_pairs


class MotionCaptureTests(unittest.TestCase):
    def test_adb_device_parser_accepts_spaces_or_tabs(self):
        header='List of devices attached\n'
        self.assertEqual(authorized_devices(header+'abc device usb:1\n'),['abc'])
        self.assertEqual(authorized_devices(header+'abc\tdevice product:x\n'),['abc'])
        self.assertEqual(authorized_devices(header+'abc unauthorized usb:1\n'),[])

    def pair(self,*,paired=True,source='run',timestamps=(100,110),age=.1,focus=(1.4,0.)):
        header={'paired':paired}
        for camera,value in zip(('20','21'),focus):
            header[camera]={'image':{'width':1280,'height':960},
                            'capture':{'crop':f'crop{camera}','focusDiopters':value}}
        return SimpleNamespace(header=header,source=source,timestamps=timestamps,age_upper=lambda now:age)

    def test_packet_admission_and_reasons(self):
        report={'geometrySignature':[['crop20',1.4],['crop21',0.]]}
        self.assertIsNone(packet_reason(self.pair(),report,(1280,960),None,None,1.))
        self.assertEqual(packet_reason(self.pair(paired=False),report,(1280,960),None,None,1.),'unpaired')
        self.assertEqual(packet_reason(self.pair(source='new'),report,(1280,960),'old',None,1.),'source_changed')
        self.assertEqual(packet_reason(self.pair(timestamps=(100,120)),report,(1280,960),'run',(100,110),1.),'non_increasing_timestamp')
        self.assertEqual(packet_reason(self.pair(age=1.01),report,(1280,960),None,None,1.),'stale')
        self.assertEqual(packet_reason(self.pair(focus=(float('nan'),0)),report,(1280,960),None,None,1.),'geometry_mismatch')

    def test_invalid_geometry_or_source_is_not_hidden_by_unpaired(self):
        report={'geometrySignature':[['crop20',1.4],['crop21',0.]]}
        pair=self.pair(paired=False,source='new')
        self.assertEqual(packet_reason(pair,report,(1280,960),'old',None,1.),'source_changed')
        pair=self.pair(paired=False,focus=(float('nan'),0))
        self.assertEqual(packet_reason(pair,report,(1280,960),None,None,1.),'geometry_mismatch')

    def test_capture_writer_preserves_source_and_cancelled_frames(self):
        with tempfile.TemporaryDirectory() as directory:
            path=Path(directory)/'record';writer=CaptureWriter(path,{},dict(calibrationSha256='test'))
            pair=self.pair()
            for camera,stamp in zip(('20','21'),(100,110)):
                pair.header[camera]['image'].update(sequence=1,imageTimestampNs=stamp)
                pair.header[camera]['capture']['sensorTimestampNs']=stamp
            pair.blobs=(b'left',b'right');pair.read_started=1.;pair.received=1.1
            pair.callback_age_upper=lambda now:.2
            writer.save(pair,'sabit',1.2);writer.finish('cancelled')
            result=json.loads((path/'manifest.json').read_text())
            self.assertEqual(result['status'],'cancelled');self.assertEqual(result['frames'],1)
            self.assertEqual(result['jpegBytes'],9)
            self.assertEqual((path/'camera_20_1_100.jpg').read_bytes(),b'left')
            self.assertEqual(json.loads((path/'frames.jsonl').read_text())['sourceTimestampsNs'],[100,110])
            self.assertEqual(len(list(committed_pairs(path))),1)
            with self.assertRaises(FileExistsError):CaptureWriter(path,{})

    def test_quota_rejection_happens_before_image_write(self):
        with tempfile.TemporaryDirectory() as directory:
            path=Path(directory)/'record';writer=CaptureWriter(path,{})
            writer.bytes=128*1024*1024
            pair=self.pair();pair.blobs=(b'left',b'right')
            with self.assertRaises(RuntimeError):writer.save(pair,'sabit',1.)
            self.assertEqual(list(committed_pairs(path)),[])
            self.assertEqual(list(path.glob('*.jpg')),[])

    def test_failed_pair_never_enters_commit_index(self):
        with tempfile.TemporaryDirectory() as directory:
            path=Path(directory)/'record';writer=CaptureWriter(path,{})
            pair=self.pair();pair.blobs=(b'left',b'right');pair.read_started=1.;pair.received=1.1
            pair.callback_age_upper=lambda now:.2
            for camera,stamp in zip(('20','21'),(100,110)):
                pair.header[camera]['image'].update(sequence=1,imageTimestampNs=stamp)
                pair.header[camera]['capture']['sensorTimestampNs']=stamp
            with patch('host.motion_capture.atomic_json',side_effect=OSError('disk full')):
                with self.assertRaises(OSError):writer.save(pair,'sabit',1.2)
            self.assertEqual(writer.frames,0)
            self.assertEqual(list(committed_pairs(path)),[])
            self.assertEqual(len(list(path.glob('*.jpg'))),2) # recoverable, not audited


class GuideTests(unittest.TestCase):
    def test_pause_gap_and_duplicate_click_never_advance_stage(self):
        guide=CaptureGuide(duration=1.)
        self.assertTrue(guide.command('start',0))
        self.assertFalse(guide.command('start',0))
        guide.saved(1.);guide.saved(1.25)
        self.assertTrue(guide.command('pause',1))
        guide.saved(20.)
        self.assertEqual(guide.counts,[2,0,0,0]);self.assertEqual(guide.elapsed,.25)
        self.assertTrue(guide.command('start',2))
        guide.saved(30.);self.assertEqual(guide.elapsed,.25)
        guide.saved(31.);self.assertEqual(guide.elapsed,.25)
        guide.saved(31.25);guide.saved(31.5);self.assertTrue(guide.saved(31.75))
        self.assertFalse(guide.active);self.assertEqual(guide.index,1)
        self.assertFalse(guide.command('start',2))

    def test_each_stage_needs_explicit_start_and_interruption_breaks_time(self):
        guide=CaptureGuide(duration=.25)
        now=1.
        for index in range(4):
            self.assertFalse(guide.active)
            guide.command('start',guide.revision)
            guide.saved(now);guide.interrupt();guide.saved(now+.1)
            self.assertEqual(guide.elapsed,0.)
            self.assertTrue(guide.saved(now+.35));now+=1
            self.assertEqual(guide.index,index+1)
        self.assertTrue(guide.finished);self.assertFalse(guide.command('start',guide.revision))


if __name__=='__main__':unittest.main()

class MappingRecordingQuotaTests(unittest.TestCase):
    def test_mapping_can_use_separate_bound_without_changing_diagnostic_default(self):
        with tempfile.TemporaryDirectory() as tmp:
            default=CaptureWriter(Path(tmp)/'diagnostic',{})
            mapping=CaptureWriter(Path(tmp)/'mapping',{},max_frames=12000,max_bytes=1024**3)
            self.assertEqual(default.max_frames,600)
            self.assertEqual(default.max_bytes,128*1024**2)
            mapping.frames=12000
            with self.assertRaises(RuntimeError):mapping.save(SimpleNamespace(blobs=(b'left',b'right')),'room_mapping',1.)
            self.assertEqual(list(mapping.directory.glob('*.jpg')),[])

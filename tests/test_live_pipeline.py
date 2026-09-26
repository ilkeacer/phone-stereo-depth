import copy
import json
import queue
import socket
import struct
import subprocess
import sys
import threading
import time
import unittest
from unittest.mock import patch
import numpy as np

from host.contracts import ReceivedPair, check_geometry
from host.pipeline import LivePipeline, LatestOutputs
from host.transport import read_pair
from host.device import run_command


def packet(left=1_000_000_000, right=None, source='run-a', age_ns=0):
    right = left + 1_000_000 if right is None else right
    now = time.monotonic_ns()
    h = dict(ok=True, paired=abs(left-right) <= 20_000_000,
             deltaNs=abs(left-right), sourceRun=source, deviceElapsedNs=now)
    for c, ts in zip(('20', '21'), (left, right)):
        h[c] = dict(length=3, image=dict(imageTimestampNs=ts, arrivalElapsedNs=now-age_ns,
                                        width=2, height=2),
                    capture=dict(sensorTimestampNs=ts, crop='fixed', focusDiopters=0.))
    return h, [b'jpg', b'jpg']


def result(blobs):
    im = np.zeros((2, 2), np.uint8)
    return [im, im], np.ones((2, 2), np.float32), np.ones((2, 2), bool), 1.


def wait_for(predicate, timeout=3.):
    deadline = time.monotonic()+timeout
    while time.monotonic() < deadline:
        if predicate():
            return
        time.sleep(.005)
    raise AssertionError('Timed out waiting for pipeline condition')


class FakeSource:
    def __init__(self):
        self.queue = queue.Queue()

    def __call__(self):
        try:
            item = self.queue.get(timeout=.01)
        except queue.Empty:
            return None
        if isinstance(item, Exception):
            raise item
        return item


class TransportTests(unittest.TestCase):
    def test_device_command_can_be_cancelled_during_startup(self):
        stop = threading.Event()
        timer = threading.Timer(.15, stop.set)
        timer.start();started=time.monotonic()
        try:
            with self.assertRaises(InterruptedError):
                run_command([sys.executable,'-c','import time; time.sleep(10)'],stop)
            self.assertLess(time.monotonic()-started, 2.)
        finally:timer.cancel()

    def test_transport_import_has_no_gui_or_calibration_dependency(self):
        code = "import sys; import host.transport; assert 'tkinter' not in sys.modules; assert 'host.calibrate' not in sys.modules"
        subprocess.run([sys.executable, '-c', code], check=True)

    def test_split_packet_reads_complete_payload(self):
        h, blobs = packet()
        header = json.dumps(h).encode()
        wire = struct.pack('>I', len(header))+header+b''.join(blobs)
        class Fragmented:
            def __enter__(self):return self
            def __exit__(self, *args):pass
            def recv(self, length):
                nonlocal wire
                part, wire = wire[:min(length, 7)], wire[min(length, 7):]
                return part
        with patch('host.transport.socket.create_connection', return_value=Fragmented()):
            self.assertEqual(read_pair(), (h, blobs))

    def test_invalid_headers_and_truncated_body_rejected(self):
        original, blobs = packet()
        cases = []
        h = copy.deepcopy(original);h['paired'] = 'yes';cases.append(h)
        h = copy.deepcopy(original);h['deltaNs'] += 1;cases.append(h)
        h = copy.deepcopy(original);h['20']['length'] = 9_000_000;cases.append(h)
        h = copy.deepcopy(original);h['21']['capture']['sensorTimestampNs'] += 1;cases.append(h)
        for header in cases + [original]:
            a, b = socket.socketpair()
            try:
                data = json.dumps(header).encode()
                a.sendall(struct.pack('>I', len(data))+data+b'j');a.shutdown(socket.SHUT_WR)
                with patch('host.transport.socket.create_connection', return_value=b):
                    with self.assertRaises((ValueError, ConnectionError)):read_pair()
            finally:
                a.close();b.close()

    def test_inertial_batch_uses_device_clock_and_bounded_finite_samples(self):
        h,blobs=packet()
        h['imuSamples']=[dict(sequence=1,kind='gyro',timestampNs=h['deviceElapsedNs']-1_000_000,
                              x=0.1,y=0.2,z=0.3,accuracy=3)]
        h['imuDroppedTotal']=0
        self.assertEqual(ReceivedPair.create((h,blobs),10.,10.1).header['imuSamples'][0]['x'],0.1)
        for change in (dict(timestampNs=h['deviceElapsedNs']+1),dict(x=float('nan')),
                       dict(kind='unknown'),dict(sequence=0)):
            bad=copy.deepcopy(h);bad['imuSamples'][0].update(change)
            with self.assertRaises(ValueError):ReceivedPair.create((bad,blobs),10.,10.1)

    def test_large_header_rejected_before_allocation(self):
        a, b = socket.socketpair()
        try:
            a.sendall(struct.pack('>I', 65_537))
            with patch('host.transport.socket.create_connection', return_value=b):
                with self.assertRaises(ValueError):read_pair()
        finally:
            a.close();b.close()

    def test_callback_age_includes_usb_and_processing_but_is_not_exposure_age(self):
        frame = ReceivedPair.create(packet(age_ns=200_000_000), 10., 10.1)
        self.assertAlmostEqual(frame.callback_age_upper(10.3), .5)
        h, blobs = packet();h['20']['image']['arrivalElapsedNs'] = h['deviceElapsedNs']+1
        with self.assertRaises(ValueError):ReceivedPair.create((h, blobs), 10., 10.1)

    def test_nan_focus_rejected(self):
        h, _ = packet();h['20']['capture']['focusDiopters'] = float('nan')
        with self.assertRaises(ValueError):
            check_geometry(h, {'geometrySignature': [['fixed', 0.], ['fixed', 0.]]}, (2, 2))

    def test_realtime_source_age_detects_sensor_to_callback_backlog(self):
        h,blobs=packet()
        for c in ('20','21'):
            h[c]['image']['timestampSource']=1
            h[c]['image']['imageTimestampNs']=h['deviceElapsedNs']-2_000_000_000
            h[c]['capture']['sensorTimestampNs']=h[c]['image']['imageTimestampNs']
        h['deltaNs']=0
        f=ReceivedPair.create((h,blobs),10.,10.1)
        self.assertAlmostEqual(f.callback_age_upper(10.2),.2)
        self.assertAlmostEqual(f.age_upper(10.2),2.2)
        for c in ('20','21'):h[c]['image']['timestampSource']=0
        f=ReceivedPair.create((h,blobs),10.,10.1)
        self.assertIsNone(f.sensor_age_seconds)
        self.assertAlmostEqual(f.age_upper(10.2),.2)


class PipelineTests(unittest.TestCase):
    def setUp(self):
        self.source = FakeSource()
        self.release = threading.Event()
        self.pipelines = []

    def tearDown(self):
        self.release.set()
        for p in self.pipelines:
            p.request_stop();p.join();p.diagnostics.close()

    def start(self, process=result, **kwargs):
        p = LivePipeline(process, lambda h: None, reader=self.source, warmup_seconds=0,
                         poll_seconds=.001, **kwargs)
        self.pipelines.append(p);p.start()
        return p

    def count(self, p, kind):
        return p.diagnostics.snapshot()['counts'].get(kind, 0)

    def test_blocked_compute_does_not_block_receiver_and_only_latest_job_waits(self):
        entered = threading.Event()
        def slow(blobs):
            entered.set();self.release.wait(2)
            return result(blobs)
        p = self.start(slow)
        self.source.queue.put(packet());self.assertTrue(entered.wait(1))
        for i in range(1, 9):self.source.queue.put(packet(1_000_000_000+i*70_000_000))
        wait_for(lambda:self.count(p, 'received') == 9)
        self.assertEqual(self.count(p, 'preview'), 9)
        self.assertEqual(self.count(p, 'pending_replaced'), 7)
        with p.condition:
            self.assertEqual(p.pending[0].timestamps[0], 1_560_000_000)
        self.release.set();wait_for(lambda:self.count(p, 'depth') == 2)
        self.assertEqual(self.count(p, 'eligible'), 9)

    def test_unpaired_preview_and_reused_right_frame_never_reach_compute(self):
        p = self.start()
        self.source.queue.put(packet());wait_for(lambda:self.count(p, 'depth') == 1)
        self.source.queue.put(packet(1_002_000_000, 1_001_000_000))
        self.source.queue.put(packet(1_070_000_000, 1_100_000_000))
        wait_for(lambda:self.count(p, 'received') == 3)
        self.assertEqual(self.count(p, 'duplicate_pair'), 1)
        self.assertEqual(self.count(p, 'unpaired'), 1)
        self.assertEqual(self.count(p, 'depth'), 1)
        self.assertEqual(self.count(p, 'preview'), 3)

    def test_session_change_discards_inflight_result(self):
        entered = threading.Event()
        def slow(blobs):
            entered.set();self.release.wait(2)
            return result(blobs)
        p = self.start(slow)
        self.source.queue.put(packet());self.assertTrue(entered.wait(1))
        self.source.queue.put(packet(source='run-b'))
        wait_for(lambda:self.count(p, 'session_started') == 2)
        self.release.set();wait_for(lambda:self.count(p, 'depth') == 1)
        self.assertEqual(self.count(p, 'obsolete_result'), 1)
        depth = p.outputs.drain()['depth']
        self.assertEqual(depth[-1]['sourceRun'], 'run-b')

    def test_disconnect_clears_pending_and_invalidates_inflight(self):
        entered = threading.Event()
        def slow(blobs):
            entered.set();self.release.wait(2)
            return result(blobs)
        p = self.start(slow)
        self.source.queue.put(packet());self.assertTrue(entered.wait(1))
        self.source.queue.put(ConnectionError('unplugged'))
        wait_for(lambda:self.count(p, 'connection_error') == 1)
        self.release.set();wait_for(lambda:self.count(p, 'obsolete_result') == 1)
        self.source.queue.put(packet(1_070_000_000))
        wait_for(lambda:self.count(p, 'depth') == 1)
        self.assertIsNone(p.error)

    def test_stale_callback_does_not_become_fresh_when_processing_finishes(self):
        p = self.start()
        self.source.queue.put(packet(age_ns=2_000_000_000))
        wait_for(lambda:self.count(p, 'stale_input') == 1)
        self.assertEqual(self.count(p, 'eligible'), 0)
        self.assertEqual(self.count(p, 'depth'), 0)

    def test_old_sessions_and_out_of_order_frames_are_not_replayed(self):
        p = self.start()
        self.source.queue.put(packet(1_140_000_000));wait_for(lambda:self.count(p, 'depth') == 1)
        self.source.queue.put(packet(1_070_000_000))
        wait_for(lambda:self.count(p, 'out_of_order') == 1)
        self.source.queue.put(packet(source='run-b'));wait_for(lambda:self.count(p, 'depth') == 2)
        self.source.queue.put(packet(1_210_000_000, source='run-a'))
        wait_for(lambda:self.count(p, 'retired_session_packet') == 1)
        self.assertEqual(self.count(p, 'depth'), 2)
        self.assertEqual(p.source, 'run-b')

    def test_old_completion_is_not_published(self):
        entered = threading.Event()
        def slow(blobs):
            entered.set();self.release.wait(2)
            return result(blobs)
        p = self.start(slow, max_age_seconds=.05)
        self.source.queue.put(packet());self.assertTrue(entered.wait(1))
        time.sleep(.06);self.release.set()
        wait_for(lambda:self.count(p, 'stale_result') == 1)
        self.assertNotIn('depth', p.outputs.drain())

    def test_processing_failure_is_visible_and_stops_receiver(self):
        def broken(blobs):raise ValueError('bad geometry')
        p = self.start(broken)
        self.source.queue.put(packet());self.assertTrue(p.stop.wait(1))
        p.join()
        self.assertEqual(p.error, 'bad geometry')
        self.assertIn('bad geometry', p.outputs.drain()['status'][1])

    def test_startup_settling_allows_preview_but_never_bypasses_geometry_gate(self):
        report={'geometrySignature': [['fixed', 0.], ['fixed', 0.]]}
        p=LivePipeline(result,lambda h:check_geometry(h,report,(2,2)), reader=self.source,
                       warmup_seconds=.04,poll_seconds=.001)
        self.pipelines.append(p);p.start()
        h,blobs=packet();h['20']['capture']['focusDiopters']=1.4
        self.source.queue.put((h,blobs))
        wait_for(lambda:self.count(p,'warmup')==1)
        self.assertIsNone(p.error);self.assertEqual(self.count(p,'depth'),0)
        time.sleep(.05)
        self.source.queue.put(packet(1_070_000_000))
        wait_for(lambda:self.count(p,'depth')==1)
        h,blobs=packet(1_140_000_000);h['20']['capture']['focusDiopters']=1.4
        self.source.queue.put((h,blobs));self.assertTrue(p.stop.wait(1))
        self.assertIn('kalibrasyonla farklı',p.error)

    def test_status_preview_and_depth_have_independent_slots(self):
        out = LatestOutputs()
        out.put(('status', 'error'));out.put(('depth', 1))
        for i in range(100):out.put(('preview_packet', i))
        items = out.drain()
        self.assertEqual(len(items), 3)
        self.assertEqual(items['status'][1], 'error')
        self.assertEqual(items['depth'][1], 1)
        self.assertEqual(items['preview_packet'][1], 99)
        self.assertEqual(out.drain(), {})

    def test_quality_counters_travel_with_their_exact_pair(self):
        quality={'totalPixels':4,'stages':{'positiveFiniteZ':4}}
        p=self.start(lambda blobs:(*result(blobs),quality))
        self.source.queue.put(packet(left=2_000_000_000))
        wait_for(lambda:self.count(p,'depth')==1)
        depth=p.outputs.drain()['depth']
        self.assertEqual(depth[-1]['leftTimestampNs'],2_000_000_000)
        self.assertEqual(depth[-1]['quality'],quality)

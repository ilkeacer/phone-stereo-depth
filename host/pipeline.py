"""Independent v1 receiver and depth worker. One pending job, no Tk dependency."""
import threading
import time
from collections import deque
from host.contracts import ReceivedPair, MAX_PAIR_DELTA_NS
from host.diagnostics import Diagnostics
from host.transport import read_pair


class LatestOutputs:
    """Independent last-value channels; status is not evicted by frame traffic."""
    def __init__(self):
        self.lock = threading.Lock()
        self.items = {}

    def put(self, item):
        with self.lock:
            self.items[item[0]] = item

    def drain(self):
        with self.lock:
            result = self.items
            self.items = {}
            return result

    def clear(self):
        with self.lock:
            self.items.clear()


class LivePipeline:
    def __init__(self, process, validate, reader=read_pair, diagnostics=None,
                 outputs=None, warmup_seconds=4., poll_seconds=.06, max_age_seconds=1.):
        self.process, self.validate, self.reader = process, validate, reader
        self.diagnostics = diagnostics or Diagnostics()
        self.outputs = outputs or LatestOutputs()
        self.warmup_seconds, self.poll_seconds, self.max_age_seconds = warmup_seconds, poll_seconds, max_age_seconds
        self.stop = threading.Event()
        self.condition = threading.Condition()
        self.pending = None
        self.epoch = 0
        self.source = None
        self.retired_sources = deque(maxlen=32)
        self.source_started = 0.
        self.last_preview = (-1, -1)
        self.last_pair = (-1, -1)
        self.connected = False
        self.waiting_reported = False
        self.error = None
        self.threads = []

    def start(self):
        if self.threads or self.stop.is_set():
            raise RuntimeError('Pipeline instances cannot be restarted')
        self.threads = [threading.Thread(target=self._receive, name='stereo-receiver', daemon=True),
                        threading.Thread(target=self._compute, name='stereo-depth', daemon=True)]
        for thread in self.threads:
            thread.start()

    def request_stop(self):
        self.stop.set()
        with self.condition:
            self.condition.notify_all()

    def join(self, timeout=5.):
        deadline = time.monotonic() + timeout
        for thread in self.threads:
            thread.join(max(0., deadline - time.monotonic()))
        if any(thread.is_alive() for thread in self.threads):
            raise RuntimeError('Akış işçisi henüz kapanmadı.')

    def _invalidate(self, status):
        # Caller holds condition. An in-flight result from this epoch is invalid.
        self.epoch += 1
        self.pending = None
        self.outputs.clear()
        self.outputs.put(('reset',))
        self.outputs.put(('status', status))

    def _fail(self, error):
        with self.condition:
            self.error = str(error)
            self._invalidate(f'Canlı akış durdu: {error}')
            self.diagnostics.event('fatal', reason=str(error))
        self.request_stop()

    def _receive(self):
        try:
            while not self.stop.is_set():
                start = time.monotonic()
                try:
                    packet = self.reader()
                except OSError as error:
                    self.diagnostics.event('connection_error', reason=str(error))
                    with self.condition:
                        if self.connected:
                            self.connected = False
                            self._invalidate('USB bağlantısı bekleniyor…')
                        elif not self.waiting_reported:
                            self.outputs.put(('status','Kamera görüntüsü bekleniyor; telefon ekranı ve uygulama açık olmalı.'))
                        self.waiting_reported = True
                    self.stop.wait(.1)
                    continue
                if self.stop.is_set():
                    break
                if packet is None:
                    self.diagnostics.event('waiting_packet')
                    with self.condition:
                        if not self.waiting_reported:
                            self.outputs.put(('status','Kamera görüntüsü bekleniyor; telefon ekranı ve uygulama açık olmalı.'))
                            self.waiting_reported = True
                    self.stop.wait(self.poll_seconds)
                    continue
                frame = ReceivedPair.create(packet, start, time.monotonic())
                self.diagnostics.event('received', sourceRun=frame.source,
                                       leftTimestampNs=frame.timestamps[0], rightTimestampNs=frame.timestamps[1],
                                       deltaMs=abs(frame.timestamps[0]-frame.timestamps[1])/1e6,
                                       readMs=(frame.received-start)*1000,
                                       sensorAgeAtSnapshotMs=frame.sensor_age_seconds*1000 if frame.sensor_age_seconds is not None else None,
                                       callbackAgeAtSnapshotMs=frame.callback_age_seconds*1000)
                with self.condition:
                    if frame.source != self.source:
                        if frame.source in self.retired_sources:
                            self.diagnostics.event('retired_session_packet')
                            continue
                        if self.source is not None:
                            self.retired_sources.append(self.source)
                        self._invalidate('Yeni kamera oturumu; uygun çift bekleniyor…')
                        self.source = frame.source
                        self.source_started = frame.received
                        self.last_preview = self.last_pair = (-1, -1)
                        self.diagnostics.event('session_started', sourceRun=frame.source)
                    if self.waiting_reported:
                        self.outputs.put(('status','Kamera görüntüleri geldi; uygun derinlik çifti bekleniyor…'))
                        self.waiting_reported = False
                    self.connected = True
                    warming_up = frame.received-self.source_started < self.warmup_seconds
                    # Focus/crop metadata can be transitional immediately after
                    # opening Camera2. Preview is allowed during the settle time;
                    # no depth is admitted until that time AND validation pass.
                    if not warming_up:
                        self.validate(frame.header)
                    if frame.age_upper(time.monotonic()) > self.max_age_seconds:
                        self.diagnostics.event('stale_input')
                    elif any(ts < old for ts, old in zip(frame.timestamps, self.last_preview)):
                        self.diagnostics.event('out_of_order')
                    else:
                        if frame.timestamps != self.last_preview:
                            self.last_preview = frame.timestamps
                            self.outputs.put(('preview_packet', frame, self.epoch))
                            self.diagnostics.event('preview')
                        eligible = (frame.header['paired'] and
                                    abs(frame.timestamps[0]-frame.timestamps[1]) <= MAX_PAIR_DELTA_NS)
                        if not eligible:
                            self.diagnostics.event('unpaired')
                        elif warming_up:
                            self.diagnostics.event('warmup')
                        elif any(ts <= old for ts, old in zip(frame.timestamps, self.last_pair)):
                            self.diagnostics.event('duplicate_pair')
                        else:
                            self.last_pair = frame.timestamps
                            if self.pending is not None:
                                self.diagnostics.event('pending_replaced')
                            self.pending = (frame, self.epoch)
                            self.diagnostics.event('eligible')
                            self.condition.notify()
                self.stop.wait(self.poll_seconds)
        except Exception as error:
            self._fail(error)

    def _compute(self):
        try:
            while not self.stop.is_set():
                with self.condition:
                    self.condition.wait_for(lambda: self.pending is not None or self.stop.is_set())
                    if self.stop.is_set():
                        break
                    frame, epoch = self.pending
                    self.pending = None
                started = time.monotonic()
                if frame.age_upper(started) > self.max_age_seconds:
                    self.diagnostics.event('stale_before_compute')
                    continue
                result = self.process(frame.blobs)
                rect, z, mask, matcher_ms = result[:4]
                quality = result[4] if len(result)==5 else None
                completed = time.monotonic()
                with self.condition:
                    if self.stop.is_set() or epoch != self.epoch:
                        self.diagnostics.event('obsolete_result')
                        continue
                    age = frame.age_upper(completed)
                    if age > self.max_age_seconds:
                        self.diagnostics.event('stale_result')
                        continue
                    record = dict(sourceRun=frame.source, leftTimestampNs=frame.timestamps[0],
                                  rightTimestampNs=frame.timestamps[1], deltaMs=abs(frame.timestamps[0]-frame.timestamps[1])/1e6,
                                  matcherMs=matcher_ms, processingMs=(completed-started)*1000,
                                  readMs=(frame.received-frame.read_started)*1000,
                                  queueWaitMs=(started-frame.received)*1000,
                                  callbackAgeUpperMs=frame.callback_age_upper(completed)*1000,
                                  freshnessAgeUpperMs=age*1000,
                                  sensorAgeUpperMs=age*1000 if frame.sensor_age_seconds is not None else None,
                                  hostCompletionMonotonic=completed,
                                  validPixelRatio=float(mask.mean()), epoch=epoch,
                                  unit='checker_square', metricScaleVerified=False, endToEndLatencyMeasured=False)
                    if quality is not None:record['quality']=quality
                    self.diagnostics.event('depth', **record)
                    self.outputs.put(('depth', rect, z, mask, record))
        except Exception as error:
            self._fail(error)

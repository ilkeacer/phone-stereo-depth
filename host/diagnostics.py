"""Streaming numeric evidence with bounded in-memory timing samples."""
from collections import Counter, deque
import json
import statistics
import threading
import time


class Diagnostics:
    def __init__(self, path=None):
        self.started = time.monotonic()
        self.lock = threading.Lock()
        self.counts = Counter()
        self.last_depth = None
        self.first_depth = None
        self.max_gap = 0.
        self.samples = {}
        self.file = open(path, 'w', buffering=1) if path else None

    def event(self, kind, **fields):
        with self.lock:
            now = time.monotonic()
            self.counts[kind] += 1
            if kind == 'depth':
                if self.first_depth is None:
                    self.first_depth = now
                if self.last_depth is not None:
                    self.max_gap = max(self.max_gap, now - self.last_depth)
                self.last_depth = now
                for key in ('matcherMs', 'processingMs', 'queueWaitMs', 'readMs', 'callbackAgeUpperMs', 'deltaMs', 'sensorAgeUpperMs'):
                    if fields.get(key) is not None:
                        self.samples.setdefault(key, deque(maxlen=512)).append(fields[key])
            if self.file:
                self.file.write(json.dumps(dict(event=kind, hostMonotonic=now, **fields), allow_nan=False) + '\n')

    def snapshot(self):
        with self.lock:
            now = time.monotonic()
            duration = now - self.started
            leading = (self.first_depth or now) - self.started
            trailing = now - (self.last_depth or self.started)
            return dict(totalSeconds=duration, counts=dict(self.counts),
                        averageDepthUpdatesHzIncludingWaits=self.counts['depth'] / max(duration, 1e-9),
                        maxGapBetweenDepthUpdatesSeconds=self.max_gap,
                        secondsUntilFirstDepth=leading, secondsAfterLastDepth=trailing,
                        maxNoDepthIntervalIncludingEdgesSeconds=max(leading, trailing, self.max_gap),
                        mediansLast512={key: statistics.median(values) for key, values in self.samples.items()},
                        metricScaleVerified=False, endToEndLatencyMeasured=False,
                        ageMeaning='callbackAgeUpperMs excludes sensor-to-callback delay; sensorAgeUpperMs uses declared Android REALTIME clock when available; neither measures display latency')

    def close(self):
        with self.lock:
            if self.file:
                self.file.close()
                self.file = None

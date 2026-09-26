"""Mapping duration starts on the first admitted pair, after camera settling."""
import math


class MappingClock:
    def __init__(self, duration):
        self.duration = duration
        self.started = None

    def admit(self, now):
        if self.started is None:
            self.started = now

    def elapsed(self, now):
        return 0. if self.started is None else max(0., now-self.started)

    def expired(self, now):
        return self.started is not None and self.elapsed(now) >= self.duration

    def status(self, now):
        elapsed = self.elapsed(now)
        return dict(durationSeconds=self.duration, elapsedSeconds=elapsed,
                    remainingSeconds=max(0, math.ceil(self.duration-elapsed)))

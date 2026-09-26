"""Camera packet contracts without GUI, ROS or CUDA dependencies."""
from dataclasses import dataclass
import math

CAMERAS = ('20', '21')
MAX_PAIR_DELTA_NS = 20_000_000


def integer(value, name, minimum=0):
    if type(value) is not int or value < minimum:
        raise ValueError(f'Invalid {name}')
    return value


def validate_header(header):
    if not isinstance(header, dict) or type(header.get('ok')) is not bool:
        raise ValueError('Invalid packet status')
    if not header['ok']:
        return
    if type(header.get('paired')) is not bool:
        raise ValueError('Missing or invalid pair eligibility')
    try:
        timestamps = []
        for camera in CAMERAS:
            row = header[camera]
            ts = integer(row['image']['imageTimestampNs'], 'image timestamp')
            if integer(row['capture']['sensorTimestampNs'], 'capture timestamp') != ts:
                raise ValueError('Metadata timestamp mismatch')
            length = integer(row['length'], 'JPEG length', 1)
            if length > 8_000_000:
                raise ValueError('JPEG too large')
            timestamps.append(ts)
        delta = abs(timestamps[0] - timestamps[1])
        if 'deltaNs' in header and integer(header['deltaNs'], 'pair delta') != delta:
            raise ValueError('Incorrect pair delta')
        if header['paired'] and delta > MAX_PAIR_DELTA_NS:
            raise ValueError('Pair exceeds timestamp threshold')
        if 'imuSamples' in header:
            samples=header['imuSamples']
            if not isinstance(samples,list) or len(samples)>128:
                raise ValueError('Invalid inertial batch')
            snapshot=integer(header.get('deviceElapsedNs'),'device elapsed time')
            previous_sequence=0
            for sample in samples:
                if not isinstance(sample,dict):raise ValueError('Invalid inertial sample')
                sequence=integer(sample.get('sequence'),'inertial sequence',1)
                timestamp=integer(sample.get('timestampNs'),'inertial timestamp',1)
                if sequence<=previous_sequence or timestamp>snapshot or sample.get('kind') not in ('gyro','accel'):
                    raise ValueError('Invalid inertial ordering or type')
                previous_sequence=sequence
                for axis in ('x','y','z'):
                    value=sample.get(axis)
                    if type(value) not in (int,float) or not math.isfinite(value):
                        raise ValueError('Invalid inertial axis')
                if type(sample.get('accuracy')) is not int:
                    raise ValueError('Invalid inertial accuracy')
            integer(header.get('imuDroppedTotal'),'dropped inertial samples')
    except (KeyError, TypeError) as error:
        raise ValueError('Incomplete camera packet') from error


def check_geometry(header, report, size):
    expected = report['geometrySignature']
    for i, camera in enumerate(CAMERAS):
        row, meta = header[camera]['image'], header[camera]['capture']
        if (row['width'], row['height']) != tuple(size):
            raise ValueError('Görüntü boyutu kalibrasyonla farklı.')
        focus = meta['focusDiopters']
        # NaN must fail this comparison too.
        if meta['crop'] != expected[i][0] or focus is None or not abs(focus - expected[i][1]) <= .01:
            raise ValueError('Kamera odağı veya kırpması kalibrasyonla farklı.')


@dataclass(frozen=True)
class ReceivedPair:
    header: dict
    blobs: tuple
    read_started: float
    received: float
    source: str
    timestamps: tuple
    callback_age_seconds: float
    sensor_age_seconds: float | None = None

    @classmethod
    def create(cls, packet, started, received):
        header, blobs = packet
        validate_header(header)
        source = header.get('sourceRun')
        if not header['ok'] or not isinstance(source, str) or not source:
            raise ValueError('Missing camera session')
        if len(blobs) != 2 or any(len(b) != header[c]['length'] for c, b in zip(CAMERAS, blobs)):
            raise ValueError('JPEG payload length mismatch')
        now = integer(header.get('deviceElapsedNs'), 'device elapsed time')
        ages = []
        for camera in CAMERAS:
            arrival = integer(header[camera]['image'].get('arrivalElapsedNs'), 'callback time')
            if arrival > now:
                raise ValueError('Camera callback is later than packet snapshot')
            ages.append((now - arrival) / 1e9)
        timestamps=tuple(header[c]['image']['imageTimestampNs'] for c in CAMERAS)
        sensor_age=None
        # Android REALTIME timestamps share elapsedRealtimeNanos' timebase.
        # Other/absent timestamp sources are deliberately left unknown.
        if all(type(header[c]['image'].get('timestampSource')) is int and
               header[c]['image']['timestampSource']==1 for c in CAMERAS):
            if any(ts>now for ts in timestamps):raise ValueError('Future realtime sensor timestamp')
            sensor_age=max(now-ts for ts in timestamps)/1e9
        return cls(header, tuple(blobs), started, received, source,timestamps,max(ages),sensor_age)

    def age_upper(self, now):
        age=self.sensor_age_seconds if self.sensor_age_seconds is not None else self.callback_age_seconds
        return age+max(0.,now-self.read_started)

    def callback_age_upper(self, now):
        # Includes the whole USB request round trip, conservatively. This is NOT
        # exposure-to-display age: callback delivery delay is not measured here.
        return self.callback_age_seconds + max(0., now - self.read_started)

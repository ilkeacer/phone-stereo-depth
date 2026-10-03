"""Shared stereo scale, bound to a calibration; scale is not map accuracy."""
import hashlib
import json
import math
from pathlib import Path

NOMINAL_SQUARE_MM = 21.979195166653625
SOURCES = ('measured', 'nominal_display_estimate')


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def explicit_scale(square_mm, source, calibration):
    if isinstance(square_mm, bool) or not isinstance(square_mm, (int, float)) or not math.isfinite(square_mm) or square_mm <= 0:
        raise ValueError('Positive finite square size in mm required')
    if source not in SOURCES:
        raise ValueError('Unknown stereo scale source')
    calibration = Path(calibration)
    return dict(schemaVersion=1, squareMm=float(square_mm), scaleSource=source,
                measuredSquareMm=float(square_mm) if source == 'measured' else None,
                metricAccuracyValidated=False,
                calibrationSha256=sha(calibration),
                calibrationReportSha256=sha(calibration.with_suffix('.json')))


def load_scale(path, calibration):
    path = Path(path)
    data = json.loads(path.read_text())
    if not isinstance(data, dict) or data.get('schemaVersion') != 1:
        raise ValueError('Unsupported scale configuration version')
    result = explicit_scale(data['squareMm'], data['scaleSource'], calibration)
    for key in ('calibrationSha256', 'calibrationReportSha256'):
        if data.get(key) != result[key]:
            raise ValueError('Scale configuration calibration hashes differ')
    if data.get('metricAccuracyValidated') is not False:
        raise ValueError('Scale configuration cannot certify map accuracy')
    measurement = data.get('scaleMeasurement')
    if measurement is not None:
        if not isinstance(measurement, dict):
            raise ValueError('Invalid screen measurement object')
        if result['scaleSource'] != 'measured' or measurement.get('samePhysicalDisplayConfirmed') is not True:
            raise ValueError('Screen measurement needs confirmed display identity')
        spans = measurement.get('reportedApproximateSpansMm', {})
        count = measurement.get('squaresPerSpan')
        if type(count) is not int or count <= 0 or not isinstance(spans, dict) or set(spans) != {'AB', 'CD'}:
            raise ValueError('Invalid screen measurement spans')
        for value in spans.values():
            if type(value) not in (int, float) or not math.isfinite(value) or not math.isclose(value / count, result['squareMm'], rel_tol=0, abs_tol=1e-9):
                raise ValueError('Screen measurement differs from configured scale')
        result['scaleMeasurement'] = measurement
    result.update(scaleConfigPath=str(path.resolve()), scaleConfigSha256=sha(path))
    return result


def default_scale(calibration, config=None):
    """A present but invalid sidecar fails; it never silently becomes nominal."""
    path = Path(config) if config is not None else Path(calibration).with_name('scale.json')
    if config is not None or path.exists():
        return load_scale(path, calibration)
    return explicit_scale(NOMINAL_SQUARE_MM, 'nominal_display_estimate', calibration)


def add_scale_options(parser):
    group = parser.add_mutually_exclusive_group()
    group.add_argument('--scale-config', type=Path, help='Calibration-bound scale JSON (default: scale.json beside calibration)')
    group.add_argument('--square-mm', type=float, help='Explicit physically measured square edge')
    group.add_argument('--estimated-square-mm', type=float, help='Explicit unverified estimate')


def scale_from_args(args):
    if args.square_mm is not None:
        return explicit_scale(args.square_mm, 'measured', args.calibration)
    if args.estimated_square_mm is not None:
        return explicit_scale(args.estimated_square_mm, 'nominal_display_estimate', args.calibration)
    path = args.scale_config or args.calibration.with_name('scale.json')
    return load_scale(path, args.calibration)


def scale_label(scale):
    if scale.get('scaleSource') == 'arcore_depth_api':
        return 'ARCore derinliği metre biriminde · Fiziksel doğruluk henüz doğrulanmadı'
    if scale.get('scaleSource') == 'measured':
        approximate = (scale.get('scaleMeasurement') or {}).get('approximate', False)
        return f"Kare kenarı: {'yaklaşık ' if approximate else ''}{scale['squareMm']:g} mm · Fiziksel ölçüm · Harita doğruluğu henüz doğrulanmadı"
    return 'Ölçek tahmini · Harita doğruluğu henüz doğrulanmadı'


def saved_map_scale(cloud):
    """Use that export's metadata, never today's live configuration."""
    path = Path(cloud).parent/'result.json'
    if path.is_file():
        result = json.loads(path.read_text())
        if result.get('scaleSource') == 'arcore_depth_api' and result.get('metricAccuracyValidated') is False:
            return result
        if result.get('scaleSource') == 'measured' and result.get('squareMm') is not None:
            return result
    return dict(scaleSource='unknown')

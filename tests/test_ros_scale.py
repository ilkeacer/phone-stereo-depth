import argparse
import json
from pathlib import Path
import tempfile
import unittest
import numpy as np
from host.ros_scale import (explicit_scale, load_scale, default_scale,
                            add_scale_options, scale_from_args, scale_label, saved_map_scale)
from host.ros_stereo import metric_projection


class ScaleTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.root = Path(self.temp.name)
        self.cal = self.root/'calibration.npz'
        self.cal.write_bytes(b'calibration fixture')
        self.cal.with_suffix('.json').write_text('{"lengthUnit":"checker_square"}')
        self.config = self.root/'scale.json'

    def measured(self):
        data = explicit_scale(21.44, 'measured', self.cal)
        data['scaleMeasurement'] = dict(approximate=True, samePhysicalDisplayConfirmed=True,
            reportedApproximateSpansMm=dict(AB=107.2, CD=107.2), squaresPerSpan=5,
            instrumentResolutionMm=None, measurementUncertaintyMm=None)
        return data

    def write(self, data):
        self.config.write_text(json.dumps(data))

    def test_default_is_nominal_only_when_sidecar_is_absent(self):
        self.assertEqual(default_scale(self.cal)['scaleSource'], 'nominal_display_estimate')
        self.write(self.measured())
        result = default_scale(self.cal)
        self.assertEqual(result['squareMm'], 21.44)
        self.assertEqual(result['measuredSquareMm'], 21.44)
        self.assertFalse(result['metricAccuracyValidated'])
        self.assertTrue(result['scaleMeasurement']['approximate'])
        self.assertIn('yaklaşık', scale_label(result))

    def test_changed_calibration_or_report_rejected_without_fallback(self):
        for target in (self.cal, self.cal.with_suffix('.json')):
            with self.subTest(target=target):
                self.write(self.measured())
                old = target.read_bytes()
                target.write_bytes(old+b'changed')
                with self.assertRaisesRegex(ValueError, 'hashes differ'):default_scale(self.cal)
                target.write_bytes(old)

    def test_unknown_display_or_inconsistent_spans_do_not_become_measured(self):
        for key, value in [('samePhysicalDisplayConfirmed', False), ('squaresPerSpan', 10),
                           ('reportedApproximateSpansMm', dict(AB=107.2, CD=108))]:
            data = self.measured();data['scaleMeasurement'][key] = value
            self.write(data)
            with self.subTest(key=key), self.assertRaises(ValueError):load_scale(self.config, self.cal)

    def test_nan_zero_negative_boolean_scale_and_accuracy_claim_rejected(self):
        for value in (float('nan'), float('inf'), 0, -1, True):
            with self.subTest(value=value), self.assertRaises(ValueError):
                explicit_scale(value, 'measured', self.cal)
        data = self.measured();data['metricAccuracyValidated'] = True;self.write(data)
        with self.assertRaisesRegex(ValueError, 'accuracy'):load_scale(self.config, self.cal)

    def test_new_configuration_does_not_replace_explicit_historical_scale(self):
        self.write(self.measured())
        parser = argparse.ArgumentParser();add_scale_options(parser)
        args = parser.parse_args(['--estimated-square-mm', '22']);args.calibration = self.cal
        result = scale_from_args(args)
        self.assertEqual(result['squareMm'], 22)
        self.assertEqual(result['scaleSource'], 'nominal_display_estimate')
        args = parser.parse_args([]);args.calibration = self.cal
        self.assertEqual(scale_from_args(args)['squareMm'], 21.44)

    def test_projection_and_baseline_use_same_measured_unit_without_changing_intrinsics(self):
        self.write(self.measured())
        settings = load_scale(self.config, self.cal)
        p = np.array([[100.,0,30,-200],[0,100,20,0],[0,0,1,0]])
        result = metric_projection(p, settings['squareMm'])
        self.assertAlmostEqual(-result[0,3]/result[0,0], 2*21.44/1000)
        np.testing.assert_array_equal(result[:,:3], p[:,:3])
        self.assertEqual(p[0,3], -200)

    def test_explicit_missing_config_fails_instead_of_becoming_nominal(self):
        with self.assertRaises(FileNotFoundError):default_scale(self.cal, self.config)

    def test_malformed_json_structure_is_rejected(self):
        self.write([])
        with self.assertRaises(ValueError):load_scale(self.config, self.cal)
        for value in ('not an object', dict(squaresPerSpan=5, samePhysicalDisplayConfirmed=True, reportedApproximateSpansMm=None)):
            data=self.measured();data['scaleMeasurement']=value;self.write(data)
            with self.assertRaises(ValueError):load_scale(self.config, self.cal)

    def test_saved_map_does_not_inherit_current_measured_sidecar(self):
        self.write(self.measured())
        cloud=self.root/'map_cloud.ply'
        self.assertEqual(saved_map_scale(cloud)['scaleSource'],'unknown')
        result=self.root/'result.json';result.write_text(json.dumps(self.measured()))
        self.assertEqual(saved_map_scale(cloud)['squareMm'],21.44)


if __name__ == '__main__':unittest.main()

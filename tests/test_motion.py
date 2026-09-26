import unittest
import json
import tempfile
from pathlib import Path
from unittest.mock import patch
from types import SimpleNamespace
import cv2
import numpy as np
from host.motion import fit_motion,estimate_pair,transform_disagreement
from host.motion_audit import temporal_reason,audit,interior_disparity_mask
from host.motion_summary import distribution,summarize


class MotionTests(unittest.TestCase):
    def setUp(self):
        cv2.setRNGSeed(7)
        self.rng=np.random.default_rng(42)
        self.k=np.array([[400.,0,320],[0,400,240],[0,0,1]])
        self.xyz=np.column_stack([self.rng.uniform(-4,4,160),self.rng.uniform(-3,3,160),self.rng.uniform(8,16,160)])
        self.r=np.array([.01,-.02,.005]);self.t=np.array([.2,-.1,.08])
        self.uv=cv2.projectPoints(self.xyz,self.r,self.t,self.k,None)[0].reshape(-1,2)

    def test_known_pose_and_inverse_with_outliers(self):
        uv=self.uv.copy();uv[:25]=self.rng.uniform([0,0],[640,480],(25,2))
        result=fit_motion(self.xyz,uv,self.k,(640,480))
        self.assertEqual(result['status'],'candidate')
        transform=np.array(result['T_current_previous'])
        np.testing.assert_allclose(transform[:3,3],self.t,atol=1e-5)
        np.testing.assert_allclose(transform[:3,:3],cv2.Rodrigues(self.r)[0],atol=1e-5)
        np.testing.assert_allclose(result['cameraPositionInPrevious'],-transform[:3,:3].T@self.t,atol=1e-5)
        self.assertFalse(result['motionAccuracyValidated'])

    def test_too_few_nonfinite_and_negative_depth(self):
        xyz=self.xyz.copy();xyz[20:,2]=-1;xyz[0]=np.nan
        result=fit_motion(xyz,self.uv,self.k,(640,480))
        self.assertEqual(result['reason'],'insufficient_correspondences')
        self.assertIsNone(result['T_current_previous'])

    def test_unrelated_correspondences_rejected(self):
        result=fit_motion(self.xyz,self.rng.uniform([0,0],[640,480],(160,2)),self.k,(640,480))
        self.assertEqual(result['status'],'rejected');self.assertIsNone(result['T_current_previous'])

    def test_clustered_support_rejected(self):
        result=fit_motion(self.xyz,self.uv*.01+[310,230],self.k,(640,480))
        self.assertEqual(result['reason'],'poor_image_coverage')

    def test_collinear_support_rejected(self):
        xyz=np.column_stack([np.linspace(-3,3,160),np.zeros(160),np.full(160,10)])
        result=fit_motion(xyz,self.uv,self.k,(640,480))
        self.assertEqual(result['reason'],'degenerate_3d_support')

    def test_blank_and_no_depth_rejected(self):
        image=np.zeros((480,640),np.uint8);z=np.full(image.shape,10.)
        for mask in [np.ones(image.shape,bool),np.zeros(image.shape,bool)]:
            result=estimate_pair(image,image,z,mask,self.k)
            self.assertEqual(result['reason'],'no_features')

    def test_identical_textured_frame_is_identity_control(self):
        image=self.rng.integers(0,256,(480,640),dtype=np.uint8)
        z=np.full(image.shape,10.);valid=np.ones(image.shape,bool)
        result=estimate_pair(image,image,z,valid,self.k)
        self.assertEqual(result['status'],'candidate')
        np.testing.assert_allclose(result['T_current_previous'],np.eye(4),atol=1e-4)
        self.assertTrue(result['nearPlanarSupport'])

    def test_timestamp_guards(self):
        self.assertEqual(temporal_reason((100,105),(101,105)),'non_increasing_timestamp')
        self.assertEqual(temporal_reason((100,105),(99,106)),'non_increasing_timestamp')
        self.assertEqual(temporal_reason((100,105),(600000100,600000105)),'temporal_gap')
        self.assertIsNone(temporal_reason((100,105),(100000100,100000105)))

    def test_weak_corners_with_bright_detail_recover_known_translation(self):
        image=np.full((480,640),128,np.uint8)
        for y in range(60,430,50):
            for x in range(60,590,50):
                image[y:y+7,x:x+7]=136
        image[220:235,310:325]=255
        current=cv2.warpAffine(image,np.float32([[1,0,2],[0,1,0]]),(640,480),borderValue=128)
        result=estimate_pair(image,current,np.full(image.shape,10.),np.ones(image.shape,bool),self.k)
        self.assertEqual(result['status'],'candidate')
        expected=np.eye(4);expected[0,3]=.05
        np.testing.assert_allclose(result['T_current_previous'],expected,atol=.002)

    def test_search_endpoints_excluded_for_both_disparity_signs(self):
        for minimum in [0,-64]:
            d=np.array([minimum-1,minimum,minimum+.25,minimum+62.75,minimum+63,np.nan])
            np.testing.assert_array_equal(interior_disparity_mask(d,minimum,64),
                                          [False,False,True,True,False,False])

    def test_malformed_inputs(self):
        with self.assertRaises(ValueError):fit_motion(self.xyz,self.uv[:2],self.k,(640,480))
        with self.assertRaises(ValueError):fit_motion(self.xyz,self.uv,np.zeros((3,3)),(640,480))
        image=np.zeros((30,40),np.uint8)
        with self.assertRaises(ValueError):estimate_pair(image,image,np.ones((2,2)),np.ones((2,2),bool),self.k)

    def test_transform_disagreement(self):
        first=np.eye(4);second=np.eye(4)
        second[:3,:3]=cv2.Rodrigues(np.array([0,0,np.deg2rad(3.)]))[0]
        second[:3,3]=[1,2,2]
        result=transform_disagreement(first,second)
        self.assertAlmostEqual(result['rotationDegrees'],3.)
        self.assertAlmostEqual(result['translationCheckerSquare'],3.)
        with self.assertRaises(ValueError):transform_disagreement(np.zeros((4,4)),second)

    def test_forward_reverse_and_three_frame_composition(self):
        a=np.eye(4);b=np.eye(4);c=np.eye(4)
        b[:3,3]=[1,0,0];c[:3,3]=[0,2,0]
        forward=b@a;reverse=np.linalg.inv(forward)
        closure=transform_disagreement(np.eye(4),reverse@forward)
        self.assertLess(closure['translationCheckerSquare'],1e-12)
        direct=c@b;composed=c@b
        self.assertLess(transform_disagreement(direct,composed)['rotationDegrees'],1e-9)

class AuditTests(unittest.TestCase):
    def setUp(self):
        self.temp=tempfile.TemporaryDirectory();self.addCleanup(self.temp.cleanup)
        self.path=Path(self.temp.name)/'calibration.npz'
        np.savez(self.path,lengthUnit='checker_square')
        self.report=dict(ids=['20','21'],lengthUnit='checker_square',geometrySignature=[['crop',1.4],['crop',0]])
        self.write_report()
        self.processor=patch('host.motion_audit.StereoProcessor',return_value=SimpleNamespace(input_size=(1280,960)))
        self.processor.start();self.addCleanup(self.processor.stop)

    def write_report(self):
        self.path.with_suffix('.json').write_text(json.dumps(self.report))

    def test_nonfinite_trial_focus_rejected_through_audit(self):
        left=dict(imageTimestampNs=100,capture=dict(crop='crop',focusDiopters=float('nan')))
        right=dict(imageTimestampNs=105,capture=dict(crop='crop',focusDiopters=0))
        with patch('host.motion_audit.candidates',return_value=[(left,right,5)]):
            result=audit(Path(self.temp.name),self.path)
        self.assertEqual(result['rows'][0]['reason'],'calibration_geometry_mismatch')
        self.assertIsNone(result['rows'][0]['T_current_previous'])

    def test_sidecar_unit_mismatch_rejected(self):
        np.savez(self.path,lengthUnit='meter')
        with self.assertRaisesRegex(ValueError,'unit differs'):audit(Path(self.temp.name),self.path)

    def test_nonfinite_calibration_focus_rejected(self):
        self.report['geometrySignature'][0][1]=float('nan');self.write_report()
        with self.assertRaisesRegex(ValueError,'geometry signature'):audit(Path(self.temp.name),self.path)

    def test_sidecar_hash_covers_admission_parameters(self):
        with patch('host.motion_audit.candidates',return_value=[]):
            first=audit(Path(self.temp.name),self.path)
            self.report['geometrySignature'][0][1]=1.5;self.write_report()
            second=audit(Path(self.temp.name),self.path)
        self.assertEqual(first['calibrationSha256'],second['calibrationSha256'])
        self.assertNotEqual(first['calibrationReportSha256'],second['calibrationReportSha256'])

    def test_invalid_start_pair(self):
        with self.assertRaisesRegex(ValueError,'Start pair'):audit(Path(self.temp.name),self.path,start_pair=-1)


class MotionSummaryTests(unittest.TestCase):
    def test_distribution_and_aggregate(self):
        self.assertEqual(distribution([]),dict(count=0,p50=None,p95=None,maximum=None))
        with tempfile.TemporaryDirectory() as directory:
            path=Path(directory)/'audit.json'
            row=dict(status='candidate',reason=None,sourceTimestampsNs=[10,11],
                     forwardBackward=dict(rotationDegrees=1.,translationCheckerSquare=2.),
                     threeFrame=None,reverse=dict(status='rejected',reason='pnp_failed'))
            audit=dict(calibrationSha256='a',calibrationReportSha256='b',outputScale=.5,
                       unit='checker_square',gates={'maxTemporalGapMs':500},motionGroundTruthAvailable=False,
                       metricAccuracyValidated=False,processedPairs=1,startPair=4,rows=[row])
            path.write_text(json.dumps(audit))
            result=summarize([path])
        self.assertEqual(result['totalPairs'],1)
        self.assertEqual(result['statusCounts'],{'candidate':1})
        self.assertEqual(result['reverseRejectionReasons'],{'pnp_failed':1})
        self.assertEqual(result['forwardBackward']['rotationDegrees']['p95'],1.)
        self.assertEqual(result['trackingStateCounts'],{'new_segment':1})

    def test_mismatched_provenance_rejected(self):
        with tempfile.TemporaryDirectory() as directory:
            paths=[]
            for index,unit in enumerate(('checker_square','meter')):
                path=Path(directory)/f'{index}.json';paths.append(path)
                path.write_text(json.dumps(dict(calibrationSha256='a',calibrationReportSha256='b',
                    outputScale=.5,unit=unit,gates={'maxTemporalGapMs':500},motionGroundTruthAvailable=False,
                    metricAccuracyValidated=False,processedPairs=0,rows=[])))
            with self.assertRaisesRegex(ValueError,'differ'):summarize(paths)

if __name__=='__main__':unittest.main()

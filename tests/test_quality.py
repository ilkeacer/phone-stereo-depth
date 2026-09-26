import unittest
from unittest.mock import Mock, patch

import cv2
import numpy as np

from host.depth import StereoProcessor, scaled_rectification
from host.viewer import Viewer
from host.corner_quality import sample_supported


def calibration():
    k=np.array([[100.,0,160],[0,100,48],[0,0,1]])
    p1=np.column_stack((k,np.zeros(3)));p2=p1.copy();p2[0,3]=-10
    return dict(K1=k,K2=k.copy(),D1=np.zeros(5),D2=np.zeros(5),R1=np.eye(3),R2=np.eye(3),
                P1=p1,P2=p2,Q=np.array([[1.,0,0,-160],[0,1,0,-48],[0,0,0,100],[0,0,10,0]]),
                size=np.array([320,96]),rotate90=np.array(False),roi1=np.array([0,0,320,96]),
                roi2=np.array([0,0,320,96]),lengthUnit=np.array('m'))


class QualityTests(unittest.TestCase):
    def test_corner_sampling_interpolates_a_plane_without_crossing_invalid_pixels(self):
        yy,xx=np.indices((6,8));d=(3+2*xx+yy).astype(np.float32)
        valid=np.ones_like(d,bool)
        points=np.array([[2.25,1.5],[7,2],[-.1,1],[np.nan,1]])
        values,good=sample_supported(d,valid,points)
        self.assertAlmostEqual(values[0],9.)
        np.testing.assert_array_equal(good,[True,False,False,False])
        self.assertTrue(np.isnan(values[1:]).all())
        valid[2,3]=False
        values,good=sample_supported(d,valid,points[:1])
        self.assertFalse(good[0]);self.assertTrue(np.isnan(values[0]))

    def test_scaled_projection_preserves_known_xyz_and_source_rays(self):
        c=calibration();scaled=scaled_rectification(c,.5)
        for xyz in ([.2,.1,1.],[-.15,-.2,2.]):
            homogeneous=np.r_[xyz,1.]
            a=scaled['P1']@homogeneous;b=scaled['P2']@homogeneous
            a=a[:2]/a[2];b=b[:2]/b[2]
            result=scaled['Q']@np.array([a[0],a[1],a[0]-b[0],1.])
            np.testing.assert_allclose(result[:3]/result[3],xyz,atol=1e-9)
        full=StereoProcessor(c);half=StereoProcessor(c,.5)
        for full_maps,half_maps in zip(full.maps,half.maps):
            for a,b in zip(full_maps,half_maps):np.testing.assert_allclose(a[::2,::2],b,atol=1e-6)
        image=np.zeros((96,320),np.uint8)
        self.assertEqual(half.rectify([image,image])[0].shape,(48,160))
        self.assertEqual(c['P1'][0,0],100.)
        with self.assertRaises(ValueError):scaled_rectification(c,.333)

    def test_counters_do_not_change_disparity_and_known_plane_depth(self):
        cv2.setNumThreads(2)
        rng=np.random.default_rng(21)
        left=rng.integers(0,256,(96,320),np.uint8)
        right=np.zeros_like(left);right[:,:-8]=left[:,8:]
        processor=StereoProcessor(calibration())
        a=processor.compute([left,right],16);b=processor.compute_diagnostics([left,right],16)
        for old,new in zip(a[:3],b[:3]):np.testing.assert_array_equal(old,new)
        counts=list(b[4]['stages'].values())
        self.assertTrue(all(x>=y for x,y in zip(counts,counts[1:])))
        self.assertEqual(counts[-1],int(b[2].sum()))
        mask=b[2][10:-10,40:-30]
        self.assertGreater(mask.mean(),.95)
        self.assertAlmostEqual(float(np.median(b[1][10:-10,40:-30][mask])),1.25,places=2)
        blank=processor.compute_diagnostics([np.zeros_like(left),np.zeros_like(left)],16)
        self.assertFalse(blank[2].any());self.assertEqual(blank[4]['stages']['positiveFiniteZ'],0)

    def test_raw_preview_cannot_replace_computed_pair_even_in_same_poll(self):
        viewer=Viewer.__new__(Viewer)
        viewer.root=Mock();viewer.messages=Mock();viewer.status=Mock();viewer.quality_readout=Mock()
        viewer.show=Mock();viewer.camera_images=Mock();viewer.z=None;viewer.mode='live'
        rect=[np.ones((2,2),np.uint8)]*2
        row=dict(hostCompletionMonotonic=10,callbackAgeUpperMs=0,leftTimestampNs=100,
                 rightTimestampNs=101,deltaMs=.000001,matcherMs=1,validPixelRatio=1)
        newer=Mock();newer.age_upper.return_value=0;newer.blobs=[b'new',b'new']
        viewer.messages.drain.return_value={'depth':('depth',rect,np.ones((2,2)),np.ones((2,2),bool),row),
                                             'preview_packet':('preview_packet',newer,1)}
        with patch('host.viewer.cv2.imdecode',return_value=np.zeros((2,2),np.uint8)) as decode:
            viewer.poll()
        self.assertIs(viewer.show.call_args.args[0],rect)
        viewer.camera_images.assert_not_called();decode.assert_not_called()

    def test_source_reset_clears_cameras_depth_selection_and_inspector_pair(self):
        viewer=Viewer.__new__(Viewer)
        viewer.messages=Mock();viewer.messages.drain.return_value={'reset':('reset',)}
        for name in ('root','pair_note','quality_readout','readout','canvas','draw_inspector'):
            setattr(viewer,name,Mock())
        viewer.camera_views=[Mock(),Mock()];viewer.mode='live'
        viewer.z=np.ones((2,2));viewer.mask=np.ones((2,2),bool);viewer.pair_rect=[viewer.z]*2
        viewer.transform=(0,0,1);viewer.poll()
        self.assertIsNone(viewer.z);self.assertIsNone(viewer.mask);self.assertIsNone(viewer.pair_rect)
        self.assertIsNone(viewer.transform)
        for view in viewer.camera_views:
            view.config.assert_called_with(image='');self.assertIsNone(view.image)
        viewer.canvas.delete.assert_called_with('all')


if __name__=='__main__':unittest.main()

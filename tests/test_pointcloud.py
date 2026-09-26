import json,tempfile,unittest
from pathlib import Path
import numpy as np
from host.pointcloud import points_from_z,save_cloud,read_cloud,disparity_support_mask
from host.cloud_view import display_subset


class PointCloudTests(unittest.TestCase):
    def test_disparity_floor_is_resolution_invariant_and_never_fills_holes(self):
        p=np.column_stack((np.eye(3),[-80.,0,0]))
        z=np.array([[5,10,20,np.nan,0,5]],np.float32)
        mask=np.ones_like(z,bool);mask[0,-1]=False
        full,stats=disparity_support_mask(z,mask,p,1.,8.)
        half_p=p.copy();half_p[:2]*=.5
        half,_=disparity_support_mask(z,mask,half_p,.5,8.)
        np.testing.assert_array_equal(full,[[True,True,False,False,False,False]])
        np.testing.assert_array_equal(full,half)
        self.assertEqual(stats['removed'],1);self.assertEqual(stats['maximumRetainedZ'],10.)
        p[0,3]=80
        np.testing.assert_array_equal(full,disparity_support_mask(z,mask,p,1.,8.)[0])
        for threshold in [-1,float('nan')]:
            with self.assertRaises(ValueError):disparity_support_mask(z,mask,p,1.,threshold)

    def test_axial_depth_axes_and_same_pixel_grayscale(self):
        p=np.array([[2.,0,1,0],[0,2,1,0],[0,0,1,0]])
        z=np.full((3,3),2.,np.float32);left=np.arange(9,dtype=np.uint8).reshape(3,3)
        xyz,rgb,pixels=points_from_z(z,np.ones_like(z,bool),left,p)
        np.testing.assert_allclose(xyz[[0,4,8]],[[-1,-1,2],[0,0,2],[1,1,2]])
        np.testing.assert_array_equal(rgb[:,0],left[pixels[:,1],pixels[:,0]])
        self.assertGreater(np.linalg.norm(xyz[0]),z[0,0])
        projected=xyz@p[:,:3].T
        np.testing.assert_allclose(projected[:,:2]/projected[:,2,None],pixels)

    def test_invalid_values_never_become_points_and_empty_cloud_is_valid(self):
        p=np.column_stack((np.eye(3),np.zeros(3)))
        z=np.array([[1,np.nan,0,-2,np.inf,2]],np.float32)
        valid=np.ones_like(z,bool);valid[0,-1]=False
        xyz,_,pixels=points_from_z(z,valid,np.zeros_like(z,np.uint8),p)
        self.assertEqual(len(xyz),1);np.testing.assert_array_equal(pixels,[[0,0]])
        self.assertEqual(len(points_from_z(z,np.zeros_like(valid),np.zeros_like(z,np.uint8),p)[0]),0)

    def test_binary_ply_and_metadata_roundtrip_no_overwrite(self):
        with tempfile.TemporaryDirectory() as tmp:
            path=Path(tmp)/'capture';p=np.column_stack((np.eye(3),np.zeros(3)))
            z=np.array([[2.,3.]],np.float32);valid=np.ones_like(z,bool);left=np.array([[17,245]],np.uint8)
            r=save_cloud(path,z,valid,left,p,{'unit':'checker_square','frameMetadata':{'leftTimestampNs':123}})
            raw=(path/'cloud.ply').read_bytes();head,data=raw.split(b'end_header\n',1)
            self.assertIn(b'element vertex 2',head);self.assertEqual(len(data),2*15)
            dtype=np.dtype([('xyz','<f4',(3,)),('rgb','u1',(3,))])
            parsed=np.frombuffer(data,dtype=dtype)
            np.testing.assert_allclose(parsed['xyz'],[[0,0,2],[3,0,3]])
            np.testing.assert_array_equal(parsed['rgb'],[[17]*3,[245]*3])
            self.assertEqual(json.loads((path/'metadata.json').read_text())['provenance']['frameMetadata']['leftTimestampNs'],123)
            self.assertFalse(r['accumulatedMap']);self.assertFalse(r['metricAccuracyValidated'])
            xyz,rgb,meta=read_cloud(path)
            np.testing.assert_allclose(xyz,parsed['xyz']);np.testing.assert_array_equal(rgb,parsed['rgb'])
            with self.assertRaises(FileExistsError):save_cloud(path,z,valid,left,p,{'unit':'checker_square'})
            (path/'cloud.ply').write_bytes(raw[:-1])
            with self.assertRaises(ValueError):read_cloud(path)

    def test_display_sampling_preserves_colors_and_does_not_change_cloud(self):
        xyz=np.column_stack((np.arange(100),np.zeros(100),np.arange(1,101))).astype(np.float32)
        rgb=np.repeat(np.arange(100,dtype=np.uint8)[:,None],3,axis=1)
        original=xyz.copy()
        points,colors,count=display_subset(xyz,rgb,limit=10,max_z=50)
        self.assertEqual(count,50);self.assertEqual(len(points),10)
        np.testing.assert_array_equal(colors[:,0],points[:,0]);np.testing.assert_array_equal(xyz,original)
        self.assertEqual(display_subset(xyz,rgb,max_z=.5)[2],0)
        with self.assertRaises(ValueError):display_subset(xyz,rgb,max_z=float('nan'))


if __name__=='__main__':unittest.main()

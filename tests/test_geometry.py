import unittest
import numpy as np
from host.depth import reproject
from host.calibrate import heldout_error,solve
import cv2
class GeometryTests(unittest.TestCase):
 def test_metric_scale_and_invalid_mask(self):
  # Synthetic f=100px, baseline=.1m. d=10px -> Z=1m, d=5px -> Z=2m.
  q=np.array([[1,0,0,0],[0,1,0,0],[0,0,0,100],[0,0,10,0]],np.float64)
  d=np.array([[10,5,0]],np.float32)
  xyz,mask=reproject(d,q,np.array([[True,True,False]]))
  np.testing.assert_allclose(xyz[0,:2,2],[1,2]);self.assertTrue(np.isnan(xyz[0,2]).all())
 def test_axis_depth_is_not_range(self):
  q=np.array([[1,0,0,10],[0,1,0,0],[0,0,0,100],[0,0,10,0]],np.float64)
  xyz,_=reproject(np.array([[10]],np.float32),q,np.array([[True]]))
  self.assertGreater(np.linalg.norm(xyz[0,0]),xyz[0,0,2])
 def test_heldout_epipolar_error(self):
  k=np.array([[100,0,50],[0,100,50],[0,0,1]],np.float64)
  cal=dict(K1=k,K2=k,D1=np.zeros(5),D2=np.zeros(5),R1=np.eye(3),R2=np.eye(3),P1=k,P2=k)
  a=np.array([[[20,30]],[[30,40]]],np.float32);b=a.copy();b[:,:,0]-=5;b[:,:,1]+=2
  np.testing.assert_allclose(heldout_error(a,b,cal),2,atol=1e-5)
 def test_synthetic_stereo_recovers_known_scale(self):
  grid=np.zeros((54,3),np.float32);grid[:,:2]=np.mgrid[:9,:6].T.reshape(-1,2)*.025
  k=np.array([[600,0,320],[0,600,240],[0,0,1]],np.float64)
  objects=[];left=[];right=[]
  for i in range(18):
   rv=np.array([.1*(i%3-1),.12*(i%4-1.5),.03*i],np.float64)
   tv=np.array([-.1+.03*(i%4),-.08+.025*(i%3),.65+.05*(i%5)],np.float64)
   left.append(cv2.projectPoints(grid,rv,tv,k,np.zeros(5))[0])
   right.append(cv2.projectPoints(grid,rv,tv+np.array([-.06,0,0]),k,np.zeros(5))[0]);objects.append(grid)
  cal,rms=solve(objects,left,right,(640,480))
  self.assertLess(rms,.01);self.assertAlmostEqual(np.linalg.norm(cal['T']),.06,places=4)
if __name__=='__main__':unittest.main()

class SeededTargetTests(unittest.TestCase):
 def test_nested_preview_copy_does_not_replace_target(self):
  from host.calibrate import detect_board
  image=np.full((650,1100),255,np.uint8)
  def board(x,y,side):
   for r in range(7):
    for c in range(10):
     if (r+c)%2==0:image[y+r*side:y+(r+1)*side,x+c*side:x+(c+1)*side]=0
   return (np.mgrid[:9,:6].T.reshape(-1,1,2)*side+np.array([x+side-.5,y+side-.5])).astype(np.float32)
  seed=board(50,80,65);board(800,250,22)
  ok,points=detect_board(image,(9,6),seed)
  self.assertTrue(ok)
  self.assertLess(np.linalg.norm(points-seed,axis=2).mean(),.5)
  self.assertLess(points[:,:,0].max(),750)

import json,tempfile,unittest
from pathlib import Path
import numpy as np
from host.calibrate import candidates
from host.viewer import region_stats,check_geometry
from host.depth import consistency_mask

class ResultPipelineTests(unittest.TestCase):
 def test_guided_first_saved_pose_is_not_camera_start(self):
  with tempfile.TemporaryDirectory() as tmp:
   p=Path(tmp);(p/'selected-poses.jsonl').write_text('')
   for c,offset in [('20',0),('21',1_000_000)]:
    images=[];meta=[]
    for i,sequence in enumerate([10,100,130]):
     ts=10_000_000_000+i*1_000_000_000+offset
     images.append(dict(sequence=sequence,imageTimestampNs=ts,sampleFile=f'{c}_{i}.jpg'))
     meta.append(dict(sensorTimestampNs=ts,crop='fixed',focusDiopters=1.4))
    (p/f'camera_{c}_images.jsonl').write_text('\n'.join(map(json.dumps,images)))
    (p/f'camera_{c}_metadata.jsonl').write_text('\n'.join(map(json.dumps,meta)))
   matches=list(candidates(p,['20','21'],20))
   self.assertEqual([r[0]['sequence'] for r in matches],[100,130])
 def test_left_right_mask_and_occlusion(self):
  left=np.full((2,8),2,np.float32);right=-left.copy();right[0,2]=0
  mask=consistency_mask(left,right)
  self.assertFalse(mask[:,:2].any());self.assertFalse(mask[0,4]);self.assertTrue(mask[1,2:].all())
  left[0,7]=np.nan;self.assertFalse(consistency_mask(left,right)[0,7])
 def test_roi_median_ignores_invalid_values(self):
  z=np.full((10,10),5.,np.float32);mask=np.ones_like(z,bool);z[:3]=9999;mask[:3]=False
  stats=region_stats(z,mask,5,5,20);self.assertEqual(stats['median'],5);self.assertEqual(stats['validPixels'],70)
  self.assertIsNone(region_stats(z,np.zeros_like(mask),5,5));self.assertIsNone(region_stats(z,mask,-1,0))
 def test_live_geometry_changes_rejected(self):
  report={'geometrySignature':[['a',1.4],['b',0.]]};h={}
  for c,crop,focus in [('20','a',1.4),('21','b',0.)]:h[c]={'image':{'width':1280,'height':960},'capture':{'crop':crop,'focusDiopters':focus}}
  check_geometry(h,report,(1280,960));h['20']['capture']['focusDiopters']=2
  with self.assertRaises(ValueError):check_geometry(h,report,(1280,960))

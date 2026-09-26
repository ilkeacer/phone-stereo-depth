import json
import socket
import struct
from unittest.mock import patch
import tempfile
import unittest
from pathlib import Path
import numpy as np
from host.assistant import BurstSelector,receive_exact,save_pair,align,read_pair
from host.calibrate import board_contained

class AssistantTests(unittest.TestCase):
 def setUp(self):
  self.p=(np.mgrid[:9,:6].T.reshape(-1,1,2)*20+100).astype(np.float32)
 def packet(self,i,paired=True):
  h={'deltaNs':10_000_000,'paired':paired}
  for c,offset in [('20',0),('21',10_000_000)]:
   ts=10_000_000_000+i*200_000_000+offset
   h[c]={'image':{'imageTimestampNs':ts},'capture':{'exposureTimeNs':20_000_000,'rollingShutterSkewNs':32_000_000}}
  return h,[bytes([i%255])]*2
 def test_hand_tremor_selects_a_real_buffered_pair(self):
  gate=BurstSelector();chosen=None
  for i in range(10):
   shift=3 if i%2 else -3
   progress,chosen=gate.update(self.packet(i),[self.p+shift,self.p+shift],[100,100])
   if chosen:break
  self.assertEqual(progress,1)
  self.assertIsNotNone(chosen)
  self.assertLess(chosen['packet'][0]['20']['image']['imageTimestampNs'],self.packet(i)[0]['20']['image']['imageTimestampNs'])
  self.assertLessEqual(max(chosen['quality']['estimatedMotionRiskPx']),3)
 def test_fast_movement_and_blur_are_not_saved(self):
  for step,sharpness in [(30,100),(0,2)]:
   gate=BurstSelector()
   for i in range(15):
    _,chosen=gate.update(self.packet(i),[self.p+i*step]*2,[sharpness]*2)
    self.assertIsNone(chosen)
 def test_unpaired_burst_and_expired_samples_are_not_selected(self):
  gate=BurstSelector()
  for i in range(12):
   _,chosen=gate.update(self.packet(i,False),[self.p]*2,[100]*2)
   self.assertIsNone(chosen)
  _,chosen=gate.update(self.packet(40),[self.p]*2,[100]*2)
  self.assertIsNone(chosen)
  self.assertEqual(len(gate.samples),1)
 def test_full_outer_board_must_fit(self):
  self.assertTrue(board_contained(self.p,(9,6),(640,480)))
  clipped=self.p.copy();clipped[:,:,1]-=85
  self.assertFalse(board_contained(clipped,(9,6),(640,480)))
 def test_corner_order_stability(self):
  np.testing.assert_array_equal(align(self.p[::-1],self.p),self.p)
 def test_packet_truncation_and_bounds(self):
  a,b=socket.socketpair()
  try:
   a.sendall(b'abc');a.shutdown(socket.SHUT_WR)
   with self.assertRaises(ConnectionError):receive_exact(b,4)
   with self.assertRaises(ValueError):receive_exact(b,9_000_000)
  finally:a.close();b.close()
 def test_capture_preserves_jpeg_bytes_and_exact_timestamps(self):
  h={'deltaNs':10}
  for c,ts in [('20',100),('21',110)]:
   h[c]={'image':{'sequence':1,'imageTimestampNs':ts},'capture':{'sensorTimestampNs':ts}}
  with tempfile.TemporaryDirectory() as tmp:
   path=Path(tmp);save_pair(path,(h,[b'original20',b'original21']),[self.p,self.p],0,{})
   self.assertEqual((path/'camera_20_1_100.jpg').read_bytes(),b'original20')
   self.assertEqual(json.loads((path/'camera_21_metadata.jsonl').read_text())['sensorTimestampNs'],110)
   self.assertFalse(json.loads((path/'selected-poses.jsonl').read_text())['metricScaleVerified'])

 def test_unpaired_preview_never_claims_capture_eligible(self):
  for eligible in [False,True]:
   h={'ok':True,'paired':eligible}
   for c,ts in [('20',100_000_000),('21',130_000_000)]:
    h[c]={'length':3,'image':{'imageTimestampNs':ts},'capture':{'sensorTimestampNs':ts}}
   wire=json.dumps(h).encode();a,b=socket.socketpair()
   try:
    a.sendall(struct.pack('>I',len(wire))+wire+b'jpgjpg')
    with patch('host.assistant.socket.create_connection',return_value=b):
     if eligible:
      with self.assertRaises(ValueError):read_pair()
     else:
      result=read_pair();self.assertFalse(result[0]['paired'])
   finally:a.close();b.close()

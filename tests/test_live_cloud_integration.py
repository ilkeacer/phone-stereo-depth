"""Regression checks for source freshness and renderer replacement, without a phone."""
import multiprocessing as mp
import threading
import time
import unittest
from types import SimpleNamespace
from unittest.mock import Mock, patch
import numpy as np
from host.live_cloud import CloudMailbox, create_window, renderable
from host.viewer import Viewer


class CloudIntegrationTests(unittest.TestCase):
    def viewer(self):
        v=Viewer.__new__(Viewer)
        box=CloudMailbox(mp.get_context('spawn'),3)
        v.live_cloud=SimpleNamespace(mailbox=box,alive=lambda:True)
        v.pipeline=SimpleNamespace(condition=threading.Condition(),stop=threading.Event(),epoch=1,source='test')
        v.current_row=dict(hostCompletionMonotonic=100.,freshnessAgeUpperMs=200.,callbackAgeUpperMs=100.,
                           epoch=1,sourceRun='test',leftTimestampNs=10,rightTimestampNs=11)
        p1=np.c_[np.eye(3),np.zeros(3)];p2=p1.copy();p2[0,3]=-100
        v.cloud_frame=(np.ones((2,2)),np.ones((2,2),bool),np.zeros((2,2),np.uint8),p1,
                       dict(rectificationP2=p2,outputScale=1.))
        v.mode='live';v.last_cloud_sequence=None;v.closing=False;v.root=Mock()
        return v,box

    def test_source_expiry_and_duplicate_do_not_refresh_cloud(self):
        v,box=self.viewer()
        with patch('host.viewer.time.monotonic',return_value=100.1):
            v.poll_live_cloud();first=box.read();v.poll_live_cloud()
        self.assertEqual(first['sequence'],box.read()['sequence'])
        self.assertAlmostEqual(first['expires'],100.8)
        self.assertEqual(len(first['xyz']),3)
        with patch('host.viewer.time.monotonic',return_value=100.9):v.poll_live_cloud()
        self.assertFalse(renderable(box.read(),100.9))

    def test_epoch_change_during_conversion_discards_result(self):
        v,box=self.viewer()
        def changed(*args):
            v.pipeline.epoch+=1
            return np.ones((2,3)),None,None
        with patch('host.viewer.time.monotonic',return_value=100.1),patch('host.viewer.points_from_z',side_effect=changed):
            v.poll_live_cloud()
        self.assertFalse(renderable(box.read(),100.1))
        self.assertIsNone(v.last_cloud_sequence)

    def test_stop_or_wrong_source_clears_existing_cloud(self):
        for stop in (True,False):
            v,box=self.viewer();box.publish(np.ones((1,3)),200.,1,1)
            if stop:v.pipeline.stop.set()
            else:v.pipeline.source='replacement'
            with patch('host.viewer.time.monotonic',return_value=100.1):v.poll_live_cloud()
            self.assertFalse(renderable(box.read(),100.1))

    def test_renderer_replaces_cloud_preserves_view_and_hides_expired_data(self):
        import matplotlib.pyplot as plt
        box=CloudMailbox(mp.get_context('spawn'),3)
        fig=create_window(box,backend='Agg');_,state,tick=fig._live_cloud_controls
        try:
            ax=fig.axes[0];ax.view_init(elev=12,azim=34);limits=(ax.get_xlim(),ax.get_ylim(),ax.get_zlim())
            box.publish(np.ones((3,3)),time.monotonic()+10,3,4);tick()
            box.publish(np.ones((1,3))*2,time.monotonic()+10,1,4);tick()
            self.assertEqual(len(ax.collections[0]._offsets3d[0]),1)
            self.assertEqual((ax.elev,ax.azim),(12,34))
            self.assertEqual((ax.get_xlim(),ax.get_ylim(),ax.get_zlim()),limits)
            with patch('host.live_cloud.time.monotonic',return_value=time.monotonic()+20):tick()
            self.assertFalse(state['visible'])
            self.assertEqual(len(ax.collections[0]._offsets3d[0]),0)
            box.stop.set();self.assertFalse(tick())
        finally:plt.close(fig)

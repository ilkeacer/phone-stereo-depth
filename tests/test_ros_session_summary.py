import json
from pathlib import Path
import sqlite3
import tempfile
import unittest
from host.ros_session_summary import summarize


class SessionSummaryTests(unittest.TestCase):
    def test_complete_capture_can_coexist_with_rejected_map(self):
        with tempfile.TemporaryDirectory() as tmp:
            root=Path(tmp)
            (root/'capture.json').write_text(json.dumps(dict(recordingComplete=True,recordingPairs=1000,publishedPairs=1000)))
            (root/'lifecycle.json').write_text(json.dumps(dict(captureContinuedAfterTrackingLoss=True,error='tracking lost',recoveryPlanAvailable=True)))
            result=summarize(root)
            self.assertTrue(result['captureContinuedAfterTrackingLoss']);self.assertTrue(result['recordingComplete'])
            self.assertTrue(result['recoveryPlanAvailable'])
            self.assertEqual(result['recordingPairs'],1000);self.assertIsNotNone(result['sessionError'])

    def test_unmatched_guard_loss_is_reported_separately(self):
        with tempfile.TemporaryDirectory() as tmp:
            root=Path(tmp)
            (root/'odometry-status.jsonl').write_text('{"lost": false}\n')
            (root/'hybrid-tracking-failure.json').write_text('{"lost": true, "stampNs": 9, "error": "lost"}')
            result=summarize(root)
            self.assertEqual(result['lostResults'],0)
            self.assertTrue(result['hybridTrackingFailure']['lost'])
            self.assertIn('Matched',result['trackingCountScope'])
            self.assertIn('Session failed',result['conclusion'])

    def test_mapping_timeline_separates_processing_gap_from_active_graph(self):
        with tempfile.TemporaryDirectory() as tmp:
            root=Path(tmp);(root/'map').mkdir()
            with sqlite3.connect(root/'map/map.db') as db:
                db.execute('CREATE TABLE Node (id INTEGER, weight INTEGER, stamp FLOAT)')
                db.execute('CREATE TABLE Link (from_id INTEGER,to_id INTEGER)')
                db.executemany('INSERT INTO Node VALUES (?,?,?)',[(1,0,1.),(2,-9,2.),(3,0,13.)])
                db.execute('INSERT INTO Link VALUES (1,3)')
            result=summarize(root)
            self.assertEqual(result['graph']['activeNodes'],2)
            self.assertEqual(result['mappingTimeline']['storedNodeSamples'],3)
            self.assertEqual(result['mappingTimeline']['maxGapSeconds'],11.)

    def test_explicit_lost_status_wins_over_console_quality_proxy(self):
        with tempfile.TemporaryDirectory() as tmp:
            root=Path(tmp)
            (root/'mapping.log').write_text('Odom: quality=0\nOdom: quality=0\n')
            (root/'capture.json').write_text(json.dumps(dict(publishedPairs=4)))
            (root/'odometry-status.jsonl').write_text(json.dumps(dict(lost=False,inliers=0))+'\n'+json.dumps(dict(lost=True,inliers=0))+'\n')
            result=summarize(root)
            self.assertEqual(result['trackingResults'],1);self.assertEqual(result['lostResults'],1)
            self.assertEqual(result['trackingSource'],'synchronized_OdomInfo')
            self.assertEqual(result['odometryInputCoverageRatio'],.5)

    def test_rgbd_lost_status_invalidates_map_without_guard_marker(self):
        with tempfile.TemporaryDirectory() as tmp:
            root=Path(tmp)
            (root/'lifecycle.json').write_text(json.dumps(dict(inputKind='rgbd')))
            (root/'odometry-status.jsonl').write_text('{"lost": false}\n{"lost": true}\n')
            result=summarize(root)
            self.assertEqual(result['lostResults'],1)
            self.assertTrue(result['depthTrackingInvalid'])
            self.assertIn('diagnostic evidence',result['conclusion'])

    def test_live_ai_depth_scale_does_not_inherit_stereo_measurement_label(self):
        with tempfile.TemporaryDirectory() as tmp:
            root=Path(tmp)
            (root/'capture.json').write_text(json.dumps(dict(scaleSource='measured',squareMm=21.44)))
            (root/'lifecycle.json').write_text(json.dumps(dict(inputKind='hybrid',
                depthScaleSource='model_predicted_metres',odometryScaleSource='measured')))
            result=summarize(root)
            self.assertEqual(result['scaleSource'],'model_predicted_metres')
            self.assertEqual(result['odometryScaleSource'],'measured')

    def test_missing_capture_is_not_success(self):
        with tempfile.TemporaryDirectory() as temp:
            result=summarize(Path(temp))
            self.assertFalse(result['captureReportPresent'])
            self.assertFalse(result['accumulatedGraphPresent'])

    def test_crashed_mapper_is_reported_even_with_successful_tracking(self):
        with tempfile.TemporaryDirectory() as temp:
            root=Path(temp)
            (root/'mapping.log').write_text('Odom: quality=100\n[ERROR] [rtabmap-2]: process has died [exit code -6]\n')
            result=summarize(root)
            self.assertEqual(result['trackingResults'],1)
            self.assertEqual(len(result['processFailures']),1)
            self.assertFalse(result['accumulatedGraphPresent'])

    def test_deleted_nodes_do_not_count_as_a_map(self):
        with tempfile.TemporaryDirectory() as temp:
            root=Path(temp);(root/'map').mkdir()
            (root/'capture.json').write_text(json.dumps(dict(publishedPairs=4,error=None)))
            (root/'mapping.log').write_text('Odom: quality=20\nOdom: quality=0\n')
            with sqlite3.connect(root/'map/map.db') as db:
                db.execute('CREATE TABLE Node (weight INTEGER)')
                db.execute('CREATE TABLE Link (from_id INTEGER, to_id INTEGER)')
                db.executemany('INSERT INTO Node VALUES (?)',[(0,),(-9,),(-9,)])
            result=summarize(root)
            self.assertEqual(result['trackingResults'],1)
            self.assertEqual(result['lostResults'],1)
            self.assertFalse(result['accumulatedGraphPresent'])
            with sqlite3.connect(root/'map/map.db') as db:
                db.execute('INSERT INTO Node VALUES (0)')
                db.execute('INSERT INTO Link VALUES (1,4)')
            result=summarize(root)
            self.assertTrue(result['accumulatedGraphPresent'])
            self.assertFalse(result['mapAccuracyValidated'])

    def test_disconnected_graph_is_not_reported_as_one_map(self):
        with tempfile.TemporaryDirectory() as temp:
            root=Path(temp);(root/'map').mkdir()
            with sqlite3.connect(root/'map/map.db') as db:
                db.execute('CREATE TABLE Node (id INTEGER, weight INTEGER)')
                db.execute('CREATE TABLE Link (from_id INTEGER,to_id INTEGER)')
                db.executemany('INSERT INTO Node VALUES (?,0)',[(1,),(2,),(3,),(4,)])
                db.executemany('INSERT INTO Link VALUES (?,?)',[(1,2),(3,4),(4,99)])
            result=summarize(root);g=result['graph']
            self.assertEqual(result['mapState'],'partial')
            self.assertIn('not one continuous map',result['conclusion'])
            self.assertEqual(g['connectedComponents'],2)
            self.assertEqual(g['largestComponentNodes'],2)
            self.assertEqual(g['danglingLinks'],1)
            self.assertFalse(g['allActiveNodesConnected'])

class ShutdownSummaryTests(unittest.TestCase):
    def test_only_expected_post_shutdown_sigint_is_not_a_crash(self):
        with tempfile.TemporaryDirectory() as tmp:
            root=Path(tmp);prefix='Odom: quality=1\n'
            sigint='[ERROR] [node]: process has died [exit code -2, cmd abc]\n'
            (root/'mapping.log').write_text(prefix+sigint)
            (root/'lifecycle.json').write_text(json.dumps(dict(mappingLogBytesBeforeShutdown=len(prefix.encode()))))
            self.assertEqual(summarize(root)['processFailures'],[])
            (root/'mapping.log').write_text(prefix+sigint.replace('-2,','-11,'))
            self.assertEqual(len(summarize(root)['processFailures']),1)
            (root/'lifecycle.json').write_text(json.dumps(dict(mappingLogBytesBeforeShutdown=9999)))
            (root/'mapping.log').write_text(prefix+sigint)
            self.assertEqual(len(summarize(root)['processFailures']),1)

    def test_known_input_count_is_denominator_not_lossy_observer(self):
        with tempfile.TemporaryDirectory() as tmp:
            root=Path(tmp)
            (root/'capture.json').write_text(json.dumps(dict(publishedPairs=1,inputPairs=3)))
            (root/'odometry-status.jsonl').write_text((json.dumps(dict(lost=False,inliers=20))+'\n')*2)
            self.assertAlmostEqual(summarize(root)['odometryInputCoverageRatio'],2/3)

import tempfile
import unittest
from pathlib import Path
from host.timing import feasibility,read_complete_rows,audit
import json


class TimingTests(unittest.TestCase):
    def test_source_phase_gap_even_with_every_frame_available(self):
        left=[i*100_000_000 for i in range(10)]
        right=[t+(0 if i in (0,9) else 40_000_000) for i,t in enumerate(left)]
        result=feasibility(left,right)
        self.assertEqual(result['eligibleLeftFrames'],2)
        self.assertAlmostEqual(result['maxInternalEligibleGapSeconds'],.9)

    def test_no_eligible_pairs_and_edges_are_counted(self):
        left=[i*100_000_000 for i in range(10)];right=[t+40_000_000 for t in left]
        result=feasibility(left,right)
        self.assertEqual(result['eligibleLeftFrames'],0)
        self.assertAlmostEqual(result['maxNoEligibleIntervalIncludingEdgesSeconds'],.86)

    def test_full_match_and_large_integer_timestamps(self):
        left=[10**17+i*66_666_667 for i in range(10)]
        r=feasibility(left,left)
        self.assertEqual(r['eligibleLeftFrames'],10)
        self.assertAlmostEqual(r['maxInternalEligibleGapSeconds'],.066666667)
        with self.assertRaises(ValueError):feasibility([1,1],[1,2])

    def test_only_incomplete_final_record_may_be_excluded(self):
        with tempfile.TemporaryDirectory() as temp:
            p=Path(temp)/'rows.jsonl';p.write_bytes(b'{"ok":1}\n{"x":')
            self.assertEqual(read_complete_rows(p),([{'ok':1}],1))
            p.write_bytes(b'broken\n{"ok":1}\n')
            with self.assertRaises(ValueError):read_complete_rows(p)
            p.write_bytes(b'{"ok":1}\nbroken\n')
            with self.assertRaises(ValueError):read_complete_rows(p)

    def test_audit_preserves_all_images_and_reports_exact_metadata_subset(self):
        with tempfile.TemporaryDirectory() as temp:
            p=Path(temp)
            for camera in ('20','21'):
                ts=[i*66_666_667 for i in range(5)]
                (p/f'camera_{camera}_images.jsonl').write_text(''.join(json.dumps({'imageTimestampNs':t})+'\n' for t in ts))
                (p/f'camera_{camera}_metadata.jsonl').write_text(''.join(json.dumps({'sensorTimestampNs':t,'frameDurationNs':66_666_667})+'\n' for t in ts[1:]))
            report=audit(p)
            self.assertEqual(report['allImageTimestamps']['eligibleLeftFrames'],5)
            self.assertEqual(report['exactMetadataJoinedTimestamps']['eligibleLeftFrames'],4)
            self.assertEqual(report['inputCounts']['20']['exactJoinedFrames'],4)
            with (p/'camera_20_metadata.jsonl').open('a') as f:f.write(json.dumps({'sensorTimestampNs':66_666_667})+'\n')
            with self.assertRaises(ValueError):audit(p)

import unittest
from host.analyze import nearest_unique,parse_dumpsys,analyze_trial,continuous_evidence
import tempfile,json
from pathlib import Path

class PairingTests(unittest.TestCase):
    def test_no_reuse(self):
        self.assertEqual(nearest_unique([0,9],[8],20),[(1,0,1)])
    def test_threshold_and_unmatched(self):
        self.assertEqual(nearest_unique([0,10,30],[3,13,99],3),[(0,0,3),(1,1,3)])
    def test_empty(self): self.assertEqual(nearest_unique([],[],1),[])
    def test_large_absolute_timestamps(self):
        b=10**18;self.assertEqual(nearest_unique([b,b+100],[b+1,b+99],2),[(0,0,1),(1,1,1)])
    def test_duplicates_and_order(self):
        for a,b in [([1,1],[1]),([2,1],[1])]:
            with self.assertRaises(ValueError):nearest_unique(a,b,1)
    def test_ordered_maximum_pairs(self):
        self.assertEqual(nearest_unique([0,10],[9,20],20),[(0,0,9),(1,1,10)])
        self.assertEqual(nearest_unique([0,10],[-5,1],9),[(0,0,5),(1,1,9)])
    def test_sparse_stream_is_not_evidence(self):
        m=dict(spanSeconds=65,fps=.03,gapsOver200ms=1,monotonic=True,duplicateImageTimestamps=0,frames=2,metadataMatches=2)
        self.assertFalse(continuous_evidence([m,m],65))
        m.update(fps=15,gapsOver200ms=0,frames=975,metadataMatches=974)
        self.assertTrue(continuous_evidence([m,m],65))
        self.assertFalse(continuous_evidence([m,m],59))
    def test_report_ignores_dynamic_and_vendor_sections(self):
        text='== Camera HAL device device@3.5/legacy/20 (v3.5) static information: ==\n  Resource cost: 99\n android.lens.facing (80005): byte[1]\n [BACK ]\n== Camera HAL device device@3.5/legacy/20 dump state: ==\n'
        parsed=parse_dumpsys(text)
        self.assertEqual(len(parsed),1);self.assertEqual(parsed[0]['id'],'20');self.assertEqual(parsed[0]['facing'],['BACK'])
    def test_exact_metadata_join_no_fuzzy_join(self):
        with tempfile.TemporaryDirectory() as d:
            p=Path(d);(p/'summary.json').write_text(json.dumps({'feeds':[{'cameraId':'0'}]}))
            (p/'camera_0_images.jsonl').write_text('{"imageTimestampNs":100}\n{"imageTimestampNs":200}\n')
            (p/'camera_0_metadata.jsonl').write_text('{"sensorTimestampNs":100}\n{"sensorTimestampNs":201}\n')
            m=analyze_trial(p)['measurements'][0]
            self.assertEqual(m['metadataMatches'],1);self.assertEqual(m['unmatchedMetadata'],1)

if __name__=='__main__':unittest.main()

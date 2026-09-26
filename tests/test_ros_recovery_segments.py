"""Boundary selection is from image/OdomInfo timestamps, never image counts."""
import json
from pathlib import Path
import tempfile
import unittest

from host.ros_recovery_segments import plan, materialize, FORMULA
from host.ros_recovery_map import validate_segments


class RecoverySegmentsTests(unittest.TestCase):
    def fixture(self, root, *, offset=True, status='failed'):
        session = root/'session'; raw = session/'raw';raw.mkdir(parents=True)
        rows = []
        for i in range(8):
            names = [f'camera_{camera}_{i}.jpg' for camera in (20, 21)]
            for name in names:(raw/name).write_bytes(('source-'+name).encode())
            rows.append(dict(frameIndex=i, sourceRun='same-run',
                             sourceTimestampsNs=[i*100_000_000+1, i*100_000_000+2], sampleFiles=names))
        (raw/'committed-pairs.json').write_text(json.dumps(rows))
        (raw/'manifest.json').write_text(json.dumps(dict(status=status, error='camera stalled' if status=='failed' else None,
                                                       frames=8, sourceRun='same-run', provenance={'calibrationSha256':'a','calibrationReportSha256':'b'})))
        capture=dict(sourceRun='same-run',publishedPairs=8)
        if offset:capture.update(rosStampOffsetNs=1_000_000_000,rosStampFormula=FORMULA)
        (session/'capture.json').write_text(json.dumps(capture))
        (session/'hybrid-tracking-failure.json').write_text(json.dumps(dict(lost=True,stampNs=1_450_000_000)))
        (session/'mapping-fallback.json').write_text(json.dumps(dict(mappingStopped=True,policy='keep-capture',observedImagesAtLoss=2)))
        return session

    def test_time_boundary_copies_two_independent_sections(self):
        with tempfile.TemporaryDirectory() as temporary:
            root=Path(temporary);source=self.fixture(root,status='complete')
            result=plan(source,margin_ms=100)
            self.assertEqual([(s['start'],s['end']) for s in result['sections']],[(0,3),(6,7)])
            self.assertEqual(result['excludedFrameIndexRange'],[4,5])
            out=root/'derived';materialize(source,out,result)
            self.assertEqual(json.loads((out/'before-loss/manifest.json').read_text())['originalCaptureStatus'],'complete')
            self.assertEqual(json.loads((out/'after-loss/manifest.json').read_text())['originalFrameIndexRange'],[6,7])
            self.assertEqual([r['frameIndex'] for r in json.loads((out/'after-loss/committed-pairs.json').read_text())],[0,1])
            self.assertEqual((source/'raw/camera_20_6.jpg').read_bytes(),(out/'after-loss/camera_20_6.jpg').read_bytes())
            with self.assertRaises(FileExistsError):materialize(source,out,result)

    def test_missing_exact_offset_refuses_old_count_based_guess(self):
        with tempfile.TemporaryDirectory() as temporary:
            source=self.fixture(Path(temporary),offset=False)
            with self.assertRaisesRegex(ValueError,'offset unavailable'):plan(source)

    def test_changed_commit_index_rejected_before_copy(self):
        with tempfile.TemporaryDirectory() as temporary:
            root=Path(temporary);source=self.fixture(root)
            result=plan(source,100)
            (source/'raw/committed-pairs.json').write_text('[]')
            with self.assertRaisesRegex(ValueError,'changed after plan'):materialize(source,root/'derived',result)
            self.assertFalse((root/'derived').exists())

    def test_map_pipeline_rejects_overlap_and_accepts_disjoint_source_ranges(self):
        with tempfile.TemporaryDirectory() as temporary:
            root=Path(temporary)
            directories=[]
            for name,range_ in [('first',[0,3]),('second',[6,7])]:
                part=root/name;part.mkdir()
                (part/'manifest.json').write_text(json.dumps(dict(status='complete',derivedSubset=True,
                    sourceTrackingLossNotRecovered=True,separateMapOriginRequired=True,
                    originalSourceRun='run',sourceDirectory='/same/source',sourceFailureStampNs=123,
                    originalFrameIndexRange=range_,frames=range_[1]-range_[0]+1)))
                directories.append(part)
            ordered=validate_segments(directories[::-1])
            self.assertEqual([m['originalFrameIndexRange'] for _,m in ordered],[[0,3],[6,7]])
            manifest=json.loads((directories[1]/'manifest.json').read_text())
            manifest.update(originalFrameIndexRange=[3,6],frames=4)
            (directories[1]/'manifest.json').write_text(json.dumps(manifest))
            with self.assertRaisesRegex(ValueError,'overlap'):validate_segments(directories)


if __name__=='__main__':unittest.main()

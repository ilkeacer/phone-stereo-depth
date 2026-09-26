import json
from pathlib import Path
import tempfile
from types import SimpleNamespace
from unittest.mock import patch
import unittest
from host.ros_diagnostic import DiagnosticWriter,phase_at
from host.motion_recording import committed_pairs


class DiagnosticTests(unittest.TestCase):
    def test_phase_boundaries(self):
        self.assertEqual([phase_at(t) for t in (0,9.99,10,19.99,20,29.99,30)],
                         ['sabit','sabit','otele','otele','sabit_son','sabit_son','complete'])
        with self.assertRaises(ValueError):phase_at(-1)

    def test_sample_limit_committed_phases_and_terminal_status(self):
        with tempfile.TemporaryDirectory() as tmp:
            path=Path(tmp)/'raw';writer=DiagnosticWriter(path,{},dict(calibrationSha256='fixture'))
            def pair(i):
                header={c:dict(image=dict(sequence=i,imageTimestampNs=i*1000+j),
                               capture=dict(sensorTimestampNs=i*1000+j))
                        for j,c in enumerate(('20','21'))}
                return SimpleNamespace(header=header,blobs=(b'left',b'right'),source='fixture',
                    timestamps=(i*1000,i*1000+1),read_started=0,received=0,
                    callback_age_upper=lambda now:0,age_upper=lambda now:0)
            with patch('builtins.print') as output:
                for i,t in enumerate((0,.01,.2,10,20,30),1):
                    complete=writer.observe(pair(i),100+t)
            lines=[str(call.args[0]) for call in output.call_args_list]
            counters=[line for line in lines if 'Toplam:' in line]
            self.assertEqual(len(counters),3)
            self.assertIn('Toplam: 30 sn kaldı',counters[0])
            self.assertIn('[2/3]',counters[1])
            self.assertIn('Kaydedilen çift: 4',counters[2])
            self.assertTrue(complete)
            writer.finish('complete')
            result=json.loads((path/'manifest.json').read_text())
            self.assertEqual(result['frames'],4)
            self.assertEqual(result['phaseCounts'],dict(sabit=2,otele=1,sabit_son=1))
            self.assertEqual(len(list(committed_pairs(path))),4)
            self.assertEqual(result['status'],'complete')
            self.assertFalse(result['metricScaleVerified'])

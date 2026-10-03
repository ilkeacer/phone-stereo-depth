from pathlib import Path
import unittest
from unittest.mock import patch

from host.depth_pro_model import UPSTREAM_COMMIT, validate_source


class SourcePinTests(unittest.TestCase):
    def test_clean_pin_is_allowed_and_extra_code_or_bytecode_is_not(self):
        with patch('host.depth_pro_model.subprocess.check_output',
                   side_effect=[UPSTREAM_COMMIT+'\n', '', '']):
            self.assertEqual(validate_source(Path('/synthetic/upstream')), UPSTREAM_COMMIT)
        for extra in ('src/depth_pro/shadow.py\n', 'src/depth_pro/__pycache__/model.cpython-310.pyc\n'):
            with patch('host.depth_pro_model.subprocess.check_output', side_effect=[UPSTREAM_COMMIT+'\n', '', extra]):
                with self.assertRaises(ValueError):
                    validate_source(Path('/synthetic/upstream'))

    def test_modified_tracked_source_or_wrong_commit_is_rejected(self):
        for revision, dirty in [(UPSTREAM_COMMIT, ' M src/depth_pro/depth_pro.py\n'), ('wrong', '')]:
            with patch('host.depth_pro_model.subprocess.check_output', side_effect=[revision, dirty, '']):
                with self.assertRaises(ValueError):
                    validate_source(Path('/synthetic/upstream'))

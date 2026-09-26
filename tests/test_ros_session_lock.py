import subprocess
import tempfile
from pathlib import Path
import unittest


class SessionLockTests(unittest.TestCase):
    def test_closed_lock_fd_is_not_inherited_by_child(self):
        with tempfile.TemporaryDirectory() as temp:
            lock=Path(temp)/'lock'
            command=f'exec 9>"{lock}"; flock -n 9; sh -c "test ! -e /proc/\\$\\$/fd/9" 9>&-'
            result=subprocess.run(['bash','-c',command],capture_output=True,text=True)
            self.assertEqual(result.returncode,0,result.stderr)

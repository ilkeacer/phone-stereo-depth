import os
from pathlib import Path
import subprocess
import sys
import tempfile
import time
import unittest
from host.ros_process import ProcessGroup,active_group,session_lock


class ProcessTests(unittest.TestCase):
    def test_shutdown_wait_services_observer_callback(self):
        with tempfile.TemporaryDirectory() as tmp:
            marker=Path(tmp)/'ready'
            code='import signal,time,pathlib,sys; signal.signal(signal.SIGINT,lambda *_:(time.sleep(.25),sys.exit(0))); pathlib.Path('+repr(str(marker))+').touch(); time.sleep(60)'
            child=ProcessGroup([sys.executable,'-c',code],stdout=subprocess.DEVNULL)
            calls=[]
            try:
                end=time.monotonic()+3
                while not marker.exists() and time.monotonic()<end:time.sleep(.02)
                self.assertTrue(marker.exists())
                result=child.stop(grace=1,wait_hook=lambda:calls.append(time.monotonic()))
                self.assertGreater(len(calls),1);self.assertFalse(result['remaining']);self.assertFalse(result['forcedKill'])
            finally:child.stop(grace=.1,terminate_grace=.1)

    def test_normal_child_closes_gracefully(self):
        child=ProcessGroup([sys.executable,'-c','import time; time.sleep(60)'],stdout=subprocess.DEVNULL)
        try:
            result=child.stop(grace=1)
            self.assertFalse(result['remaining']);self.assertFalse(result['forcedKill'])
        finally:child.stop(grace=.1,terminate_grace=.1)

    def test_exited_leader_does_not_hide_stubborn_descendant(self):
        with tempfile.TemporaryDirectory() as tmp:
            marker=Path(tmp)/'ready'
            code='import signal,time,pathlib; signal.signal(signal.SIGINT,signal.SIG_IGN); signal.signal(signal.SIGTERM,signal.SIG_IGN); pathlib.Path('+repr(str(marker))+').touch(); time.sleep(60)'
            launcher='import subprocess,sys; subprocess.Popen([sys.executable,"-c",'+repr(code)+'])'
            child=ProcessGroup([sys.executable,'-c',launcher],stdout=subprocess.DEVNULL)
            try:
                child.process.wait(timeout=3)
                end=time.monotonic()+3
                while not marker.exists() and time.monotonic()<end:time.sleep(.02)
                self.assertTrue(marker.exists());self.assertTrue(active_group(child.pgid))
                result=child.stop(grace=.1,terminate_grace=.1)
                self.assertFalse(result['remaining']);self.assertTrue(result['forcedKill'])
            finally:child.stop(grace=.1,terminate_grace=.1)

    def test_lock_excludes_other_owner_then_releases(self):
        with tempfile.TemporaryDirectory() as tmp:
            lock=Path(tmp)/'lock'
            with session_lock(lock):
                with self.assertRaisesRegex(RuntimeError,'Başka'):
                    with session_lock(lock):pass
            with session_lock(lock):pass

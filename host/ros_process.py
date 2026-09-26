"""Own and reap ROS process groups, including descendants of exited launchers."""
import fcntl
import os
from pathlib import Path
import signal
import subprocess
import time
from contextlib import contextmanager


@contextmanager
def session_lock(path):
    Path(path).parent.mkdir(parents=True,exist_ok=True)
    with Path(path).open('a') as lock:
        try:fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
        except BlockingIOError:raise RuntimeError('Başka bir haritalama oturumu açık.') from None
        # Python descriptors are non-inheritable, and Popen also uses close_fds.
        try:yield
        finally:fcntl.flock(lock,fcntl.LOCK_UN)


def active_group(pgid):
    """Linux: exclude unreaped zombies which can no longer touch the database."""
    for entry in Path('/proc').iterdir():
        if not entry.name.isdigit():continue
        try:
            fields=(entry/'stat').read_text().rsplit(')',1)[1].split()
            if int(fields[2])==pgid and fields[0] not in ('Z','X'):return True
        except (OSError,ValueError,IndexError):continue
    return False


class ProcessGroup:
    def __init__(self,command,*,env=None,cwd=None,stdout=None):
        self.command=list(command)
        self.process=subprocess.Popen(command,env=env,cwd=cwd,stdout=stdout,
            stderr=subprocess.STDOUT,start_new_session=True,close_fds=True)
        self.pgid=self.process.pid

    def stop(self,grace=10.,terminate_grace=3.,wait_hook=None):
        escalated=False
        for sig,timeout in [(signal.SIGINT,grace),(signal.SIGTERM,terminate_grace),(signal.SIGKILL,2.)]:
            self.process.poll()
            if not active_group(self.pgid):break
            if sig==signal.SIGKILL:escalated=True
            try:os.killpg(self.pgid,sig)
            except ProcessLookupError:break
            deadline=time.monotonic()+timeout
            while active_group(self.pgid) and time.monotonic()<deadline:
                if wait_hook is not None:wait_hook()
                self.process.poll();time.sleep(.05)
        self.process.poll()
        return dict(pid=self.pgid,returnCode=self.process.returncode,
                    forcedKill=escalated,remaining=active_group(self.pgid))

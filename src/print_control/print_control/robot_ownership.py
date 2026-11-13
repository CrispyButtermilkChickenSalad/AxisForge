"""Cross-process ownership shared by standalone motion and the sync worker."""
import fcntl
import os
from pathlib import Path


class RobotControlLock:
    def __init__(self, path=None):
        self.path = Path(path or os.getenv('AXISFORGE_ROBOT_LOCK','.axisforge-robot-control.lock')).resolve()
        self.fd = None

    def _open(self): return os.open(self.path, os.O_CREAT | os.O_RDWR, 0o600)

    def acquire(self):
        if self.fd is not None: return
        fd = self._open()
        try:fcntl.flock(fd,fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError:
            os.close(fd)
            raise ValueError('Robot is owned by bed probing or another AxisForge controller')
        except Exception:os.close(fd);raise
        self.fd = fd

    def release(self):
        if self.fd is not None:
            fd,self.fd = self.fd,None
            os.close(fd)

    def external_busy(self):
        if self.fd is not None:return False
        fd = self._open()
        try:
            try:fcntl.flock(fd,fcntl.LOCK_EX | fcntl.LOCK_NB)
            except BlockingIOError:return True
            return False
        finally:os.close(fd)

    def __del__(self):self.release()

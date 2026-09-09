import os
from pathlib import Path
import signal
import subprocess
import sys
import tempfile
import time
import unittest

SUPERVISOR = Path(__file__).resolve().parents[1] / 'tools/runtime/process_supervisor.py'


def alive(pid):
    try:
        return Path(f'/proc/{pid}/stat').read_text().rsplit(')', 1)[1].split()[0] != 'Z'
    except FileNotFoundError:
        return False


class SupervisorTests(unittest.TestCase):
    def test_parent_exit_cleans_stubborn_descendants(self):
        for sig in (signal.SIGHUP, signal.SIGTERM, signal.SIGKILL):
            with self.subTest(signal=sig), tempfile.TemporaryDirectory() as directory:
                marker = Path(directory) / 'pid'
                worker = "import os,signal,time;from pathlib import Path;signal.signal(signal.SIGTERM,signal.SIG_IGN);Path(" + repr(str(marker)) + ").write_text(str(os.getpid()));time.sleep(60)"
                launcher = "import subprocess,os,time;subprocess.Popen(" + repr([sys.executable, str(SUPERVISOR), '--parent']) + "+[str(os.getpid()),'--grace','0.2','--'," + repr(sys.executable) + ",' -c'.strip()," + repr(worker) + "],start_new_session=True);time.sleep(60)"
                parent = subprocess.Popen([sys.executable, '-c', launcher])
                try:
                    deadline = time.monotonic() + 3
                    while not marker.exists() and time.monotonic() < deadline:
                        time.sleep(.02)
                    self.assertTrue(marker.exists())
                    pid = int(marker.read_text())
                    parent.send_signal(sig)
                    parent.wait(timeout=2)
                    deadline = time.monotonic() + 3
                    while alive(pid) and time.monotonic() < deadline:
                        time.sleep(.02)
                    self.assertFalse(alive(pid))
                finally:
                    if parent.poll() is None:
                        parent.kill()
                    parent.wait()
                    if marker.exists() and alive(int(marker.read_text())):
                        os.kill(int(marker.read_text()), signal.SIGKILL)

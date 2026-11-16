import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import tempfile
import time
import unittest


GUARD = (Path(__file__).resolve().parents[1] /
         'tools/deploy/hc_x1_camera_dds/camera_child_guard.py')


def alive(pid):
    try:
        return Path(f'/proc/{pid}/stat').read_text().rsplit(')', 1)[1].split()[0] != 'Z'
    except FileNotFoundError:
        return False


class CameraGuardTests(unittest.TestCase):
    def test_absent_launch_never_spawns_child(self):
        launch = subprocess.Popen([sys.executable, '-c', 'pass'])
        launch.wait(timeout=3)
        with tempfile.TemporaryDirectory() as directory:
            marker = Path(directory) / 'unexpected'
            result = subprocess.run([
                sys.executable, str(GUARD), '--launch-pid', str(launch.pid), '--',
                sys.executable, '-c',
                'import sys;from pathlib import Path;Path(sys.argv[1]).touch()',
                str(marker),
            ], timeout=3)
            self.assertEqual(result.returncode, 1)
            self.assertFalse(marker.exists())

    def test_launch_or_child_exit_cleans_only_owned_group(self):
        for exit_target in ('launch', 'child'):
            with self.subTest(exit_target=exit_target), tempfile.TemporaryDirectory() as directory:
                pidfile = Path(directory) / 'guard.json'
                descendant_file = Path(directory) / 'descendant.pid'
                sleeper = 'import signal,time;signal.signal(signal.SIGTERM,signal.SIG_IGN);time.sleep(30)'
                worker = (
                    'import subprocess,sys,time;from pathlib import Path;'
                    f'p=subprocess.Popen([sys.executable,"-c",{sleeper!r}]);'
                    'Path(sys.argv[1]).write_text(str(p.pid));time.sleep(30)'
                )
                launch = subprocess.Popen([sys.executable, '-c', 'import time;time.sleep(30)'])
                unrelated = subprocess.Popen([sys.executable, '-c', 'import time;time.sleep(30)'])
                guard = subprocess.Popen([
                    sys.executable, str(GUARD), '--launch-pid', str(launch.pid),
                    '--pid-file', str(pidfile), '--grace', '0.2', '--',
                    sys.executable, '-c', worker, str(descendant_file),
                ])
                owned = []
                try:
                    deadline = time.monotonic() + 3
                    while (not pidfile.exists() or not descendant_file.exists()) and time.monotonic() < deadline:
                        time.sleep(.02)
                    self.assertTrue(pidfile.exists())
                    self.assertTrue(descendant_file.exists())
                    child_pid = json.loads(pidfile.read_text())['child_pid']
                    owned = [child_pid, int(descendant_file.read_text())]
                    os.kill(launch.pid if exit_target == 'launch' else child_pid, signal.SIGKILL)
                    guard.wait(timeout=3)
                    self.assertTrue(all(not alive(pid) for pid in owned))
                    self.assertIsNone(unrelated.poll())
                    if exit_target == 'child':
                        self.assertIsNone(launch.poll())
                finally:
                    for process in (guard, launch, unrelated):
                        if process.poll() is None:
                            process.kill()
                        process.wait()
                    for pid in owned:
                        if alive(pid):
                            os.kill(pid, signal.SIGKILL)

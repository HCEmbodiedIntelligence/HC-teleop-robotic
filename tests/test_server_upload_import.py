"""A checkout upgrade must not run a stale installed upload client."""
import os
import subprocess
import sys
from pathlib import Path


def test_server_bootstrap_prefers_bundled_client_over_old_installation(tmp_path):
    root = Path(__file__).resolve().parents[1]
    stale = tmp_path / 'humanoid_upload'
    stale.mkdir()
    (stale / '__init__.py').write_text('')
    script = '''
import runpy, sys
from pathlib import Path
root = Path(sys.argv[1])
runpy.run_path(str(root / 'middleware/server.py'))
import humanoid_upload
from middleware.core.platform_connection import PlatformConnection
from middleware.core.task_recording import TaskRecorder
assert Path(humanoid_upload.__file__).resolve().parent == root / 'upload_client/humanoid_upload'
'''
    result = subprocess.run(
        [sys.executable, '-c', script, str(root)],
        env=dict(os.environ, PYTHONPATH=str(tmp_path)), capture_output=True, text=True,
    )
    assert result.returncode == 0, result.stderr

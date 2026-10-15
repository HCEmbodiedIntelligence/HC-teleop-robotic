"""Installation in a filesystem without platform sources; HTTP is the only bridge."""
import json
import os
import shutil
import subprocess
from pathlib import Path

from humanoid_upload.common import atomic_json
from conftest import G0


def test_install_and_upload_without_platform_files(tmp_path, cfg, server):
    workspace = Path(__file__).resolve().parents[4]
    wheels = Path(os.environ.get('OPENARM_UPLOAD_ARTIFACTS',str(workspace/"artifacts/g0")))
    isolated = tmp_path/"isolated"
    isolated.mkdir()
    shutil.copytree(G0/"fixtures/valid-openarm", isolated/"data"/"sample")
    cfg.update(data_directory="/robot/data", queue_directory="/robot/queue", processing_endpoint=True)
    atomic_json(isolated/"config.json", cfg)
    prefix = ["bwrap", "--die-with-parent", "--ro-bind", "/usr", "/usr", "--ro-bind", "/lib", "/lib",
              "--ro-bind", "/lib64", "/lib64", "--proc", "/proc", "--dev", "/dev", "--tmpfs", "/tmp",
              "--ro-bind", shutil.which("uv"), "/bin/uv", "--ro-bind", str(wheels/"dist"), "/dist",
              "--ro-bind", str(wheels/"wheelhouse"), "/wheels", "--bind", str(isolated), "/robot",
              "--chdir", "/robot", "--setenv", "HOME", "/tmp", "--setenv", "TMPDIR", "/tmp",
              "--unsetenv", "PYTHONPATH"]
    def run(*args):
        result = subprocess.run(prefix + list(args), capture_output=True, text=True, timeout=60)
        assert result.returncode == 0, result.stdout+result.stderr
        return result.stdout
    run("/bin/uv", "venv", "--python", "/usr/bin/python3", "/robot/venv")
    run("/bin/uv", "pip", "install", "--python", "/robot/venv/bin/python", "--no-cache", "--no-index",
        "--find-links", "/dist", "--find-links", "/wheels", "humanoid-upload-client=="+os.environ.get('OPENARM_UPLOAD_VERSION','1.0.0'))
    evidence = run("/robot/venv/bin/python", "-c",
        "import importlib.util,json,pathlib; "
        "names=['hc_data_platform','sqlalchemy','psycopg','temporalio','rclpy']; "
        "assert all(importlib.util.find_spec(n) is None for n in names); "
        "assert not pathlib.Path('/home').exists(); "
        "print(json.dumps({'platform_directory_present':False,'absent_packages':names}))")
    run("/robot/venv/bin/humanoid-upload", "--config", "/robot/config.json", "scan")
    server.faults["commit"] = 1
    result = json.loads(run("/robot/venv/bin/humanoid-upload", "--config", "/robot/config.json", "step"))[0]
    assert result["state"] == "retry_wait"
    run("/robot/venv/bin/humanoid-upload", "--config", "/robot/config.json", "retry", result["id"])
    result = json.loads(run("/robot/venv/bin/humanoid-upload", "--config", "/robot/config.json", "step"))[0]
    assert result["state"] == "processing_failed" and len(server.raws) == 1
    assert server.calls["commit"] == 1
    atomic_json(wheels/"isolated-install.json", json.loads(evidence))
    atomic_json(wheels/"queue-receipt-example.json", result)

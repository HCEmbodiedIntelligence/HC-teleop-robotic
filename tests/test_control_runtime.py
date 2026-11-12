import json
import os
from pathlib import Path
import shutil
import tempfile
import unittest
from unittest.mock import patch

from middleware.core.control_runtime import ControlRuntime


class ControlRuntimeTests(unittest.IsolatedAsyncioTestCase):
    async def asyncSetUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.root = Path(self.tmp.name)
        for folder in ("middleware", "adapters/robots/x1", "tools/runtime"):
            (self.root / folder).mkdir(parents=True)
        (self.root / "adapters/robots/x1/arm_teleop.yaml").write_text("control:\n  backend: v23\n")
        shutil.copy(Path(__file__).resolve().parents[1] / "tools/runtime/process_supervisor.py",
                    self.root / "tools/runtime/process_supervisor.py")
        self.env = patch.dict(os.environ, {"HC_MANAGED_CONTROL": "1", "HC_CONTROL_LOG_DIR": str(self.root / "logs"),
                                          "HC_ROBOT_NAME": "stale_profile", "HC_ARM_TELEOP_CONFIG": "/stale.yaml"})
        self.env.start()
        self.manager = ControlRuntime(self.root / "middleware/config.yaml")
        self.manager.project = self.root
        self.config = {"robot_profiles": {"root": "../adapters/robots", "active": "x1"}, "ros": {"domain_id": 13}}
        self.write_launcher('''
import json, os, pathlib, signal, sys, time
p=pathlib.Path(sys.argv[sys.argv.index('--log-dir')+1])
(p/'environment.json').write_text(json.dumps(dict(os.environ)))
(p/'teleop_controller.log').write_text('HC-TJ teleop ready: disabled')
(p/'v23_solver.log').write_text('reconstructed controller_v2_3 started:')
signal.signal(signal.SIGTERM, lambda *_: sys.exit(0))
while True: time.sleep(.05)
''')

    def write_launcher(self, body):
        path = self.root / "adapters/start.sh"
        path.write_text("#!/usr/bin/python3\n" + body)
        path.chmod(0o700)

    async def asyncTearDown(self):
        await self.manager.stop()
        self.env.stop()
        self.tmp.cleanup()

    async def test_restart_reaps_old_process_and_uses_saved_selection(self):
        await self.manager.start(self.config, timeout=3)
        self.assertEqual(self.manager.status()["state"], "running")
        old = self.manager.process
        env = json.loads((self.manager.log_dir / "environment.json").read_text())
        self.assertNotIn("HC_ROBOT_NAME", env)
        self.assertNotIn("HC_ARM_TELEOP_CONFIG", env)
        self.assertEqual(env["HC_CONTROL_START_DISABLED"], "1")
        self.assertEqual(env["ROS_DOMAIN_ID"], "13")
        await self.manager.stop()
        self.assertIsNotNone(old.returncode)
        await self.manager.start(self.config, timeout=3)
        self.assertEqual(self.manager.status()["state"], "running")
        self.assertNotEqual(old.pid, self.manager.process.pid)

    async def test_initialization_failure_is_not_reported_as_running(self):
        self.write_launcher("raise SystemExit(7)\n")
        await self.manager.start(self.config, timeout=3)
        self.assertEqual(self.manager.status()["state"], "failed")
        self.assertIsNone(self.manager.process)
        self.assertIn("exited", self.manager.error)

    async def test_initialization_timeout_stops_child(self):
        self.write_launcher("import time\ntime.sleep(30)\n")
        await self.manager.start(self.config, timeout=.3)
        self.assertEqual(self.manager.status()["state"], "failed")
        self.assertIsNone(self.manager.process)
        self.assertIn("timed out", self.manager.error)

    async def test_exit_after_startup_is_visible(self):
        await self.manager.start(self.config, timeout=3)
        self.manager.process.terminate()
        await self.manager.process.wait()
        self.assertEqual(self.manager.status()["state"], "failed")

    async def test_no_implicit_controller_in_middleware_only_mode(self):
        self.manager.enabled = False
        await self.manager.start(self.config)
        self.assertIsNone(self.manager.process)

    async def test_unsupported_backend_cannot_report_false_readiness(self):
        (self.root / "adapters/robots/x1/arm_teleop.yaml").write_text("control:\n  backend: motion\n")
        await self.manager.start(self.config)
        self.assertEqual(self.manager.status()["state"], "failed")
        self.assertIsNone(self.manager.process)


if __name__ == '__main__':
    unittest.main()

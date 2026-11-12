"""Own the teleop controller lifetime alongside the web runtime configuration."""
from __future__ import annotations

import asyncio
import os
from pathlib import Path
import signal
import time

import yaml


class ControlRuntime:
    def __init__(self, config_path: Path):
        self.config_path = Path(config_path).resolve()
        self.project = Path(__file__).resolve().parents[2]
        self.enabled = os.environ.get("HC_MANAGED_CONTROL") == "1"
        self.process = None
        self.state = "stopped" if self.enabled else "unmanaged"
        self.profile_id = ""
        self.error = ""
        self.log_dir = None
        self._log = None

    def status(self):
        if self.process is not None and self.process.returncode is not None and self.state == "running":
            self.state = "failed"
            self.error = f"control process exited: {self.process.returncode}"
        return {"state": self.state, "profile_id": self.profile_id,
                "pid": self.process.pid if self.process else None,
                "error": self.error, "log_dir": str(self.log_dir or "")}

    async def start(self, config, timeout=30.0):
        if not self.enabled:
            return
        if self.process is not None:
            raise RuntimeError("old control process has not stopped")
        self.state, self.error = "starting", ""
        self.profile_id = config["robot_profiles"]["active"]
        try:
            root = Path(config["robot_profiles"]["root"]).expanduser()
            if not root.is_absolute():
                root = self.config_path.parent / root
            profile = root.resolve() / self.profile_id
            arm = yaml.safe_load((profile / "arm_teleop.yaml").read_text())
            backend = arm.get("control", {}).get("backend", "v23")
            if backend != "v23":
                raise ValueError(f"网页自动重启尚不支持 {backend} 后端；请使用 HC_MANAGED_CONTROL=0 手动启动入口")
            base_log = Path(os.environ.get("HC_CONTROL_LOG_DIR", str(self.project / "runtime/control_logs")))
            self.log_dir = base_log / f"reload_{time.time_ns()}"
            self.log_dir.mkdir(parents=True, mode=0o700)
            env = os.environ.copy()
            for key in ("HC_ROBOT_NAME", "HC_ROBOT_CONFIG_ROOT", "HC_CONTROLLER_CONFIG", "HC_ARM_TELEOP_CONFIG", "HC_CONTROL_BACKEND"):
                env.pop(key, None)
            env.update(ROS_DOMAIN_ID=str(config["ros"]["domain_id"]),
                       HC_CONTROL_START_DISABLED="1")
            self._log = (self.log_dir / "launcher.log").open("wb")
            self.process = await asyncio.create_subprocess_exec(
                "/usr/bin/python3", str(self.project / "tools/runtime/process_supervisor.py"),
                "--parent", str(os.getpid()), "--grace", "6", "--",
                str(self.project / "adapters/start.sh"), "--config", str(self.config_path),
                "--log-dir", str(self.log_dir), cwd=str(self.project), env=env,
                stdout=self._log, stderr=asyncio.subprocess.STDOUT,
                start_new_session=True,
            )
            deadline = asyncio.get_running_loop().time() + timeout
            while asyncio.get_running_loop().time() < deadline:
                if self.process.returncode is not None:
                    raise RuntimeError(f"control startup exited: {self.process.returncode}; see {self.log_dir}")
                teleop_log = self.log_dir / "teleop_controller.log"
                solver_log = self.log_dir / f"{backend}_solver.log"
                teleop_ready = teleop_log.exists() and "HC-TJ teleop ready:" in teleop_log.read_text(errors="replace")
                solver_ready = solver_log.exists() and "reconstructed controller_v2_3 started:" in solver_log.read_text(errors="replace")
                if teleop_ready and solver_ready:
                    self.state = "running"
                    return
                await asyncio.sleep(0.1)
            raise RuntimeError(f"control startup timed out; see {self.log_dir}")
        except Exception as exc:
            await self.stop()
            self.state, self.error = "failed", str(exc)

    async def stop(self):
        process = self.process
        if process is not None:
            if process.returncode is None:
                try:
                    process.send_signal(signal.SIGTERM)
                except ProcessLookupError:
                    pass
                try:
                    await asyncio.wait_for(process.wait(), 10.0)
                except asyncio.TimeoutError:
                    # The supervisor owns this group; never signal by process name.
                    try:
                        os.killpg(process.pid, signal.SIGKILL)
                    except ProcessLookupError:
                        pass
                    await process.wait()
            self.process = None
        if self._log:
            self._log.close()
            self._log = None
        if self.enabled:
            self.state = "stopped"

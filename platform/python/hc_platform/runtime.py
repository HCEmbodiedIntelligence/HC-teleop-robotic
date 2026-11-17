"""Compose independent services; configuration changes are stop/restart transactions."""

from __future__ import annotations
import asyncio
import contextlib
import fcntl
import hashlib
import json
import os
from pathlib import Path
import socket
import sys
import time
import uuid

import httpx

from .configuration import PackageRegistry, atomic_json, ConfigurationError
from .ipc import CoreClient
from .supervisor import Supervisor
from .topics import TopicRegistry
from .data import DataClient
from .settings import apply_overrides, seal_config
from .capture_plan import validate_capture_plan
from .replay import ACTIVE as REPLAY_ACTIVE


class Runtime:
    def __init__(
        self,
        repo: Path,
        state_dir: Path,
        *,
        robot=None,
        mode="sim",
        domain=14,
        groups=None,
        vr_port=5005,
        discovery_port=5006,
    ):
        self.repo, self.state_dir = repo.resolve(), state_dir.resolve()
        self.state_dir.mkdir(parents=True, exist_ok=True)
        os.chmod(self.state_dir, 0o700)
        suffix = hashlib.sha256(str(self.state_dir).encode()).hexdigest()[:10]
        self.run_dir = Path(f"/tmp/hc-platform-{os.getuid()}-{suffix}")
        if self.run_dir.exists() and (
            self.run_dir.is_symlink() or self.run_dir.stat().st_uid != os.getuid()
        ):
            raise RuntimeError("Unsafe runtime directory")
        self.run_dir.mkdir(mode=0o700, exist_ok=True)
        os.chmod(self.run_dir, 0o700)
        self.mode = mode
        self.domain = (
            int(domain) if mode == "teleop" else 180 + int(suffix[:2], 16) % 30
        )
        self.robot, self.groups = robot, groups
        self.vr_port, self.discovery_port = vr_port, discovery_port
        self.registry = PackageRegistry(repo / "adapters/robots", self.state_dir)
        if self.robot is None:
            try:
                self.robot = json.loads(self.registry.active_path.read_text())[
                    "robot_id"
                ]
            except (OSError, KeyError, ValueError):
                self.robot = "x1"
        self.supervisor = Supervisor(repo, self.state_dir / "logs")
        self.core = CoreClient(str(self.run_dir / "control.sock"))
        self.audit = CoreClient(
            str(self.run_dir / "control.sock"), "recorder", event_queue_size=4096
        )
        self.source_clients = {
            role: CoreClient(str(self.run_dir / "control.sock"), role)
            for role in ("program", "exo", "replay")
        }
        self.data = DataClient(str(self.run_dir / "data.sock"))
        # Large downloads / platform calls must not occupy ownership capture or
        # status RPC connections while the robot and ROS capture keep running.
        self.audit_data = DataClient(str(self.run_dir / "data.sock"))
        self.status_data = DataClient(str(self.run_dir / "data.sock"), timeout=3)
        self.media = httpx.AsyncClient(
            transport=httpx.AsyncHTTPTransport(uds=str(self.run_dir / "media.sock")),
            base_url="http://media",
            timeout=3,
        )
        self.config = {}
        self.vr_events_task = None
        self.tasks = []
        self.event_socket = None
        self.active_recording = None
        self.recording_mode = "ordinary"
        self.recording_cutoff_ns = None
        self.intervention_active = False
        self.audit_watermark_boot = None
        self.audit_watermark_ns = 0
        self.audit_progress = asyncio.Event()
        self.audit_error = None
        self.recording_policy = {
            "enabled": False,
            "mode": "corrections",
            "required": False,
        }
        if (self.state_dir / "dagger-recording.json").exists():
            self.recording_policy.update(
                json.loads((self.state_dir / "dagger-recording.json").read_text())
            )
        self.mutation_lock = asyncio.Lock()
        self.management_generation = 0
        self.shutdown_requested = False
        self._closing_task = None
        self._home_resume_task = None
        self._recording_transition = None
        self.instance_lock = None
        self.hardware_lock = None
        self.motion = self.replay = None
        self.control_started = False
        self.api_url = "http://127.0.0.1:7876"
        self.capture_started = False
        self._status = None
        self._status_at = 0
        self._status_lock = asyncio.Lock()

    def overrides_path(self, robot):
        self.registry.validate_id(robot)
        return self.state_dir / "settings" / (robot + ".json")

    def overrides(self, robot=None):
        path = self.overrides_path(robot or self.robot)
        return json.loads(path.read_text()) if path.exists() else {}

    def environment(self):
        env = os.environ.copy()
        env["ROS_DOMAIN_ID"] = str(self.domain)
        env["ROS_LOCALHOST_ONLY"] = "0" if self.mode == "teleop" else "1"
        env["PYTHONUNBUFFERED"] = "1"
        return env

    def compile(self, robot, overrides=None):
        config = self.registry.compile(robot, refresh=True)
        config["runtime_mode"] = self.mode
        config["ros_domain_id"] = self.domain
        config["runtime_state_dir"] = str(self.run_dir)
        config["vr_udp_port"], config["discovery_port"], config["status_port"] = (
            self.vr_port,
            self.discovery_port,
            5007,
        )
        config["vr"] = {
            "pose_port": self.vr_port,
            "discovery_port": self.discovery_port,
            "feedback_port": 5007,
        }
        allowed = self.groups or (
            ["right_arm", "right_gripper"]
            if self.mode == "teleop"
            else list(config["groups"])
            + [
                side + "_gripper"
                for side in ("left", "right")
                if side in config["grippers"]
            ]
            + (["base"] if config["enable_base"] else [])
        )
        config["hardware"]["allowed_groups"] = list(allowed)
        config = apply_overrides(
            config, self.overrides(robot) if overrides is None else overrides
        )
        if "HC_CONTROL_BACKEND" in os.environ:
            config["control_backend"] = os.environ["HC_CONTROL_BACKEND"]
        if "HC_X1_CONTRACT" in os.environ:
            config["hardware"]["contract"] = os.environ["HC_X1_CONTRACT"]
        config = apply_overrides(config, {})
        if self.mode == "teleop" and config["hardware"]["adapter"] != "x1":
            raise ConfigurationError(
                "This build provides the X1 hardware adapter; other robot packages can run in simulation"
            )
        allowed = config["hardware"]["allowed_groups"]
        if self.groups:
            # An explicit process launch limit is an upper bound for settings.
            if set(allowed) - set(self.groups):
                raise ConfigurationError(
                    "Settings exceed --groups launch authorization"
                )
        config["allowed_groups"] = [
            group for group in allowed if group in config["groups"]
        ]
        config["enable_grippers"] = any("gripper" in group for group in allowed)
        config["enable_base"] = "base" in allowed and config["enable_base"]
        body = config["teleop"].get("body", {})
        config["allow_independent_base"] = bool(
            body.get("left_stick_independent_base")
            or body.get("right_stick_independent_base")
        )
        return seal_config(config)

    async def start(self):
        self.supervisor.start_logging()
        self.instance_lock = (self.run_dir / "instance.lock").open("a+")
        fcntl.flock(self.instance_lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        if self.mode == "teleop":
            self.hardware_lock = open(
                f"/tmp/hc-platform-hardware-{os.getuid()}-{self.domain}.lock", "a+"
            )
            fcntl.flock(self.hardware_lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        self.config = await asyncio.to_thread(self.compile, self.robot)
        self.topics = TopicRegistry(
            self.state_dir / "topics.json", self.config.get("topics", [])
        )
        self.config["topics"] = self.topics.items
        atomic_json(self.run_dir / "config.json", self.config)
        for name in ("data.sock", "media.sock", "api-events.sock"):
            (self.run_dir / name).unlink(missing_ok=True)
        argv = [
            sys.executable,
            "-m",
            "hc_platform.data",
            "--state-dir",
            str(self.state_dir / "data"),
            "--socket",
            str(self.run_dir / "data.sock"),
        ]
        converter_python = os.environ.get("HC_CONVERTER_PYTHON")
        if not converter_python:
            candidate_python = self.repo / ".venv-lerobot-platform" / "bin" / "python"
            if candidate_python.is_file():
                converter_python = str(candidate_python)
        if converter_python:
            argv += ["--converter-python", converter_python]
        data = await self.supervisor.start("management", argv, self.environment())
        await self.supervisor.wait_path(self.run_dir / "data.sock", data)
        await self.start_control()
        await self.start_media()
        self.event_socket = socket.socket(socket.AF_UNIX, socket.SOCK_DGRAM)
        self.event_socket.setblocking(False)
        self.event_socket.bind(str(self.run_dir / "api-events.sock"))
        os.chmod(self.run_dir / "api-events.sock", 0o600)
        self.vr_events_task = asyncio.create_task(self._vr_events())
        self.tasks = [
            self.vr_events_task,
            asyncio.create_task(self._control_events()),
            asyncio.create_task(self._health_watch()),
        ]
        atomic_json(
            self.registry.active_path,
            {
                "robot_id": self.robot,
                "config_hash": self.config["config_hash"],
                "runtime_mode": self.mode,
                "ros_domain_id": self.domain,
            },
        )

    async def start_control(self):
        binary = self.repo / "platform/control/build/hc-control"
        if not binary.exists():
            raise RuntimeError("Build platform/control before starting")
        (self.run_dir / "control.sock").unlink(missing_ok=True)
        child = await self.supervisor.start(
            "control",
            [
                str(binary),
                "--socket",
                str(self.run_dir / "control.sock"),
                "--config",
                str(self.run_dir / "config.json"),
            ],
            self.environment(),
        )
        await self.supervisor.wait_path(self.run_dir / "control.sock", child)
        await self.core.connect()
        await self.audit.connect()
        observer = [
            sys.executable,
            "-m",
            "hc_platform.adapters.observations",
            "--config",
            str(self.run_dir / "config.json"),
            "--state-dir",
            str(self.run_dir),
        ]
        if self.mode == "teleop":
            observer += ["--vr-only"]
        await self.supervisor.start("observations", observer, self.environment())
        for role in ("backend", "ik", "vr"):
            (self.run_dir / (role + ".json")).unlink(missing_ok=True)
            argv = [
                sys.executable,
                "-m",
                "hc_platform.adapters.runtime",
                "--role",
                role,
                "--config",
                str(self.run_dir / "config.json"),
                "--socket",
                str(self.run_dir / "control.sock"),
                "--state-dir",
                str(self.run_dir),
            ]
            if role == "backend":
                argv += ["--backend", "x1" if self.mode == "teleop" else "simulation"]
            await self.supervisor.start(role, argv, self.environment())
        # A ready process must have actually loaded the new generation. Hardware
        # readiness remains a separate condition and never triggers motion.
        deadline = time.monotonic() + 15
        while True:
            for role in ("control", "backend", "ik", "vr"):
                child = self.supervisor.children[role]
                if child.process.returncode is not None:
                    raise RuntimeError(f"{role} failed to start; inspect runtime logs")
            states = {
                role: self.read_adapter_state(role) for role in ("backend", "ik", "vr")
            }
            if all(
                state.get("pid") == self.supervisor.children[role].process.pid
                and state.get("core_connected")
                for role, state in states.items()
            ):
                break
            if time.monotonic() >= deadline:
                raise TimeoutError(
                    "Adapter generation did not become ready within 15 seconds"
                )
            await asyncio.sleep(0.05)
        self.control_started = True
        if self.config.get("exoskeleton", {}).get("enabled"):
            await self.supervisor.start(
                "exoskeleton",
                [
                    sys.executable,
                    "-m",
                    "hc_platform.exoskeleton",
                    "--config",
                    str(self.run_dir / "config.json"),
                    "--api-url",
                    self.api_url,
                    "--state-dir",
                    str(self.run_dir),
                ],
                self.environment(),
            )

    async def stop_control(self):
        self.management_generation += 1
        with contextlib.suppress(Exception):
            await self.core.request("stop")
        if self.motion:
            await self.motion.close()
        if self.replay:
            await self.replay.stop()
        with contextlib.suppress(Exception):
            await self.core.request("stop")
            await asyncio.sleep(0.1)
        for client in self.source_clients.values():
            await client.close()
        for name in ("exoskeleton", "vr", "ik", "backend", "observations"):
            await self.supervisor.stop(name)
        await self.core.close()
        await self.audit.close()
        await self.supervisor.stop("control")
        self.control_started = False

    async def start_media(self):
        (self.run_dir / "media.sock").unlink(missing_ok=True)
        child = await self.supervisor.start(
            "media",
            [
                sys.executable,
                "-m",
                "hc_platform.media",
                "--config",
                str(self.run_dir / "config.json"),
                "--socket",
                str(self.run_dir / "media.sock"),
            ],
            self.environment(),
        )
        await self.supervisor.wait_path(self.run_dir / "media.sock", child)

    async def apply(self, robot, overrides=None):
        async with self.mutation_lock:
            data_status = await self.status_data.status()
            if data_status.get("active_session_id"):
                raise ConfigurationError(
                    "Stop the active recording before applying a robot"
                )
            candidate = await asyncio.to_thread(self.compile, robot, overrides)
            previous, previous_robot, previous_topics = (
                self.config,
                self.robot,
                self.topics,
            )
            await self.stop_control()
            await self.supervisor.stop("media")
            try:
                self.config, self.robot = candidate, robot
                if robot != previous_robot and not self.topics.path.exists():
                    self.topics = TopicRegistry(
                        self.topics.path, candidate.get("topics", [])
                    )
                self.config["topics"] = self.topics.items
                atomic_json(self.run_dir / "config.json", self.config)
                await self.start_control()
                await self.start_media()
                if overrides is not None:
                    await asyncio.to_thread(
                        atomic_json, self.overrides_path(robot), overrides
                    )
                atomic_json(
                    self.registry.active_path,
                    {"robot_id": robot, "config_hash": candidate["config_hash"]},
                )
            except Exception:
                await self.stop_control()
                await self.supervisor.stop("media")
                self.config, self.robot, self.topics = (
                    previous,
                    previous_robot,
                    previous_topics,
                )
                atomic_json(self.run_dir / "config.json", previous)
                await self.start_control()
                await self.start_media()
                raise
            return {
                "applied_config_hash": candidate["config_hash"],
                "robot_id": robot,
                "requires_explicit_recovery": True,
            }

    def read_adapter_state(self, name):
        try:
            value = json.loads((self.run_dir / f"{name}.json").read_text())
            child = self.supervisor.children.get(name)
            value["process_alive"] = bool(child and child.process.returncode is None)
            value["status_stale"] = (
                value.get("pid") != (child.process.pid if child else None)
                or time.monotonic_ns() - value.get("updated_mono_ns", 0) > 2_000_000_000
            )
            return value
        except (OSError, ValueError):
            return {"process_alive": False, "reason": "NO_ADAPTER_STATUS"}

    async def status(self):
        async with self._status_lock:
            if self._status and time.monotonic() - self._status_at < 0.08:
                return self._status
            self._status = await self._get_status()
            self._status_at = time.monotonic()
            return self._status

    async def _get_status(self):
        try:
            response = await self.core.request("status")
            core = response.get("state", self.core.state)
        except Exception as exc:
            core = {"safety": "unavailable", "reason": str(exc)}
        try:
            data = await self.status_data.status()
        except Exception as exc:
            data = {"reason": str(exc)}
        return {
            "schema": "hc-platform-state/v1",
            "runtime_mode": self.mode,
            "ros_domain_id": self.domain,
            "robot_id": self.robot,
            "config_hash": self.config.get("config_hash"),
            "control": core,
            "vr": self.read_adapter_state("vr"),
            "backend": self.read_adapter_state("backend"),
            "ik": self.read_adapter_state("ik"),
            "recording": data,
            "processes": self.supervisor.status(),
            "dagger_recording": self.recording_policy,
            "log_session": self.supervisor.session_id,
        }

    async def recording_preflight(self, spec):
        selected = spec.get("topics", self.topics.selected())
        if not isinstance(selected, list) or len(selected) > 256:
            raise ConfigurationError(
                "Recording topics must be a list of at most 256 entries"
            )
        for item in selected:
            TopicRegistry.validate(item)
        data_status = await self.status_data.status()
        try:
            media = (await self.media.get("/status")).json()
        except Exception as exc:
            media = {"reason": str(exc)}
        graph = {item["topic"]: item["types"] for item in media.get("topics", [])}
        metrics = media.get("topic_metrics", {})
        type_errors = {}
        missing_types = sorted(
            {
                item["type"]
                for item in selected
                if media.get("ros_available") and item["topic"] not in metrics
            }
        )
        if missing_types:
            with contextlib.suppress(Exception):
                validation = await self.media.get(
                    "/validate-types", params={"types": ",".join(missing_types)}
                )
                if validation.status_code == 200:
                    payload = validation.json()
                    if isinstance(payload.get("results"), dict):
                        type_errors = payload["results"]
        problems = []
        for item in selected:
            name = item["topic"]
            metric = metrics.get(name, {})
            age = metric.get("age_ms")
            reason = (
                "SUBSCRIPTION_UNAVAILABLE"
                if str(metric.get("error", "")).startswith("SUBSCRIPTION_UNAVAILABLE")
                or type_errors.get(item["type"])
                else "TOPIC_UNDISCOVERED"
                if name not in graph
                else "TYPE_MISMATCH"
                if item["type"] not in graph[name]
                else "NO_FRESH_DATA"
                if age is None or age > 2000
                else None
            )
            if reason:
                problems.append(
                    {
                        "topic": name,
                        "reason": reason,
                        "required": bool(item.get("required")),
                    }
                )
        blockers = [
            item
            for item in problems
            if item["required"]
            or item["reason"] in ("TYPE_MISMATCH", "SUBSCRIPTION_UNAVAILABLE")
        ]
        if selected and not media.get("ros_available"):
            blockers.append({"reason": "ROS_CAPTURE_UNAVAILABLE"})
        if data_status.get("active_session_id"):
            blockers.append({"reason": "RECORDING_BUSY"})
        if self.replay and (await self.replay.status()).get("phase") in REPLAY_ACTIVE:
            blockers.append({"reason": "REPLAY_ACTIVE"})
        if spec.get("auto_upload") and not spec.get("platform_task_id"):
            blockers.append({"reason": "PLATFORM_TASK_REQUIRED"})
        result = {
            "ready": not blockers,
            "blockers": blockers,
            "warnings": problems,
            "selected_count": len(selected),
            "data": data_status,
            "config_hash": self.config["config_hash"],
        }
        task_id = spec.get("platform_task_id")
        if task_id:
            source_format = (
                "LEROBOT_V3"
                if spec.get("upload_format", "mcap") == "lerobot"
                else "MCAP"
            )
            try:
                task = await self.data.platform_preflight(
                    task_id, source_format, self.robot
                )
                result["platform_task"] = task
                plan_preview = task.get("capture_plan") or {}
                if isinstance(plan_preview, dict) and plan_preview.get("output"):
                    control_state = (await self.core.request("status")).get(
                        "state", self.core.state
                    )
                    preview_snapshot = {
                        **plan_preview,
                        "source_format": plan_preview.get(
                            "source_format", source_format
                        ),
                        "effective_adapter": task.get(
                            "effective_adapter",
                            plan_preview.get("effective_adapter", {}),
                        ),
                    }
                    result["conversion_preview"] = validate_capture_plan(
                        preview_snapshot,
                        self.config,
                        selected,
                        result,
                        spec,
                        control_state,
                    )
            except Exception as exc:
                blockers.append(
                    {
                        "reason": (
                            getattr(exc, "code", "CAPTURE_PLAN_INCOMPATIBLE")
                            if isinstance(exc, ConfigurationError)
                            else getattr(exc, "code", "PLATFORM_PREFLIGHT_FAILED")
                        ),
                        "message": str(exc),
                    }
                )
                result["ready"] = False
        return result

    async def start_recording(self, spec, request_id):
        selected = spec.get("topics", self.topics.selected())
        for item in selected:
            TopicRegistry.validate(item)
        previous_transition = self._recording_transition
        self._recording_transition = "starting"
        try:
            return await self._start_recording_impl(spec, request_id, selected)
        finally:
            if self._recording_transition == "starting":
                self._recording_transition = previous_transition

    async def _start_recording_impl(self, spec, request_id, selected):
        preflight = await self.recording_preflight(spec)
        if not preflight["ready"]:
            raise ConfigurationError(
                "Recording preflight failed: " + json.dumps(preflight["blockers"])
            )
        package_archive = (
            self.state_dir / "config-snapshots" / (self.config["package_hash"] + ".zip")
        )
        if not package_archive.exists():
            package_archive.parent.mkdir(parents=True, exist_ok=True)
            temporary = package_archive.with_suffix(".pending.zip")
            current_package = await asyncio.to_thread(
                self.registry.compile, self.robot, refresh=True
            )
            if current_package["config_hash"] != self.config["package_hash"]:
                raise ConfigurationError(
                    "Robot package changed on disk; apply the new version before recording"
                )
            await asyncio.to_thread(self.registry.export_zip, self.robot, temporary)
            await asyncio.to_thread(os.replace, temporary, package_archive)
        frozen = {
            **spec,
            "recording_id": str(
                uuid.uuid5(uuid.NAMESPACE_URL, "hc-platform:" + request_id)
            ),
            "topics": selected,
            "robot_id": self.robot,
            "config_hash": self.config["config_hash"],
            "robot_config": self.config,
            "runtime_mode": self.mode,
            "ros_domain_id": self.domain,
            "preflight": preflight,
        }
        frozen["robot_package_archive"] = str(package_archive)
        frozen.pop("robot_package_archive_sha256", None)
        frozen["control_audit_semantics"] = {
            "actions": "core_gate_targets_not_hardware_execution",
            "ownership": "core_ordered_until_recording_cutoff",
            "backend_receipts": "best_effort_received_before_cutoff_not_execution_proof",
        }
        task_id = frozen.get("platform_task_id")
        if frozen.get("auto_upload") and not task_id:
            raise ConfigurationError(
                "Automatic upload requires a selected platform task"
            )
        if task_id:
            source_format = (
                "LEROBOT_V3"
                if frozen.get("upload_format", "mcap") == "lerobot"
                else "MCAP"
            )
            task = await self.data.platform_preflight(
                task_id, source_format, self.robot
            )
            snapshot = await self.data.platform_prepare_recording(
                task_id,
                {
                    "recording_id": frozen["recording_id"],
                    "expected_plan_revision": task["capture_plan"]["revision"],
                    "source_format": source_format,
                },
            )
            frozen["capture_snapshot"] = snapshot
            frozen["upload_target"] = {
                "robot_id": snapshot["robot_id"],
                "collection_task_id": task_id,
                "capture_plan": {
                    key: snapshot[key]
                    for key in (
                        "plan_revision",
                        "plan_sha256",
                        "snapshot_sha256",
                        "recording_id",
                    )
                },
            }
            control_state = (await self.core.request("status")).get(
                "state", self.core.state
            )
            conversion = validate_capture_plan(
                snapshot, self.config, selected, preflight, frozen, control_state
            )
            if conversion:
                frozen["conversion"] = conversion
        result = await self.audit_data.start_session(spec=frozen, request_id=request_id)
        session_id = result.get("id", result.get("session_id"))
        if not session_id:
            raise RuntimeError("Recorder did not return a session ID")
        self.audit.dropped_events = 0
        self.active_recording = session_id
        self.recording_mode = frozen.get("mode", "ordinary")
        self.recording_cutoff_ns = None
        self.intervention_active = False
        self.audit_error = None
        atomic_json(self.run_dir / "capture-topics.json", selected)
        if selected:
            try:
                ready_path = self.run_dir / "capture-ready.json"
                ready_path.unlink(missing_ok=True)
                child = await self.supervisor.start(
                    "capture",
                    [
                        sys.executable,
                        "-m",
                        "hc_platform.capture",
                        "--socket",
                        str(self.run_dir / "data.sock"),
                        "--session-id",
                        session_id,
                        "--topics",
                        str(self.run_dir / "capture-topics.json"),
                        "--ready-file",
                        str(ready_path),
                    ],
                    self.environment(),
                )
                await self.supervisor.wait_path(ready_path, child)
                self.capture_started = True
            except BaseException as error:
                with contextlib.suppress(Exception):
                    await self.audit_data.append_sample(
                        session_id=session_id,
                        topic="/capture/gaps",
                        payload={
                            "reason": "CAPTURE_START_FAILED",
                            "message": str(error),
                            "data_integrity": "incomplete",
                        },
                    )
                await self.stop_recording()
                raise
        self.notify_vr("recording_started", result)
        return result

    async def stop_recording(self):
        previous_transition = self._recording_transition
        self._recording_transition = "stopping"
        try:
            return await self._stop_recording_impl()
        finally:
            if self._recording_transition == "stopping":
                self._recording_transition = previous_transition

    async def _stop_recording_impl(self):
        status = await self.status_data.status()
        session_id = status.get("active_session_id") or self.active_recording
        if not session_id:
            raise ValueError("No active recording")
        await self.supervisor.stop("capture", timeout=8)
        self.capture_started = False
        cutoff = time.monotonic_ns()
        self.recording_cutoff_ns = cutoff
        try:
            await self.wait_audit_watermark(cutoff, self.core.state.get("boot_id"))
        except (asyncio.TimeoutError, TimeoutError, RuntimeError) as error:
            with contextlib.suppress(Exception):
                await asyncio.wait_for(
                    self.audit_data.append_sample(
                        session_id=session_id,
                        topic="/capture/gaps",
                        payload={
                            "reason": "AUDIT_BARRIER_INCOMPLETE",
                            "message": str(error),
                            "data_integrity": "incomplete",
                        },
                        sample_monotonic_ns=cutoff,
                    ),
                    1,
                )
        self.active_recording = None
        # Capture has already stopped; clear VR headset recording state immediately
        # rather than keeping the HUD stuck on "Recording" during artifact sealing.
        self.notify_vr(
            "recording_status",
            {
                "recording": False,
                "finalizing": True,
                "session_id": session_id,
                "message": "正在保存录制...",
            },
        )
        result = await self.audit_data.stop_session(
            session_id=session_id, cutoff_monotonic_ns=cutoff
        )
        self.notify_vr("recording_stopped", result)
        return result

    async def wait_audit_watermark(self, cutoff, boot_id, timeout=1.0):
        """Broadcast state follows both audit rings on the recorder socket.

        A status *response* is not a barrier: the core prioritizes responses.
        Only a consumed periodic state proves all earlier queued audit writes
        completed. The rings are bounded and fully drained before that state.
        """
        deadline = time.monotonic() + timeout
        while True:
            if self.audit_error:
                raise RuntimeError(self.audit_error)
            if (
                self.audit_watermark_boot == boot_id
                and self.audit_watermark_ns >= cutoff
            ):
                return
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                raise TimeoutError("Control audit did not reach the recording cutoff")
            self.audit_progress.clear()
            await asyncio.wait_for(self.audit_progress.wait(), remaining)

    def notify_vr(self, kind, payload):
        if self.event_socket:
            with contextlib.suppress(OSError):
                self.event_socket.sendto(
                    json.dumps({"kind": kind, "payload": payload}).encode(),
                    str(self.run_dir / "vr-events.sock"),
                )

    async def validate_vr_management(self, event, *, max_age_ns=500_000_000):
        if self.shutdown_requested:
            raise ValueError("Runtime is shutting down")
        now = time.monotonic_ns()
        if (
            max_age_ns is not None
            and not 0 <= now - int(event.get("sample_mono_ns", 0)) <= max_age_ns
        ):
            raise ValueError("Expired VR management event")
        current = (await self.core.request("status"))["state"]
        if event.get("boot_id") != current.get("boot_id") or event.get(
            "epoch"
        ) != current.get("epoch"):
            raise ValueError(
                "VR management event belongs to a previous control generation"
            )
        return current

    async def _wait_transition_settled(self, generation, current, timeout=1.0):
        deadline = time.monotonic() + timeout
        while current.get("transition_pending") and time.monotonic() < deadline:
            if generation != self.management_generation or self.shutdown_requested:
                raise ValueError(
                    "VR operation was cancelled by stop or configuration change"
                )
            await asyncio.sleep(0.02)
            current = (await self.core.request("status"))["state"]
        if generation != self.management_generation or self.shutdown_requested:
            raise ValueError(
                "VR operation was cancelled by stop or configuration change"
            )
        return current

    async def _handle_vr_control_recover(self, event, generation, current):
        safety = current.get("safety")
        transitioned = False
        if safety in ("stopped", "fault"):
            resp = await self.core.request(
                "recover",
                expected_boot_id=event["boot_id"],
                expected_epoch=event["epoch"],
            )
            current = resp.get("state") or current
            current = await self._wait_transition_settled(generation, current)
            transitioned = True
        elif safety == "recovering" or current.get("transition_pending"):
            current = await self._wait_transition_settled(generation, current)

        if current.get("safety") != "ready" or current.get("transition_pending"):
            return transitioned
        tokens = {
            "expected_boot_id": current["boot_id"],
            "expected_epoch": current["epoch"],
        }
        mode = current.get("mode")
        if mode == "idle" and self.mode == "teleop":
            await self.core.request("session_start", mode="teleop", **tokens)
            return True
        if current.get("paused") and mode == "teleop":
            await self.core.request("resume", **tokens)
            return True
        return transitioned

    async def _resume_teleop_after_home(self, generation):
        try:
            async with self.mutation_lock:
                if (
                    self.mode != "teleop"
                    or self.shutdown_requested
                    or generation != self.management_generation
                ):
                    return
                if (
                    self.replay
                    and (await self.replay.status()).get("phase") in REPLAY_ACTIVE
                ):
                    return
                current = (await self.core.request("status"))["state"]
                current = await self._wait_transition_settled(generation, current)
                if (
                    current.get("safety") == "ready"
                    and current.get("mode") == "idle"
                    and not current.get("transition_pending")
                ):
                    await self.core.request(
                        "session_start",
                        mode="teleop",
                        expected_boot_id=current["boot_id"],
                        expected_epoch=current["epoch"],
                    )
        except Exception as exc:
            if not self.shutdown_requested:
                self.supervisor.log("vr-events", str(exc), "WARN")

    async def _handle_vr_event(self, event, kind, generation):
        try:
            await self.validate_vr_management(event)
            if kind == "record_toggle":
                async with self.mutation_lock:
                    current = await self.validate_vr_management(
                        event, max_age_ns=None
                    )
                    if generation != self.management_generation:
                        raise ValueError(
                            "VR operation was cancelled by stop or configuration change"
                        )
                    if self.active_recording:
                        await self.stop_recording()
                    elif current.get("mode", self.core.state.get("mode")) == "program":
                        raise ValueError(
                            "Ordinary recording is unavailable during an autonomous session"
                        )
                    else:
                        await self.start_recording(
                            {"format": "mcap"}, event.get("id", str(time.time_ns()))
                        )
            elif kind == "record_stop":
                async with self.mutation_lock:
                    await self.validate_vr_management(event, max_age_ns=None)
                    if generation != self.management_generation:
                        raise ValueError(
                            "VR operation was cancelled by stop or configuration change"
                        )
                    await self.stop_recording()
            elif kind == "record_mark" and self.active_recording:
                await self.audit_data.append_sample(
                    session_id=self.active_recording,
                    topic="/capture/marks",
                    payload={
                        "kind": "operator_mark",
                        "event": event,
                        "task_success": None,
                    },
                )
                self.notify_vr(
                    "recording_marked",
                    {
                        "marked": True,
                        "session_id": self.active_recording,
                        "filename": self.active_recording,
                        "message": "已标记关键片段",
                    },
                )
            elif kind in (
                "control_recover",
                "control_resume",
                "control_pause",
                "home",
            ):
                async with self.mutation_lock:
                    current = await self.validate_vr_management(
                        event, max_age_ns=None
                    )
                    if (
                        self.replay
                        and (await self.replay.status()).get("phase")
                        in REPLAY_ACTIVE
                    ):
                        raise ValueError("Replay owns control; stop replay first")
                    if generation != self.management_generation:
                        raise ValueError(
                            "VR operation was cancelled by stop or configuration change"
                        )
                    changed = False
                    if kind == "control_recover":
                        changed = await self._handle_vr_control_recover(
                            event, generation, current
                        )
                    else:
                        tokens = (
                            await self.motion.validate_home()
                            if kind == "home"
                            else {
                                "expected_boot_id": event["boot_id"],
                                "expected_epoch": event["epoch"],
                            }
                        )
                        if generation != self.management_generation:
                            raise ValueError(
                                "VR operation was cancelled by stop or configuration change"
                            )
                        await self.core.request(
                            {
                                "control_resume": "resume",
                                "control_pause": "pause",
                                "home": "home",
                            }[kind],
                            **tokens,
                        )
                        changed = kind == "control_resume"
                    if changed and self.replay:
                        mark_reset = getattr(self.replay, "mark_reset", None)
                        if callable(mark_reset):
                            await mark_reset()
        except asyncio.CancelledError:
            raise
        except Exception as exc:
            if self.shutdown_requested:
                return
            self.supervisor.log("vr-events", str(exc), "ERROR")
            self.notify_vr("recording_error", {"message": str(exc)})

    async def _vr_events(self):
        in_flight = set()
        recording_toggle_task = None

        def spawn(coro):
            task = asyncio.create_task(coro)
            in_flight.add(task)
            task.add_done_callback(in_flight.discard)
            return task

        try:
            while True:
                raw = await asyncio.get_running_loop().sock_recv(
                    self.event_socket, 65536
                )
                if self.shutdown_requested:
                    continue
                try:
                    event = json.loads(raw)
                    kind = event.get("kind")
                    generation = self.management_generation
                    now = time.monotonic_ns()
                    if not 0 <= now - int(event.get("sample_mono_ns", 0)) <= 500_000_000:
                        raise ValueError("Expired VR management event")
                    if kind == "record_toggle" and (
                        self._recording_transition is not None
                        or (
                            recording_toggle_task is not None
                            and not recording_toggle_task.done()
                        )
                    ):
                        if self._recording_transition == "stopping":
                            self.notify_vr(
                                "recording_status",
                                {
                                    "recording": False,
                                    "finalizing": True,
                                    "message": "正在保存录制，请稍候",
                                },
                            )
                        elif self._recording_transition == "starting":
                            self.notify_vr(
                                "recording_status",
                                {
                                    "recording": False,
                                    "starting": True,
                                    "message": "正在启动录制，请稍候",
                                },
                            )
                        continue
                    if kind == "record_stop" and self._recording_transition == "stopping":
                        self.notify_vr(
                            "recording_status",
                            {
                                "recording": False,
                                "finalizing": True,
                                "message": "正在保存录制，请稍候",
                            },
                        )
                        continue
                    task = spawn(self._handle_vr_event(event, kind, generation))
                    if kind == "record_toggle":
                        recording_toggle_task = task
                except Exception as exc:
                    if self.shutdown_requested:
                        continue
                    self.supervisor.log("vr-events", str(exc), "ERROR")
                    self.notify_vr("recording_error", {"message": str(exc)})
        finally:
            for task in list(in_flight):
                task.cancel()
            if in_flight:
                await asyncio.gather(*in_flight, return_exceptions=True)

    async def _flush_audit_batch(self, session_id, batch):
        if not batch or not session_id:
            return
        if len(batch) > 1 and callable(getattr(self.audit_data, "append_samples", None)):
            await self.audit_data.append_samples(session_id=session_id, samples=batch)
        else:
            for sample in batch:
                await self.audit_data.append_sample(session_id=session_id, **sample)

    async def _control_events(self):
        while True:
            events = [await self.audit.events.get()]
            for _ in range(31):
                try:
                    events.append(self.audit.events.get_nowait())
                except asyncio.QueueEmpty:
                    break
            batch = []
            batch_session = None
            for event in events:
                session_id = self.active_recording
                sample_ns = event.get(
                    "mono_ns", event.get("issued_mono_ns", event.get("monotonic_ns", 0))
                )
                beyond_cutoff = (
                    self.recording_cutoff_ns is not None
                    and isinstance(sample_ns, int)
                    and sample_ns > self.recording_cutoff_ns
                )
                if not session_id:
                    self.audit.dropped_events = 0
                if (
                    self.mode == "teleop"
                    and event.get("type") == "ownership_transition"
                    and event.get("reason") == "HOME_COMPLETE"
                    and not self.shutdown_requested
                ):
                    if self._home_resume_task is None or self._home_resume_task.done():
                        self._home_resume_task = asyncio.create_task(
                            self._resume_teleop_after_home(self.management_generation)
                        )
                if session_id and not beyond_cutoff:
                    try:
                        if (
                            self.recording_mode.startswith("dagger")
                            and event.get("type") == "ownership_transition"
                        ):
                            owner = event.get("owner", event.get("new_owner"))
                            active = owner == "human"
                            if owner is not None and active != self.intervention_active:
                                if batch:
                                    await self._flush_audit_batch(batch_session, batch)
                                    batch = []
                                    batch_session = None
                                confirmed = {
                                    **event,
                                    "confirmed": True,
                                    "event_id": str(event.get("boot_id"))
                                    + ":"
                                    + str(event.get("event_seq", event.get("seq"))),
                                    "control_epoch": event["epoch"],
                                    "monotonic_ns": event.get(
                                        "mono_ns",
                                        event.get(
                                            "issued_mono_ns", event.get("monotonic_ns")
                                        ),
                                    ),
                                }
                                result = await self.audit_data.set_intervention(
                                    session_id=session_id, active=active, event=confirmed
                                )
                                self.intervention_active = bool(
                                    result.get("active", active)
                                )
                        topic = (
                            "/platform/control/commands"
                            if event.get("type") == "output"
                            else "/platform/control/events"
                            if event.get("type") != "state"
                            else "/platform/control/state"
                        )
                        if batch_session and batch_session != session_id:
                            await self._flush_audit_batch(batch_session, batch)
                            batch = []
                        batch_session = session_id
                        batch.append({"topic": topic, "payload": event})
                        dropped = self.audit.dropped_events
                        if dropped:
                            self.audit.dropped_events = 0
                            batch.append(
                                {
                                    "topic": "/capture/gaps",
                                    "payload": {
                                        "reason": "CONTROL_AUDIT_QUEUE_OVERFLOW",
                                        "dropped": dropped,
                                    },
                                }
                            )
                    except Exception as exc:
                        batch = []
                        batch_session = None
                        self.supervisor.log("capture", str(exc), "ERROR")
                        if self.active_recording == session_id:
                            self.audit_error = str(exc)
                        if (
                            self.recording_policy.get("required")
                            and self.active_recording == session_id
                        ):
                            with contextlib.suppress(Exception):
                                await self.core.request("pause")
            if batch:
                try:
                    await self._flush_audit_batch(batch_session, batch)
                except Exception as exc:
                    self.supervisor.log("capture", str(exc), "ERROR")
                    if self.active_recording == batch_session:
                        self.audit_error = str(exc)
                    if (
                        self.recording_policy.get("required")
                        and self.active_recording == batch_session
                    ):
                        with contextlib.suppress(Exception):
                            await self.core.request("pause")
            for event in events:
                if event.get("type") == "state":
                    stamp = event.get("issued_mono_ns", 0)
                    if isinstance(stamp, int) and stamp > 0:
                        self.audit_watermark_boot = event.get("boot_id")
                        self.audit_watermark_ns = stamp
                        self.audit_progress.set()

    async def _health_watch(self):
        reported = set()
        while True:
            await asyncio.sleep(0.5)
            for name in ("management", "capture", "backend", "ik", "vr"):
                child = self.supervisor.children.get(name)
                if (
                    child
                    and child.process.returncode is not None
                    and not child.stopping
                    and child.process.pid not in reported
                ):
                    reported.add(child.process.pid)
                    self.supervisor.log(
                        name,
                        "Unexpected process exit; explicit restart is required",
                        "ERROR",
                    )
                    if name in ("backend", "ik", "vr") or self.recording_policy.get(
                        "required"
                    ):
                        self.management_generation += 1
                        with contextlib.suppress(Exception):
                            await self.core.request(
                                "stop", reason="PROCESS_EXIT_" + name.upper()
                            )
                    if name == "capture" and self.active_recording:
                        self.notify_vr(
                            "recording_error", {"reason": "ROS_CAPTURE_EXITED"}
                        )
                        with contextlib.suppress(Exception):
                            async with self.mutation_lock:
                                if self.active_recording:
                                    await self.stop_recording()

    async def close(self):
        self.shutdown_requested = True
        if self._closing_task is None:
            self.management_generation += 1
            self._closing_task = asyncio.create_task(
                self._close_owned(), name="runtime-cleanup"
            )
        try:
            await asyncio.shield(self._closing_task)
        except asyncio.CancelledError:
            # A cancelled lifespan must still reap only our own subprocesses.
            await asyncio.shield(self._closing_task)
            raise

    async def _close_owned(self):
        self.shutdown_requested = True
        try:
            for task in (self.vr_events_task, self._home_resume_task):
                if task:
                    task.cancel()
                    await asyncio.gather(task, return_exceptions=True)
            with contextlib.suppress(Exception):
                await self.core.request("stop")
                # Keep audit capture alive for the stop, fence and dispatch ack.
                deadline = time.monotonic() + 1
                await asyncio.sleep(0.1)
                while (
                    self.core.state.get("transition_pending")
                    and time.monotonic() < deadline
                ):
                    await asyncio.sleep(0.02)
            if self.active_recording:
                with contextlib.suppress(Exception):
                    async with self.mutation_lock:
                        if self.active_recording:
                            await self.stop_recording()
        finally:
            for task in self.tasks:
                task.cancel()
            await asyncio.gather(*self.tasks, return_exceptions=True)
            try:
                await self.stop_control()
            finally:
                await asyncio.gather(
                    self.media.aclose(),
                    self.data.close(),
                    self.audit_data.close(),
                    self.status_data.close(),
                    return_exceptions=True,
                )
                try:
                    await self.supervisor.close()
                finally:
                    if self.event_socket:
                        self.event_socket.close()
                    for lock in (self.instance_lock, self.hardware_lock):
                        if lock:
                            lock.close()

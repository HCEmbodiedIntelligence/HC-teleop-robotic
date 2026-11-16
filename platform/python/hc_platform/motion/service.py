"""Asynchronous motion jobs; CPU work is isolated in a killable subprocess."""

from __future__ import annotations

import asyncio
from collections import OrderedDict, deque
import contextlib
import copy
import json
import math
import os
from pathlib import Path
import sys
import time
import uuid

from .errors import MotionError
from ..configuration import atomic_json

TOKENS = ("boot_id", "epoch", "lease_id", "reference_epoch")
TERMINAL = {"succeeded", "failed", "cancelled", "aborted"}


class PlannerProcess:
    def __init__(self, runtime):
        self.runtime = runtime
        self.process = self.reader = self.error_reader = None
        self.pending = {}
        self.lock = asyncio.Lock()
        self.start_lock = asyncio.Lock()
        self.logs = deque(maxlen=20)
        self.signature = None

    async def start(self):
        async with self.start_lock:
            signature = json.dumps(
                {
                    "config_hash": self.runtime.config.get("config_hash"),
                    "motion": self.runtime.config.get("motion", {}),
                    "allowed": self.runtime.config.get("allowed_groups"),
                },
                sort_keys=True,
            )
            if (
                self.process
                and self.process.returncode is None
                and self.signature == signature
                and self.reader
                and not self.reader.done()
            ):
                return
            await self.close()
            path = Path(self.runtime.run_dir) / "motion-worker-config.json"
            await asyncio.to_thread(
                atomic_json, path, copy.deepcopy(self.runtime.config)
            )
            env = (
                self.runtime.environment()
                if hasattr(self.runtime, "environment")
                else os.environ.copy()
            )
            # Bound native BLAS pools; one planner must not occupy every control CPU.
            env.update(
                {
                    "OPENBLAS_NUM_THREADS": "1",
                    "OMP_NUM_THREADS": "1",
                    "MKL_NUM_THREADS": "1",
                }
            )
            self.process = await asyncio.create_subprocess_exec(
                sys.executable,
                "-m",
                "hc_platform.motion.worker",
                "--config",
                str(path),
                stdin=asyncio.subprocess.PIPE,
                stdout=asyncio.subprocess.PIPE,
                stderr=asyncio.subprocess.PIPE,
                env=env,
                limit=8 * 1024 * 1024,
            )
            self.signature = signature
            self.reader = asyncio.create_task(
                self._read(self.process), name="motion-worker-reader"
            )
            self.error_reader = asyncio.create_task(
                self._errors(self.process), name="motion-worker-errors"
            )

    async def _errors(self, process):
        while True:
            line = await process.stderr.readline()
            if not line:
                return
            self.logs.append(line.decode(errors="replace")[:1000].rstrip())

    async def _read(self, process):
        try:
            while True:
                line = await process.stdout.readline()
                if not line:
                    raise MotionError("PLANNER_DISCONNECTED", "; ".join(self.logs))
                if len(line) > 8 * 1024 * 1024:
                    raise MotionError("PLANNER_RESULT_TOO_LARGE")
                message = await asyncio.to_thread(json.loads, line)
                future = self.pending.get(message.get("id"))
                if future and not future.done():
                    if message.get("ok"):
                        future.set_result(message["result"])
                    else:
                        error = message.get("error", {})
                        future.set_exception(
                            MotionError(
                                error.get("code", "PLANNER_ERROR"),
                                error.get("message", ""),
                                error.get("details"),
                            )
                        )
        except asyncio.CancelledError:
            raise
        except Exception as exc:
            for future in list(self.pending.values()):
                if not future.done():
                    future.set_exception(exc)

    async def request(self, op, *, timeout=25, **payload):
        async with self.lock:
            await self.start()
            request_id = uuid.uuid4().hex
            future = asyncio.get_running_loop().create_future()
            self.pending[request_id] = future
            try:
                raw = (
                    json.dumps(
                        {"id": request_id, "op": op, **payload},
                        allow_nan=False,
                        separators=(",", ":"),
                    ).encode()
                    + b"\n"
                )
                if len(raw) > 4 * 1024 * 1024:
                    raise MotionError("PLANNER_REQUEST_TOO_LARGE")
                self.process.stdin.write(raw)
                await asyncio.wait_for(self.process.stdin.drain(), 1)
                return await asyncio.wait_for(future, timeout)
            except asyncio.TimeoutError as exc:
                await self.close()
                raise MotionError("PLANNING_TIMEOUT") from exc
            finally:
                self.pending.pop(request_id, None)

    async def close(self):
        process, self.process = self.process, None
        if process and process.returncode is None:
            with contextlib.suppress(ProcessLookupError):
                process.terminate()
            try:
                await asyncio.wait_for(process.wait(), 0.8)
            except asyncio.TimeoutError:
                with contextlib.suppress(ProcessLookupError):
                    process.kill()
                await process.wait()
        for task in (self.reader, self.error_reader):
            if task and task is not asyncio.current_task():
                task.cancel()
        tasks = [
            task
            for task in (self.reader, self.error_reader)
            if task and task is not asyncio.current_task()
        ]
        if tasks:
            await asyncio.gather(*tasks, return_exceptions=True)
        self.reader = self.error_reader = None
        for future in list(self.pending.values()):
            if not future.done():
                future.set_exception(MotionError("PLANNER_CANCELLED"))
        self.signature = None


class MotionService:
    """One program motion at a time, always using the core's existing lease.

    No method recovers safety or selects program ownership implicitly. Completed
    or cancelled jobs revoke their motion epoch through pause; explicit resume is
    required for the next job. close() is reusable after a runtime config restart.
    """

    def __init__(self, runtime):
        self.runtime = runtime
        self.worker = PlannerProcess(runtime)
        self.lock = asyncio.Lock()
        self.tasks = OrderedDict()
        self.runners = {}
        self.cancel_events = {}
        self._active = None
        self._servo = False
        self._seq = 0
        self._description = None
        self._description_hash = None

    @property
    def busy(self):
        return self._servo or self._active is not None

    def _enabled(self):
        backend = self.runtime.config.get(
            "control_backend", os.environ.get("HC_CONTROL_BACKEND", "v23")
        )
        if backend != "motion":
            raise MotionError(
                "MOTION_BACKEND_REQUIRED", "Select control_backend=motion explicitly"
            )

    async def describe(self):
        self._enabled()
        fingerprint = json.dumps(
            {
                "hash": self.runtime.config.get("config_hash"),
                "motion": self.runtime.config.get("motion", {}),
                "allowed": self.runtime.config.get("allowed_groups"),
            },
            sort_keys=True,
        )
        if fingerprint != self._description_hash:
            self._description = await self.worker.request("describe", timeout=8)
            self._description_hash = fingerprint
        return copy.deepcopy(self._description)

    async def capabilities(self):
        commands = ("ServoJ", "ServoP", "MoveJ", "MoveL", "MoveP")
        try:
            description = await self.describe()
            state = self.runtime.core.state
            enabled = {
                kind: any(
                    kind in group["commands"]
                    for group in description["groups"].values()
                )
                for kind in commands
            }
            return {
                kind: {
                    "declared": True,
                    "configured": enabled[kind],
                    "backend_supported": enabled[kind],
                    "runtime_ready": enabled[kind]
                    and state.get("safety") == "ready"
                    and state.get("owner") == "program",
                    "backend": "native-platform-motion",
                    "blocking_reasons": [] if enabled[kind] else ["GROUP_UNSUPPORTED"],
                    "collision": description["collision"],
                    "verification": self.runtime.config.get("verification", {}),
                }
                for kind in commands
            }
        except Exception as exc:
            code = getattr(exc, "code", "MOTION_DEPENDENCY_UNAVAILABLE")
            return {
                kind: {
                    "declared": True,
                    "configured": False,
                    "backend_supported": False,
                    "runtime_ready": False,
                    "blocking_reasons": [code],
                    "reason": str(exc),
                }
                for kind in commands
            }

    async def _state(self):
        result = await self.runtime.core.request("status")
        return result.get("state", self.runtime.core.state)

    @staticmethod
    def _tokens(state):
        return {key: state.get(key) for key in TOKENS}

    def _owned(self, state, tokens=None):
        if (
            state.get("safety") != "ready"
            or state.get("mode") != "program"
            or state.get("transition_pending")
        ):
            raise MotionError("PROGRAM_NOT_READY", details={"state": state})
        if state.get("owner") != "program" and not (
            state.get("handback_waiting") and state.get("requested_owner") == "program"
        ):
            raise MotionError(
                "PROGRAM_NOT_OWNER", details={"owner": state.get("owner")}
            )
        if tokens and any(state.get(key) != value for key, value in tokens.items()):
            raise MotionError("CONTROL_EPOCH_CHANGED")

    def _feedback(self, state, group):
        names = self.runtime.config.get("groups", {}).get(group)
        if (
            not names
            or group not in state.get("available_groups", [])
            or group not in state.get("leased_groups", [])
        ):
            raise MotionError("FRESH_LEASED_GROUP_REQUIRED", str(group))
        fresh = set(state.get("fresh_joint_names", []))
        if not fresh:
            # Compatibility with the first control-core build: only complete
            # available groups may contribute; never fill from configured home.
            fresh = {
                name
                for available in state.get("available_groups", [])
                for name in self.runtime.config["groups"].get(available, [])
            }
        values = {
            name: float(value)
            for name, value in zip(
                state.get("joint_names", []), state.get("positions", [])
            )
            if name in fresh
        }
        if not set(names).issubset(values) or not all(
            math.isfinite(value) for value in values.values()
        ):
            raise MotionError("FRESH_GROUP_FEEDBACK_REQUIRED")
        return values

    async def _guard(self, tokens, group, cancel=None):
        if cancel and cancel.is_set():
            raise MotionError("CANCEL_REQUESTED")
        state = await self._state()
        self._owned(state, tokens)
        return state, self._feedback(state, group)

    async def _send(self, names, positions, tokens, *, deadline_ns=None, state=None):
        now = time.monotonic_ns()
        if deadline_ns is None:
            deadline_ns = (
                now
                + min(100, self.runtime.config.get("command_timeout_ms", 150))
                * 1_000_000
            )
        remaining_ms = (deadline_ns - now) / 1e6
        if remaining_ms <= 1:
            raise MotionError("TARGET_EXPIRED")
        self._seq = max(
            self._seq + 1,
            now,
            int((state or self.runtime.core.state).get("last_command_seq", 0)) + 1,
        )
        return await self.runtime.source_clients["program"].request(
            "command",
            source="program",
            **tokens,
            seq=self._seq,
            ttl_ms=min(remaining_ms, 100),
            sample_mono_ns=now,
            deadline_mono_ns=int(deadline_ns),
            joint_names=names,
            positions=positions,
        )

    async def _servo_request(self, kind, payload):
        self._enabled()
        received = time.monotonic_ns()
        ttl = payload.get("ttl_ms", 100)
        if (
            isinstance(ttl, bool)
            or not isinstance(ttl, (int, float))
            or not math.isfinite(ttl)
            or not 10 <= ttl <= 1000
        ):
            raise MotionError("INVALID_TTL")
        deadline = received + int(ttl * 1e6)
        async with self.lock:
            if self.busy:
                raise MotionError("MOTION_BUSY")
            self._servo = True
            try:
                # Cold initialization may be slow. Keep the worker warm but do
                # not make this old target fresh by resetting its deadline.
                await self.describe()
                if time.monotonic_ns() >= deadline:
                    raise MotionError("TARGET_EXPIRED")
                state = await self._state()
                self._owned(state)
                group = payload.get("group")
                feedback = self._feedback(state, group)
                tokens = self._tokens(state)
                request = dict(payload, deadline_mono_ns=deadline)
                result = await self.worker.request(
                    "servo",
                    kind=kind,
                    request=request,
                    feedback=feedback,
                    timeout=max(0.1, ttl / 1000),
                )
                current, _ = await self._guard(tokens, group)
                response = await self._send(
                    result["joint_names"],
                    result["positions"],
                    tokens,
                    deadline_ns=deadline,
                    state=current,
                )
                return {
                    "status": "accepted",
                    "kind": kind,
                    "command_seq": self._seq,
                    "deadline_mono_ns": deadline,
                    "collision_checked": result["collision_checked"],
                    "state": response.get("state"),
                }
            finally:
                self._servo = False

    async def servo_j(self, payload):
        return await self._servo_request("ServoJ", payload)

    async def servo_p(self, payload):
        return await self._servo_request("ServoP", payload)

    async def validate_external(self, payload):
        """Validate a raw command in either backend, without renewing its age.

        Return the same command with a monotonic sample/deadline envelope. The
        caller still submits it to the core, which rechecks all authority tokens.
        """
        checked = copy.deepcopy(payload)
        received = time.monotonic_ns()
        sample = checked.get("sample_mono_ns", received)
        ttl = checked.get("ttl_ms")
        if (
            isinstance(sample, bool)
            or not isinstance(sample, int)
            or sample <= 0
            or sample > received
        ):
            raise MotionError("INVALID_SAMPLE_TIME")
        if (
            isinstance(ttl, bool)
            or not isinstance(ttl, (int, float))
            or not math.isfinite(ttl)
            or ttl <= 0
            or ttl > 5000
        ):
            raise MotionError("INVALID_TTL")
        deadline = min(received, sample) + int(
            min(ttl, self.runtime.config.get("command_timeout_ms", 150)) * 1e6
        )
        supplied_deadline = checked.get("deadline_mono_ns", deadline)
        if (
            isinstance(supplied_deadline, bool)
            or not isinstance(supplied_deadline, int)
            or supplied_deadline <= 0
        ):
            raise MotionError("INVALID_DEADLINE")
        deadline = min(deadline, supplied_deadline)
        if deadline <= received:
            raise MotionError("TARGET_EXPIRED")
        checked.update(sample_mono_ns=sample, deadline_mono_ns=deadline)
        names, values = checked.get("joint_names"), checked.get("positions")
        if (
            not isinstance(names, list)
            or not isinstance(values, list)
            or not names
            or len(names) > 64
            or len(names) != len(values)
        ):
            raise MotionError("JOINT_ARRAY_LENGTH")
        if any(not isinstance(name, str) for name in names) or len(set(names)) != len(
            names
        ):
            raise MotionError("INVALID_JOINT_NAMES")
        state = await self._state()
        source = checked.get("source")
        handback = (
            source == "program"
            and state.get("handback_waiting")
            and state.get("requested_owner") == "program"
        )
        if (
            source not in ("program", "exo", "replay")
            or (state.get("owner") != source and not handback)
            or state.get("safety") != "ready"
            or state.get("transition_pending")
        ):
            raise MotionError("NOT_OWNER")
        tokens = {key: checked.get(key) for key in TOKENS}
        if any(state.get(key) != value for key, value in tokens.items()):
            raise MotionError("CONTROL_EPOCH_CHANGED")
        allowed = {
            name
            for group in state.get("leased_groups", [])
            for name in self.runtime.config["groups"].get(group, [])
        }
        fresh = set(state.get("fresh_joint_names", []))
        if not fresh:
            fresh = {
                name
                for group in state.get("available_groups", [])
                for name in self.runtime.config["groups"].get(group, [])
            }
        if not set(names).issubset(allowed & fresh):
            raise MotionError("FRESH_LEASED_GROUP_REQUIRED")
        indices = {name: i for i, name in enumerate(self.runtime.config["joint_names"])}
        for name, value in zip(names, values):
            if (
                name not in indices
                or isinstance(value, bool)
                or not isinstance(value, (int, float))
                or not math.isfinite(value)
            ):
                raise MotionError("INVALID_JOINT_VALUE")
            index = indices[name]
            if (
                not self.runtime.config["lower"][index]
                <= value
                <= self.runtime.config["upper"][index]
            ):
                raise MotionError("JOINT_LIMIT", name)
        collision = bool(
            self.runtime.config.get("motion", {}).get("collision_enabled", False)
            or self.runtime.config.get("teleop", {})
            .get("control", {})
            .get("collision_avoidance_enabled", False)
        )
        if collision:
            feedback = {
                name: value
                for name, value in zip(
                    state.get("joint_names", []), state.get("positions", [])
                )
                if name in fresh
            }
            if not self.worker.process or self.worker.process.returncode is not None:
                # Warm even when this first old target expires; never shift its
                # capture time to make initialization fit the command lease.
                await self.worker.request("describe", timeout=8)
            remaining = (deadline - time.monotonic_ns()) / 1e9
            if remaining <= 0:
                raise MotionError("TARGET_EXPIRED")
            await self.worker.request(
                "validate_external",
                request=checked,
                feedback=feedback,
                timeout=remaining,
            )
            after = await self._state()
            if any(after.get(key) != value for key, value in tokens.items()):
                raise MotionError("CONTROL_EPOCH_CHANGED")
            observed_after = dict(
                zip(after.get("joint_names", []), after.get("positions", []))
            )
            fresh_after = set(after.get("fresh_joint_names", []))
            if not fresh_after:
                fresh_after = {
                    name
                    for group in after.get("available_groups", [])
                    for name in self.runtime.config["groups"].get(group, [])
                }
            if not set(self.runtime.config["joint_names"]).issubset(fresh_after):
                raise MotionError("COLLISION_FEEDBACK_INCOMPLETE")
            if any(
                abs(observed_after[name] - value) > 0.02
                for name, value in feedback.items()
            ):
                raise MotionError("COLLISION_START_STATE_CHANGED")
        if time.monotonic_ns() >= deadline:
            raise MotionError("TARGET_EXPIRED")
        return checked

    async def validate_home(self, groups=None):
        """Preflight native Home without bypassing configured geometry checks."""
        config = self.runtime.config
        state = await self._state()
        collision = bool(
            config.get("motion", {}).get("collision_enabled", False)
            or config.get("teleop", {})
            .get("control", {})
            .get("collision_avoidance_enabled", False)
        )
        if not collision:
            return {
                "expected_boot_id": state.get("boot_id"),
                "expected_epoch": state.get("epoch"),
            }
        if state.get("safety") != "ready":
            raise MotionError("HOME_NOT_READY")
        allowed = {
            name
            for group in config.get("allowed_groups", config["groups"])
            for name in config["groups"].get(group, [])
        }
        selected = allowed & set(config.get("home_joint_names", config["joint_names"]))
        if "home_groups" in config:
            selected &= {
                name
                for group in config["home_groups"]
                for name in config["groups"].get(group, [])
            }
        if groups is not None:
            if isinstance(groups, dict):
                groups = groups.get("groups")
            if groups is not None:
                if not isinstance(groups, list) or any(
                    group not in config["groups"] for group in groups
                ):
                    raise MotionError("INVALID_HOME_GROUPS")
                selected &= {
                    name for group in groups for name in config["groups"][group]
                }
        names = [name for name in config["joint_names"] if name in selected]
        if not names:
            raise MotionError("HOME_SCOPE_UNAVAILABLE")
        if state.get("owner") != "none" or state.get("transition_pending"):
            await self.runtime.core.request(
                "pause",
                expected_boot_id=state["boot_id"],
                expected_epoch=state["epoch"],
            )
        limit = time.monotonic() + 1
        anchor = None
        stable_since = None
        previous_seq = None
        observations = 0
        while time.monotonic() < limit:
            state = await self._state()
            if state.get("safety") != "ready":
                raise MotionError("HOME_NOT_READY")
            if state.get("transition_pending"):
                await asyncio.sleep(0.02)
                continue
            if state.get("owner") != "none":
                raise MotionError("HOME_OWNER_CHANGED")
            fresh = set(state.get("fresh_joint_names", []))
            if not fresh:
                fresh = {
                    name
                    for group in state.get("available_groups", [])
                    for name in config["groups"].get(group, [])
                }
            if not set(config["joint_names"]).issubset(fresh):
                raise MotionError("COLLISION_FEEDBACK_INCOMPLETE")
            feedback = {
                name: float(value)
                for name, value in zip(
                    state.get("joint_names", []), state.get("positions", [])
                )
                if name in fresh
            }
            values = [feedback[name] for name in names]
            if (
                anchor is None
                or max(abs(a - b) for a, b in zip(anchor, values)) > 0.003
            ):
                anchor = values
                stable_since = time.monotonic()
                observations = 0
            if state.get("feedback_seq") != previous_seq:
                observations += 1
                previous_seq = state.get("feedback_seq")
            if observations >= 2 and time.monotonic() - stable_since >= 0.1:
                break
            await asyncio.sleep(0.02)
        else:
            raise MotionError("HOME_FEEDBACK_NOT_STABLE")
        tokens = {
            "expected_boot_id": state["boot_id"],
            "expected_epoch": state["epoch"],
        }
        positions = {
            name: value for name, value in zip(config["joint_names"], config["home"])
        }
        deadline = time.monotonic_ns() + int(
            float(config.get("motion", {}).get("planning_timeout_s", 20)) * 1e9
        )
        await self.worker.request(
            "validate_home",
            request={
                "joint_names": names,
                "positions": [positions[name] for name in names],
                "deadline_mono_ns": deadline,
            },
            feedback=feedback,
            timeout=max(0.1, (deadline - time.monotonic_ns()) / 1e9) + 1,
        )
        after = await self._state()
        if (
            after.get("boot_id") != tokens["expected_boot_id"]
            or after.get("epoch") != tokens["expected_epoch"]
            or after.get("owner") != "none"
        ):
            raise MotionError("HOME_OWNER_CHANGED")
        current = dict(zip(after.get("joint_names", []), after.get("positions", [])))
        fresh = set(after.get("fresh_joint_names", []))
        if not fresh:
            fresh = {
                name
                for group in after.get("available_groups", [])
                for name in config["groups"].get(group, [])
            }
        if not set(config["joint_names"]).issubset(fresh):
            raise MotionError("COLLISION_FEEDBACK_INCOMPLETE")
        if any(abs(current[name] - value) > 0.003 for name, value in feedback.items()):
            raise MotionError("HOME_START_STATE_CHANGED")
        return tokens

    async def _submit(self, kind, payload):
        self._enabled()
        async with self.lock:
            if self.busy:
                raise MotionError("MOTION_BUSY")
            description = await self.describe()
            group = payload.get("group")
            if (
                group not in description["groups"]
                or kind not in description["groups"][group]["commands"]
            ):
                raise MotionError("GROUP_COMMAND_UNSUPPORTED")
            state = await self._state()
            self._owned(state)
            feedback = self._feedback(state, group)
            task_id = uuid.uuid4().hex
            task = {
                "id": task_id,
                "kind": kind,
                "group": group,
                "status": "queued",
                "progress": 0.0,
                "created_at_unix_ns": time.time_ns(),
                "config_hash": self.runtime.config.get("config_hash"),
                "control_tokens": self._tokens(state),
                "feedback": {},
                "result": None,
            }
            self.tasks[task_id] = task
            while len(self.tasks) > 128:
                first = next(iter(self.tasks))
                if self.tasks[first]["status"] not in TERMINAL:
                    break
                self.tasks.pop(first)
                self.runners.pop(first, None)
                self.cancel_events.pop(first, None)
            cancel = asyncio.Event()
            self.cancel_events[task_id] = cancel
            self._active = task_id
            self.runners[task_id] = asyncio.create_task(
                self._run(task, copy.deepcopy(payload), feedback, cancel),
                name="motion-" + task_id,
            )
            return copy.deepcopy(task)

    async def move_j(self, payload):
        return await self._submit("MoveJ", payload)

    async def move_l(self, payload):
        return await self._submit("MoveL", payload)

    async def move_p(self, payload):
        return await self._submit("MoveP", payload)

    async def get_task(self, task_id):
        if task_id not in self.tasks:
            raise MotionError("TASK_NOT_FOUND")
        return copy.deepcopy(self.tasks[task_id])

    async def list_tasks(self):
        return [copy.deepcopy(task) for task in reversed(self.tasks.values())]

    async def _pause_if_owned(self, tokens):
        try:
            state = await self._state()
            if (
                any(state.get(key) != value for key, value in tokens.items())
                or state.get("owner") != "program"
            ):
                return {
                    "revoked": True,
                    "reason": "OWNER_ALREADY_CHANGED",
                    "physical_stop_ack": state.get("physical_stop_ack", False),
                }
            await self.runtime.core.request(
                "pause",
                expected_boot_id=tokens["boot_id"],
                expected_epoch=tokens["epoch"],
            )
            until = time.monotonic() + 2
            while time.monotonic() < until:
                state = await self._state()
                if (
                    not state.get("transition_pending")
                    and state.get("owner") != "program"
                ):
                    return {
                        "revoked": True,
                        "reason": state.get("reason"),
                        "physical_stop_ack": state.get("physical_stop_ack", False),
                    }
                await asyncio.sleep(0.02)
            return {"revoked": False, "reason": "CANCEL_CONFIRMATION_TIMEOUT"}
        except Exception as exc:
            return {"revoked": False, "reason": str(exc)}

    async def _planning_hold(self, task, feedback, cancel, done):
        names = self.runtime.config["groups"][task["group"]]
        positions = [feedback[name] for name in names]
        while not done.is_set():
            state, _ = await self._guard(task["control_tokens"], task["group"], cancel)
            await self._send(names, positions, task["control_tokens"], state=state)
            try:
                await asyncio.wait_for(done.wait(), 0.04)
            except asyncio.TimeoutError:
                pass

    async def _run(self, task, payload, feedback, cancel):
        task["status"] = "planning"
        done = asyncio.Event()
        hold = asyncio.create_task(self._planning_hold(task, feedback, cancel, done))
        plan_future = asyncio.create_task(
            self.worker.request(
                "plan",
                kind=task["kind"],
                request=payload,
                feedback=feedback,
                timeout=float(
                    self.runtime.config.get("motion", {}).get("planning_timeout_s", 20)
                )
                + 2,
            )
        )
        try:
            finished, _ = await asyncio.wait(
                (hold, plan_future), return_when=asyncio.FIRST_COMPLETED
            )
            if hold in finished:
                await hold
                raise MotionError("PLANNING_INTERRUPTED")
            plan = await plan_future
            done.set()
            await hold
            if cancel.is_set():
                raise MotionError("CANCEL_REQUESTED")
            state, current = await self._guard(
                task["control_tokens"], task["group"], cancel
            )
            names = plan["joint_names"]
            if (
                max(
                    abs(current[name] - initial)
                    for name, initial in zip(names, plan["positions"][0])
                )
                > 0.05
            ):
                raise MotionError("PLANNING_START_STATE_CHANGED")
            task.update(
                status="running",
                started_at_unix_ns=time.time_ns(),
                duration_s=plan["duration_s"],
                collision_checked=plan["collision_checked"],
                planning_elapsed_ms=plan["planning_elapsed_ms"],
            )
            await self._execute(task, plan, cancel)
            task["status"] = "succeeded"
            task["result"] = {
                "code": "SUCCEEDED",
                "reached": True,
                "requires_resume": True,
                "collision_checked": plan["collision_checked"],
                "feedback_confirmed": True,
            }
        except asyncio.CancelledError:
            task["status"] = "cancelled"
            task["result"] = {"code": "CANCELLED", "requires_resume": True}
        except Exception as exc:
            code = getattr(exc, "code", "MOTION_FAILED")
            if cancel.is_set() or code in ("CANCEL_REQUESTED", "PLANNER_CANCELLED"):
                status, code = "cancelled", "CANCELLED"
            else:
                status = (
                    "aborted"
                    if code
                    in (
                        "CONTROL_EPOCH_CHANGED",
                        "PROGRAM_NOT_OWNER",
                        "PROGRAM_NOT_READY",
                    )
                    else "failed"
                )
            task["status"] = status
            task["result"] = {
                "code": code,
                "message": str(exc),
                "details": getattr(exc, "details", {}),
                "requires_resume": True,
            }
        finally:
            terminal_status = task["status"]
            task["status"] = "finalizing"
            done.set()
            for running in (hold, plan_future):
                if not running.done():
                    running.cancel()
            await asyncio.gather(hold, plan_future, return_exceptions=True)
            if terminal_status in ("cancelled", "aborted", "failed"):
                await self.worker.close()
                self._description_hash = None
            boundary = await self._pause_if_owned(task["control_tokens"])
            task.setdefault("result", {}).update(cancel_boundary=boundary)
            task["status"] = terminal_status
            if not boundary["revoked"]:
                task["status"] = "failed"
                task["result"]["code"] = "OUTPUT_REVOCATION_UNCONFIRMED"
            task["finished_at_unix_ns"] = time.time_ns()
            if self._active == task["id"]:
                self._active = None

    async def _execute(self, task, plan, cancel):
        options = self.runtime.config.get("motion", {})
        hz = min(100, max(10, float(options.get("stream_hz", 50))))
        settle_timeout = float(options.get("settle_timeout_s", 8))
        tolerance = float(options.get("joint_tolerance", 0.015))
        settle_duration = float(options.get("settle_duration_s", 0.1))
        max_tracking_error = float(options.get("max_tracking_error", 0.3))
        names, times, points = plan["joint_names"], plan["times"], plan["positions"]
        started = time.monotonic()
        next_tick = started
        index = 0
        stable_since = None
        previous_feedback = None
        previous_feedback_time = None
        previous_feedback_seq = None
        last_observe = 0
        pose_reached = "pose_goal" not in plan
        while True:
            now = time.monotonic()
            elapsed = now - started
            if elapsed > plan["duration_s"] + settle_timeout:
                raise MotionError("MOTION_FEEDBACK_TIMEOUT")
            state, feedback = await self._guard(
                task["control_tokens"], task["group"], cancel
            )
            while index + 1 < len(times) - 1 and times[index + 1] < elapsed:
                index += 1
            if elapsed >= times[-1]:
                values = points[-1]
            else:
                fraction = max(
                    0,
                    min(
                        1,
                        (elapsed - times[index])
                        / max(1e-9, times[index + 1] - times[index]),
                    ),
                )
                values = [
                    a + (b - a) * fraction
                    for a, b in zip(points[index], points[index + 1])
                ]
            measured = [feedback[name] for name in names]
            error = max(abs(a - b) for a, b in zip(measured, values))
            if error > max_tracking_error:
                raise MotionError("TRACKING_ERROR", details={"max_joint_error": error})
            await self._send(names, values, task["control_tokens"], state=state)
            task["progress"] = min(1, elapsed / max(plan["duration_s"], 1e-9))
            task["feedback"] = {
                "joint_names": names,
                "positions": measured,
                "target_positions": values,
                "max_error": error,
                "feedback_seq": state.get("feedback_seq"),
                "elapsed_s": elapsed,
            }
            if (
                "pose_goal" in plan or plan.get("collision_checked")
            ) and now - last_observe >= 0.1:
                observe_request = {"group": task["group"]}
                if "pose_goal" in plan:
                    observe_request["goal_pose"] = plan["pose_goal"]["pose"]
                if "cartesian" in plan:
                    cart = plan["cartesian"]
                    observe_request.update(
                        line_start=cart["start_pose"], line_goal=cart["goal_pose"]
                    )
                observation = await self.worker.request(
                    "observe", request=observe_request, feedback=feedback, timeout=0.5
                )
                task["feedback"].update(observation)
                if observation.get("line_deviation_m", 0) > float(
                    options.get("max_cartesian_deviation_m", 0.015)
                ):
                    raise MotionError("CARTESIAN_PATH_DEVIATION", details=observation)
                if observation.get("line_orientation_error_rad", 0) > float(
                    options.get("max_cartesian_orientation_deviation_rad", 0.15)
                ):
                    raise MotionError(
                        "CARTESIAN_ORIENTATION_DEVIATION", details=observation
                    )
                if "pose_goal" in plan:
                    pose_reached = observation["position_error_m"] <= float(
                        options.get("position_tolerance_m", 0.005)
                    ) and observation["orientation_error_rad"] <= float(
                        options.get("orientation_tolerance_rad", 0.03)
                    )
                last_observe = now
            fresh = state.get("feedback_seq") != previous_feedback_seq
            velocity = None
            if fresh and previous_feedback is not None:
                dt = max(0.001, now - previous_feedback_time)
                velocity = max(
                    abs(a - b) / dt for a, b in zip(measured, previous_feedback)
                )
            if fresh:
                previous_feedback, previous_feedback_time, previous_feedback_seq = (
                    measured,
                    now,
                    state.get("feedback_seq"),
                )
            goal_error = max(abs(a - b) for a, b in zip(measured, points[-1]))
            if (
                elapsed >= plan["duration_s"]
                and goal_error <= tolerance
                and pose_reached
                and fresh
                and velocity is not None
                and velocity <= 0.03
            ):
                stable_since = stable_since or now
                if now - stable_since >= settle_duration:
                    return
            elif fresh:
                stable_since = None
            next_tick += 1 / hz
            if time.monotonic() - next_tick > 0.12:
                raise MotionError("STREAM_DEADLINE_MISSED")
            await asyncio.sleep(max(0, next_tick - time.monotonic()))

    async def cancel(self, task_id):
        if task_id not in self.tasks:
            raise MotionError("TASK_NOT_FOUND")
        task = self.tasks[task_id]
        if task["status"] in TERMINAL:
            return await self.get_task(task_id)
        task["status"] = "cancelling"
        self.cancel_events[task_id].set()
        await self.worker.close()
        self._description_hash = None
        runner = self.runners[task_id]
        try:
            await asyncio.wait_for(asyncio.shield(runner), 4)
        except asyncio.TimeoutError:
            runner.cancel()
            await asyncio.gather(runner, return_exceptions=True)
        return await self.get_task(task_id)

    async def close(self):
        if self._active and self.tasks[self._active]["status"] not in TERMINAL:
            await self.cancel(self._active)
        await self.worker.close()
        self._description = None
        self._description_hash = None

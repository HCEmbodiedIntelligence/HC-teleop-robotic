"""Independent observation/drive replay worker and console-facing manager.

Drive playback is a fresh fenced source. Recorded epochs, leases, sequence
numbers and deadlines are evidence only and are never reused as authority.
"""

from __future__ import annotations
import argparse
import asyncio
import bisect
import contextlib
from dataclasses import dataclass
import json
import math
import os
from pathlib import Path
import signal
import sys
import time
import uuid

from .configuration import atomic_json
from .data import DataClient, DataError
from .data.artifacts import safe_asset, verify_directory
from .data.sample_codec import iter_samples
from .ipc import CoreClient

COMMAND_TOPIC = "/platform/control/commands"
INTERFACE = "hc-control-ipc/v1"
ACTIVE = {"PREPARING", "RUNNING", "PAUSING", "PAUSED", "RESUMING", "STOPPING"}


class ReplayError(ValueError):
    def __init__(self, code, message=""):
        self.code = code
        super().__init__(message or code)


def checked_speed(value):
    value = float(value)
    if not math.isfinite(value) or not 0.05 <= value <= 4:
        raise ReplayError(
            "REPLAY_SPEED_INVALID", "Speed must be finite and between 0.05 and 4"
        )
    return value


def validate_contract(manifest, config):
    spec = manifest.get("session", {}).get("spec", {})
    recorded = spec.get("robot_config", {})
    if spec.get("config_hash") != config.get("config_hash") or not config.get(
        "config_hash"
    ):
        raise ReplayError("REPLAY_CONFIG_MISMATCH")
    if spec.get("robot_id") != config.get("robot_id"):
        raise ReplayError("REPLAY_ROBOT_MISMATCH")
    if (
        recorded.get("command_interface_version") != INTERFACE
        or config.get("command_interface_version") != INTERFACE
    ):
        raise ReplayError(
            "REPLAY_INTERFACE_VERSION_MISMATCH",
            "A declared hc-control-ipc/v1 recording is required",
        )
    for item in (recorded, config):
        names, units = item.get("joint_names", []), item.get("joint_units", [])
        if (
            not names
            or len(names) != len(units)
            or any(unit not in {"rad", "m"} for unit in units)
        ):
            raise ReplayError("REPLAY_JOINT_UNITS_REQUIRED")
    if dict(zip(recorded["joint_names"], recorded["joint_units"])) != dict(
        zip(config["joint_names"], config["joint_units"])
    ):
        raise ReplayError("REPLAY_JOINT_UNITS_MISMATCH")


@dataclass
class Point:
    time_s: float
    positions: tuple
    grippers: tuple | None = None
    base: tuple | None = None
    grippers_valid: tuple | None = None


class Trajectory:
    def __init__(self, names, points, groups, config):
        self.names, self.points, self.groups = names, points, groups
        self.config = config
        self.times = [point.time_s for point in points]
        if len(points) < 2 or self.times[-1] <= 0:
            raise ReplayError("REPLAY_TRAJECTORY_TOO_SHORT")
        self.duration = self.times[-1]

    def validate(self, speed, loop=False):
        speed = checked_speed(speed)
        indices = [self.config["joint_names"].index(name) for name in self.names]
        velocities = []
        for left, right in zip(self.points, self.points[1:]):
            dt = (right.time_s - left.time_s) / speed
            if dt <= 0 or right.time_s - left.time_s > float(
                self.config.get("replay_max_gap_s", 0.15)
            ):
                raise ReplayError("REPLAY_COMMAND_GAP")
            velocity = [(b - a) / dt for a, b in zip(left.positions, right.positions)]
            if any(
                abs(v) > self.config["velocity"][i] + 1e-6
                for v, i in zip(velocity, indices)
            ):
                raise ReplayError("REPLAY_VELOCITY_LIMIT")
            velocities.append((velocity, dt))
        for index, (velocity, dt) in enumerate(velocities):
            previous = velocities[index - 1][0] if index else [0.0] * len(indices)
            interval = (dt + velocities[index - 1][1]) / 2 if index else dt
            if any(
                abs(v - p) / interval > self.config["acceleration"][i] + 1e-5
                for v, p, i in zip(velocity, previous, indices)
            ):
                raise ReplayError("REPLAY_ACCELERATION_LIMIT")
        last_v, last_dt = velocities[-1]
        if any(
            abs(v) / last_dt > self.config["acceleration"][i] + 1e-5
            for v, i in zip(last_v, indices)
        ):
            raise ReplayError("REPLAY_END_NOT_AT_REST")
        if any(point.base is not None for point in self.points):
            limits = self.config.get("base_velocity_limit", [0.2, 0.2, 0.3])
            for point in self.points:
                if point.base is not None and any(
                    abs(value * speed) > limit + 1e-9
                    for value, limit in zip(point.base, limits)
                ):
                    raise ReplayError("REPLAY_BASE_VELOCITY_LIMIT")
            if any(
                any(abs(value) > 1e-9 for value in point.base or ())
                for point in self.points
            ):
                acceleration = self.config.get("base_acceleration_limit")
                if (
                    not isinstance(acceleration, list)
                    or len(acceleration) != 3
                    or any(not math.isfinite(v) or v <= 0 for v in acceleration)
                ):
                    raise ReplayError("REPLAY_BASE_ACCELERATION_CONFIGURATION_REQUIRED")
                if any(
                    abs(v) > 1e-9
                    for point in (self.points[0], self.points[-1])
                    for v in point.base or ()
                ):
                    raise ReplayError("REPLAY_BASE_ENDPOINT_NOT_AT_REST")
                for left, right in zip(self.points, self.points[1:]):
                    dt = (right.time_s - left.time_s) / speed
                    if any(
                        abs((b - a) * speed) / dt > limit + 1e-6
                        for a, b, limit in zip(
                            left.base or (0, 0, 0),
                            right.base or (0, 0, 0),
                            acceleration,
                        )
                    ):
                        raise ReplayError("REPLAY_BASE_ACCELERATION_LIMIT")
        if loop:
            if any(
                abs(a - b) > float(self.config.get("replay_loop_tolerance", 0.001))
                for a, b in zip(self.points[0].positions, self.points[-1].positions)
            ):
                raise ReplayError("REPLAY_LOOP_ENDPOINT_MISMATCH")
            # Exact endpoint equality avoids manufacturing a wrap trajectory.
            if any(
                abs(a - b) > 1e-8
                for a, b in zip(self.points[0].positions, self.points[-1].positions)
            ):
                raise ReplayError("REPLAY_LOOP_REQUIRES_ALIGNED_ENDPOINTS")
            if (
                self.points[0].grippers != self.points[-1].grippers
                or self.points[0].base != self.points[-1].base
                or self.points[0].grippers_valid != self.points[-1].grippers_valid
            ):
                raise ReplayError("REPLAY_LOOP_AUX_MISMATCH")
        return {
            "duration_s": self.duration / speed,
            "points": len(self.points),
            "joint_names": self.names,
            "groups": self.groups,
        }

    def at(self, elapsed):
        if elapsed >= self.duration:
            return self.points[-1], len(self.points) - 1
        index = max(0, bisect.bisect_right(self.times, elapsed) - 1)
        left, right = (
            self.points[index],
            self.points[min(index + 1, len(self.points) - 1)],
        )
        alpha = (
            0
            if right.time_s == left.time_s
            else max(
                0.0, min(1.0, (elapsed - left.time_s) / (right.time_s - left.time_s))
            )
        )
        return Point(
            elapsed,
            tuple(a + alpha * (b - a) for a, b in zip(left.positions, right.positions)),
            left.grippers,
            left.base,
            left.grippers_valid,
        ), index

    def check_alignment(self, state, elapsed=0):
        age = state.get("feedback_age_ns", float("inf"))
        if (
            not isinstance(age, (int, float))
            or not math.isfinite(age)
            or not 0 <= age <= self.config.get("feedback_timeout_ms", 200) * 1_000_000
        ):
            raise ReplayError("REPLAY_FEEDBACK_STALE")
        feedback = dict(zip(state.get("joint_names", []), state.get("positions", [])))
        point, _ = self.at(elapsed)
        tolerance = float(self.config.get("replay_start_tolerance", 0.02))
        if any(
            name not in feedback
            or not math.isfinite(feedback[name])
            or abs(feedback[name] - value) > tolerance
            for name, value in zip(self.names, point.positions)
        ):
            raise ReplayError(
                "REPLAY_START_POSE_MISMATCH",
                "Move the robot to the selected trajectory start before replay",
            )


def load_trajectory(path, manifest, config, options):
    validate_contract(manifest, config)
    groups = options.get("groups") or [
        name for name in config.get("allowed_groups", []) if name in config["groups"]
    ]
    if not groups or any(
        group not in config.get("allowed_groups", []) or group not in config["groups"]
        for group in groups
    ):
        raise ReplayError("REPLAY_GROUP_NOT_AUTHORIZED")
    names = [
        name
        for name in config["joint_names"]
        if any(name in config["groups"][group] for group in groups)
    ]
    if not names:
        raise ReplayError("REPLAY_EMPTY_JOINT_SET")
    points, first_ns, boot, last_seq, stopped = [], None, None, None, False
    indices = [config["joint_names"].index(name) for name in names]
    for sample in iter_samples(path, topics=[COMMAND_TOPIC, "/capture/gaps"]):
        if sample["topic"] == "/capture/gaps":
            raise ReplayError("REPLAY_RECORDED_DATA_GAP")
        payload = sample["payload"]
        if (
            not isinstance(payload, dict)
            or payload.get("stage") != "core_gate"
            or payload.get("type") != "output"
        ):
            raise ReplayError("REPLAY_UNVERIFIED_COMMAND_SOURCE")
        if payload.get("config_hash") != config["config_hash"]:
            raise ReplayError("REPLAY_CONFIG_MISMATCH")
        if payload.get("mode") == "stop":
            stopped = bool(points)
            continue
        if payload.get("mode") not in {"hold", "position"}:
            continue
        if stopped:
            raise ReplayError(
                "REPLAY_MULTIPLE_MOTION_SEGMENTS",
                "Select a continuous recording without an intervening stop",
            )
        if boot is None:
            boot = payload.get("boot_id")
        if not boot or payload.get("boot_id") != boot:
            raise ReplayError("REPLAY_RECORDED_CORE_RESTART")
        sequence = payload.get("seq")
        if type(sequence) is not int or (last_seq is not None and sequence <= last_seq):
            raise ReplayError("REPLAY_RECORDED_SEQUENCE_INVALID")
        last_seq = sequence
        source_names, positions = (
            payload.get("joint_names", []),
            payload.get("positions", []),
        )
        if len(source_names) != len(set(source_names)) or len(source_names) != len(
            positions
        ):
            raise ReplayError("REPLAY_JOINT_MAPPING_INVALID")
        mapped = dict(zip(source_names, positions))
        if not all(name in mapped for name in names):
            raise ReplayError("REPLAY_REQUIRED_JOINT_MISSING")
        values = tuple(float(mapped[name]) for name in names)
        if any(
            not math.isfinite(value)
            or not config["lower"][i] <= value <= config["upper"][i]
            for value, i in zip(values, indices)
        ):
            raise ReplayError("REPLAY_JOINT_LIMIT")
        stamp = payload.get("issued_mono_ns")
        if type(stamp) is not int or stamp < 0:
            raise ReplayError("REPLAY_COMMAND_TIMESTAMP_REQUIRED")
        if first_ns is None:
            first_ns = stamp
        elapsed = (stamp - first_ns) / 1e9
        if points and elapsed <= points[-1].time_s:
            raise ReplayError("REPLAY_COMMAND_TIME_REVERSED")
        grippers = base = grip_valid = None
        if options.get("include_grippers"):
            validity = payload.get("grippers_valid", [False, False])
            if len(validity) != 2 or any(type(v) is not bool for v in validity):
                raise ReplayError("REPLAY_GRIPPER_VALIDITY_INVALID")
            if any(validity):
                if not config.get("enable_grippers"):
                    raise ReplayError("REPLAY_GRIPPER_UNAVAILABLE")
                grippers = tuple(float(v) for v in payload["grippers"])
                if len(grippers) != 2 or any(
                    not math.isfinite(v) or not 0 <= v <= 1 for v in grippers
                ):
                    raise ReplayError("REPLAY_GRIPPER_INVALID")
                grip_valid = tuple(validity)
        if options.get("include_base"):
            if not config.get("enable_base"):
                raise ReplayError("REPLAY_BASE_UNAVAILABLE")
            base = tuple(float(v) for v in payload.get("base_velocity", [0, 0, 0]))
            limits = config.get("base_velocity_limit", [0.2, 0.2, 0.3])
            if len(base) != 3 or any(
                not math.isfinite(v) or abs(v) > limit for v, limit in zip(base, limits)
            ):
                raise ReplayError("REPLAY_BASE_LIMIT")
        points.append(Point(elapsed, values, grippers, base, grip_valid))
        if len(points) * (len(names) * 32 + 320) > int(
            options.get("max_plan_bytes", 128 * 1024 * 1024)
        ):
            raise ReplayError("REPLAY_PLAN_MEMORY_LIMIT")
    trajectory = Trajectory(names, points, groups, config)
    trajectory.validate(options.get("speed", 1), options.get("loop", False))
    return trajectory


def validate_collisions(trajectory, state, config, start_elapsed=None):
    options = config.get("teleop", {}).get("control", {})
    if not options.get("collision_avoidance_enabled"):
        return {"enabled": False}
    feedback = dict(zip(state.get("joint_names", []), state.get("positions", [])))
    if (
        not set(config["joint_names"]).issubset(feedback)
        or not all(math.isfinite(value) for value in feedback.values())
        or state.get("feedback_age_ns", float("inf"))
        > config.get("feedback_timeout_ms", 200) * 1_000_000
    ):
        raise ReplayError("COLLISION_FEEDBACK_INCOMPLETE")
    try:
        from .adapters.model import UrdfModel
        from .adapters.collision import CollisionChecker

        checker = CollisionChecker(
            UrdfModel(config.get("urdf", config.get("urdf_path"))), config
        )
    except Exception as error:
        raise ReplayError(
            "REPLAY_COLLISION_UNAVAILABLE", type(error).__name__
        ) from error
    budget = float(options.get("replay_collision_timeout_s", 30))
    if not math.isfinite(budget) or not 0 < budget <= 120:
        checker.close()
        raise ReplayError("REPLAY_COLLISION_BUDGET_INVALID")
    deadline = time.monotonic_ns() + round(budget * 1e9)
    checked = samples = 0
    try:
        current = dict(feedback)
        points = (
            trajectory.points
            if start_elapsed is None
            else [trajectory.at(start_elapsed)[0]]
            + [point for point in trajectory.points if point.time_s > start_elapsed]
        )
        for point in points:
            target = dict(current)
            target.update(zip(trajectory.names, point.positions))
            result = checker.check_path(current, target, deadline)
            if not result.safe:
                raise ReplayError(result.reason)
            checked += 1
            samples += result.samples
            current = target
        return {"enabled": True, "checked_segments": checked, "samples": samples}
    finally:
        checker.close()


class ReplayClock:
    def __init__(self, speed=1):
        self.speed = checked_speed(speed)
        self.offset = 0.0
        self.anchor = time.monotonic()
        self.paused = True

    def elapsed(self):
        return (
            self.offset
            if self.paused
            else self.offset + (time.monotonic() - self.anchor) * self.speed
        )

    def pause(self):
        self.offset = self.elapsed()
        self.paused = True

    def resume(self):
        self.anchor = time.monotonic()
        self.paused = False

    def rebase(self, elapsed):
        self.offset, self.anchor = elapsed, time.monotonic()

    def set_speed(self, value):
        self.offset = self.elapsed()
        self.anchor = time.monotonic()
        self.speed = checked_speed(value)


class RosObservationPublisher:
    def __init__(self, mapping):
        try:
            import rclpy
            from rclpy.context import Context
            from rclpy.node import Node
        except ImportError:
            raise ReplayError(
                "REPLAY_ROS_UNAVAILABLE",
                "Source the intended ROS distribution before observation playback",
            ) from None
        self.rclpy, self.mapping = rclpy, mapping
        self.context = Context()
        rclpy.init(context=self.context)
        self.node = Node("hc_observation_replay", context=self.context)
        self.publishers = {}

    def publish(self, schema, channel, message):
        from rclpy.serialization import deserialize_message
        from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
        from rosidl_runtime_py.utilities import get_message

        target = self.mapping.get(channel.topic, "/replay/" + channel.topic.lstrip("/"))
        if not target.startswith("/replay/") or ".." in target or "//" in target:
            raise ReplayError("REPLAY_OBSERVATION_NAMESPACE_REQUIRED")
        if (
            channel.message_encoding != "cdr"
            or schema is None
            or schema.encoding != "ros2msg"
        ):
            raise ReplayError("REPLAY_ROS_SCHEMA_REQUIRED", channel.topic)
        signature = (target, schema.name)
        if signature not in self.publishers:
            try:
                msg_type = get_message(schema.name)
            except (ImportError, AttributeError, ValueError):
                raise ReplayError(
                    "REPLAY_ROS_MESSAGE_TYPE_UNAVAILABLE", schema.name
                ) from None
            qos = QoSProfile(
                depth=1,
                reliability=ReliabilityPolicy.BEST_EFFORT,
                durability=DurabilityPolicy.VOLATILE,
            )
            self.publishers[signature] = (
                self.node.create_publisher(msg_type, target, qos),
                msg_type,
            )
        publisher, msg_type = self.publishers[signature]
        publisher.publish(deserialize_message(message.data, msg_type))
        self.rclpy.spin_once(self.node, timeout_sec=0)

    def close(self):
        self.node.destroy_node()
        self.rclpy.shutdown(context=self.context)


class ReplayWorker:
    def __init__(self, job):
        self.job, self.options = job, job["options"]
        self.mode = self.options["mode"]
        self.clock = ReplayClock(self.options.get("speed", 1))
        self.state = {
            "id": job["id"],
            "artifact_id": job["artifact"]["id"],
            "mode": self.mode,
            "phase": "PREPARING",
            "speed": self.clock.speed,
            "loop": self.options.get("loop", False),
            "loop_count": 0,
            "elapsed_s": 0.0,
            "duration_s": 0.0,
            "messages": 0,
            "total_messages": None,
            "error": None,
            "requires_explicit_recovery": True,
        }
        self.api = CoreClient(job["core_socket"])
        self.source = CoreClient(job["core_socket"], "replay")
        self.data = DataClient(job["data_socket"])
        self.lock, self.state_lock = asyncio.Lock(), asyncio.Lock()
        self.stop_event = asyncio.Event()
        self.trajectory = None
        self.tokens = None
        self.sequence = 0
        self.publisher = None
        self.server = None
        self.lease_keeper = None
        self.renew_at = self.status_at = 0.0

    async def report(self):
        self.state.update(elapsed_s=self.clock.elapsed(), speed=self.clock.speed)
        async with self.state_lock:
            await asyncio.to_thread(
                atomic_json, Path(self.job["status_path"]), dict(self.state)
            )

    async def core_state(self):
        return (await self.api.request("status"))["state"]

    def check_initial_fence(self, state):
        expected = self.job.get("expected_control", {})
        if state.get("boot_id") != expected.get("boot_id") or state.get(
            "epoch"
        ) != expected.get("epoch"):
            raise ReplayError(
                "REPLAY_START_FENCED", "Control state changed after the replay request"
            )

    async def wait_core(self, predicate):
        deadline = time.monotonic() + 3
        while time.monotonic() < deadline:
            state = await self.core_state()
            if predicate(state):
                return state
            if state.get("safety") == "fault":
                raise ReplayError("REPLAY_CONTROL_FAULT", state.get("reason", ""))
            await asyncio.sleep(0.01)
        raise ReplayError("REPLAY_CONTROL_TRANSITION_TIMEOUT")

    def use_tokens(self, state):
        self.tokens = {
            key: state[key]
            for key in ("boot_id", "epoch", "lease_id", "reference_epoch")
        }
        self.sequence = 0
        self.state.update(
            control_boot_id=state["boot_id"],
            control_epoch=state["epoch"],
            control_session_id=state["session_id"],
            control_reserved=True,
        )

    async def dispatch(self, point):
        state = self.source.state
        if state and (
            state.get("boot_id") != self.tokens["boot_id"]
            or state.get("epoch", 0) > self.tokens["epoch"]
        ):
            raise ReplayError("REPLAY_AUTHORITY_CHANGED")
        self.sequence += 1
        arguments = {
            **self.tokens,
            "source": "replay",
            "seq": self.sequence,
            "ttl_ms": min(100, int(self.job["config"].get("command_timeout_ms", 150))),
            "sample_mono_ns": time.monotonic_ns(),
            "joint_names": self.trajectory.names,
            "positions": point.positions,
        }
        if point.grippers is not None:
            arguments["grippers"] = point.grippers
            arguments["grippers_valid"] = point.grippers_valid
        if point.base is not None:
            arguments["base_velocity"] = [
                value * self.clock.speed for value in point.base
            ]
        await self.source.request("command", **arguments)

    async def command(self, operation, params):
        async with self.lock:
            if operation == "status":
                return dict(self.state)
            if operation == "stop":
                self.clock.pause()
                self.state["phase"] = "STOPPING"
                self.stop_event.set()
                await self.api.request("stop", reason="REPLAY_STOP_REQUESTED")
            elif operation == "pause":
                if self.state["phase"] != "RUNNING":
                    raise ReplayError("REPLAY_NOT_RUNNING")
                self.clock.pause()
                self.state["phase"] = "PAUSING"
                if self.mode == "drive":
                    await self.api.request(
                        "pause",
                        expected_boot_id=self.tokens["boot_id"],
                        expected_epoch=self.tokens["epoch"],
                    )
                    await self.wait_core(
                        lambda s: s["paused"] and not s["transition_pending"]
                    )
                self.state["phase"] = "PAUSED"
            elif operation == "resume":
                if self.state["phase"] != "PAUSED":
                    raise ReplayError("REPLAY_NOT_PAUSED")
                self.state["phase"] = "RESUMING"
                if self.mode == "drive":
                    state = await self.core_state()
                    if (
                        state.get("boot_id") != self.state.get("control_boot_id")
                        or state.get("session_id")
                        != self.state.get("control_session_id")
                        or state.get("mode") != "replay"
                        or state.get("safety") != "ready"
                        or not state.get("paused")
                        or state.get("transition_pending")
                    ):
                        raise ReplayError("REPLAY_AUTHORITY_CHANGED")
                    elapsed = min(self.clock.elapsed(), self.trajectory.duration)
                    self.trajectory.check_alignment(state, elapsed)
                    self.state["collision"] = await asyncio.to_thread(
                        validate_collisions,
                        self.trajectory,
                        state,
                        self.job["config"],
                        elapsed,
                    )
                    checked = await self.core_state()
                    if (
                        checked.get("boot_id") != state["boot_id"]
                        or checked.get("epoch") != state["epoch"]
                    ):
                        raise ReplayError("REPLAY_AUTHORITY_CHANGED")
                    self.trajectory.check_alignment(checked, elapsed)
                    if self.state["collision"]["enabled"]:
                        previous = dict(zip(state["joint_names"], state["positions"]))
                        current = dict(
                            zip(checked["joint_names"], checked["positions"])
                        )
                        if any(
                            name not in current or abs(value - current[name]) > 0.01
                            for name, value in previous.items()
                        ):
                            raise ReplayError("REPLAY_COLLISION_FEEDBACK_CHANGED")
                    await self.api.request(
                        "resume",
                        expected_boot_id=state["boot_id"],
                        expected_epoch=state["epoch"],
                    )
                    state = await self.wait_core(
                        lambda s: s["owner"] == "replay" and not s["transition_pending"]
                    )
                    self.use_tokens(state)
                    await self.dispatch(self.trajectory.at(self.clock.elapsed())[0])
                self.clock.resume()
                self.state["phase"] = "RUNNING"
            elif operation == "speed":
                speed = checked_speed(params["speed"])
                if self.mode == "drive":
                    if self.state["phase"] != "PAUSED":
                        raise ReplayError(
                            "REPLAY_PAUSE_REQUIRED",
                            "Pause drive replay before changing speed",
                        )
                    self.trajectory.validate(speed, self.options.get("loop", False))
                self.clock.set_speed(speed)
            else:
                raise ReplayError("REPLAY_OPERATION_INVALID")
            await self.report()
            return dict(self.state)

    async def handle(self, reader, writer):
        try:
            request = json.loads(await asyncio.wait_for(reader.readline(), 5))
            try:
                result = await self.command(request["op"], request)
                response = {"result": result}
            except Exception as error:
                response = {
                    "error": {
                        "code": getattr(error, "code", "REPLAY_COMMAND_FAILED"),
                        "message": str(error),
                    }
                }
                if self.state["phase"] in {"PAUSING", "RESUMING"}:
                    self.state.update(phase="FAILED", error=response["error"])
                    self.stop_event.set()
                    await self.report()
            writer.write(json.dumps(response).encode() + b"\n")
            await writer.drain()
        except (ValueError, OSError, asyncio.TimeoutError):
            pass
        finally:
            writer.close()
            with contextlib.suppress(OSError):
                await writer.wait_closed()

    async def heartbeat(self):
        now = time.monotonic()
        if now >= self.status_at:
            await self.report()
            self.status_at = now + 0.2

    async def keep_lease(self):
        try:
            while not self.stop_event.is_set():
                await self.data.acquire_artifact(
                    self.job["artifact"]["id"], self.job["owner"], 300
                )
                try:
                    await asyncio.wait_for(self.stop_event.wait(), 30)
                except asyncio.TimeoutError:
                    pass
        except Exception as error:
            self.state.update(
                phase="FAILED",
                error={"code": "REPLAY_ARTIFACT_LEASE_LOST", "message": str(error)},
            )
            self.stop_event.set()

    async def drive(self):
        self.trajectory = await asyncio.to_thread(
            load_trajectory,
            self.job["path"],
            self.job["artifact"]["manifest"],
            self.job["config"],
            self.options,
        )
        if self.stop_event.is_set():
            return
        state = await self.core_state()
        self.check_initial_fence(state)
        if (
            state.get("safety") != "ready"
            or state.get("mode") != "idle"
            or state.get("transition_pending")
        ):
            raise ReplayError(
                "REPLAY_REQUIRES_READY_IDLE",
                "Explicit recovery and idle ownership are required",
            )
        self.trajectory.check_alignment(state)
        self.state["collision"] = await asyncio.to_thread(
            validate_collisions, self.trajectory, state, self.job["config"]
        )
        if self.stop_event.is_set():
            return
        current = await self.core_state()
        self.check_initial_fence(current)
        self.trajectory.check_alignment(current)
        if self.state["collision"]["enabled"]:
            before = dict(zip(state["joint_names"], state["positions"]))
            after = dict(zip(current["joint_names"], current["positions"]))
            if any(
                name not in after or abs(after[name] - value) > 0.01
                for name, value in before.items()
            ):
                raise ReplayError("REPLAY_COLLISION_FEEDBACK_CHANGED")
        state = current
        await self.source.connect()
        self.state.update(
            control_boot_id=state["boot_id"],
            control_session_id=state["session_id"] + 1,
            control_reserved=True,
        )
        await self.api.request(
            "session_start",
            mode="replay",
            groups=self.trajectory.groups,
            expected_boot_id=state["boot_id"],
            expected_epoch=state["epoch"],
        )
        state = await self.wait_core(
            lambda s: s["owner"] == "replay" and not s["transition_pending"]
        )
        self.use_tokens(state)
        self.state.update(
            duration_s=self.trajectory.duration,
            total_messages=len(self.trajectory.points),
            phase="RUNNING",
        )
        self.clock.resume()
        period = 1 / max(1, min(200, int(self.job["config"].get("control_hz", 200))))
        while not self.stop_event.is_set():
            async with self.lock:
                started = time.monotonic()
                if self.state["phase"] == "RUNNING":
                    elapsed = self.clock.elapsed()
                    point, index = self.trajectory.at(elapsed)
                    await self.dispatch(point)
                    self.state["messages"] = index + 1
                    if elapsed >= self.trajectory.duration:
                        if self.options.get("loop"):
                            self.clock.rebase(0)
                            self.state["loop_count"] += 1
                        else:
                            self.clock.pause()
                            self.clock.rebase(self.trajectory.duration)
                            self.state["phase"] = "COMPLETED"
                            break
            await self.heartbeat()
            elapsed_work = time.monotonic() - started
            if self.state["phase"] == "RUNNING" and elapsed_work > float(
                self.options.get("max_lateness_s", 0.1)
            ):
                raise ReplayError("REPLAY_SCHEDULER_STALLED")
            await asyncio.sleep(max(0.001, period - elapsed_work))

    async def observe(self):
        from mcap.reader import make_reader

        state = await self.core_state()
        self.check_initial_fence(state)
        if state.get("mode") != "idle" or state.get("owner") != "none":
            raise ReplayError("REPLAY_REQUIRES_IDLE")
        await self.api.request("stop", reason="OBSERVATION_REPLAY")
        self.state["control_reserved"] = True
        self.publisher = RosObservationPublisher(self.options.get("mapping") or {})
        self.state["phase"] = "RUNNING"
        while not self.stop_event.is_set():
            stream = open(self.job["path"], "rb")
            try:
                reader = make_reader(stream, validate_crcs=True)
                summary = await asyncio.to_thread(reader.get_summary)
                if summary and summary.statistics:
                    stats = summary.statistics
                    selected = self.options.get("topics")
                    self.state["total_messages"] = (
                        sum(
                            count
                            for channel_id, count in stats.channel_message_counts.items()
                            if summary.channels[channel_id].topic in selected
                        )
                        if selected
                        else stats.message_count
                    )
                    self.state["duration_s"] = (
                        stats.message_end_time - stats.message_start_time
                    ) / 1e9
                iterator = iter(reader.iter_messages(topics=self.options.get("topics")))
                first = None
                self.clock.rebase(0)
                if self.state["phase"] == "RUNNING":
                    self.clock.resume()

                def next_message():
                    return next(iterator, None)

                while not self.stop_event.is_set():
                    item = await asyncio.to_thread(next_message)
                    if item is None:
                        break
                    schema, channel, message = item
                    first = message.log_time if first is None else first
                    due = (message.log_time - first) / 1e9
                    while not self.stop_event.is_set() and (
                        self.clock.paused or self.clock.elapsed() < due
                    ):
                        await self.heartbeat()
                        await asyncio.sleep(0.005)
                    if self.stop_event.is_set():
                        break
                    async with self.lock:
                        if self.clock.elapsed() - due > float(
                            self.options.get("max_lateness_s", 0.1)
                        ):
                            self.clock.rebase(due)
                        self.publisher.publish(schema, channel, message)
                        self.state["messages"] += 1
                    await self.heartbeat()
                if first is None:
                    raise ReplayError("REPLAY_NO_SELECTED_MESSAGES")
                self.state["duration_s"] = due
            finally:
                stream.close()
            if not self.options.get("loop") or self.stop_event.is_set():
                break
            self.state["loop_count"] += 1
            self.state["messages"] = 0
        if not self.stop_event.is_set():
            self.state["phase"] = "COMPLETED"
        self.clock.pause()

    async def run(self):
        path = Path(self.job["socket_path"])
        try:
            self.server = await asyncio.start_unix_server(
                self.handle, path=str(path), limit=65536
            )
            os.chmod(path, 0o600)
            self.lease_keeper = asyncio.create_task(
                self.keep_lease(), name="replay-artifact-lease"
            )
            await self.report()
            await asyncio.to_thread(
                verify_directory,
                self.job["artifact"]["root"],
                self.job["artifact"]["manifest"],
            )
            if not self.stop_event.is_set():
                await (self.drive() if self.mode == "drive" else self.observe())
            if self.stop_event.is_set() and self.state["phase"] != "FAILED":
                self.state["phase"] = "STOPPED"
        except BaseException as error:
            self.clock.pause()
            self.state.update(
                phase="FAILED",
                error={
                    "code": getattr(error, "code", "REPLAY_FAILED"),
                    "message": str(error),
                },
            )
        finally:
            if self.lease_keeper:
                self.lease_keeper.cancel()
                await asyncio.gather(self.lease_keeper, return_exceptions=True)
            if self.mode == "drive" and self.state.get("control_reserved"):
                with contextlib.suppress(Exception):
                    current = await self.core_state()
                    if (
                        current["mode"] == "replay"
                        and current["boot_id"] == self.state.get("control_boot_id")
                        and current["session_id"]
                        == self.state.get("control_session_id")
                    ):
                        await self.api.request(
                            "stop",
                            reason="REPLAY_FINISHED_REQUIRES_RECOVERY",
                            expected_boot_id=current["boot_id"],
                            expected_epoch=current["epoch"],
                        )
            with contextlib.suppress(Exception):
                await self.data.release_artifact(
                    self.job["artifact"]["id"], self.job["owner"]
                )
            if self.publisher:
                self.publisher.close()
            with contextlib.suppress(Exception):
                await self.report()
            if self.server:
                self.server.close()
                await self.server.wait_closed()
            path.unlink(missing_ok=True)
            await self.api.close()
            await self.source.close()
            await self.data.close()


class ReplayManager:
    def __init__(self, runtime):
        self.runtime = runtime
        self.socket_path = runtime.run_dir / "replay.sock"
        self.status_path = runtime.run_dir / "replay-state.json"
        self.lock = asyncio.Lock()
        self.monitor = None
        self.current = None
        self.generation = 0
        self.pending_start = None
        self.local_state = None

    async def status(self):
        if self.pending_start is not None and not self.status_path.exists():
            return dict(self.local_state or {"phase": "PREPARING"})
        try:
            state = json.loads(await asyncio.to_thread(self.status_path.read_text))
        except (OSError, ValueError):
            return dict(self.local_state or {"phase": "IDLE"})
        if self.local_state and self.local_state.get("phase") == "STOPPED":
            return dict(self.local_state)
        if self.pending_start is not None and state.get("id") != self.local_state.get(
            "id"
        ):
            return dict(self.local_state)
        child = self.runtime.supervisor.children.get("replay")
        state["process_alive"] = bool(child and child.process.returncode is None)
        if state.get("phase") in ACTIVE and not state["process_alive"]:
            state.update(
                phase="FAILED",
                error={"code": "REPLAY_WORKER_EXITED"},
                requires_explicit_recovery=True,
            )
        return state

    async def start(
        self,
        artifact_id,
        mode="observe",
        topics=None,
        mapping=None,
        speed=1.0,
        loop=False,
        groups=None,
        include_grippers=False,
        include_base=False,
    ):
        async with self.lock:
            child = self.runtime.supervisor.children.get("replay")
            if child and child.process.returncode is None:
                raise ReplayError("REPLAY_BUSY")
            self.generation += 1
            generation = self.generation
            canceled = asyncio.Event()
            self.pending_start = canceled
            identity = str(uuid.uuid4())
            self.local_state = {
                "id": identity,
                "phase": "PREPARING",
                "mode": mode,
                "artifact_id": artifact_id,
            }
            try:
                return await self._start(
                    generation,
                    canceled,
                    identity,
                    artifact_id,
                    mode,
                    topics,
                    mapping,
                    speed,
                    loop,
                    groups,
                    include_grippers,
                    include_base,
                )
            except BaseException as error:
                if self.generation == generation:
                    self.local_state = {
                        **self.local_state,
                        "phase": "FAILED",
                        "error": {
                            "code": getattr(error, "code", "REPLAY_START_FAILED")
                        },
                    }
                    self._notify_vr("replay_stopped", self.local_state, error=str(error))
                raise
            finally:
                if self.pending_start is canceled:
                    self.pending_start = None

    def _check_start(self, generation, canceled):
        if canceled.is_set() or generation != self.generation:
            raise ReplayError("REPLAY_START_CANCELED")

    async def _start(
        self,
        generation,
        canceled,
        identity,
        artifact_id,
        mode,
        topics,
        mapping,
        speed,
        loop,
        groups,
        include_grippers,
        include_base,
    ):
        # Capture authority before artifact IO/planning. A later stop and
        # recovery cannot authorize this earlier start request.
        initial = (await self.runtime.core.request("status"))["state"]
        self._check_start(generation, canceled)
        if mode not in {"observe", "drive"}:
            raise ReplayError("REPLAY_MODE_INVALID")
        if (
            initial.get("mode") != "idle"
            or initial.get("transition_pending")
            or (mode == "drive" and initial.get("safety") != "ready")
        ):
            raise ReplayError(
                "REPLAY_REQUIRES_READY_IDLE"
                if mode == "drive"
                else "REPLAY_REQUIRES_IDLE"
            )
        checked_speed(speed)
        if any(
            not str(target).startswith("/replay/")
            for target in (mapping or {}).values()
        ):
            raise ReplayError("REPLAY_OBSERVATION_NAMESPACE_REQUIRED")
        if (await self.runtime.data.status()).get("active_session_id"):
            raise ReplayError("REPLAY_RECORDING_CONFLICT")
        artifact = await self.runtime.data.get_dataset(artifact_id)
        self._check_start(generation, canceled)
        if artifact["state"] != "READY" or artifact["format"] not in {
            "mcap",
            "mcap_mp4",
        }:
            raise ReplayError("REPLAY_MCAP_REQUIRED")
        assets = [
            asset["path"]
            for asset in artifact["manifest"]["assets"]
            if asset["path"].endswith(".mcap")
        ]
        if len(assets) != 1:
            raise ReplayError("REPLAY_MCAP_AMBIGUOUS")
        if mode == "drive":
            validate_contract(artifact["manifest"], self.runtime.config)
        mcap_path = str(safe_asset(artifact["root"], assets[0]))
        owner = "replay:" + identity
        await self.runtime.data.acquire_artifact(artifact_id, owner, 300)
        try:
            self._check_start(generation, canceled)
        except BaseException:
            await self.runtime.data.release_artifact(artifact_id, owner)
            raise
        job = {
            "id": identity,
            "owner": owner,
            "artifact": artifact,
            "config": self.runtime.config,
            "path": mcap_path,
            "core_socket": str(self.runtime.run_dir / "control.sock"),
            "data_socket": str(self.runtime.run_dir / "data.sock"),
            "socket_path": str(self.socket_path),
            "status_path": str(self.status_path),
            "expected_control": {
                "boot_id": initial["boot_id"],
                "epoch": initial["epoch"],
            },
            "options": {
                "mode": mode,
                "topics": topics,
                "mapping": mapping,
                "speed": speed,
                "loop": bool(loop),
                "groups": groups,
                "include_grippers": bool(include_grippers),
                "include_base": bool(include_base),
            },
        }
        self.current = job
        job_path = self.runtime.run_dir / "replay-job.json"
        try:
            self.socket_path.unlink(missing_ok=True)
            self.status_path.unlink(missing_ok=True)
            await asyncio.to_thread(atomic_json, job_path, job)
            self._check_start(generation, canceled)
            child = await self.runtime.supervisor.start(
                "replay",
                [sys.executable, "-m", "hc_platform.replay", "--job", str(job_path)],
                self.runtime.environment(),
            )
            self._check_start(generation, canceled)
            self.monitor = asyncio.create_task(
                self._monitor(child, artifact_id, owner, identity)
            )
            deadline = time.monotonic() + 5
            while time.monotonic() < deadline:
                self._check_start(generation, canceled)
                state = await self.status()
                if state.get("id") == identity and state.get("phase") != "PREPARING":
                    if state["phase"] == "FAILED":
                        raise ReplayError(
                            state["error"]["code"], state["error"].get("message", "")
                        )
                    self._notify_vr("replay_started", state)
                    return state
                if child.process.returncode is not None:
                    raise ReplayError("REPLAY_WORKER_START_FAILED")
                await asyncio.sleep(0.025)
            preparing = {"id": identity, "phase": "PREPARING", "mode": mode, "artifact_id": artifact_id}
            self._notify_vr("replay_started", preparing)
            return {"id": identity, "phase": "PREPARING"}
        except BaseException:
            await self.runtime.supervisor.stop("replay")
            await self.runtime.data.release_artifact(artifact_id, owner)
            raise

    async def _monitor(self, child, artifact_id, owner, identity):
        await child.process.wait()
        with contextlib.suppress(Exception):
            status = await self.status()
            current = (await self.runtime.core.request("status"))["state"]
            if (
                status.get("id") == identity
                and status.get("mode") == "drive"
                and status.get("control_reserved")
                and current.get("mode") == "replay"
                and current.get("boot_id") == status.get("control_boot_id")
                and current.get("session_id") == status.get("control_session_id")
            ):
                await self.runtime.core.request(
                    "stop",
                    reason="REPLAY_WORKER_EXITED",
                    expected_boot_id=current["boot_id"],
                    expected_epoch=current["epoch"],
                )
            if status.get("id") == identity and not (
                self.local_state and self.local_state.get("phase") == "STOPPED"
            ):
                self._notify_vr("replay_stopped", status)
        with contextlib.suppress(Exception):
            await self.runtime.data.release_artifact(artifact_id, owner)

    def _notify_vr(self, kind, state_dict=None, *, error=""):
        notify = getattr(self.runtime, "notify_vr", None)
        if not callable(notify):
            return
        info = state_dict or self.local_state or {}
        phase = str(info.get("phase", "")).lower()
        if not error and isinstance(info.get("error"), dict):
            error = info["error"].get("message") or info["error"].get("code") or ""
        notify(
            kind,
            {
                "state": phase or ("idle" if kind == "replay_reset" else ""),
                "mode": info.get("mode"),
                "artifact_id": info.get("artifact_id"),
                "requires_reset": bool(info.get("requires_explicit_recovery")),
                "message": error or f"replay {phase or kind}",
                "error": str(error),
            },
        )

    async def mark_reset(self):
        state = await self.status()
        if state.get("phase") in {"STOPPED", "COMPLETED", "FAILED"} or state.get(
            "requires_explicit_recovery"
        ):
            self.local_state = {"phase": "IDLE", "requires_explicit_recovery": False}
            self.status_path.unlink(missing_ok=True)
            self._notify_vr("replay_reset", self.local_state)

    async def _command(self, operation, **params):
        reader, writer = await asyncio.wait_for(
            asyncio.open_unix_connection(str(self.socket_path)), 2
        )
        try:
            writer.write(json.dumps({"op": operation, **params}).encode() + b"\n")
            await writer.drain()
            response = json.loads(await asyncio.wait_for(reader.readline(), 5))
            if "error" in response:
                raise ReplayError(
                    response["error"]["code"], response["error"].get("message", "")
                )
            return response["result"]
        finally:
            writer.close()
            with contextlib.suppress(OSError):
                await writer.wait_closed()

    async def pause(self):
        result = await self._command("pause")
        self._notify_vr("replay_status", result)
        return result

    async def resume(self):
        result = await self._command("resume")
        self._notify_vr(
            "replay_started" if result.get("phase") == "RUNNING" else "replay_status",
            result,
        )
        return result

    async def set_speed(self, speed):
        return await self._command("speed", speed=checked_speed(speed))

    async def stop(self):
        self.generation += 1
        if self.pending_start is not None:
            self.pending_start.set()
            # Do not wait for the preparation lock. Fence a worker that races
            # subprocess creation, even if replay.sock does not exist yet.
            with contextlib.suppress(Exception):
                await self.runtime.core.request(
                    "stop", reason="REPLAY_PENDING_START_CANCELED"
                )
        else:
            # The worker's command lock may be busy validating a resume. Fence
            # an active owned session before waiting for its management RPC.
            with contextlib.suppress(Exception):
                replay_state = await self.status()
                current = (await self.runtime.core.request("status"))["state"]
                if (
                    replay_state.get("phase") in ACTIVE
                    and replay_state.get("control_reserved")
                    and current.get("mode") == "replay"
                    and current.get("boot_id") == replay_state.get("control_boot_id")
                    and current.get("session_id")
                    == replay_state.get("control_session_id")
                ):
                    await self.runtime.core.request(
                        "stop",
                        reason="REPLAY_STOP_REQUESTED",
                        expected_boot_id=current["boot_id"],
                        expected_epoch=current["epoch"],
                    )
        with contextlib.suppress(OSError, ConnectionError, asyncio.TimeoutError):
            await self._command("stop")
        await self.runtime.supervisor.stop("replay", timeout=5)
        if self.monitor:
            await self.monitor
        if self.local_state:
            self.local_state = {
                **self.local_state,
                "phase": "STOPPED",
                "requires_explicit_recovery": True,
            }
        state = await self.status()
        if state.get("phase") in {"STOPPED", "FAILED"}:
            self._notify_vr("replay_stopped", state)
        return state

    async def close(self):
        await self.stop()


async def _main(job_path):
    worker = ReplayWorker(json.loads(Path(job_path).read_text()))
    for signum in (signal.SIGINT, signal.SIGTERM):
        asyncio.get_running_loop().add_signal_handler(signum, worker.stop_event.set)
    await worker.run()


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--job", required=True)
    asyncio.run(_main(parser.parse_args().job))

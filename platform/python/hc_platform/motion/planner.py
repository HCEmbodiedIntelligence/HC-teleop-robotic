"""Bounded trajectory planning, evaluated only in the planner subprocess.

MoveL is a sampled Cartesian straight line with shortest-arc orientation. MoveP
is point-to-point to a Cartesian pose through IK; it does not imply vendor blend
semantics. All coordinates are SI in the group frame advertised by describe().
"""

from __future__ import annotations

import copy
import math
import time

import numpy as np

from ..adapters.model import (
    UrdfModel,
    pose_matrix,
    matrix_quaternion,
    rotation_vector,
    rotation,
)


from .errors import MotionError


def finite(value, label="value"):
    if isinstance(value, bool):
        raise MotionError("INVALID_NUMBER", label)
    try:
        value = float(value)
    except (ValueError, TypeError) as exc:
        raise MotionError("INVALID_NUMBER", label) from exc
    if not math.isfinite(value):
        raise MotionError("NONFINITE_NUMBER", label)
    return value


def bounded(value, low, high, label):
    value = finite(value, label)
    if not low <= value <= high:
        raise MotionError("INVALID_PARAMETER", label)
    return value


def quintic(u):
    return u * u * u * (10 + u * (-15 + 6 * u))


def slerp_matrix(a, b, u):
    vector = rotation_vector(b @ a.T)
    angle = np.linalg.norm(vector)
    return a if angle < 1e-12 else rotation(vector / angle, angle * u) @ a


class Planner:
    def __init__(self, config):
        self.config = copy.deepcopy(config)
        self.options = self.config.get("motion", {})
        self.model = UrdfModel(
            config.get("urdf", config.get("urdf_path")),
            config.get("package_root"),
            backend=self.options.get("kinematics_backend", "auto"),
        )
        self.names = tuple(config["joint_names"])
        self.index = {name: i for i, name in enumerate(self.names)}
        self.groups = config["groups"]
        self.allowed = set(config.get("allowed_groups", self.groups))
        self.hz = bounded(self.options.get("stream_hz", 50), 10, 100, "stream_hz")
        self.max_samples = int(
            bounded(self.options.get("max_samples", 5000), 2, 10000, "max_samples")
        )
        self.max_duration = bounded(
            self.options.get("max_duration_s", 120), 0.1, 600, "max_duration_s"
        )
        self.planning_timeout = bounded(
            self.options.get("planning_timeout_s", 20), 0.1, 120, "planning_timeout_s"
        )
        self.joint_step = bounded(
            self.options.get("path_joint_step", 0.02), 0.001, 0.1, "path_joint_step"
        )
        self.position_tolerance = bounded(
            self.options.get("ik_position_tolerance", 0.001),
            0.00001,
            0.02,
            "ik_position_tolerance",
        )
        self.orientation_tolerance = bounded(
            self.options.get("ik_orientation_tolerance", 0.01),
            0.0001,
            0.1,
            "ik_orientation_tolerance",
        )
        self.collision_enabled = bool(
            self.options.get("collision_enabled", False)
            or config.get("teleop", {})
            .get("control", {})
            .get("collision_avoidance_enabled", False)
        )
        self.checker = None
        if self.collision_enabled:
            from ..adapters.collision import CollisionChecker

            self.checker = CollisionChecker(self.model, self.config)

    def close(self):
        if self.checker:
            self.checker.close()

    def group(self, request):
        group = request.get("group")
        if group not in self.groups or group not in self.allowed:
            raise MotionError("GROUP_UNAVAILABLE", str(group))
        names = list(self.groups[group])
        if not names or any(name not in self.index for name in names):
            raise MotionError("INVALID_GROUP_CONFIGURATION")
        return group, names

    def describe(self):
        result = {}
        for group in self.groups:
            if group not in self.allowed:
                continue
            names = self.groups[group]
            arm = (
                self.config.get("arms", {}).get(group.removesuffix("_arm"))
                if group.endswith("_arm")
                else None
            )
            result[group] = {
                "joint_names": names,
                "commands": ["ServoJ", "MoveJ"],
                "joints": [
                    {
                        "name": name,
                        "unit": "m"
                        if self.model.joints[name].kind == "prismatic"
                        else "rad",
                        "lower": self.config["lower"][self.index[name]],
                        "upper": self.config["upper"][self.index[name]],
                        "velocity": self.config["velocity"][self.index[name]],
                        "acceleration": self.config["acceleration"][self.index[name]],
                    }
                    for name in names
                ],
            }
            if arm:
                self.model.chain(arm["ee_link"], arm["base_link"])
                result[group].update(
                    {
                        "base_frame": arm["base_link"],
                        "tip_frame": arm["ee_link"],
                        "commands": ["ServoJ", "ServoP", "MoveJ", "MoveL", "MoveP"],
                    }
                )
        return {
            "schema": "hc-platform-motion/v1",
            "groups": result,
            "joint_name_convention": "URDF model names",
            "pose_convention": "metres and quaternion xyzw in the declared group base frame",
            "kinematics_backend": self.model.backend,
            "kinematics_fallback_reason": self.model.backend_reason,
            "collision": {
                "configured": self.collision_enabled,
                "available": bool(self.checker),
                "scope": "configured static URDF self-collision pairs",
                "dynamic_world": False,
            },
            "MoveL": {
                "path": "sampled Cartesian straight line",
                "rotation": "shortest-arc SO(3)",
                "continuous_swept_volume": False,
            },
            "MoveP": {
                "path": "Cartesian goal solved by IK then joint-space point-to-point",
                "blending": False,
            },
            "stream_hz": self.hz,
            "physical_execution_verified": False,
        }

    def seed(self, names, feedback):
        if not isinstance(feedback, dict) or any(
            name not in feedback for name in names
        ):
            raise MotionError("FRESH_GROUP_FEEDBACK_REQUIRED")
        q = {name: finite(value, name) for name, value in feedback.items()}
        if any(name not in self.model.joints for name in q):
            raise MotionError("UNKNOWN_FEEDBACK_JOINT")
        self.limits({name: q[name] for name in names})
        return q

    def limits(self, values):
        for name, value in values.items():
            if name not in self.index:
                raise MotionError("UNKNOWN_JOINT", name)
            i = self.index[name]
            if (
                not self.config["lower"][i]
                <= finite(value, name)
                <= self.config["upper"][i]
            ):
                raise MotionError("JOINT_LIMIT", name)

    def target_joints(self, request, names, seed):
        input_names, values = request.get("joint_names"), request.get("positions")
        if (
            not isinstance(input_names, list)
            or not isinstance(values, list)
            or not input_names
            or len(input_names) != len(values)
        ):
            raise MotionError("JOINT_ARRAY_LENGTH")
        if len(set(input_names)) != len(input_names) or any(
            name not in names for name in input_names
        ):
            raise MotionError("INVALID_GROUP_JOINTS")
        target = {name: seed[name] for name in names}
        target.update(
            {name: finite(value, name) for name, value in zip(input_names, values)}
        )
        self.limits(target)
        return target

    def frame_and_target(self, request, group, seed):
        arm = (
            self.config.get("arms", {}).get(group.removesuffix("_arm"))
            if group.endswith("_arm")
            else None
        )
        if not arm:
            raise MotionError("CARTESIAN_GROUP_UNSUPPORTED")
        target = pose_matrix(request.get("pose", []))
        base, tip = arm["base_link"], arm["ee_link"]
        frame = request.get("frame_id", base)
        if frame not in self.model.links:
            raise MotionError("UNKNOWN_FRAME", str(frame))
        if frame != base:
            # Shared ancestors cancel. Every moving joint between the two frames
            # must have fresh observed values; never use home to fill feedback.
            relative = set(self.model.chain(frame)) ^ set(self.model.chain(base))
            required = {
                name
                for name in relative
                if self.model.joints[name].kind != "fixed"
                and not self.model.joints[name].mimic
            }
            if not required.issubset(seed):
                raise MotionError(
                    "FRAME_FEEDBACK_INCOMPLETE",
                    details={"missing": sorted(required - set(seed))},
                )
            target = self.model.fk(seed, frame, base) @ target
        return base, tip, target

    def ik(self, target, seed, names, tip, base, deadline):
        remaining_ms = (deadline - time.monotonic_ns()) / 1e6
        if remaining_ms <= 0:
            raise MotionError("PLANNING_DEADLINE")
        result = self.model.ik(
            target,
            seed,
            names,
            tip,
            base,
            max_iterations=120,
            timeout_ms=min(remaining_ms, self.options.get("ik_budget_ms", 80)),
            damping=0.01,
            max_step=0.08,
            position_tolerance=self.position_tolerance,
            orientation_tolerance=self.orientation_tolerance,
            orientation_weight=0.5,
        )
        if not result.converged:
            raise MotionError(
                "IK_NO_SOLUTION",
                result.reason,
                {
                    "position_error": result.position_error,
                    "orientation_error": result.orientation_error,
                },
            )
        target_q = {name: result.positions[name] for name in names}
        self.limits(target_q)
        return target_q

    def check_path(self, current, target, deadline):
        if time.monotonic_ns() >= deadline:
            raise MotionError("PLANNING_DEADLINE")
        self.limits(target)
        if self.checker:
            result = self.checker.check_path(current, target, deadline)
            if not result.safe:
                raise MotionError(
                    "COLLISION_REJECTED",
                    result.reason,
                    {
                        "minimum_distance": result.minimum_distance,
                        "pair": list(result.pair),
                    },
                )

    def servo(self, kind, request, feedback):
        group, names = self.group(request)
        seed = self.seed(names, feedback)
        deadline = min(
            time.monotonic_ns() + int(self.planning_timeout * 1e9),
            int(request.get("deadline_mono_ns", 2**63 - 1)),
        )
        if kind == "ServoJ":
            target = self.target_joints(request, names, seed)
        else:
            base, tip, pose = self.frame_and_target(request, group, seed)
            target = self.ik(pose, seed, names, tip, base, deadline)
        self.check_path(seed, target, deadline)
        return {
            "joint_names": names,
            "positions": [target[name] for name in names],
            "collision_checked": self.collision_enabled,
            "deadline_mono_ns": deadline,
        }

    def validate_external(self, request, feedback):
        names = request.get("joint_names", [])
        allowed = {
            name for group in self.allowed for name in self.groups.get(group, [])
        }
        if (
            not isinstance(names, list)
            or not names
            or any(name not in allowed for name in names)
        ):
            raise MotionError("GROUP_UNAVAILABLE")
        seed = self.seed(names, feedback)
        target = self.target_joints(request, names, seed)
        self.check_path(seed, target, int(request["deadline_mono_ns"]))
        return {"collision_checked": self.collision_enabled, "validated": True}

    def validate_home(self, request, feedback):
        names = request["joint_names"]
        q = self.seed(names, feedback)
        goal = self.target_joints(request, names, q)
        deadline = int(request["deadline_mono_ns"])
        dt = 1 / float(self.config.get("control_hz", 200))
        velocity = {name: 0.0 for name in names}
        max_steps = min(
            60000, int(float(self.config.get("home_timeout_ms", 30000)) / 1000 / dt) + 1
        )
        previous = dict(q)
        checked_samples = 0
        for step in range(max_steps):
            if time.monotonic_ns() >= deadline:
                raise MotionError("HOME_VALIDATION_DEADLINE")
            settled = True
            for name in names:
                index = self.index[name]
                a = min(
                    float(self.config["acceleration"][index]),
                    float(self.config.get("home_acceleration", 1.4)),
                )
                maximum_v = min(
                    float(self.config["velocity"][index]),
                    float(self.config.get("home_velocity", 0.7)),
                )
                if a <= 0 or maximum_v <= 0:
                    raise MotionError("INVALID_HOME_DYNAMICS")
                delta = goal[name] - q[name]
                braking = max(
                    0, math.sqrt(a * a * dt * dt + 2 * a * abs(delta)) - a * dt
                )
                desired = math.copysign(min(maximum_v, braking), delta)
                velocity[name] = min(
                    velocity[name] + a * dt, max(velocity[name] - a * dt, desired)
                )
                q[name] += velocity[name] * dt
                settled = (
                    settled and abs(delta) < 0.0001 and abs(velocity[name]) < 0.001
                )
            self.limits({name: q[name] for name in names})
            # Preserve curved multi-joint home motion instead of checking only
            # one straight line between its endpoints. Bound skipped segments.
            if (
                step == 0
                or settled
                or max(abs(q[name] - previous[name]) for name in names) >= 0.003
            ):
                self.check_path(previous, {name: q[name] for name in names}, deadline)
                previous = dict(q)
                checked_samples += 1
            if settled:
                self.check_path(previous, goal, deadline)
                return {
                    "validated": True,
                    "collision_checked": self.collision_enabled,
                    "samples": checked_samples,
                    "nominal_duration_s": step * dt,
                    "path": "core independent joint home dynamics",
                }
        raise MotionError("HOME_VALIDATION_TIMEOUT")

    def plan(self, kind, request, feedback):
        started = time.monotonic_ns()
        deadline = started + int(self.planning_timeout * 1e9)
        group, names = self.group(request)
        seed = self.seed(names, feedback)
        speed = bounded(request.get("speed_scale", 0.25), 0.01, 1, "speed_scale")
        acceleration_scale = bounded(
            request.get("acceleration_scale", speed), 0.01, 1, "acceleration_scale"
        )
        maximum_v = np.array(
            [self.config["velocity"][self.index[name]] * speed for name in names]
        )
        maximum_a = np.array(
            [
                self.config["acceleration"][self.index[name]] * acceleration_scale
                for name in names
            ]
        )
        start_q = np.array([seed[name] for name in names])
        cartesian = None
        if kind == "MoveJ":
            target = self.target_joints(request, names, seed)
        elif kind in ("MoveP", "MoveL"):
            base, tip, pose = self.frame_and_target(request, group, seed)
            if kind == "MoveP":
                target = self.ik(pose, seed, names, tip, base, deadline)
            else:
                cartesian = (base, tip, self.model.fk(seed, tip, base), pose)
        else:
            raise MotionError("UNKNOWN_MOTION_KIND")
        if cartesian is None:
            goal = np.array([target[name] for name in names])
            delta = np.abs(goal - start_q)
            duration = max(
                0.1,
                float(np.max(1.875 * delta / maximum_v)),
                float(np.max(np.sqrt(5.774 * delta / maximum_a))),
            )
            count = max(
                2,
                math.ceil(duration * self.hz),
                math.ceil(float(np.max(delta)) / self.joint_step),
            )
            if count > self.max_samples or duration > self.max_duration:
                raise MotionError("TRAJECTORY_TOO_LONG")
            points = [
                start_q + (goal - start_q) * quintic(i / count)
                for i in range(count + 1)
            ]
        else:
            base, tip, origin, pose = cartesian
            distance = float(np.linalg.norm(pose[:3, 3] - origin[:3, 3]))
            angle = float(
                np.linalg.norm(rotation_vector(pose[:3, :3] @ origin[:3, :3].T))
            )
            linear_v = (
                bounded(
                    request.get("linear_velocity", 0.05), 0.001, 0.5, "linear_velocity"
                )
                * speed
            )
            angular_v = (
                bounded(
                    request.get("angular_velocity", 0.3), 0.005, 2, "angular_velocity"
                )
                * speed
            )
            duration = max(0.1, 1.875 * distance / linear_v, 1.875 * angle / angular_v)
            count = max(
                2,
                math.ceil(duration * self.hz),
                math.ceil(distance / 0.003),
                math.ceil(angle / 0.02),
            )
            if count > self.max_samples or duration > self.max_duration:
                raise MotionError("TRAJECTORY_TOO_LONG")
            points = [start_q]
            previous = dict(seed)
            for i in range(1, count + 1):
                progress = quintic(i / count)
                pose_i = np.eye(4)
                pose_i[:3, 3] = origin[:3, 3] + progress * (pose[:3, 3] - origin[:3, 3])
                pose_i[:3, :3] = slerp_matrix(origin[:3, :3], pose[:3, :3], progress)
                target = self.ik(pose_i, previous, names, tip, base, deadline)
                value = np.array([target[name] for name in names])
                if np.max(np.abs(value - points[-1])) > self.joint_step * 2:
                    raise MotionError("IK_BRANCH_JUMP")
                points.append(value)
                previous.update(target)
            # Stretch time without changing the geometric path. Joint finite
            # differences obey the requested dynamic bounds after scaling.
            point_array = np.asarray(points)
            dt = duration / count
            speeds = np.diff(point_array, axis=0) / dt
            accelerations = (
                np.diff(
                    np.vstack(
                        (np.zeros((1, len(names))), speeds, np.zeros((1, len(names))))
                    ),
                    axis=0,
                )
                / dt
            )
            factor = max(
                1,
                float(np.max(np.abs(speeds) / maximum_v)),
                math.sqrt(float(np.max(np.abs(accelerations) / maximum_a))),
            )
            duration *= factor * 1.01
            if duration > self.max_duration:
                raise MotionError("TRAJECTORY_TOO_LONG")
        previous = dict(seed)
        for values in points:
            target = dict(zip(names, map(float, values)))
            self.check_path(previous, target, deadline)
            previous.update(target)
        result = {
            "kind": kind,
            "group": group,
            "joint_names": names,
            "duration_s": duration,
            "times": [duration * i / count for i in range(count + 1)],
            "positions": [list(map(float, values)) for values in points],
            "goal_positions": list(map(float, points[-1])),
            "planning_elapsed_ms": (time.monotonic_ns() - started) / 1e6,
            "collision_checked": self.collision_enabled,
            "collision_scope": "static URDF self-collision",
            "config_hash": self.config.get("config_hash"),
            "sampled_path": True,
        }
        if cartesian is not None:
            result["cartesian"] = {
                "base_frame": base,
                "tip_frame": tip,
                "start_pose": list(map(float, origin[:3, 3]))
                + list(map(float, matrix_quaternion(origin[:3, :3]))),
                "goal_pose": list(map(float, pose[:3, 3]))
                + list(map(float, matrix_quaternion(pose[:3, :3]))),
            }
        if kind in ("MoveP", "MoveL"):
            result["pose_goal"] = {
                "base_frame": base,
                "tip_frame": tip,
                "pose": list(map(float, pose[:3, 3]))
                + list(map(float, matrix_quaternion(pose[:3, :3]))),
            }
        return result

    def observe(self, request, feedback):
        group, names = self.group(request)
        seed = self.seed(names, feedback)
        arm = (
            self.config.get("arms", {}).get(group.removesuffix("_arm"))
            if group.endswith("_arm")
            else None
        )
        result = {"joint_names": names, "positions": [seed[name] for name in names]}
        if arm:
            frame = self.model.fk(seed, arm["ee_link"], arm["base_link"])
            result["pose"] = list(map(float, frame[:3, 3])) + list(
                map(float, matrix_quaternion(frame[:3, :3]))
            )
            if "line_start" in request:
                start, goal = (
                    pose_matrix(request["line_start"]),
                    pose_matrix(request["line_goal"]),
                )
                delta = goal[:3, 3] - start[:3, 3]
                if np.dot(delta, delta) > 1e-12:
                    fraction = float(
                        np.clip(
                            np.dot(frame[:3, 3] - start[:3, 3], delta)
                            / np.dot(delta, delta),
                            0,
                            1,
                        )
                    )
                else:
                    total = rotation_vector(goal[:3, :3] @ start[:3, :3].T)
                    actual = rotation_vector(frame[:3, :3] @ start[:3, :3].T)
                    fraction = float(
                        np.clip(
                            np.dot(actual, total) / max(np.dot(total, total), 1e-18),
                            0,
                            1,
                        )
                    )
                result["line_deviation_m"] = float(
                    np.linalg.norm(frame[:3, 3] - (start[:3, 3] + fraction * delta))
                )
                expected = slerp_matrix(start[:3, :3], goal[:3, :3], fraction)
                result["line_orientation_error_rad"] = float(
                    np.linalg.norm(rotation_vector(frame[:3, :3] @ expected.T))
                )
            if "goal_pose" in request:
                goal = pose_matrix(request["goal_pose"])
                result["position_error_m"] = float(
                    np.linalg.norm(frame[:3, 3] - goal[:3, 3])
                )
                result["orientation_error_rad"] = float(
                    np.linalg.norm(rotation_vector(frame[:3, :3] @ goal[:3, :3].T))
                )
        if self.checker:
            self.check_path(
                seed,
                {name: seed[name] for name in names},
                time.monotonic_ns() + 100000000,
            )
        return result

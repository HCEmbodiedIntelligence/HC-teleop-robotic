"""Configuration-driven VR relative targets and X1 optional body mapping."""

import math
import time
from dataclasses import asdict
import numpy as np

from .model import UrdfModel, pose_matrix, relative_target, rotation, rotation_vector


class TeleopSolver:
    def __init__(self, config):
        self.config = config
        self.teleop = config.get("teleop", {})
        self.options = self.teleop.get("control", self.teleop)
        self.arms = config.get("arms", self.teleop.get("arms", {}))
        self.body = self.teleop.get("body", {})
        self.model = UrdfModel(
            config.get("urdf_path", config.get("urdf")),
            config.get("package_root"),
            backend=config.get("kinematics_backend", "native"),
        )
        self.mapping = self.options.get(
            "axis_mapping", [[0, 0, 1], [-1, 0, 0], [0, 1, 0]]
        )
        self.last_status = {
            "ready": True,
            "solver": self.model.backend,
            "collision_checked": False,
        }
        self._target_history = {}
        self.collision = None
        self._collision_latch = None
        if self.options.get("collision_avoidance_enabled", False):
            from .collision import CollisionChecker

            self.collision = CollisionChecker(self.model, config)
        if self.body.get("neck_tracking_enabled", False):
            yaw, pitch = (
                self.body.get("neck_yaw_joint"),
                self.body.get("neck_pitch_joint"),
            )
            names = self.body.get("neck_joint_names", [])
            if (
                not yaw
                or not pitch
                or yaw == pitch
                or set((yaw, pitch)) != set(names)
                or any(name not in self.model.joint_names for name in names)
            ):
                raise ValueError(
                    "neck tracking requires two valid configured yaw/pitch joints"
                )

    def _smooth_target(self, target, reference, side, request):
        key = (request["boot_id"], request["epoch"], request["reference_epoch"], side)
        now = time.monotonic_ns()
        previous, timestamp = self._target_history.get(
            key, (reference, now - int(0.01 * 1e9))
        )
        dt = min(0.05, max(0.001, (now - timestamp) / 1e9))
        delta = target[:3, 3] - previous[:3, 3]
        if np.linalg.norm(delta) < float(
            self.options.get("target_position_deadband", 0.003)
        ):
            target[:3, 3] = previous[:3, 3]
        else:
            amount = float(self.options.get("target_filter_alpha", 0.4))
            delta *= max(0, min(1, amount))
            limit = float(self.options.get("target_max_linear_velocity", 0.8)) * dt
            if np.linalg.norm(delta) > limit:
                delta *= limit / np.linalg.norm(delta)
            target[:3, 3] = previous[:3, 3] + delta
        angular = rotation_vector(target[:3, :3] @ previous[:3, :3].T)
        size = np.linalg.norm(angular)
        if size < float(self.options.get("target_orientation_deadband", 0.01)):
            target[:3, :3] = previous[:3, :3]
        else:
            size = min(
                size, float(self.options.get("target_max_angular_velocity", 3.0)) * dt
            )
            target[:3, :3] = rotation(angular, size) @ previous[:3, :3]
        self._target_history = {
            k: v for k, v in self._target_history.items() if k[:3] == key[:3]
        }
        self._target_history[key] = (target.copy(), now)
        return target

    def solve(self, request):
        deadline = int(request.get("deadline_mono_ns", 0))
        if time.monotonic_ns() >= deadline:
            self.last_status = {"ready": True, "reason": "REQUEST_EXPIRED"}
            return None
        names = request["joint_names"]
        q = dict(zip(names, request["positions"]))
        initial_q = dict(q)
        reference_key = (
            request["boot_id"],
            request["epoch"],
            request["reference_epoch"],
        )
        if self._collision_latch == reference_key:
            self.last_status.update(reason="COLLISION_LATCHED_RELEASE_GRIP")
            return None
        output = {}
        details = {}
        for side in ("left", "right"):
            if not request.get("active_arms", {}).get(side):
                continue
            arm = self.arms.get(side)
            if arm is None:
                raise ValueError("missing arm configuration: " + side)
            group = arm["joint_names"]
            if any(name not in q for name in group):
                raise ValueError("missing active arm feedback")
            reference_q = dict(zip(names, request["reference_positions_" + side]))
            reference = self.model.fk(reference_q, arm["ee_link"], arm.get("base_link"))
            target = relative_target(
                reference,
                request[side + "_pose"],
                request[side + "_reference_pose"],
                self.mapping,
                float(self.options.get("position_scale", 0.85)),
                float(self.options.get("rotation_scale", 1.0))
                if self.options.get("orientation_enabled", True)
                else 0.0,
                float(self.options.get("max_displacement", 0.9)),
            )
            target = self._smooth_target(target, reference, side, request)
            remaining_ms = (deadline - time.monotonic_ns()) / 1e6
            if remaining_ms <= 1:
                return None
            result = self.model.ik(
                target,
                q,
                group,
                arm["ee_link"],
                arm.get("base_link"),
                max_iterations=int(self.options.get("ik_iterations", 20)),
                timeout_ms=min(30, remaining_ms),
                orientation_weight=float(self.options.get("orientation_weight", 0.35)),
            )
            details[side] = {
                "converged": result.converged,
                "position_error": result.position_error,
                "orientation_error": result.orientation_error,
                "iterations": result.iterations,
                "elapsed_ms": result.elapsed_ms,
                "reason": result.reason,
            }
            if not result.converged:
                self.last_status = {
                    "ready": True,
                    "arms": details,
                    "reason": "IK_NOT_CONVERGED",
                    "collision_checked": False,
                    "solver": self.model.backend,
                }
                return None
            output.update({name: result.positions[name] for name in group})
            q.update(output)
        if request.get("allow_body") and (
            self.body.get("enabled") or self.body.get("neck_tracking_enabled")
        ):
            body = (
                self._body_target(request, q, deadline)
                if self.body.get("waist_joint_names")
                else {}
            )
            if body is None:
                return None
            output.update(body)
            if self.body.get("neck_tracking_enabled"):
                output.update(self._neck_target(request, q))
        if not output or time.monotonic_ns() >= deadline:
            return None
        collision_result = None
        if self.collision:
            collision_result = self.collision.check_path(initial_q, output, deadline)
            if not collision_result.safe:
                self.last_status = {
                    "ready": True,
                    "reason": collision_result.reason,
                    "arms": details,
                    "solver": self.model.backend,
                    "collision_checked": True,
                    "collision": asdict(collision_result),
                }
                if (
                    self.options.get("collision_latch_until_release", True)
                    and collision_result.reason == "COLLISION_BLOCKED"
                ):
                    self._collision_latch = reference_key
                return None
        self.last_status = {
            "ready": True,
            "reason": "OK",
            "arms": details,
            "collision_checked": bool(self.collision),
            "solver": self.model.backend,
        }
        if collision_result:
            self.last_status["collision"] = asdict(collision_result)
        result = {
            key: request[key]
            for key in ("boot_id", "epoch", "lease_id", "reference_epoch", "seq")
        }
        result.update(
            op="ik_result",
            joint_names=list(output),
            positions=list(output.values()),
            ttl_ms=max(1, int((deadline - time.monotonic_ns()) / 1e6)),
        )
        return result

    def _body_target(self, request, q, deadline):
        group = self.body.get("waist_joint_names", [])
        if not group or any(name not in q for name in group):
            return None
        head, head_reference = (
            request.get("head_pose"),
            request.get("head_reference_pose"),
        )
        if not head or not head_reference:
            return None
        reference_q = dict(
            zip(request["joint_names"], request["reference_positions_left"])
        )
        tip = self.body.get("target_link", "zhi_Link")
        reference = self.model.fk(reference_q, tip)
        axis = np.asarray(self.body.get("pitch_axis_world", [-1, 0, 0]), dtype=float)
        axis /= np.linalg.norm(axis)
        height_axis = int(self.body.get("head_height_axis", 1))
        height = (
            (head[height_axis] - head_reference[height_axis])
            * float(self.body.get("height_scale", 0.5))
            * float(self.body.get("height_direction", 1))
        )
        height = float(
            np.clip(
                height,
                -float(self.body.get("max_height_delta", 0.18)),
                float(self.body.get("max_height_delta", 0.18)),
            )
        )
        # Unity pitch uses its local X axis; the configured world axis describes
        # the corresponding robot torso rotation direction.
        head_delta = rotation_vector(
            pose_matrix(head)[:3, :3] @ pose_matrix(head_reference)[:3, :3].T
        )
        pitch = (
            -float(head_delta[0])
            * float(self.body.get("pitch_scale", 1))
            * float(self.body.get("pitch_direction", 1))
        )
        pitch = float(
            np.clip(
                pitch,
                -float(self.body.get("max_pitch_delta", 0.35)),
                float(self.body.get("max_pitch_delta", 0.35)),
            )
        )
        target_height = reference[2, 3] + height
        target_rotation = rotation(axis, pitch) @ reference[:3, :3]
        lower, upper = (
            self.body.get("joint_lower", []),
            self.body.get("joint_upper", []),
        )
        for _ in range(min(40, int(self.body.get("ik_iterations", 15)))):
            if time.monotonic_ns() >= deadline:
                return None
            current = self.model.fk(q, tip)
            error = np.array(
                [
                    target_height - current[2, 3],
                    np.dot(rotation_vector(target_rotation @ current[:3, :3].T), axis),
                ]
            )
            if abs(error[0]) < 0.005 and abs(error[1]) < 0.04:
                return {name: q[name] for name in group}
            jacobian = self.model.jacobian(q, tip, group)
            jacobian = np.vstack([jacobian[2, :], axis @ jacobian[3:, :]])
            delta = jacobian.T @ np.linalg.solve(
                jacobian @ jacobian.T + 0.0025 * np.eye(2), error
            )
            delta = np.clip(delta, -0.08, 0.08)
            for i, name in enumerate(group):
                joint = self.model.joints[name]
                lo = (
                    max(joint.lower, lower[i])
                    if len(lower) == len(group)
                    else joint.lower
                )
                hi = (
                    min(joint.upper, upper[i])
                    if len(upper) == len(group)
                    else joint.upper
                )
                q[name] = float(np.clip(q[name] + delta[i], lo, hi))
        return None

    def _neck_target(self, request, q):
        head, reference = request.get("head_pose"), request.get("head_reference_pose")
        if not head or not reference:
            raise ValueError("neck tracking requires head reference")
        mapping = np.asarray(self.options.get("head_axis_mapping", self.mapping))
        delta = (
            mapping
            @ (pose_matrix(head)[:3, :3] @ pose_matrix(reference)[:3, :3].T)
            @ mapping.T
        )
        vector = rotation_vector(delta)
        reference_q = dict(
            zip(request["joint_names"], request["reference_positions_left"])
        )
        output = {}
        for kind, axis in (("yaw", 2), ("pitch", 1)):
            name = self.body["neck_" + kind + "_joint"]
            if name not in q or name not in reference_q:
                raise ValueError("neck tracking requires fresh named neck feedback")
            joint = self.model.joints[name]
            limit = float(
                self.body.get("neck_max_" + kind, 1.57 if kind == "yaw" else 0.8)
            )
            value = reference_q[name] + float(vector[axis]) * float(
                self.body.get("neck_" + kind + "_scale", 1)
            )
            output[name] = float(
                np.clip(value, max(joint.lower, -limit), min(joint.upper, limit))
            )
        return output

    def close(self):
        if self.collision:
            self.collision.close()


class BaseMapper:
    """X1 planar velocity mapping; no invented odometry from command integration."""

    def __init__(self, config):
        self.body = config.get("teleop", {}).get("body", {})
        self.limits = config.get("base_velocity_limit", [0.1, 0.1, 0.2])
        self.reference = None
        self.previous_grip = False

    def map(self, frame, left_grip):
        if not self.body.get("enabled") or not frame.tracked("head"):
            self.reference = None
            return [0.0, 0.0, 0.0], False
        if left_grip and not self.previous_grip:
            self.reference = frame.head.as_list()
        if not left_grip:
            self.reference = None
        self.previous_grip = left_grip
        x, y = frame.left_input.primary_axis
        yaw = frame.right_input.primary_axis[0]
        deadzone = float(self.body.get("stick_deadzone", 0.08))
        x, y = (x if abs(x) > deadzone else 0), (y if abs(y) > deadzone else 0)
        yaw = (
            yaw
            if abs(yaw) > float(self.body.get("right_stick_yaw_deadzone", 0.08))
            else 0
        )
        independent = bool(
            (x or y) and self.body.get("left_stick_independent_base", False)
        ) or bool(yaw and self.body.get("right_stick_independent_base", False))
        forward = (
            y * float(self.body.get("stick_forward_direction", 1)) * self.limits[0]
            if left_grip or self.body.get("left_stick_independent_base")
            else 0
        )
        lateral = (
            x * float(self.body.get("stick_lateral_direction", -1)) * self.limits[1]
            if left_grip or self.body.get("left_stick_independent_base")
            else 0
        )
        turn = (
            yaw * float(self.body.get("right_stick_yaw_direction", -1)) * self.limits[2]
            if self.body.get("right_stick_yaw_enabled", False)
            and (left_grip or self.body.get("right_stick_independent_base"))
            else 0
        )
        settings = self.body.get("head_pose_base", {})
        if left_grip and settings.get("enabled") and self.reference is not None:
            if x or y or yaw:
                self.reference = (
                    frame.head.as_list()
                )  # stick releases do not resurrect old head offsets
            else:
                delta = np.asarray(frame.head.position) - np.asarray(self.reference[:3])
                delta *= float(settings.get("position_scale", 1))
                maximum = float(settings.get("max_displacement", 0.5))
                if maximum > 0 and np.linalg.norm(delta) > maximum:
                    delta *= maximum / np.linalg.norm(delta)
                head_rotation = (
                    pose_matrix(frame.head.as_list())[:3, :3]
                    @ pose_matrix(self.reference)[:3, :3].T
                )
                angle = rotation_vector(head_rotation)[1] * float(
                    settings.get("yaw_scale", 1)
                )
                maximum_yaw = float(settings.get("max_yaw", 1.57))
                if maximum_yaw > 0:
                    angle = float(np.clip(angle, -maximum_yaw, maximum_yaw))
                gain = float(settings.get("position_gain", 2))
                forward = delta[2] * gain
                lateral = -delta[0] * gain
                turn = (
                    -angle
                    * float(settings.get("yaw_gain", 2))
                    * float(self.body.get("yaw_direction", 1))
                )
                if (
                    abs(forward)
                    < float(settings.get("position_tolerance", 0.01)) * gain
                ):
                    forward = 0
                if (
                    abs(lateral)
                    < float(settings.get("position_tolerance", 0.01)) * gain
                ):
                    lateral = 0
                if abs(angle) < float(settings.get("yaw_tolerance", 0.03)):
                    turn = 0
        return [
            float(np.clip(value, -limit, limit))
            for value, limit in zip((forward, lateral, turn), self.limits)
        ], independent

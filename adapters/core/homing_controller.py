"""Homing gesture, trajectory interpolation, and backend-specific home sequencing."""

from __future__ import annotations

from contextlib import nullcontext
import json
import math
from typing import Any

import numpy as np
from sensor_msgs.msg import JointState
from std_msgs.msg import Header, String

from .arm_teleop_math import ordered_homing_joint_names, sticks_outward
from .body_base_controller import clear_body_reference
from .pybullet_kinematics import reset_named_joints


def home_allowed(node: Any) -> bool:
    return (
        node.control_source == "vr"
        and not node.source_feedback_pending
        and (
            not (getattr(node, "intervention", None) and node.intervention.active)
            or node.intervention.can_home()
        )
        and node._monotonic() - node.joint_state_stamp
        <= float(node.control["joint_state_timeout"])
        and all(name in node.joint_state for name in node._homing_joint_names())
    )


def home_gesture_triggered(node: Any, now: float) -> bool:
    if not all(node._input_fresh(node.arms[side], now) for side in ("left", "right")):
        return False
    axis = int(node.control.get("home_gesture_axis", 2))
    left_x = node._axis(node.arms["left"], axis)
    right_x = node._axis(node.arms["right"], axis)
    threshold = float(node.control.get("home_gesture_threshold", 0.8))
    release = float(node.control.get("home_gesture_release_threshold", 0.35))
    if node.home_gesture_latched:
        if abs(left_x) <= release and abs(right_x) <= release:
            node.home_gesture_latched = False
        return False
    if sticks_outward(left_x, right_x, threshold):
        node.home_gesture_latched = True
        return True
    return False


def homing_joint_names(node: Any) -> list[str]:
    configured = node.control.get("home_joint_names")
    if configured is not None:
        return list(configured)
    return (
        ordered_homing_joint_names(
            node.arms["right"].joint_names,
            node.arms["left"].joint_names,
            node.body.joint_names,
        )
        + node.neck_joint_names
    )


def begin_joint_home_trajectory(node: Any) -> None:
    now = node._monotonic()
    node.homing_velocity = dict.fromkeys(node._homing_joint_names(), 0.0)
    node.homing_last_time = now
    node.homing_target_reached_time = None
    node.homing_start_time = now


def check_initial_homing(node: Any) -> None:
    if node.initial_homing_done:
        return
    if node.backend == "motion":
        # Enabling hardware never implicitly starts a MoveJ.
        node.initial_homing_done = True
        return
    if not node.control.get("home_on_start", True):
        node.initial_homing_done = True
        return
    homing_names = node._homing_joint_names()
    if not homing_names or not all(name in node.joint_state for name in homing_names):
        return
    tolerance = float(node.control.get("home_tolerance", 0.08))
    needs_homing = any(
        abs(node.joint_state[name] - node.initial_joints.get(name, 0.0)) > tolerance
        for name in homing_names
    )
    node.initial_homing_done = True
    if needs_homing:
        node.get_logger().info(
            "joint feedback deviates from initial_joints on startup; triggering initial homing"
        )
        if node.backend in {"generic", "v23"}:
            node._set_generic_home_targets()
        else:
            node._start_homing()


def _capture_home_targets(
    node: Any,
    names: list[str],
    *,
    deactivate_arms: bool = False,
) -> None:
    """Reset the PyBullet model to initial_joints and capture arm/torso home targets."""
    reset_named_joints(node, names, node.initial_joints)
    for arm in node.arms.values():
        arm.target_local = node._relative_pose(
            node._link_pose(arm.base_index), node._link_pose(arm.ee_index)
        )
        arm.reference_local_ee = arm.target_local
        if deactivate_arms:
            arm.active = False
    node.body.target_base = node._relative_pose(
        node._root_pose(), node._link_pose(node.body.torso_index)
    )


def start_homing(node: Any) -> None:
    node._release_all(send_base_zero=True)
    node.base_zero_pending = False
    for name in node._homing_joint_names():
        if name in node.joint_state:
            node.last_command[name] = node.joint_state[name]
    node.homing = True
    node._begin_joint_home_trajectory()
    node.get_logger().info(
        "both sticks outward: homing "
        f"{', '.join(node._homing_joint_names())} to initial_joints"
    )


def update_homing(node: Any, command: dict[str, float]) -> None:
    names = node._homing_joint_names()
    target = np.asarray([node.initial_joints[name] for name in names], dtype=float)
    previous = np.asarray(
        [
            node.last_command.get(
                name, node.joint_state.get(name, node.initial_joints[name])
            )
            for name in names
        ],
        dtype=float,
    )
    now = node._monotonic()
    nominal_dt = 1.0 / float(node.control["rate_hz"])
    dt = min(0.05, max(0.001, now - node.homing_last_time))
    if not node.homing_velocity or set(node.homing_velocity) != set(names):
        node.homing_velocity = dict.fromkeys(names, 0.0)
        dt = nominal_dt
    node.homing_last_time = now
    maximum_velocity = float(node.control["home_joint_velocity"])
    maximum_acceleration = float(
        node.control.get("home_joint_acceleration", 2.0 * maximum_velocity)
    )
    next_values = previous.copy()
    for index, name in enumerate(names):
        error = target[index] - previous[index]
        old_velocity = float(node.homing_velocity.get(name, 0.0))
        # Start discrete braking one acceleration step before the
        # continuous stopping-distance boundary, so the final sample can
        # reach zero velocity without snapping.
        stopping_velocity = max(
            0.0,
            math.sqrt(max(0.0, 2.0 * maximum_acceleration * abs(error)))
            - maximum_acceleration * dt,
        )
        desired_velocity = (
            math.copysign(min(maximum_velocity, stopping_velocity), error)
            if error
            else 0.0
        )
        velocity_delta = float(
            np.clip(
                desired_velocity - old_velocity,
                -maximum_acceleration * dt,
                maximum_acceleration * dt,
            )
        )
        new_velocity = old_velocity + velocity_delta
        step = 0.5 * (old_velocity + new_velocity) * dt
        settled_distance = 0.5 * maximum_acceleration * dt * dt
        if (
            not error
            or abs(step) >= abs(error)
            or step * error < 0.0
            or (
                abs(error) <= settled_distance
                and abs(old_velocity) <= maximum_acceleration * dt
            )
        ):
            next_values[index] = target[index]
            new_velocity = 0.0
        else:
            next_values[index] = previous[index] + step
        node.homing_velocity[name] = new_velocity
    with (getattr(node, "_command_state_lock", None) or nullcontext()):
        for name, value in zip(names, next_values):
            command[name] = float(value)
            if name in node.body.joint_names or name in node.neck_joint_names:
                node.generic_aux_command[name] = float(value)

    feedback_error = max(
        abs(node.joint_state.get(name, node.initial_joints[name]) - node.initial_joints[name])
        for name in names
    )
    reached = np.max(np.abs(next_values - target)) <= 1e-9 and all(
        abs(node.homing_velocity[name]) <= 1e-9 for name in names
    )
    if reached and node.homing_target_reached_time is None:
        node.homing_target_reached_time = now
    feedback_close = feedback_error <= float(node.control.get("home_tolerance", 0.08))
    feedback_requirement_met = feedback_close or (
        node.homing_target_reached_time is not None
        and now - node.homing_target_reached_time
        >= float(node.control.get("home_feedback_timeout", 2.0))
    )
    if reached and feedback_requirement_met:
        with (getattr(node, "_command_state_lock", None) or nullcontext()):
            for name, value in zip(names, target):
                command[name] = float(value)
                if name in node.body.joint_names or name in node.neck_joint_names:
                    node.generic_aux_command[name] = float(value)
            node.homing = False
        node.homing_velocity.clear()
        _capture_home_targets(node, names, deactivate_arms=True)
        clear_body_reference(node)
        node.neck_reference_head = None
        node.base_yaw_alignment_offset = 0.0
        if node.external_ik:
            node.solver_rearm.homing_complete()
        node.get_logger().info(
            f"{node.backend} controller homing complete: {', '.join(names)}"
        )


def set_generic_home_targets(node: Any) -> None:
    """Capture the configured arm and waist home as Cartesian tasks."""
    if node.backend == "motion":
        node._request_motion_home()
        return
    node._cancel_solver_clutch()
    node._release_all(send_base_zero=True)
    node.solver_rearm.start_homing()
    node.base_zero_pending = False
    homing_names = node._homing_joint_names()
    _capture_home_targets(node, homing_names, deactivate_arms=False)
    node._sync_model()
    clear_body_reference(node)
    node.homing = True
    for name in node.controlled_names:
        if name in node.joint_state:
            node.last_command[name] = float(node.joint_state[name])
    node._begin_joint_home_trajectory()
    node.get_logger().info(
        f"homing triggered: {node.backend} controller moving "
        f"{', '.join(homing_names)} to initial_joints"
    )


def request_motion_home(node: Any) -> bool:
    if getattr(node, "intervention", None) and node.intervention.active:
        return False
    if not node.enabled or not node.hardware_ready or node.stop_reason or node.homing:
        return False
    node._release_all(send_base_zero=True)
    node.solver_rearm.start_homing()
    node.homing = True
    node.motion_home_done = False
    node.homing_start_time = node._monotonic()
    stamp = node.get_clock().now()
    node.motion_home_request_ns = stamp.nanoseconds
    node.motion_home_pub.publish(Header(stamp=stamp.to_msg()))
    return True


def motion_home_result(node: Any, message: String) -> None:
    try:
        result = json.loads(message.data)
        if result["request_ns"] != node.motion_home_request_ns or not node.homing:
            return
        node.motion_home_done = result["success"] is True
        if not node.motion_home_done:
            node.homing = False
            node.solver_rearm.homing_complete()
            node._release_all(send_base_zero=True)
            node.get_logger().warning(
                "MoveJ home failed or canceled; release and re-engage Grip"
            )
    except (ValueError, KeyError, TypeError):
        return


def motion_home_tick(node: Any, now: float, feedback_fresh: bool) -> None:
    # Arms are owned exclusively by MoveJ. Existing auxiliary body home
    # remains independent of arm IK and goes through the common boundary.
    names = node.body.joint_names + node.neck_joint_names
    step = float(node.control["home_joint_velocity"]) / float(node.control["rate_hz"])
    for name in names:
        previous = node.last_command.get(
            name, node.joint_state.get(name, node.initial_joints[name])
        )
        value = previous + float(
            np.clip(node.initial_joints[name] - previous, -step, step)
        )
        node.generic_aux_command[name] = node.last_command[name] = value
    node._publish_motion_aux()
    tolerance = float(node.control.get("home_tolerance", 0.08))
    if node.motion_home_done and feedback_fresh and all(
        abs(node.joint_state.get(n, math.inf) - node.initial_joints[n]) <= tolerance
        for n in names
    ):
        node.homing = False
        node._release_all(send_base_zero=True)
        node.solver_rearm.homing_complete()
    node._publish_status(now, feedback_fresh)


def publish_homing_command(
    node: Any,
    command: dict[str, float],
    *,
    fallback_to_last: bool = False,
) -> None:
    """Build and publish the homing JointState message and update last_command."""
    message = JointState()
    message.header.stamp = node.get_clock().now().to_msg()
    message.name = node._homing_joint_names()
    if fallback_to_last:
        message.position = [
            float(command.get(name, node.last_command.get(name, 0.0)))
            for name in message.name
        ]
    else:
        message.position = [float(command[name]) for name in message.name]
    node.command_pub.publish(message)
    node.last_command.update(command)


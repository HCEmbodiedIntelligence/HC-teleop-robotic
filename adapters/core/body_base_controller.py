"""Body/waist tracking, neck joints, and three-layer / head-driven base control."""

from __future__ import annotations

import math
from typing import Any

import numpy as np
import pybullet as bullet
from std_msgs.msg import Float64MultiArray

from .arm_teleop_math import (
    blend_three_layer_base_velocity,
    clamp_step,
    hands_reach_assist_velocity,
    head_yaw_orientation_velocity,
    joystick_base_velocity,
    joystick_yaw_velocity,
    mapped_relative_pitch,
    mapped_relative_yaw,
    project_displacement_to_heading_frame,
    quaternion_from_axis_angle,
    quaternion_multiply,
    relative_target,
    rotate_velocity_to_base_frame,
    stabilize_pose,
)


def yaw_from_quaternion(quaternion: np.ndarray) -> float:
    x, y, z, w = quaternion
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def wrap_angle(value: float) -> float:
    return (value + math.pi) % (2.0 * math.pi) - math.pi


def clear_body_reference(node: Any, *, clear_smoothed: bool = False) -> None:
    node.body.active = False
    node.body.reference_head = None
    node.body.reference_torso = None
    node.body.reference_target_link = None
    node.body.target_head_local = None
    if clear_smoothed:
        node.body.smoothed_head_pose = None


def engage_body(node: Any) -> None:
    body = node.body
    body.active = True
    body.reference_head = body.head_pose
    body.smoothed_head_pose = body.head_pose
    target_link_idx = (
        body.target_link_index
        if body.target_link_index is not None
        else body.torso_index
    )
    body.reference_target_link = node._relative_pose(
        node._root_pose(), node._link_pose(target_link_idx)
    )
    body.reference_torso = node._link_pose(body.torso_index)
    body.last_solution = np.asarray(
        [node.joint_state[name] for name in body.joint_names], dtype=float
    )
    node.get_logger().info("left clutch engaged: base + waist")


def update_body(node: Any, command: dict[str, float]) -> None:
    node._update_body_target()
    body = node.body
    assert body.target_base is not None
    target_position, target_orientation = node._world_pose(
        node._root_pose(), body.target_base
    )
    solution = node._solve_waist(
        target_position, target_orientation, body.last_solution
    )
    previous = np.asarray(
        [node.last_command.get(name, node.joint_state[name]) for name in body.joint_names]
    )
    solution = clamp_step(
        solution,
        previous,
        body.lower,
        body.upper,
        float(node.body_config["max_joint_velocity"])
        / float(node.control["rate_hz"]),
    )
    body.last_solution = solution
    node._set_group(body.joint_indices, solution)
    for name, value in zip(body.joint_names, solution):
        command[name] = float(value)


def update_body_target(node: Any) -> None:
    """Update the body task without solving joints in the VR adapter."""
    body = node.body
    assert body.head_pose is not None
    assert body.reference_head is not None
    assert body.reference_torso is not None

    pos_deadband = float(node.body_config.get("target_position_deadband", 0.0))
    orn_deadband = float(node.body_config.get("target_orientation_deadband", 0.0))
    filter_alpha = float(node.body_config.get("target_filter_alpha", 1.0))
    if body.smoothed_head_pose is None:
        body.smoothed_head_pose = body.head_pose
    elif pos_deadband > 0.0 or orn_deadband > 0.0 or filter_alpha < 1.0:
        body.smoothed_head_pose = stabilize_pose(
            *body.head_pose,
            *body.smoothed_head_pose,
            pos_deadband,
            orn_deadband,
            filter_alpha,
        )
    else:
        body.smoothed_head_pose = body.head_pose

    effective_head = body.smoothed_head_pose

    if bool(node.body_config.get("unconstrained", False)):
        axis_mapping = node.body_config.get(
            "head_axis_mapping",
            node.control.get("head_axis_mapping", node.control["axis_mapping"]),
        )
        ref_target = (
            body.reference_target_link
            if body.reference_target_link is not None
            else node._relative_pose(
                node._root_pose(), node._link_pose(body.torso_index)
            )
        )
        target_head_local = relative_target(
            *effective_head,
            *body.reference_head,
            *ref_target,
            axis_mapping,
            position_scale=float(node.body_config.get("head_position_scale", 1.0)),
            max_displacement=float(node.body_config.get("head_max_displacement", 0.0)),
            orientation_enabled=bool(
                node.body_config.get("head_orientation_enabled", True)
            ),
        )
        body.target_base = target_head_local
        body.target_head_local = target_head_local
        return

    delta_height = float(
        effective_head[0][int(node.body_config["head_height_axis"])]
        - body.reference_head[0][int(node.body_config["head_height_axis"])]
    )
    pitch_source = str(node.body_config.get("pitch_source", "auto"))
    head_axis_mapping = node.control.get(
        "head_axis_mapping", node.control["axis_mapping"]
    )
    if pitch_source == "orientation":
        delta_pitch = mapped_relative_pitch(
            effective_head[1],
            body.reference_head[1],
            head_axis_mapping,
        )
    elif pitch_source == "position":
        delta_pitch = float(
            effective_head[0][int(node.body_config["head_pitch_position_axis"])]
            - body.reference_head[0][int(node.body_config["head_pitch_position_axis"])]
        )
    elif pitch_source == "combined":
        ori_pitch = mapped_relative_pitch(
            effective_head[1],
            body.reference_head[1],
            head_axis_mapping,
        )
        pos_axis = int(node.body_config.get("head_pitch_position_axis", 2))
        pos_delta = float(effective_head[0][pos_axis] - body.reference_head[0][pos_axis])
        delta_pitch = ori_pitch + pos_delta * float(
            node.body_config.get("pitch_pos_gain", 1.0)
        )
    else:  # "auto"
        ori_pitch = mapped_relative_pitch(
            effective_head[1],
            body.reference_head[1],
            head_axis_mapping,
        )
        if abs(ori_pitch) > 1e-4:
            delta_pitch = ori_pitch
        elif "head_pitch_position_axis" in node.body_config:
            pos_axis = int(node.body_config.get("head_pitch_position_axis", 2))
            delta_pitch = float(
                effective_head[0][pos_axis] - body.reference_head[0][pos_axis]
            )
        else:
            delta_pitch = 0.0
    head_forward_axis = int(
        node.body_config.get(
            "head_forward_axis", node.body_config.get("head_pitch_position_axis", 2)
        )
    )
    delta_forward = float(
        effective_head[0][head_forward_axis]
        - body.reference_head[0][head_forward_axis]
    )
    lift = (
        delta_height
        * float(node.body_config.get("height_direction", 1.0))
        * float(node.body_config["height_scale"])
    )
    max_height = float(node.body_config.get("max_height_delta", 0.18))
    if max_height > 0.0:
        lift = float(np.clip(lift, -max_height, max_height))

    pitch = (
        delta_pitch
        * float(node.body_config.get("pitch_direction", 1.0))
        * float(node.body_config["pitch_scale"])
    )
    max_pitch = float(node.body_config.get("max_pitch_delta", 0.35))
    if max_pitch > 0.0:
        pitch = float(np.clip(pitch, -max_pitch, max_pitch))

    forward = (
        delta_forward
        * float(node.body_config.get("forward_direction", 1.0))
        * float(node.body_config.get("forward_scale", 0.0))
    )
    max_fwd = float(node.body_config.get("max_forward_delta", 0.15))
    if max_fwd > 0.0:
        forward = float(np.clip(forward, -max_fwd, max_fwd))

    if "forward_axis_world" in node.body_config:
        forward_axis = np.asarray(node.body_config["forward_axis_world"], dtype=float)
    else:
        base_rotation = np.asarray(
            bullet.getMatrixFromQuaternion(node._root_pose()[1]), dtype=float
        ).reshape(3, 3)
        forward_axis = base_rotation[:, 0]
    norm = float(np.linalg.norm(forward_axis))
    if norm > 1e-6:
        forward_axis = forward_axis / norm

    target_position = np.array(body.reference_torso[0], dtype=float, copy=True)
    target_position += forward_axis * forward
    target_position[2] += lift
    target_orientation = quaternion_multiply(
        quaternion_from_axis_angle(node.body_config["pitch_axis_world"], pitch),
        body.reference_torso[1],
    )
    body.target_base = node._relative_pose(
        node._root_pose(), (target_position, target_orientation)
    )
    head_link = node.body_config.get("visualization_head_link")
    if head_link in node.link_by_name:
        actual_world = node._link_pose(node.link_by_name[head_link])
        head_in_torso = node._relative_pose(
            node._link_pose(body.torso_index), actual_world
        )
        body.target_head_local = node._world_pose(body.target_base, head_in_torso)
    else:
        body.target_head_local = body.target_base


def camera_heading_yaw(node: Any) -> float:
    if not bool(node.body_config.get("camera_heading_relative_base", True)):
        return 0.0
    # Translation and yaw assist must use the same headset-to-chassis map.
    # The neck's FK uses joint/optical-axis conventions and can have the
    # opposite yaw sign; it is not the headset's movement reference.
    velocity_mode = getattr(node, "head_base_mode", "pose") == "yaw_velocity"
    reference = (
        node.head_velocity_reference
        if velocity_mode
        else node.neck_reference_head or node.body.reference_head
    )
    if velocity_mode and reference is None:
        return 0.0
    if node.body.head_pose is not None and reference is not None:
        if node._monotonic() - node.body.head_stamp > float(
            node.control["pose_timeout"]
        ):
            return 0.0
        head_yaw = mapped_relative_yaw(
            node.body.head_pose[1],
            reference[1],
            node.control.get("head_axis_mapping", node.control["axis_mapping"]),
        )
        return wrap_angle(
            (head_yaw - node.base_yaw_alignment_offset)
            * float(node.body_config.get("yaw_direction", 1.0))
        )
    link_name = node.body_config.get("camera_link", "camera_head")
    if link_name in node.link_by_name:
        cam_idx = node.link_by_name[link_name]
        cam_world = node._link_pose(cam_idx)
        root_world = node._root_pose()
        cam_in_base = node._relative_pose(root_world, cam_world)
        rot = np.asarray(
            bullet.getMatrixFromQuaternion(cam_in_base[1]), dtype=float
        ).reshape(3, 3)
        # Optical forward axis of camera_head is [0, 0, 1] in camera frame
        opt_forward = rot @ np.array([0.0, 0.0, 1.0], dtype=float)
        if opt_forward[0] ** 2 + opt_forward[1] ** 2 >= 1e-4:
            yaw = float(math.atan2(opt_forward[1], opt_forward[0]))
            return 0.0 if abs(yaw) < 1e-4 else yaw

    yaw_joint = node.body_config.get("neck_yaw_joint", "head_1")
    if yaw_joint in node.last_command:
        return float(node.last_command[yaw_joint])
    if yaw_joint in node.joint_state:
        return float(node.joint_state[yaw_joint])
    return 0.0


def update_neck_joints(node: Any, command: dict[str, float], now: float) -> None:
    if not node.neck_enabled or not node.neck_joint_names:
        return
    body = node.body
    if body.head_pose is None:
        return
    if now - body.head_stamp > float(node.control["pose_timeout"]):
        return

    ref_pose = node.neck_reference_head or body.reference_head
    if ref_pose is None:
        return

    head_axis_mapping = node.control.get(
        "head_axis_mapping", node.control["axis_mapping"]
    )
    raw_relative_yaw = wrap_angle(
        mapped_relative_yaw(body.head_pose[1], ref_pose[1], head_axis_mapping)
    )
    # Neck targets follow only headset motion relative to its reference.
    # Chassis alignment remains part of base following, never neck feedback.
    relative_yaw = raw_relative_yaw
    relative_pitch = mapped_relative_pitch(
        body.head_pose[1], ref_pose[1], head_axis_mapping
    )

    yaw_joint = node.body_config.get("neck_yaw_joint", "head_1")
    pitch_joint = node.body_config.get("neck_pitch_joint", "head_2")

    yaw_scale = float(node.body_config.get("neck_yaw_scale", 1.0))
    pitch_scale = float(node.body_config.get("neck_pitch_scale", 1.0))
    yaw_dir = float(node.body_config.get("neck_yaw_direction", 1.0))
    pitch_dir = float(node.body_config.get("neck_pitch_direction", 1.0))

    max_yaw = float(node.body_config.get("neck_max_yaw", 1.57))
    max_pitch = float(node.body_config.get("neck_max_pitch", 0.8))

    target_yaw = float(np.clip(relative_yaw * yaw_scale * yaw_dir, -max_yaw, max_yaw))
    target_pitch = float(
        np.clip(relative_pitch * pitch_scale * pitch_dir, -max_pitch, max_pitch)
    )

    dt = 1.0 / float(node.control["rate_hz"])
    max_v = float(node.body_config.get("neck_max_velocity", 1.5))
    max_step = max_v * dt

    targets = {yaw_joint: target_yaw, pitch_joint: target_pitch}
    for name in node.neck_joint_names:
        if name not in targets:
            continue
        prev_val = float(
            node.last_command.get(
                name,
                node.joint_state.get(name, node.initial_joints.get(name, 0.0)),
            )
        )
        step = float(np.clip(targets[name] - prev_val, -max_step, max_step))
        cmd_val = prev_val + step
        command[name] = cmd_val
        node.last_command[name] = cmd_val
        joint_idx = node.joint_by_name.get(name)
        if joint_idx is not None:
            bullet.resetJointState(
                node.robot_id,
                joint_idx,
                cmd_val,
                physicsClientId=node.physics_client,
            )


def left_stick_active(node: Any) -> bool:
    left = node.arms["left"]
    if left.joy is None:
        return False
    axes = left.joy.axes
    x_index = int(node.body_config.get("stick_x_axis", 2))
    y_index = int(node.body_config.get("stick_y_axis", 3))
    deadzone = float(node.body_config.get("stick_deadzone", 0.08))
    x_val = axes[x_index] if len(axes) > x_index else 0.0
    y_val = axes[y_index] if len(axes) > y_index else 0.0
    return abs(float(x_val)) > deadzone or abs(float(y_val)) > deadzone


def right_stick_yaw_active(node: Any) -> bool:
    right = node.arms["right"]
    if right.joy is None:
        return False
    axes = right.joy.axes
    yaw_axis = int(node.body_config.get("right_stick_yaw_axis", 2))
    if len(axes) <= yaw_axis:
        return False
    deadzone = float(
        node.body_config.get(
            "right_stick_yaw_deadzone",
            node.body_config.get("stick_deadzone", 0.08),
        )
    )
    return abs(float(axes[yaw_axis])) > deadzone


def compute_user_base_velocity(
    node: Any,
    now: float,
    left_clutch_active: bool = False,
) -> tuple[float, float, float]:
    left = node.arms["left"]
    right = node.arms["right"]
    left_input_fresh = node._input_fresh(left, now)
    right_input_fresh = node._input_fresh(right, now)

    forward, lateral = 0.0, 0.0
    allow_left_stick = left_clutch_active or bool(
        node.body_config.get("left_stick_independent_base", True)
    )
    if left_input_fresh and allow_left_stick and left.joy is not None:
        axes = left.joy.axes
        x_index = int(node.body_config.get("stick_x_axis", 2))
        y_index = int(node.body_config.get("stick_y_axis", 3))
        stick_x = axes[x_index] if len(axes) > x_index else 0.0
        stick_y = axes[y_index] if len(axes) > y_index else 0.0
        forward, lateral = joystick_base_velocity(
            stick_x,
            stick_y,
            float(node.body_config.get("stick_deadzone", 0.08)),
            float(node.body_config.get("max_linear_velocity", 0.35)),
            float(node.body_config.get("max_lateral_velocity", 0.25)),
            float(node.body_config.get("stick_forward_direction", 1.0)),
            float(node.body_config.get("stick_lateral_direction", -1.0)),
        )

    stick_yaw = 0.0
    if (
        bool(node.body_config.get("right_stick_yaw_enabled", True))
        and right_input_fresh
        and right.joy is not None
    ):
        r_axes = right.joy.axes
        yaw_axis = int(node.body_config.get("right_stick_yaw_axis", 2))
        if len(r_axes) > yaw_axis:
            r_deadzone = float(
                node.body_config.get(
                    "right_stick_yaw_deadzone",
                    node.body_config.get("stick_deadzone", 0.08),
                )
            )
            r_direction = float(
                node.body_config.get("right_stick_yaw_direction", -1.0)
            )
            max_ang = float(node.body_config.get("max_angular_velocity", 0.45))
            stick_yaw = joystick_yaw_velocity(
                r_axes[yaw_axis],
                r_deadzone,
                max_ang,
                r_direction,
            )

    return (forward, lateral, stick_yaw)


def compute_head_orientation_velocity(
    node: Any,
    now: float,
    *,
    forward_velocity: float = 0.0,
    lateral_velocity: float = 0.0,
    track_head_yaw: bool = True,
) -> float:
    if not track_head_yaw:
        return 0.0
    if not bool(node.body_config.get("head_orientation_assist_enabled", True)):
        return 0.0

    velocity_mode = getattr(node, "head_base_mode", "pose") == "yaw_velocity"
    align_on_move_only = not velocity_mode and bool(
        node.body_config.get("head_orientation_align_on_move_only", True)
    )
    move_threshold = float(
        node.body_config.get("head_orientation_move_threshold", 0.02)
    )
    linear_speed = math.hypot(forward_velocity, lateral_velocity)
    if align_on_move_only and linear_speed <= move_threshold:
        return 0.0

    body = node.body
    ref_head = (
        node.head_velocity_reference
        if velocity_mode
        else node.neck_reference_head or body.reference_head
    )
    if body.head_pose is None or ref_head is None:
        return 0.0
    if now - body.head_stamp > float(node.control["pose_timeout"]):
        return 0.0

    raw_relative_yaw = wrap_angle(
        mapped_relative_yaw(
            body.head_pose[1],
            ref_head[1],
            node.control.get("head_axis_mapping", node.control["axis_mapping"]),
        )
    )
    relative_yaw = wrap_angle(raw_relative_yaw - node.base_yaw_alignment_offset)

    deadzone_deg = float(node.body_config.get("head_yaw_deadzone_deg", 3.0))
    active_deg = float(node.body_config.get("head_yaw_active_deg", 25.0))
    slow_gain = float(node.body_config.get("head_yaw_slow_gain", 0.50))
    fast_gain = float(node.body_config.get("head_yaw_fast_gain", 0.90))
    slow_max_vel = float(node.body_config.get("head_yaw_slow_max_vel", 0.20))
    max_angular_velocity = float(node.body_config.get("max_angular_velocity", 0.45))
    if velocity_mode:
        max_angular_velocity = min(
            max_angular_velocity, node.head_velocity_max_angular
        )
    yaw_direction = float(node.body_config.get("yaw_direction", 1.0))

    return head_yaw_orientation_velocity(
        relative_yaw=relative_yaw,
        deadzone_rad=math.radians(deadzone_deg),
        active_threshold_rad=math.radians(active_deg),
        slow_gain=slow_gain,
        fast_gain=fast_gain,
        slow_max_velocity=slow_max_vel,
        max_angular_velocity=max_angular_velocity,
        yaw_direction=yaw_direction,
    )


def compute_hands_reach_assist_velocity(
    node: Any, now: float
) -> tuple[float, float]:
    if not bool(node.body_config.get("reach_assist_enabled", True)):
        return (0.0, 0.0)

    cam_yaw = node._camera_heading_yaw()
    hand_offsets: list[tuple[float, float]] = []
    for side in ("left", "right"):
        arm = node.arms[side]
        if not arm.active:
            continue
        if arm.target_local is None or arm.reference_local_ee is None:
            continue
        if now - arm.pose_stamp > float(node.control["pose_timeout"]):
            continue

        pos = np.asarray(arm.target_local[0], dtype=float)
        ref = np.asarray(arm.reference_local_ee[0], dtype=float)
        dx = float(pos[0] - ref[0])
        dy = float(pos[1] - ref[1])
        dx_cam, dy_cam = project_displacement_to_heading_frame(dx, dy, cam_yaw)
        hand_offsets.append((dx_cam, dy_cam))

    if not hand_offsets:
        return (0.0, 0.0)

    return hands_reach_assist_velocity(
        hand_offsets=hand_offsets,
        deadzone_forward=float(node.body_config.get("reach_deadzone_forward", 0.08)),
        deadzone_backward=float(node.body_config.get("reach_deadzone_backward", 0.06)),
        deadzone_lateral=float(node.body_config.get("reach_deadzone_lateral", 0.06)),
        gain_forward=float(node.body_config.get("reach_gain_forward", 0.6)),
        gain_backward=float(node.body_config.get("reach_gain_backward", 0.5)),
        gain_lateral=float(node.body_config.get("reach_gain_lateral", 0.6)),
        max_linear_velocity=float(
            node.body_config.get("reach_max_linear_velocity", 0.20)
        ),
        max_lateral_velocity=float(
            node.body_config.get("reach_max_lateral_velocity", 0.18)
        ),
    )


def publish_head_base_pose(node: Any, now: float, engaged: bool) -> None:
    follower = node.head_base_follower
    valid_head = (
        node.body.head_pose is not None
        and now - node.body.head_stamp <= float(node.control["pose_timeout"])
    )
    valid_base = (
        node.base_pose_feedback is not None
        and now - node.base_pose_stamp <= follower.feedback_timeout
    )
    if not engaged or not valid_head or not valid_base:
        follower.reset()
        node.head_base_state = (
            "idle"
            if not engaged
            else "waiting_head_pose"
            if not valid_head
            else "waiting_base_pose"
        )
        node._stop_base_if_pending()
        return
    node.head_base_state = "tracking"
    limits = tuple(
        float(node.body_config.get(k, v))
        for k, v in (
            ("max_linear_velocity", 0.35),
            ("max_lateral_velocity", 0.25),
            ("max_angular_velocity", 0.45),
        )
    )
    yaw, forward, lateral = follower.update(
        node.body.head_pose, node.base_pose_feedback, limits
    )
    if node.body_config.get("base_command_mode", "velocity") == "delta":
        dt = 1.0 / float(node.control["rate_hz"])
        yaw, forward, lateral = yaw * dt, forward * dt, lateral * dt
    node.base_pub.publish(Float64MultiArray(data=[yaw, forward, lateral]))
    node.base_zero_pending = any(abs(v) > 1e-6 for v in (yaw, forward, lateral))


def publish_head_planar_velocity(node: Any, now: float, engaged: bool) -> None:
    follower = node.head_planar_follower
    manual = node._compute_user_base_velocity(now, left_clutch_active=engaged)
    if any(abs(v) > 1e-6 for v in manual):
        follower.reset()
        node.head_base_state = "manual_stick"
        forward, lateral, yaw = manual
    elif (
        not engaged
        or node.body.head_pose is None
        or now - node.body.head_stamp > float(node.control["pose_timeout"])
    ):
        follower.reset()
        node.head_base_state = "idle" if not engaged else "waiting_head_pose"
        node._stop_base_if_pending()
        return
    else:
        node.head_base_state = "planar_velocity_estimated"
        limits = (
            min(
                node.head_velocity_max_linear,
                float(node.body_config.get("max_linear_velocity", 0.35)),
            ),
            min(
                node.head_velocity_max_lateral,
                float(node.body_config.get("max_lateral_velocity", 0.25)),
            ),
            min(
                node.head_velocity_max_angular,
                float(node.body_config.get("max_angular_velocity", 0.45)),
            ),
        )
        yaw, forward, lateral = follower.update_velocity(
            node.body.head_pose, now, limits, float(node.control["pose_timeout"])
        )
    if max(abs(yaw), abs(forward), abs(lateral)) > 1e-4:
        if node.body_config.get("base_command_mode", "velocity") == "delta":
            period = 1.0 / float(node.control["rate_hz"])
            yaw, forward, lateral = yaw * period, forward * period, lateral * period
        node.base_pub.publish(Float64MultiArray(data=[yaw, forward, lateral]))
        node.base_zero_pending = True
    else:
        node._stop_base_if_pending()


def publish_base_command(
    node: Any,
    now: float,
    *,
    track_head_yaw: bool = True,
    left_clutch_active: bool = False,
) -> None:
    if getattr(node, "head_planar_follower", None) is not None:
        node._publish_head_planar_velocity(now, left_clutch_active and track_head_yaw)
        return
    if getattr(node, "head_base_follower", None) is not None:
        forward, lateral, _ = node._compute_user_base_velocity(
            now, left_clutch_active=left_clutch_active
        )
        if abs(forward) > 1e-6 or abs(lateral) > 1e-6:
            # Manual translation owns the base while the left stick is held.
            # Discard the pose target so releasing it cannot pull us back.
            node.head_base_follower.reset()
            node.head_base_state = "manual_stick"
            if node.body_config.get("base_command_mode", "velocity") == "delta":
                dt = 1.0 / float(node.control["rate_hz"])
                forward, lateral = forward * dt, lateral * dt
            node.base_pub.publish(Float64MultiArray(data=[0.0, forward, lateral]))
            node.base_zero_pending = True
        else:
            node._publish_head_base_pose(now, left_clutch_active)
        return
    velocity_mode = getattr(node, "head_base_mode", "pose") == "yaw_velocity"
    if velocity_mode:
        head_fresh = (
            node.body.head_pose is not None
            and now - node.body.head_stamp <= float(node.control["pose_timeout"])
        )
        if not left_clutch_active or not head_fresh or not track_head_yaw:
            node.head_velocity_reference = None
            node.last_head_velocity_time = None
            node.base_yaw_alignment_offset = 0.0
            node.head_base_state = (
                "idle" if not left_clutch_active else "waiting_head_pose"
            )
            track_head_yaw = False
        elif node.head_velocity_reference is None:
            # Reference only the new clutch press; never reuse an old neck zero.
            node.head_velocity_reference = node.body.head_pose
            node.base_yaw_alignment_offset = 0.0
            node.last_head_velocity_time = now
            node.head_base_state = "yaw_velocity"
    cmd_vel_user = node._compute_user_base_velocity(
        now, left_clutch_active=left_clutch_active
    )
    cmd_vel_assist = node._compute_hands_reach_assist_velocity(now)

    alpha_reach = float(node.body_config.get("reach_assist_alpha", 1.0))
    fwd_intent = cmd_vel_user[0] + alpha_reach * cmd_vel_assist[0]
    lat_intent = cmd_vel_user[1] + alpha_reach * cmd_vel_assist[1]

    cmd_vel_head = node._compute_head_orientation_velocity(
        now,
        forward_velocity=fwd_intent,
        lateral_velocity=lat_intent,
        track_head_yaw=track_head_yaw,
    )

    alpha_head = float(node.body_config.get("head_orientation_alpha", 1.0))
    max_linear = float(node.body_config.get("max_linear_velocity", 0.35))
    max_lateral = float(node.body_config.get("max_lateral_velocity", 0.25))
    max_angular = float(node.body_config.get("max_angular_velocity", 0.45))
    if velocity_mode:
        max_angular = min(max_angular, node.head_velocity_max_angular)

    yaw, forward, lateral = blend_three_layer_base_velocity(
        cmd_vel_user=cmd_vel_user,
        cmd_vel_assist=cmd_vel_assist,
        cmd_vel_head=cmd_vel_head,
        alpha_reach=alpha_reach,
        alpha_head=alpha_head,
        max_linear_velocity=max_linear,
        max_lateral_velocity=max_lateral,
        max_angular_velocity=max_angular,
    )

    dt = 1.0 / float(node.control["rate_hz"])
    if velocity_mode:
        dt = (
            min(
                max(now - node.last_head_velocity_time, 0.0),
                float(node.control["pose_timeout"]),
            )
            if node.last_head_velocity_time is not None
            else 0.0
        )
        node.last_head_velocity_time = (
            now if node.head_velocity_reference is not None else None
        )
    if abs(yaw) > 1e-4:
        # Command-integrated alignment is only an estimate, never measured odometry.
        # Store it in mapped-head coordinates, not chassis coordinates.
        node.base_yaw_alignment_offset = wrap_angle(
            node.base_yaw_alignment_offset
            + yaw * dt * float(node.body_config.get("yaw_direction", 1.0))
        )

    cam_yaw = node._camera_heading_yaw()
    forward, lateral = rotate_velocity_to_base_frame(forward, lateral, cam_yaw)

    is_moving = abs(yaw) > 1e-4 or abs(forward) > 1e-4 or abs(lateral) > 1e-4
    if is_moving:
        if node.body_config.get("base_command_mode", "delta") == "delta":
            period = 1.0 / float(node.control["rate_hz"])
            yaw, forward, lateral = yaw * period, forward * period, lateral * period
        node.base_pub.publish(Float64MultiArray(data=[yaw, forward, lateral]))
        node.base_zero_pending = True
    else:
        node._stop_base_if_pending()


def stop_base_if_pending(node: Any) -> None:
    if node.base_zero_pending:
        node.base_pub.publish(Float64MultiArray(data=[0.0, 0.0, 0.0]))
        node.base_zero_pending = False


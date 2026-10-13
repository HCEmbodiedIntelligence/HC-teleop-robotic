from __future__ import annotations

import math
from typing import Sequence

import numpy as np


def binary_trigger_state(trigger: float, threshold: float = 0.5) -> float:
    """Map a normalized trigger value to an open/closed recording state."""
    trigger_value = float(trigger)
    threshold_value = float(threshold)
    if not math.isfinite(trigger_value) or not math.isfinite(threshold_value):
        raise ValueError("trigger mapping values must be finite")
    if not 0.0 <= threshold_value <= 1.0:
        raise ValueError("trigger threshold must be in [0, 1]")
    return 1.0 if float(np.clip(trigger_value, 0.0, 1.0)) >= threshold_value else 0.0


def normalize_quaternion(value: Sequence[float]) -> np.ndarray:
    quaternion = np.asarray(value, dtype=float)
    norm = float(np.linalg.norm(quaternion))
    if quaternion.shape != (4,) or not math.isfinite(norm) or norm < 1e-9:
        raise ValueError("invalid quaternion")
    return quaternion / norm


def quaternion_multiply(left: Sequence[float], right: Sequence[float]) -> np.ndarray:
    x1, y1, z1, w1 = normalize_quaternion(left)
    x2, y2, z2, w2 = normalize_quaternion(right)
    return normalize_quaternion(
        [
            w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
            w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
            w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
            w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
        ]
    )


def quaternion_inverse(value: Sequence[float]) -> np.ndarray:
    x, y, z, w = normalize_quaternion(value)
    return np.asarray([-x, -y, -z, w])


def quaternion_from_axis_angle(axis: Sequence[float], angle: float) -> np.ndarray:
    vector = np.asarray(axis, dtype=float)
    norm = float(np.linalg.norm(vector))
    if vector.shape != (3,) or not math.isfinite(norm) or norm < 1e-9:
        raise ValueError("axis must be a finite non-zero 3D vector")
    vector /= norm
    half = float(angle) * 0.5
    return normalize_quaternion([*(vector * math.sin(half)), math.cos(half)])


def quaternion_to_matrix(value: Sequence[float]) -> np.ndarray:
    x, y, z, w = normalize_quaternion(value)
    return np.asarray(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
        ],
        dtype=float,
    )


def matrix_to_quaternion(matrix: Sequence[Sequence[float]]) -> np.ndarray:
    rotation = np.asarray(matrix, dtype=float)
    if rotation.shape != (3, 3) or not np.all(np.isfinite(rotation)):
        raise ValueError("rotation matrix must be finite 3x3")
    trace = float(np.trace(rotation))
    if trace > 0.0:
        scale = math.sqrt(trace + 1.0) * 2.0
        value = [
            (rotation[2, 1] - rotation[1, 2]) / scale,
            (rotation[0, 2] - rotation[2, 0]) / scale,
            (rotation[1, 0] - rotation[0, 1]) / scale,
            0.25 * scale,
        ]
    else:
        index = int(np.argmax(np.diag(rotation)))
        if index == 0:
            scale = math.sqrt(1.0 + rotation[0, 0] - rotation[1, 1] - rotation[2, 2]) * 2
            value = [
                0.25 * scale,
                (rotation[0, 1] + rotation[1, 0]) / scale,
                (rotation[0, 2] + rotation[2, 0]) / scale,
                (rotation[2, 1] - rotation[1, 2]) / scale,
            ]
        elif index == 1:
            scale = math.sqrt(1.0 + rotation[1, 1] - rotation[0, 0] - rotation[2, 2]) * 2
            value = [
                (rotation[0, 1] + rotation[1, 0]) / scale,
                0.25 * scale,
                (rotation[1, 2] + rotation[2, 1]) / scale,
                (rotation[0, 2] - rotation[2, 0]) / scale,
            ]
        else:
            scale = math.sqrt(1.0 + rotation[2, 2] - rotation[0, 0] - rotation[1, 1]) * 2
            value = [
                (rotation[0, 2] + rotation[2, 0]) / scale,
                (rotation[1, 2] + rotation[2, 1]) / scale,
                0.25 * scale,
                (rotation[1, 0] - rotation[0, 1]) / scale,
            ]
    return normalize_quaternion(value)


def orientation_error(target: Sequence[float], current: Sequence[float]) -> np.ndarray:
    difference = quaternion_multiply(target, quaternion_inverse(current))
    if difference[3] < 0.0:
        difference = -difference
    vector_norm = float(np.linalg.norm(difference[:3]))
    if vector_norm < 1e-9:
        return np.zeros(3)
    angle = 2.0 * math.atan2(vector_norm, float(difference[3]))
    return difference[:3] / vector_norm * angle


def mapped_relative_yaw(
    orientation: Sequence[float],
    reference_orientation: Sequence[float],
    axis_mapping: Sequence[Sequence[float]],
) -> float:
    """Return relative headset yaw after mapping VR axes into robot axes."""
    mapping = np.asarray(axis_mapping, dtype=float)
    if mapping.shape != (3, 3) or not np.all(np.isfinite(mapping)):
        raise ValueError("axis_mapping must be finite 3x3")
    relative = quaternion_multiply(
        orientation, quaternion_inverse(reference_orientation)
    )
    mapped = mapping @ quaternion_to_matrix(relative) @ mapping.T
    x, y, z, w = matrix_to_quaternion(mapped)
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def mapped_relative_pitch(
    orientation: Sequence[float],
    reference_orientation: Sequence[float],
    axis_mapping: Sequence[Sequence[float]],
) -> float:
    """Relative forward elevation, independent of room yaw and headset roll.

    Mapping a world-frame quaternion difference mixes pitch and roll after a
    turn. Measure each headset forward vector's elevation instead; keep the
    existing positive-down convention and subtract the clutch reference.
    """
    mapping = np.asarray(axis_mapping, dtype=float)
    if mapping.shape != (3, 3) or not np.all(np.isfinite(mapping)):
        raise ValueError("axis_mapping must be finite 3x3")

    # Honor both Unity +Z-forward and OpenXR -Z-forward mappings.
    source_forward = mapping.T @ np.array([1., 0., 0.])

    def pitch(quaternion):
        forward = mapping @ (quaternion_to_matrix(quaternion) @ source_forward)
        return math.atan2(-float(forward[2]), float(np.linalg.norm(forward[:2])))

    return pitch(orientation) - pitch(reference_orientation)


def stabilize_pose(
    target_position: Sequence[float],
    target_orientation: Sequence[float],
    previous_position: Sequence[float],
    previous_orientation: Sequence[float],
    position_deadband: float,
    orientation_deadband: float,
    filter_alpha: float,
) -> tuple[np.ndarray, np.ndarray]:
    """Apply radial deadbands and low-pass smoothing to a pose target."""
    if position_deadband < 0.0 or orientation_deadband < 0.0:
        raise ValueError("pose deadbands must be non-negative")
    if not 0.0 < filter_alpha <= 1.0:
        raise ValueError("filter_alpha must be in (0, 1]")

    target_position_array = np.asarray(target_position, dtype=float)
    previous_position_array = np.asarray(previous_position, dtype=float)
    position_delta = target_position_array - previous_position_array
    position_distance = float(np.linalg.norm(position_delta))
    if position_distance <= position_deadband:
        position = previous_position_array.copy()
    else:
        effective_delta = position_delta * (
            (position_distance - position_deadband) / position_distance
        )
        position = previous_position_array + filter_alpha * effective_delta

    previous_quaternion = normalize_quaternion(previous_orientation)
    rotation_delta = orientation_error(target_orientation, previous_quaternion)
    rotation_angle = float(np.linalg.norm(rotation_delta))
    if rotation_angle <= orientation_deadband:
        orientation = previous_quaternion
    else:
        step_angle = filter_alpha * (rotation_angle - orientation_deadband)
        step = quaternion_from_axis_angle(rotation_delta / rotation_angle, step_angle)
        orientation = quaternion_multiply(step, previous_quaternion)
    return position, orientation


def sticks_outward(left_x: float, right_x: float, threshold: float) -> bool:
    """Return true when the left stick points left and the right stick right."""
    values = (float(left_x), float(right_x), float(threshold))
    if not all(math.isfinite(value) for value in values):
        return False
    if not 0.0 < threshold <= 1.0:
        raise ValueError("stick threshold must be in (0, 1]")
    return left_x <= -threshold and right_x >= threshold


def ordered_homing_joint_names(
    right_arm: Sequence[str], left_arm: Sequence[str], waist: Sequence[str]
) -> list[str]:
    """Return the unique arm and waist joints controlled by a homing cycle."""
    return list(dict.fromkeys([*right_arm, *left_arm, *waist]))


def joystick_base_velocity(
    stick_x: float,
    stick_y: float,
    deadzone: float,
    max_linear_velocity: float,
    max_lateral_velocity: float,
    forward_direction: float = 1.0,
    lateral_direction: float = -1.0,
) -> tuple[float, float]:
    """Map a primary joystick to robot forward and left-positive velocity."""
    values = tuple(
        float(value)
        for value in (
            stick_x,
            stick_y,
            deadzone,
            max_linear_velocity,
            max_lateral_velocity,
            forward_direction,
            lateral_direction,
        )
    )
    if not all(math.isfinite(value) for value in values):
        raise ValueError("joystick base mapping values must be finite")
    x, y, deadzone, max_linear, max_lateral, forward_sign, lateral_sign = values
    if not 0.0 <= deadzone < 1.0:
        raise ValueError("joystick deadzone must be in [0, 1)")
    if max_linear < 0.0 or max_lateral < 0.0:
        raise ValueError("joystick velocity limits must be non-negative")
    if forward_sign not in (-1.0, 1.0) or lateral_sign not in (-1.0, 1.0):
        raise ValueError("joystick directions must be -1 or 1")

    def shape(value: float) -> float:
        value = float(np.clip(value, -1.0, 1.0))
        if abs(value) <= deadzone:
            return 0.0
        return math.copysign((abs(value) - deadzone) / (1.0 - deadzone), value)

    forward = shape(y) * max_linear * forward_sign
    lateral = shape(x) * max_lateral * lateral_sign
    return forward, lateral


def joystick_yaw_velocity(
    stick_yaw: float,
    deadzone: float,
    max_angular_velocity: float,
    yaw_direction: float = -1.0,
) -> float:
    """Map a secondary or right joystick axis to robot yaw velocity."""
    values = tuple(
        float(value)
        for value in (
            stick_yaw,
            deadzone,
            max_angular_velocity,
            yaw_direction,
        )
    )
    if not all(math.isfinite(value) for value in values):
        raise ValueError("joystick yaw mapping values must be finite")
    raw, deadzone, max_ang, yaw_sign = values
    if not 0.0 <= deadzone < 1.0:
        raise ValueError("joystick deadzone must be in [0, 1)")
    if max_ang < 0.0:
        raise ValueError("max_angular_velocity must be non-negative")
    if yaw_sign not in (-1.0, 1.0):
        raise ValueError("yaw_direction must be -1 or 1")

    val = float(np.clip(raw, -1.0, 1.0))
    if abs(val) <= deadzone:
        return 0.0
    shaped = math.copysign((abs(val) - deadzone) / (1.0 - deadzone), val)
    return float(shaped * max_ang * yaw_sign)


def adaptive_damping(
    smallest_singular_value: float,
    minimum: float,
    maximum: float,
    singularity_threshold: float,
) -> float:
    """Increase DLS damping smoothly as a Jacobian approaches singularity."""
    values = tuple(
        float(value)
        for value in (smallest_singular_value, minimum, maximum, singularity_threshold)
    )
    if not all(math.isfinite(value) for value in values):
        raise ValueError("adaptive damping values must be finite")
    sigma, minimum, maximum, threshold = values
    if minimum <= 0.0 or maximum < minimum or threshold <= 0.0:
        raise ValueError("invalid adaptive damping bounds")
    if sigma >= threshold:
        return minimum
    ratio = max(0.0, sigma) / threshold
    return minimum + (maximum - minimum) * (1.0 - ratio) ** 2


def joint_limit_avoidance(
    joints: Sequence[float],
    lower: Sequence[float],
    upper: Sequence[float],
    margin_fraction: float,
) -> np.ndarray:
    """Return a bounded direction that pushes joints away from nearby limits."""
    q = np.asarray(joints, dtype=float)
    low = np.asarray(lower, dtype=float)
    high = np.asarray(upper, dtype=float)
    if q.shape != low.shape or q.shape != high.shape or q.ndim != 1:
        raise ValueError("joint limit arrays must be matching vectors")
    if not np.all(np.isfinite(np.concatenate((q, low, high)))):
        raise ValueError("joint limit arrays must be finite")
    if not 0.0 < margin_fraction < 0.5:
        raise ValueError("margin_fraction must be in (0, 0.5)")
    span = high - low
    if np.any(span <= 0.0):
        raise ValueError("joint upper limits must exceed lower limits")
    normalized_low = (q - low) / span
    normalized_high = (high - q) / span
    result = np.zeros_like(q)
    low_mask = normalized_low < margin_fraction
    high_mask = normalized_high < margin_fraction
    result[low_mask] += (
        1.0 - np.maximum(normalized_low[low_mask], 0.0) / margin_fraction
    ) ** 2
    result[high_mask] -= (
        1.0 - np.maximum(normalized_high[high_mask], 0.0) / margin_fraction
    ) ** 2
    return np.clip(result, -1.0, 1.0)


def relative_target(
    vr_position: Sequence[float],
    vr_orientation: Sequence[float],
    reference_vr_position: Sequence[float],
    reference_vr_orientation: Sequence[float],
    reference_robot_position: Sequence[float],
    reference_robot_orientation: Sequence[float],
    axis_mapping: Sequence[Sequence[float]],
    position_scale: float,
    max_displacement: float,
    orientation_enabled: bool,
) -> tuple[np.ndarray, np.ndarray]:
    mapping = np.asarray(axis_mapping, dtype=float)
    if mapping.shape != (3, 3) or not np.all(np.isfinite(mapping)):
        raise ValueError("axis_mapping must be finite 3x3")
    delta = mapping @ (
        np.asarray(vr_position, dtype=float)
        - np.asarray(reference_vr_position, dtype=float)
    ) * float(position_scale)
    length = float(np.linalg.norm(delta))
    if max_displacement > 0.0 and length > max_displacement:
        delta *= max_displacement / length
    target_position = np.asarray(reference_robot_position, dtype=float) + delta

    target_orientation = normalize_quaternion(reference_robot_orientation)
    if orientation_enabled:
        vr_delta = quaternion_multiply(
            vr_orientation, quaternion_inverse(reference_vr_orientation)
        )
        mapped_delta = mapping @ quaternion_to_matrix(vr_delta) @ mapping.T
        target_orientation = quaternion_multiply(
            matrix_to_quaternion(mapped_delta), target_orientation
        )
    return target_position, target_orientation


def clamp_step(
    target: Sequence[float],
    previous: Sequence[float],
    lower: Sequence[float],
    upper: Sequence[float],
    max_step: float,
) -> np.ndarray:
    """Clamp the joint step vector proportionally so all joints move coordinately."""
    target_array = np.asarray(target, dtype=float)
    previous_array = np.asarray(previous, dtype=float)
    delta = target_array - previous_array
    max_abs = float(np.max(np.abs(delta))) if len(delta) > 0 else 0.0
    if max_abs > max_step and max_abs > 1e-12:
        delta = delta * (max_step / max_abs)
    limited = previous_array + delta
    return np.clip(limited, np.asarray(lower, dtype=float), np.asarray(upper, dtype=float))


def head_yaw_orientation_velocity(
    relative_yaw: float,
    deadzone_rad: float = math.radians(15.0),
    active_threshold_rad: float = math.radians(45.0),
    slow_gain: float = 0.25,
    fast_gain: float = 0.70,
    slow_max_velocity: float = 0.15,
    max_angular_velocity: float = 0.45,
    yaw_direction: float = 1.0,
) -> float:
    """Calculate chassis angular velocity from head-to-robot relative yaw in three zones:

    Zone 1 (< 15 deg): Deadzone -> 0.0 (no rotation)
    Zone 2 (15 ~ 45 deg): Gentle follow -> gradual, slow rotation (up to slow_max_velocity)
    Zone 3 (> 45 deg): Active follow -> prominent follow (up to max_angular_velocity)
    """
    if not math.isfinite(relative_yaw):
        return 0.0
    abs_yaw = abs(float(relative_yaw))
    if abs_yaw <= deadzone_rad or deadzone_rad >= active_threshold_rad:
        return 0.0

    sign = math.copysign(1.0, relative_yaw) * float(yaw_direction)

    if abs_yaw <= active_threshold_rad:
        # Zone 2: 15 ~ 45 deg - slow gentle follow
        excess = abs_yaw - deadzone_rad
        vel = slow_gain * excess
        vel = min(vel, slow_max_velocity)
    else:
        # Zone 3: > 45 deg - active prominent follow
        excess_fast = abs_yaw - active_threshold_rad
        vel = slow_max_velocity + fast_gain * excess_fast
        vel = min(vel, max_angular_velocity)

    return float(sign * vel)


def hands_reach_assist_velocity(
    hand_offsets: Sequence[tuple[float, float]],
    deadzone_forward: float = 0.08,
    deadzone_backward: float = 0.06,
    deadzone_lateral: float = 0.06,
    gain_forward: float = 0.6,
    gain_backward: float = 0.5,
    gain_lateral: float = 0.6,
    max_linear_velocity: float = 0.20,
    max_lateral_velocity: float = 0.18,
) -> tuple[float, float]:
    """Calculate omnidirectional base assist velocity (v_x, v_y) from active hands' displacements.

    hand_offsets: list of (delta_x, delta_y) in meters in the robot body frame.
    Returns (v_x_assist, v_y_assist).
    """
    if not hand_offsets:
        return (0.0, 0.0)

    per_hand_vx: list[float] = []
    per_hand_vy: list[float] = []

    for dx, dy in hand_offsets:
        if not (math.isfinite(dx) and math.isfinite(dy)):
            continue

        # Forward / backward assist (X)
        if dx > deadzone_forward:
            vx = min(gain_forward * (dx - deadzone_forward), max_linear_velocity)
        elif dx < -deadzone_backward:
            vx = max(gain_backward * (dx + deadzone_backward), -max_linear_velocity)
        else:
            vx = 0.0
        per_hand_vx.append(vx)

        # Lateral assist (Y)
        if dy > deadzone_lateral:
            vy = min(gain_lateral * (dy - deadzone_lateral), max_lateral_velocity)
        elif dy < -deadzone_lateral:
            vy = max(gain_lateral * (dy + deadzone_lateral), -max_lateral_velocity)
        else:
            vy = 0.0
        per_hand_vy.append(vy)

    if not per_hand_vx:
        return (0.0, 0.0)

    avg_vx = float(np.mean(per_hand_vx))
    avg_vy = float(np.mean(per_hand_vy))
    def _aggregate(velocities: list[float]) -> float:
        active = [v for v in velocities if abs(v) > 1e-4]
        if not active:
            return 0.0
        return float(sum(velocities) / len(active))

    avg_vx = _aggregate(per_hand_vx)
    avg_vy = _aggregate(per_hand_vy)
    return (avg_vx, avg_vy)


def rotate_velocity_to_base_frame(
    forward: float,
    lateral: float,
    heading_yaw: float,
) -> tuple[float, float]:
    """Rotate 2D velocity from a heading frame (e.g. camera view) into base chassis frame.

    forward: velocity along heading direction
    lateral: velocity perpendicular (left-positive) to heading direction
    heading_yaw: heading angle in radians relative to base frame (counterclockwise positive)
    Returns (v_x_base, v_y_base).
    """
    c = math.cos(heading_yaw)
    s = math.sin(heading_yaw)
    v_x = c * float(forward) - s * float(lateral)
    v_y = s * float(forward) + c * float(lateral)
    return float(v_x), float(v_y)


def project_displacement_to_heading_frame(
    dx: float,
    dy: float,
    heading_yaw: float,
) -> tuple[float, float]:
    """Project 2D displacement from base/torso frame into a heading frame (e.g. camera view).

    dx, dy: displacement in base frame
    heading_yaw: heading angle in radians relative to base frame
    Returns (dx_heading, dy_heading).
    """
    c = math.cos(heading_yaw)
    s = math.sin(heading_yaw)
    dx_h = c * float(dx) + s * float(dy)
    dy_h = -s * float(dx) + c * float(dy)
    return float(dx_h), float(dy_h)


def blend_three_layer_base_velocity(
    cmd_vel_user: Sequence[float],
    cmd_vel_assist: Sequence[float],
    cmd_vel_head: float,
    alpha_reach: float = 1.0,
    alpha_head: float = 1.0,
    max_linear_velocity: float = 0.35,
    max_lateral_velocity: float = 0.25,
    max_angular_velocity: float = 0.45,
) -> tuple[float, float, float]:
    """Blend user command, reach assist, and head orientation assist into final [yaw, forward, lateral].

    cmd_vel_user: (forward, lateral, yaw)
    cmd_vel_assist: (forward, lateral, _) from hands reach
    cmd_vel_head: yaw velocity from head orientation
    Returns (yaw, forward, lateral).
    """
    u_fwd, u_lat, u_yaw = float(cmd_vel_user[0]), float(cmd_vel_user[1]), float(cmd_vel_user[2])
    a_fwd, a_lat = float(cmd_vel_assist[0]), float(cmd_vel_assist[1])
    h_yaw = float(cmd_vel_head)

    fwd = float(np.clip(u_fwd + alpha_reach * a_fwd, -max_linear_velocity, max_linear_velocity))
    lat = float(np.clip(u_lat + alpha_reach * a_lat, -max_lateral_velocity, max_lateral_velocity))
    yaw = float(np.clip(u_yaw + alpha_head * h_yaw, -max_angular_velocity, max_angular_velocity))

    return (yaw, fwd, lat)


def clutch_base_axis_mapping(axis_mapping, head_orientation, root_orientation, task_orientation):
    """Capture VR horizontal heading -> physical base -> arm task coordinates."""
    mapping = np.asarray(axis_mapping, dtype=float)
    forward = mapping @ (quaternion_to_matrix(head_orientation) @ np.array([0., 0., 1.]))
    if np.linalg.norm(forward[:2]) < 1e-4:
        raise ValueError('headset forward has no usable horizontal heading')
    yaw = math.atan2(forward[1], forward[0])
    c, s = math.cos(yaw), math.sin(yaw)
    heading_inverse = np.array([[c, s, 0.], [-s, c, 0.], [0., 0., 1.]])
    base_to_task = quaternion_to_matrix(task_orientation).T @ quaternion_to_matrix(root_orientation)
    return base_to_task @ heading_inverse @ mapping

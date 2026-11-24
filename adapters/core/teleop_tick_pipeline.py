"""Control tick pipeline, arm/gripper updates, and generic command forwarding."""

from __future__ import annotations

from contextlib import nullcontext
import math
import time
from typing import Any, Callable

import numpy as np
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool

from .arm_teleop_config import _measure_clocks, _record_elapsed
from .arm_teleop_math import (
    binary_trigger_state,
    clutch_base_axis_mapping,
    relative_target,
    stabilize_pose,
)
from .body_base_controller import clear_body_reference
from .homing_controller import publish_homing_command
from .teleop_state import ArmRuntime


def _compute_arm_target_local(
    node: Any,
    arm: ArmRuntime,
    pose: tuple[np.ndarray, np.ndarray],
) -> tuple[np.ndarray, np.ndarray]:
    """Compute rate-limited, filtered arm target in the arm base frame."""
    assert arm.reference_vr is not None
    assert arm.reference_local_ee is not None
    raw_target_local = relative_target(
        *pose,
        *arm.reference_vr,
        *arm.reference_local_ee,
        arm.reference_axis_mapping
        if arm.reference_axis_mapping is not None
        else node.control["axis_mapping"],
        float(node.control["position_scale"]),
        float(node.control["max_displacement"]),
        bool(node.control["orientation_enabled"]),
    )
    rate_hz = float(node.control.get("rate_hz", 100.0))
    max_v = float(node.control.get("target_max_linear_velocity", 0.0))
    max_w = float(node.control.get("target_max_angular_velocity", 0.0))
    max_linear_step = (max_v / rate_hz) if max_v > 0.0 else None
    max_angular_step = (max_w / rate_hz) if max_w > 0.0 else None
    if arm.target_local is None:
        return raw_target_local
    return stabilize_pose(
        *raw_target_local,
        *arm.target_local,
        float(node.control.get("target_position_deadband", 0.0)),
        float(node.control.get("target_orientation_deadband", 0.0)),
        float(node.control.get("target_filter_alpha", 1.0)),
        max_linear_step=max_linear_step,
        max_angular_step=max_angular_step,
    )


def evaluate_clutch_and_base_gates(
    node: Any,
    now: float,
    *,
    use_solver_clutch: bool = False,
) -> tuple[bool, bool, bool, bool]:
    """Evaluate left/right grip, stick overrides, collision latch, and body gate."""
    left_input_fresh = node._input_fresh(node.arms["left"], now)
    right_input_fresh = node._input_fresh(node.arms["right"], now)
    left_base_requested = left_input_fresh and node._clutch_pressed(node.arms["left"])
    waist_active = node._update_waist_clutch(now)
    left_stick_active = (
        left_input_fresh
        and bool(node.body_config.get("left_stick_independent_base", True))
        and node._left_stick_active()
    )
    right_stick_active = (
        right_input_fresh
        and bool(node.body_config.get("right_stick_yaw_enabled", True))
        and node._right_stick_yaw_active()
    )
    arms_requested = right_input_fresh and node._clutch_pressed(node.arms["right"])

    if use_solver_clutch:
        node._update_solver_clutch(arms_requested, input_fresh=right_input_fresh)
        if node.solver_rearm.blocked:
            if right_input_fresh and node.solver_rearm.observe_clutch(arms_requested):
                node.get_logger().info(
                    "right Grip pressed after release; post-home IK control rearmed"
                )
            if node.solver_rearm.blocked:
                arms_requested = False

    right_stick_base = (
        right_stick_active
        and (
            left_base_requested
            or not arms_requested
            or bool(node.body_config.get("right_stick_with_arms", True))
        )
        and bool(node.body_config.get("right_stick_independent_base", True))
    )
    base_requested = left_base_requested or left_stick_active or right_stick_base

    base_allowed, arms_requested = node._apply_collision_latch(
        base_requested, arms_requested
    )
    body_requested = (
        waist_active
        and base_allowed
        and bool(node.body_config.get("enabled", True))
        and now - node.body.head_stamp <= float(node.control["pose_timeout"])
    )
    return left_base_requested, arms_requested, base_allowed, body_requested


def _publish_or_block_base(
    node: Any,
    now: float,
    *,
    base_allowed: bool,
    left_base_requested: bool,
) -> None:
    if not base_allowed and node.collision_blocked:
        node.head_velocity_reference = None
        node.last_head_velocity_time = None
        if getattr(node, "head_base_follower", None) is not None:
            node.head_base_follower.reset()
            node.head_base_state = "collision_blocked"
        node._stop_base_if_pending()
    else:
        node._publish_base_command(
            now,
            track_head_yaw=True,
            left_clutch_active=left_base_requested,
        )


def engage_arm(node: Any, arm: ArmRuntime) -> None:
    arm.reference_axis_mapping = None
    if node.control.get("arm_reference_frame", "fixed") == "base_at_clutch":
        if (
            node.body.head_pose is None
            or node._monotonic() - node.body.head_stamp
            > float(node.control["pose_timeout"])
        ):
            arm.active = False
            return
        try:
            arm.reference_axis_mapping = clutch_base_axis_mapping(
                node.control["axis_mapping"],
                node.body.head_pose[1],
                node._root_pose()[1],
                node._link_pose(arm.base_index)[1],
            )
        except ValueError:
            arm.active = False
            return
    arm.active = True
    arm.predictor.reset()
    arm.reference_vr = arm.pose
    arm.target_local = node._relative_pose(
        node._link_pose(arm.base_index), node._link_pose(arm.ee_index)
    )
    arm.reference_local_ee = arm.target_local
    arm.last_solution = np.asarray(
        [
            node.joint_state.get(name, node.initial_joints.get(name, 0.0))
            for name in arm.joint_names
        ],
        dtype=float,
    )
    arm.ik_converged = True
    arm.ik_within_tolerance = True
    arm.ik_rejections = 0
    arm.ik_seed = "feedback"
    arm.last_ik_attempt = 0.0
    arm.last_reseed_attempt = 0.0
    arm.recovery_solution = None
    arm.recovery_seed = ""
    arm.reseed_target_position = None
    arm.reseed_target_orientation = None


def update_arm(node: Any, arm: ArmRuntime, command: dict[str, float]) -> None:
    if not arm.active:
        return
    assert arm.pose is not None
    arm.target_local = _compute_arm_target_local(node, arm, arm.pose)
    arm.target_world = node._world_pose(
        node._link_pose(arm.base_index), arm.target_local
    )
    previous = np.asarray(
        [node.last_command.get(name, node.joint_state[name]) for name in arm.joint_names]
    )
    result = node._solve_arm_ik(arm, *arm.target_world, previous)
    was_converged = arm.ik_converged
    arm.position_error = result.position_error
    arm.orientation_error = result.orientation_error
    arm.ik_converged = result.accepted
    arm.ik_within_tolerance = result.converged
    arm.ik_seed = result.seed_name
    arm.ik_damping = result.damping
    if not result.accepted:
        arm.ik_rejections += 1
        arm.ik_rejection_total += 1
        node._set_group(arm.joint_indices, previous)
        for name, value in zip(arm.joint_names, previous):
            command[name] = float(value)
        now = node._monotonic()
        if now - arm.last_ik_log >= 1.0:
            arm.last_ik_log = now
            node.get_logger().warning(
                f"{arm.name} IK rejected; holding last valid command "
                f"(position_error={result.position_error:.4f} m, "
                f"orientation_error={result.orientation_error:.3f} rad, "
                f"damping={result.damping:.3f})"
            )
        return

    if not was_converged:
        node.get_logger().info(
            f"{arm.name} IK recovered using {result.seed_name} seed"
        )
    arm.ik_rejections = 0
    # _solve_arm_ik has already evaluated and returned the velocity-limited
    # command. Clamping it again is harmless but would hide solver mistakes,
    # so publish that exact validated step.
    solution = result.joints
    joint_deadband = float(node.control.get("command_joint_deadband", 0.0))
    solution = np.where(
        np.abs(solution - previous) <= joint_deadband, previous, solution
    )
    arm.last_solution = solution
    node._set_group(arm.joint_indices, solution)
    for name, value in zip(arm.joint_names, solution):
        command[name] = float(value)


def update_arm_target(node: Any, arm: ArmRuntime) -> None:
    """Convert VR motion into the chest-relative task consumed by sol_q."""
    assert arm.pose is not None
    pose = arm.pose
    if node.control.get("pose_prediction_enabled", False):
        predicted = arm.predictor.predict(
            node._monotonic(),
            horizon=float(node.control.get("pose_prediction_horizon", 0.10)),
        )
        if predicted is not None:
            pose = predicted
    arm.target_local = _compute_arm_target_local(node, arm, pose)


def trigger_value(node: Any, side: str) -> float:
    joy = node.arms[side].joy
    index = int(node.config["grippers"]["trigger_axis"])
    if joy is None or len(joy.axes) <= index:
        return 0.0
    return float(np.clip(joy.axes[index], 0.0, 1.0))


def gripper_position(node: Any, side: str, trigger: float) -> float:
    cfg = node.config["grippers"][side]
    previous = node.last_command.get(
        cfg["sim_joint"], node.joint_state.get(cfg["sim_joint"], 0.0)
    )
    step = float(node.config["grippers"]["max_velocity"]) / float(
        node.control["rate_hz"]
    )
    return float(
        np.clip(
            trigger * float(cfg["sim_closed_position"]),
            previous - step,
            previous + step,
        )
    )


def update_grippers(node: Any, command: dict[str, float], now: float) -> None:
    for side in ("right", "left"):
        cfg = node.config["grippers"][side]
        trigger = (
            node._trigger(side) if node._input_fresh(node.arms[side], now) else 0.0
        )
        if node.config["grippers"].get("include_sim_joints", True):
            command[cfg["sim_joint"]] = node._gripper_position(side, trigger)
        finger = JointState()
        finger.header.stamp = node.get_clock().now().to_msg()
        finger.name = list(cfg["finger_joint_names"])
        cached = getattr(node, "_gripper_open_closed", None)
        if cached is not None and side in cached:
            opened, closed = cached[side]
        else:
            opened = np.asarray(cfg["finger_open"], dtype=float)
            closed = np.asarray(cfg["finger_closed"], dtype=float)
        finger.position = (opened + trigger * (closed - opened)).tolist()
        binary_state_joint = cfg.get("binary_state_joint")
        if binary_state_joint:
            finger.name.append(str(binary_state_joint))
            finger.position.append(
                binary_trigger_state(
                    trigger,
                    float(
                        node.config["grippers"].get("binary_trigger_threshold", 0.5)
                    ),
                )
            )
        node.finger_pubs[side].publish(finger)


def command_seed(node: Any) -> dict[str, float] | None:
    required = (
        node.arms["right"].joint_names
        + node.arms["left"].joint_names
        + node.body.joint_names
    )
    if any(name not in node.joint_state for name in required):
        return None
    return {
        name: node.joint_state.get(
            name, node.last_command.get(name, node.initial_joints.get(name, 0.0))
        )
        for name in node.controlled_names
    }


def ensure_generic_targets(node: Any) -> None:
    for arm in node.arms.values():
        if arm.target_local is None:
            arm.target_local = node._relative_pose(
                node._link_pose(arm.base_index), node._link_pose(arm.ee_index)
            )
    if node.body.target_base is None:
        node.body.target_base = node._relative_pose(
            node._root_pose(), node._link_pose(node.body.torso_index)
        )


def publish_motion_aux(node: Any) -> None:
    arm_names = {n for arm in node.arms.values() for n in arm.joint_names}
    # One coherent sample survives keep-last QoS. Optional simulated
    # grippers are included only when the current hardware state has them;
    # separate real finger topics are published independently.
    values = {
        n: q
        for n, q in node.generic_aux_command.items()
        if n not in arm_names and n in node.joint_state
    }
    if values:
        msg = JointState(name=list(values), position=list(values.values()))
        msg.header.stamp = node.get_clock().now().to_msg()
        node.command_pub.publish(msg)


def publish_generic_grippers(node: Any, now: float) -> None:
    command: dict[str, float] = {}
    node._update_grippers(command, now)
    if command:
        with (getattr(node, "_command_state_lock", None) or nullcontext()):
            node.generic_aux_command.update(command)
            node.last_command.update(command)


def generic_command_callback(
    node: Any,
    message: JointState,
    *,
    measure_clocks: Callable[[], tuple[int, int]] = _measure_clocks,
) -> None:
    metrics = getattr(node, "generic_command_pipeline", None)
    if metrics is None:
        forward_generic_command(node, message, measure_clocks=measure_clocks)
        return
    started = measure_clocks()
    source_ns = (
        int(message.header.stamp.sec) * 1_000_000_000
        + int(message.header.stamp.nanosec)
    )
    received_wall_ns = time.time_ns()
    # This is observation only. Avoid an additional native ROS Clock.now()
    # read on the forwarding path. Simulated/foreign epoch headers cannot
    # be compared to wall time and must not masquerade as control latency.
    comparable = (
        source_ns > 0 and abs(received_wall_ns - source_ns) <= 86_400_000_000_000
    )
    source_age = (received_wall_ns - source_ns) / 1e6 if comparable else None
    state_lock = getattr(node, "_command_state_lock", None) or nullcontext()
    with state_lock:
        metrics["received_total"] += 1
        metrics["last_received_monotonic_ns"] = started[0]
        metrics["last_received_source_stamp_ns"] = source_ns
        metrics["last_received_wall_ns"] = received_wall_ns
        metrics["last_receive_source_age_comparable"] = comparable
        metrics["source_age_incomparable_total"] += not comparable
        metrics["last_receive_source_age_ms"] = source_age
        previous_max = metrics["max_receive_source_age_ms"]
        if source_age is not None:
            metrics["max_receive_source_age_ms"] = (
                source_age if previous_max is None else max(previous_max, source_age)
            )
            metrics["_source_ages"].append((started[0], source_age))
    try:
        rejected = forward_generic_command(
            node, message, measure_clocks=measure_clocks
        )
        if rejected is not None:
            with state_lock:
                metrics["rejected"][rejected] += 1
    except Exception:
        with state_lock:
            metrics["callback_errors"] += 1
        raise
    finally:
        finished = measure_clocks()
        with state_lock:
            _record_elapsed(
                metrics["callback"],
                finished[0] - started[0],
                finished[1] - started[1],
            )


def forward_generic_command(
    node: Any,
    message: JointState,
    *,
    measure_clocks: Callable[[], tuple[int, int]] = _measure_clocks,
) -> str | None:
    state_lock = getattr(node, "_command_state_lock", None) or nullcontext()
    with state_lock:
        if getattr(node, "intervention", None) and node.intervention.active:
            return "intervention_active"
        if node.control_source != "vr":
            return "source_not_vr"
        if node.source_feedback_pending:
            return "source_feedback_pending"
        if node.backend == "motion":
            return "motion_backend"  # Shared RTC output goes directly to the selected boundary.
        if len(message.name) != len(message.position):
            return "length_mismatch"
        if not all(math.isfinite(float(value)) for value in message.position):
            return "nonfinite_position"
        now = node._monotonic()
        node.generic_command_stamp = now
        node.generic_command_count += 1
        if not node.enabled:
            return "disabled"
        if not node.hardware_ready:
            return "hardware_not_ready"
        if node.homing:
            return "homing"
        if not node._ensure_vr_feedback():
            return "stale_feedback"
        if node.solver_rearm.blocked:
            stamp_ns = (
                int(message.header.stamp.sec) * 1_000_000_000
                + int(message.header.stamp.nanosec)
            )
            if (
                node.hardware_ready
                and message.header.frame_id != "solver_clutch_hold"
                and node.solver_rearm.observe_solver_output(stamp_ns)
            ):
                node.get_logger().info(
                    "fresh post-reset IK output received; release and press right Grip to resume"
                )
        merged = dict(zip(message.name, (float(value) for value in message.position)))
        arms_active = not node.solver_rearm.blocked and any(
            node.arms[side].active for side in ("right", "left")
        )
        body_active = node.body.active
        if not arms_active:
            for arm in node.arms.values():
                for name in arm.joint_names:
                    merged[name] = float(
                        node.last_command.get(
                            name,
                            node.joint_state.get(
                                name, node.initial_joints.get(name, 0.0)
                            ),
                        )
                    )
        merged.update(node.generic_aux_command)
        if not body_active:
            for name in node.body.joint_names:
                merged.pop(name, None)
    if arms_active or body_active:
        if not node._collision_command_allowed(merged, now):
            return "collision"
    output = JointState()
    output.header = message.header
    output.name = list(merged)
    output.position = [merged[name] for name in output.name]
    metrics = getattr(node, "generic_command_pipeline", None)
    if metrics is None:
        node.command_pub.publish(output)
    else:
        started = measure_clocks()
        try:
            node.command_pub.publish(output)
        except Exception:
            with state_lock:
                metrics["publish_errors"] += 1
            raise
        finally:
            finished = measure_clocks()
            with state_lock:
                _record_elapsed(
                    metrics["publish"],
                    finished[0] - started[0],
                    finished[1] - started[1],
                )
        with state_lock:
            previous = metrics["last_published_monotonic_ns"]
            if previous:
                gap = (started[0] - previous) / 1e6
                metrics["last_publication_gap_ms"] = gap
                metrics["max_publication_gap_ms"] = max(
                    metrics["max_publication_gap_ms"], gap
                )
                metrics["_publication_gaps"].append((started[0], gap))
            metrics["published_total"] += 1
            metrics["last_published_monotonic_ns"] = started[0]
            metrics["last_published_source_stamp_ns"] = (
                int(message.header.stamp.sec) * 1_000_000_000
                + int(message.header.stamp.nanosec)
            )
    with state_lock:
        node.last_command.update(merged)
    return None


def generic_control_tick(
    node: Any,
    now: float,
    feedback_fresh: bool,
    command: dict[str, float] | None,
) -> None:
    if not node.enabled or command is None:
        if node.homing:
            if node.backend == "motion":
                node.motion_home_cancel_pub.publish(Bool(data=True))
            node.solver_rearm.homing_complete()
        node.homing = False
        node._release_all(send_base_zero=node.base_zero_pending)
        node.base_zero_pending = False
        node._publish_status(now, feedback_fresh)
        return

    node._ensure_generic_targets()
    if node._home_gesture_triggered(now):
        node._set_generic_home_targets()
    if node.homing:
        if node.backend == "motion":
            node._motion_home_tick(now, feedback_fresh)
            return
        node._update_homing(command)
        command.update(node.generic_aux_command)
        if not node._collision_command_allowed(command, now):
            node._publish_status(now, feedback_fresh)
            return
        publish_homing_command(node, command, fallback_to_last=True)
        node._publish_target_poses()
        node._publish_generic_grippers(now)
        node._publish_status(now, feedback_fresh)
        return

    (
        left_base_requested,
        arms_requested,
        base_allowed,
        body_requested,
    ) = evaluate_clutch_and_base_gates(node, now, use_solver_clutch=True)

    body_command: dict[str, float] = {}
    if body_requested:
        with (getattr(node, "_model_lock", None) or nullcontext()):
            if not node.body.active:
                node._engage_body()
            node._update_body(body_command)
        with (getattr(node, "_command_state_lock", None) or nullcontext()):
            node.generic_aux_command.update(body_command)
            node.last_command.update(body_command)
    elif node.body.active:
        with (getattr(node, "_command_state_lock", None) or nullcontext()):
            node.body.active = False
            for name in node.body.joint_names:
                node.generic_aux_command.pop(name, None)
        clear_body_reference(node, clear_smoothed=True)
        node.get_logger().info("head/left clutch released: waist + yaw hold")
    else:
        with (getattr(node, "_command_state_lock", None) or nullcontext()):
            for name in node.body.joint_names:
                node.generic_aux_command.pop(name, None)

    neck_command: dict[str, float] = {}
    node._update_neck_joints(neck_command, now)
    if neck_command:
        with (getattr(node, "_command_state_lock", None) or nullcontext()):
            node.generic_aux_command.update(neck_command)
            node.last_command.update(neck_command)

    _publish_or_block_base(
        node,
        now,
        base_allowed=base_allowed,
        left_base_requested=left_base_requested,
    )

    arm_was_active = any(arm.active for arm in node.arms.values())
    with (getattr(node, "_model_lock", None) or nullcontext()):
        for side in ("right", "left"):
            arm = node.arms[side]
            pose_fresh = now - arm.pose_stamp <= float(node.control["pose_timeout"])
            if arms_requested and pose_fresh:
                if not arm.active:
                    node._engage_arm(arm)
                if arm.active:
                    node._update_arm_target(arm)
            elif arm.active:
                arm.active = False
                arm.reference_vr = None
                arm.reference_local_ee = None
                arm.target_world = None
            elif not arms_requested:
                if arm.target_local is None:
                    arm.target_local = node._relative_pose(
                        node._link_pose(arm.base_index), node._link_pose(arm.ee_index)
                    )
    if (
        arms_requested
        and not arm_was_active
        and any(arm.active for arm in node.arms.values())
    ):
        node.get_logger().info(
            f"right clutch engaged: {node.backend} controller owns both arms"
        )
    # 灵巧手指令始终常驻下发，食指扳机按压即可实时连续开合（无需持续按住 Grip 离合）
    node._publish_generic_grippers(now)

    if node.backend == "motion":
        node._publish_motion_aux()

    # Always publish all three finite tasks. The copied solver otherwise sees
    # an uninitialised target at startup and CasADi can produce NaN.
    node._publish_target_poses()
    node._publish_status(now, feedback_fresh)


def control_tick(
    node: Any,
    *,
    measure_clocks: Callable[[], tuple[int, int]] = _measure_clocks,
) -> None:
    metrics = getattr(node, "control_tick_metrics", None)
    if metrics is None:
        control_tick_body(node, measure_clocks=measure_clocks)
        return
    metrics["ticks"] += 1
    if metrics["ticks"] % metrics["sample_every_ticks"]:
        control_tick_body(node, measure_clocks=measure_clocks)
        return
    started = measure_clocks()
    sample = node._control_tick_sample = dict(
        sync_model=(0, 0), actual_head_fk=(0, 0)
    )
    try:
        control_tick_body(node, measure_clocks=measure_clocks)
    finally:
        finished = measure_clocks()
        total = finished[0] - started[0], finished[1] - started[1]
        sample["remainder"] = tuple(
            total[i] - sample["sync_model"][i] - sample["actual_head_fk"][i]
            for i in (0, 1)
        )
        sample["total"] = total
        for name, elapsed in sample.items():
            _record_elapsed(metrics["stages"][name], *elapsed)
        metrics["sampled_ticks"] += 1
        node._control_tick_sample = None


def control_tick_body(
    node: Any,
    *,
    measure_clocks: Callable[[], tuple[int, int]] = _measure_clocks,
) -> None:
    now = node._monotonic()
    feedback_fresh = now - node.joint_state_stamp <= float(
        node.control["joint_state_timeout"]
    )
    command = node._command_seed() if feedback_fresh else None
    if command is not None:
        sample = getattr(node, "_control_tick_sample", None)
        if sample is None:
            node._sync_model()
            node._publish_actual_poses()
        else:
            started = measure_clocks()
            node._sync_model()
            finished = measure_clocks()
            sample["sync_model"] = finished[0] - started[0], finished[1] - started[1]
            started = measure_clocks()
            node._publish_actual_poses()
            finished = measure_clocks()
            sample["actual_head_fk"] = (
                finished[0] - started[0],
                finished[1] - started[1],
            )
    if node.solver_rearm.blocked and node._input_fresh(node.arms["right"], now):
        right_grip = node._clutch_pressed(node.arms["right"])
        if node.solver_rearm.observe_clutch(right_grip):
            node.get_logger().info(
                "right Grip pressed after post-home reset; IK control rearmed"
            )
    if getattr(node, "intervention", None) and node.intervention.active:
        node.intervention.tick(now, feedback_fresh and node.hardware_ready)
        return
    if node.control_source != "vr" or node.source_feedback_pending:
        node._publish_status(now, feedback_fresh)
        return
    if not node.hardware_ready:
        if getattr(node, "head_base_follower", None) is not None:
            node.head_base_follower.reset()
            node.head_base_state = "hardware_not_ready"
            node._stop_base_if_pending()
        if any(arm.active for arm in node.arms.values()) or node.body.active:
            node._release_all(send_base_zero=node.base_zero_pending)
            node.base_zero_pending = False
        node._publish_status(now, feedback_fresh)
        return
    if node.external_ik:
        if (
            node.backend != "motion"
            and node.enabled
            and not node.homing
            and not node._ensure_vr_feedback()
        ):
            return
        node._generic_control_tick(now, feedback_fresh, command)
        return
    if not node.enabled or command is None:
        node.homing = False
        node._release_all(send_base_zero=node.base_zero_pending)
        node.base_zero_pending = False
        node._publish_status(now, feedback_fresh)
        return
    if node._home_gesture_triggered(now):
        node._start_homing()
    if node.homing:
        node._update_homing(command)
        if not node._collision_command_allowed(command, now):
            node._publish_status(now, feedback_fresh)
            return
        publish_homing_command(node, command, fallback_to_last=False)
        node._publish_status(now, feedback_fresh)
        return
    if node.home_gesture_latched:
        node._publish_status(now, feedback_fresh)
        return

    (
        left_base_requested,
        arms_requested,
        base_allowed,
        body_requested,
    ) = evaluate_clutch_and_base_gates(node, now, use_solver_clutch=False)
    any_active = False

    if body_requested:
        if not node.body.active:
            node._engage_body()
        node._update_body(command)
        any_active = True
    elif node.body.active:
        clear_body_reference(node)
        node.get_logger().info("head/left clutch released: waist + yaw hold")

    neck_command: dict[str, float] = {}
    node._update_neck_joints(neck_command, now)
    if neck_command:
        command.update(neck_command)
        any_active = True

    _publish_or_block_base(
        node,
        now,
        base_allowed=base_allowed,
        left_base_requested=left_base_requested,
    )

    arm_was_active = any(arm.active for arm in node.arms.values())
    for side in ("right", "left"):
        arm = node.arms[side]
        pose_fresh = now - arm.pose_stamp <= float(node.control["pose_timeout"])
        if arms_requested and pose_fresh:
            if not arm.active:
                node._engage_arm(arm)
            node._update_arm(arm, command)
            any_active = True
        elif arm.active:
            node._release_arm(arm)
    if (
        arms_requested
        and not arm_was_active
        and any(arm.active for arm in node.arms.values())
    ):
        node.get_logger().info("right clutch engaged: both arms + grippers")
    if not arms_requested and arm_was_active:
        right_joy = node.arms["right"].joy
        grip_index = int(node.control["clutch_axis"])
        grip_value = (
            right_joy.axes[grip_index]
            if right_joy is not None and len(right_joy.axes) > grip_index
            else float("nan")
        )
        node.get_logger().info(
            "right clutch released: both arms + grippers hold "
            f"(grip={grip_value:.3f}, input_age={now - node.arms['right'].joy_stamp:.3f}s)"
        )
    if arms_requested:
        node._update_grippers(command, now)

    if any_active:
        if not node._collision_command_allowed(command, now):
            node._publish_status(now, feedback_fresh)
            return
        message = JointState()
        message.header.stamp = node.get_clock().now().to_msg()
        if node.body.active:
            message.name = list(node.controlled_names)
        else:
            waist_names = set(node.body.joint_names)
            message.name = [
                name for name in node.controlled_names if name not in waist_names
            ]
        message.position = [float(command[name]) for name in message.name]
        node.command_pub.publish(message)
        node.last_command.update({name: command[name] for name in message.name})
        if arms_requested:
            node._publish_target_poses()
    node._publish_status(now, feedback_fresh)


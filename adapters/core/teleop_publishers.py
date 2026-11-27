"""ROS publishers for arm/head target & actual poses, hand triggers, and status."""

from __future__ import annotations

from contextlib import nullcontext
import json
import math
import time
from typing import Any

from geometry_msgs.msg import Point, Pose, PoseArray, Quaternion
import numpy as np
from sensor_msgs.msg import Joy
from std_msgs.msg import String

from .arm_teleop_config import _generic_pipeline_snapshot
from .arm_teleop_math import binary_trigger_state


def arm_base_frame(node: Any) -> str:
    cached = getattr(node, "_arm_base_frame_id", None)
    if cached is not None:
        return cached
    base_links = {config["base_link"] for config in node.config["arms"].values()}
    cached = next(iter(base_links)) if len(base_links) == 1 else "hc_tj_arm_bases"
    node._arm_base_frame_id = cached
    return cached


def pose_from_tuple(position: Any, orientation: Any) -> Pose:
    return Pose(
        position=Point(
            x=float(position[0]), y=float(position[1]), z=float(position[2])
        ),
        orientation=Quaternion(
            x=float(orientation[0]),
            y=float(orientation[1]),
            z=float(orientation[2]),
            w=float(orientation[3]),
        ),
    )


def publish_hand_triggers(node: Any, inputs: dict[str, Joy]) -> None:
    message = PoseArray()
    message.header.stamp = node.get_clock().now().to_msg()
    message.header.frame_id = "vr_hand_triggers_left_right"
    message.poses = [Pose(), Pose()]
    threshold = float(node.config["grippers"].get("binary_trigger_threshold", 0.5))
    for index, side in enumerate(("left", "right")):
        joy = inputs.get(side)
        trigger = (
            float(np.clip(joy.axes[0], 0.0, 1.0))
            if joy is not None and joy.axes
            else 0.0
        )
        message.poses[index].position.x = trigger
        message.poses[index].position.y = binary_trigger_state(trigger, threshold)
        message.poses[index].orientation.w = 1.0
    node.hand_trigger_pub.publish(message)


def publish_target_poses(node: Any) -> None:
    stamp = node.get_clock().now().to_msg()
    frame_id = node._arm_base_frame()
    message = PoseArray()
    message.header.stamp = stamp
    message.header.frame_id = frame_id
    controller_message = PoseArray()
    controller_message.header.stamp = stamp
    controller_message.header.frame_id = (
        "generic_task_bases" if node.external_ik else frame_id
    )
    link_poses: dict[int, Any] = {}

    def link_pose(index: int) -> Any:
        pose = link_poses.get(index)
        if pose is None:
            pose = node._link_pose(index)
            link_poses[index] = pose
        return pose

    for side in ("right", "left"):
        arm = node.arms[side]
        local = arm.target_local or node._relative_pose(
            link_pose(arm.base_index), link_pose(arm.ee_index)
        )
        message.poses.append(pose_from_tuple(local[0], local[1]))
        controller_local = local
        if node.external_ik:
            target_world = node._world_pose(link_pose(arm.base_index), local)
            controller_local = node._relative_pose(
                link_pose(node.generic_task_base_indices[side]),
                target_world,
            )

        # Do not impose an artificial Cartesian reach limit here. The v23
        # solver already enforces the robot's real joint limits and applies
        # singularity damping; clipping this target used to create a false
        # wall at 0.62-0.65 m and made otherwise reachable poses impossible.
        controller_message.poses.append(
            pose_from_tuple(controller_local[0], controller_local[1])
        )
    if node.external_ik and bool(
        node.body_config.get("include_in_controller_tasks", False)
    ):
        torso = node.body.target_base or node._relative_pose(
            node._root_pose(), link_pose(node.body.torso_index)
        )
        controller_message.poses.append(pose_from_tuple(torso[0], torso[1]))
    if getattr(node, "intervention", None) and node.intervention.active:
        if node.backend == "v23":
            node.intervention.poses(controller_message, action_pose=message)
        else:
            node.intervention.poses(message)
        return
    node.target_pub.publish(message)
    node.controller_target_pub.publish(controller_message)


def _publisher_has_subscribers(publisher: Any) -> bool:
    if publisher is None:
        return False
    count_fn = getattr(publisher, "get_subscription_count", None)
    if not callable(count_fn):
        return True
    return int(count_fn()) > 0


def publish_actual_poses(node: Any) -> None:
    actual_active = _publisher_has_subscribers(node.actual_pub)
    record_active = _publisher_has_subscribers(node.record_actual_pub)
    if not actual_active and not record_active:
        head_pub = getattr(node, "head_visualization_pub", None)
        if head_pub is None or _publisher_has_subscribers(head_pub):
            stamp = node.get_clock().now().to_msg()
            with (getattr(node, "_model_lock", None) or nullcontext()):
                node._publish_head_tracking_poses(stamp=stamp)
        else:
            node._publish_head_tracking_poses()
        return
    stamp = node.get_clock().now().to_msg()
    message = PoseArray()
    message.header.stamp = stamp
    message.header.frame_id = node._arm_base_frame()
    link_poses: dict[int, Any] = {}
    with (getattr(node, "_model_lock", None) or nullcontext()):
        for side in ("right", "left"):
            arm = node.arms[side]
            base_pose = link_poses.get(arm.base_index)
            if base_pose is None:
                base_pose = node._link_pose(arm.base_index)
                link_poses[arm.base_index] = base_pose
            ee_pose = link_poses.get(arm.ee_index)
            if ee_pose is None:
                ee_pose = node._link_pose(arm.ee_index)
                link_poses[arm.ee_index] = ee_pose
            local = node._relative_pose(base_pose, ee_pose)
            message.poses.append(pose_from_tuple(local[0], local[1]))
    if actual_active:
        node.actual_pub.publish(message)
    if record_active:
        node.record_actual_pub.publish(message)
    head_pub = getattr(node, "head_visualization_pub", None)
    if head_pub is None or _publisher_has_subscribers(head_pub):
        with (getattr(node, "_model_lock", None) or nullcontext()):
            node._publish_head_tracking_poses(stamp=stamp)
    else:
        node._publish_head_tracking_poses()


def publish_head_tracking_poses(node: Any, stamp: Any = None) -> None:
    """Feedback and mapped head target, relative to the model base pose."""
    if not _publisher_has_subscribers(getattr(node, "head_visualization_pub", None)):
        return
    head_link = node.body_config.get("visualization_head_link")
    if head_link not in node.link_by_name:
        return
    root = node._root_pose()
    actual_world = node._link_pose(node.link_by_name[head_link])
    actual = node._relative_pose(root, actual_world)
    target = actual
    body = node.body
    if (
        body.active
        and node.enabled
        and node.hardware_ready
        and node._monotonic() - body.head_stamp <= float(node.control["pose_timeout"])
    ):
        if body.target_head_local is not None:
            target = body.target_head_local
        elif body.target_base is not None:
            head_in_torso = node._relative_pose(
                node._link_pose(body.torso_index), actual_world
            )
            target = node._world_pose(body.target_base, head_in_torso)
    message = PoseArray()
    message.header.stamp = (
        stamp if stamp is not None else node.get_clock().now().to_msg()
    )
    message.header.frame_id = "robot_base"
    message.poses.extend(
        (
            pose_from_tuple(actual[0], actual[1]),
            pose_from_tuple(target[0], target[1]),
        )
    )
    node.head_visualization_pub.publish(message)


def publish_status(node: Any, now: float, feedback_fresh: bool) -> None:
    status_key = (
        node.enabled,
        node.hardware_ready,
        feedback_fresh,
        node.solver_rearm.state,
        node.homing,
        node.control_source,
        node.source_feedback_pending,
        node.stop_reason,
        node.collision_blocked,
    )
    if (
        now - node.last_status_publish < 0.2
        and status_key == getattr(node, "_last_status_transition_key", None)
    ):
        return
    node._last_status_transition_key = status_key
    node.last_status_publish = now
    left_clutch = node._input_fresh(
        node.arms["left"], now
    ) and node._clutch_pressed(node.arms["left"])
    right_clutch = node._input_fresh(
        node.arms["right"], now
    ) and node._clutch_pressed(node.arms["right"])
    payload = {
        "stamp_ns": time.time_ns(),
        "enabled": node.enabled,
        "control_source": node.control_source,
        "source_feedback_pending": node.source_feedback_pending,
        "backend": node.backend,
        "generic_controller": {
            "solver_fresh": not node.external_ik
            or now - node.solver_stamp <= float(node.control["joint_state_timeout"]),
            "solver_messages": node.solver_message_count,
            "command_fresh": not node.external_ik
            or now - node.generic_command_stamp
            <= float(node.control["joint_state_timeout"]),
            "command_messages": node.generic_command_count,
            # Bounded counters/timings; formatting stays in this existing
            # low-rate status publication, outside the command callback.
            "pipeline": _generic_pipeline_snapshot(
                getattr(node, "generic_command_pipeline", None),
                getattr(node, "_command_state_lock", None),
            ),
            "rearm_state": node.solver_rearm.state,
            "clutch_hold": node._clutch_solver_held,
            "clutch_resume_pending": node._clutch_resume_pending,
            "hardware_ready": node.hardware_ready,
        },
        "control_tick_timing": getattr(node, "control_tick_metrics", None),
        "qos_events": (
            node._qos_event_observer.status()
            if getattr(node, "_qos_event_observer", None) is not None
            else None
        ),
        "head_base_pose": {
            "mode": getattr(node, "head_base_mode", "pose"),
            "enabled": (
                getattr(node, "head_base_follower", None) is not None
                or getattr(node, "head_base_mode", "pose")
                in {"yaw_velocity", "planar_velocity"}
            ),
            "state": getattr(node, "head_base_state", "idle"),
            "target": (
                node.head_base_follower.target
                if getattr(node, "head_base_follower", None)
                else node.head_planar_follower.target
                if getattr(node, "head_planar_follower", None)
                else None
            ),
            "feedback_source": (
                "measured_pose" if node.head_base_mode == "pose" else "command_estimate"
            ),
        },
        "stop_reason": node.stop_reason,
        "feedback_fresh": feedback_fresh,
        "collision_avoidance": {
            "enabled": node.collision_enabled,
            "blocked": node.collision_blocked,
            "escape_mode": node.collision_escape_mode,
            "pair": node.collision_pair,
            "distance": (
                round(node.collision_distance, 5)
                if math.isfinite(node.collision_distance)
                else None
            ),
            "minimum_distance": node.collision_min_distance,
            "rejections": node.collision_rejections,
            "policy": {
                "intra_arm": node.collision_check_intra_arm,
                "inter_arm": node.collision_check_inter_arm,
                "arm_body": node.collision_check_arm_body,
            },
            "waiting_for_grip_release": {
                "right": node.collision_wait_right_release,
                "left": node.collision_wait_left_release,
            },
        },
        "mode": (
            "homing"
            if node.homing
            else "both"
            if left_clutch and right_clutch and not node.solver_rearm.blocked
            else "base_waist"
            if left_clutch
            else "ik_rearming"
            if node.solver_rearm.blocked
            else "arms_grippers"
            if right_clutch
            else "hold"
        ),
        "home_gesture_latched": node.home_gesture_latched,
        "left_clutch": left_clutch,
        "right_clutch": right_clutch,
        "body": {
            "active": node.body.active,
            "lift_error": round(node.body.lift_error, 5),
            "pitch_error": round(node.body.pitch_error, 5),
        },
        "arms": {
            side: {
                "active": arm.active,
                "pose_fresh": now - arm.pose_stamp
                <= float(node.control["pose_timeout"]),
                "position_error": round(arm.position_error, 5),
                "orientation_error": round(arm.orientation_error, 5),
                "ik_converged": arm.ik_converged
                if node.backend == "legacy"
                else None,
                "ik_within_tolerance": arm.ik_within_tolerance,
                "ik_rejections": arm.ik_rejections,
                "ik_rejection_total": arm.ik_rejection_total,
                "ik_seed": arm.ik_seed,
                "ik_damping": round(arm.ik_damping, 5),
            }
            for side, arm in node.arms.items()
        },
    }
    node.status_pub.publish(
        String(data=json.dumps(payload, separators=(",", ":")))
    )


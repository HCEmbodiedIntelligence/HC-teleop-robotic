"""Configuration validation, runtime builders, and pipeline metrics for arm teleop."""

from __future__ import annotations

from collections import deque
from contextlib import nullcontext
import math
from pathlib import Path
import time
from typing import Any

import numpy as np
import pybullet as bullet

from .teleop_state import ArmRuntime, BodyRuntime


LEGACY_IK_DEFAULTS = {
    "ik_iterations": 16,
    "ik_position_tolerance": 0.01,
    "ik_orientation_tolerance": 0.06,
    "ik_damping_min": 0.05,
    "ik_damping_max": 0.45,
    "ik_singularity_threshold": 0.08,
    "ik_limit_margin": 0.12,
    "ik_reseed_limit_margin": 0.08,
    "ik_failure_retry_interval": 0.2,
    "ik_reseed_interval": 0.5,
    "ik_reseed_target_position_delta": 0.005,
    "ik_reseed_target_orientation_delta": 0.035,
    "ik_reseed_perturbation": 0.3,
    "ik_limit_avoidance_gain": 0.035,
    "ik_posture_gain": 0.02,
    "ik_home_posture_weight": 0.8,
    "ik_candidate_home_weight": 0.5,
    "ik_solution_limit_weight": 4.0,
    "ik_min_progress_ratio": 0.001,
    "ik_min_progress_absolute": 0.0001,
    "ik_reseed_escape_progress_ratio": 0.05,
    "ik_reseed_escape_max_regression": 0.25,
    "ik_max_update": 0.08,
    "max_joint_velocity": 1.8,
    "max_joint_velocity_norm": 3.2,
}


def _timing_metric() -> dict[str, float | int]:
    return dict(
        samples=0,
        last_wall_ms=0.0,
        last_cpu_ms=0.0,
        max_wall_ms=0.0,
        max_cpu_ms=0.0,
    )


def _measure_clocks() -> tuple[int, int]:
    return time.monotonic_ns(), time.thread_time_ns()


def _record_elapsed(metric, wall_ns: int, cpu_ns: int) -> None:
    wall_ms, cpu_ms = wall_ns / 1e6, cpu_ns / 1e6
    metric.update(
        samples=metric["samples"] + 1,
        last_wall_ms=wall_ms,
        last_cpu_ms=cpu_ms,
        max_wall_ms=max(metric["max_wall_ms"], wall_ms),
        max_cpu_ms=max(metric["max_cpu_ms"], cpu_ms),
    )


def _generic_pipeline_metrics() -> dict[str, Any]:
    return dict(
        received_total=0,
        published_total=0,
        publish_errors=0,
        callback_errors=0,
        rejected={
            reason: 0
            for reason in (
                "intervention_active",
                "source_not_vr",
                "source_feedback_pending",
                "motion_backend",
                "length_mismatch",
                "nonfinite_position",
                "disabled",
                "hardware_not_ready",
                "homing",
                "stale_feedback",
                "collision",
            )
        },
        last_received_source_stamp_ns=0,
        last_received_monotonic_ns=0,
        last_received_wall_ns=0,
        receive_source_age_basis="wall_receive_minus_original_header_diagnostic_only",
        last_receive_source_age_comparable=False,
        source_age_incomparable_total=0,
        last_receive_source_age_ms=None,
        max_receive_source_age_ms=None,
        last_published_source_stamp_ns=0,
        last_published_monotonic_ns=0,
        last_publication_gap_ms=None,
        max_publication_gap_ms=0.0,
        publication_gap_basis="successful_publish_call_start_monotonic",
        callback=_timing_metric(),
        publish=_timing_metric(),
        _publication_gaps=deque(maxlen=256),
        _source_ages=deque(maxlen=256),
    )


def _generic_pipeline_snapshot(metrics, lock=None):
    if metrics is None:
        return None
    now = time.monotonic_ns()
    with (lock if lock is not None else nullcontext()):
        result = {key: value for key, value in metrics.items() if not key.startswith("_")}
        pub_gaps = list(metrics["_publication_gaps"])
        src_ages = list(metrics["_source_ages"])
    result["recent_window_seconds"] = 2.0
    result["max_publication_gap_ms_recent"] = max(
        (value for stamp, value in pub_gaps if now - stamp <= 2_000_000_000),
        default=None,
    )
    result["max_receive_source_age_ms_recent"] = max(
        (value for stamp, value in src_ages if now - stamp <= 2_000_000_000),
        default=None,
    )
    return result


def _validate_control_section(control: dict[str, Any]) -> None:
    rate = float(control.get("rate_hz", 0.0))
    if not math.isfinite(rate) or rate <= 0.0:
        raise ValueError("control.rate_hz must be positive")
    press = float(control.get("clutch_threshold", 0.55))
    release = float(control.get("clutch_release_threshold", press))
    if not (math.isfinite(press) and math.isfinite(release) and 0 <= release <= press <= 1):
        raise ValueError("clutch thresholds must satisfy 0 <= release <= press <= 1")
    horizon = float(control.get("pose_prediction_horizon", 0.10))
    if not math.isfinite(horizon) or not 0 < horizon < 0.15:
        raise ValueError("pose_prediction_horizon must be in (0, 0.15) seconds")
    vr_data_topic = control.get("vr_data_topic", "/vrdata")
    if not isinstance(vr_data_topic, str) or not vr_data_topic.startswith("/"):
        raise ValueError("control.vr_data_topic must start with /")
    for key in ("axis_mapping", "head_axis_mapping"):
        mapping = np.asarray(
            control.get(key, control["axis_mapping"]),
            dtype=float,
        )
        if mapping.shape != (3, 3) or not np.all(np.isfinite(mapping)):
            raise ValueError(f"control.{key} must be finite 3x3")
    for key in (
        "target_position_deadband",
        "target_orientation_deadband",
        "command_joint_deadband",
    ):
        value = float(control.get(key, 0.0))
        if not math.isfinite(value) or value < 0.0:
            raise ValueError(f"control.{key} must be finite and non-negative")
    filter_alpha = float(control.get("target_filter_alpha", 1.0))
    if not math.isfinite(filter_alpha) or not 0.0 < filter_alpha <= 1.0:
        raise ValueError("control.target_filter_alpha must be in (0, 1]")


def _validate_collision_section(control: dict[str, Any]) -> None:
    collision_distance = float(control.get("collision_min_distance", 0.02))
    if not math.isfinite(collision_distance) or collision_distance <= 0.0:
        raise ValueError("control.collision_min_distance must be finite and positive")
    collision_tolerance = float(control.get("collision_home_tolerance", 0.008))
    if (
        not math.isfinite(collision_tolerance)
        or collision_tolerance < 0.0
        or collision_tolerance >= collision_distance
    ):
        raise ValueError(
            "control.collision_home_tolerance must be non-negative and below "
            "collision_min_distance"
        )
    collision_graph_distance = control.get("collision_ignore_graph_distance", 2)
    if (
        isinstance(collision_graph_distance, bool)
        or not isinstance(collision_graph_distance, int)
        or collision_graph_distance < 1
    ):
        raise ValueError(
            "control.collision_ignore_graph_distance must be a positive integer"
        )
    collision_latch = control.get("collision_latch_until_release", True)
    if not isinstance(collision_latch, bool):
        raise ValueError("control.collision_latch_until_release must be boolean")
    for key, default in (
        ("collision_check_intra_arm", False),
        ("collision_check_inter_arm", True),
        ("collision_check_arm_body", True),
    ):
        value = control.get(key, default)
        if not isinstance(value, bool):
            raise ValueError(f"control.{key} must be boolean")
    collision_escape_epsilon = float(control.get("collision_escape_epsilon", 1.0e-5))
    if not math.isfinite(collision_escape_epsilon) or collision_escape_epsilon < 0.0:
        raise ValueError(
            "control.collision_escape_epsilon must be finite and non-negative"
        )


def _validate_body_section(body: dict[str, Any]) -> None:
    for key in (
        "stick_forward_direction",
        "stick_lateral_direction",
        "yaw_direction",
        "height_direction",
        "pitch_direction",
    ):
        value = float(body.get(key, 1.0))
        if value not in (-1.0, 1.0):
            raise ValueError(f"body.{key} must be -1 or 1")
    for key in ("stick_x_axis", "stick_y_axis"):
        value = body.get(key)
        if isinstance(value, bool) or not isinstance(value, int) or value < 0:
            raise ValueError(f"body.{key} must be a non-negative integer")
    stick_deadzone = float(body.get("stick_deadzone", -1.0))
    if not math.isfinite(stick_deadzone) or not 0.0 <= stick_deadzone < 1.0:
        raise ValueError("body.stick_deadzone must be finite and in [0, 1)")
    for key in ("max_linear_velocity", "max_lateral_velocity"):
        value = float(body.get(key, -1.0))
        if not math.isfinite(value) or value < 0.0:
            raise ValueError(f"body.{key} must be finite and non-negative")
    if body.get("base_command_mode", "delta") not in {"delta", "velocity"}:
        raise ValueError("body.base_command_mode must be delta or velocity")


def _validate_legacy_ik_section(control: dict[str, Any]) -> None:
    for key in (
        "ik_position_tolerance",
        "ik_orientation_tolerance",
        "ik_damping_min",
        "ik_damping_max",
        "ik_singularity_threshold",
        "ik_max_update",
        "ik_reseed_interval",
        "ik_reseed_perturbation",
        "ik_failure_retry_interval",
        "max_joint_velocity",
        "max_joint_velocity_norm",
        "ik_reseed_target_position_delta",
        "ik_reseed_target_orientation_delta",
    ):
        value = float(control.get(key, 0.0))
        if not math.isfinite(value) or value <= 0.0:
            raise ValueError(f"control.{key} must be finite and positive")
    if float(control["ik_damping_max"]) < float(control["ik_damping_min"]):
        raise ValueError("control.ik_damping_max must be >= ik_damping_min")
    for key in ("ik_limit_margin", "ik_reseed_limit_margin"):
        value = float(control.get(key, 0.0))
        if not math.isfinite(value) or not 0.0 < value < 0.5:
            raise ValueError(f"control.{key} must be in (0, 0.5)")
    for key in (
        "ik_limit_avoidance_gain",
        "ik_posture_gain",
        "ik_solution_limit_weight",
        "ik_min_progress_ratio",
        "ik_min_progress_absolute",
        "ik_reseed_escape_progress_ratio",
        "ik_reseed_escape_max_regression",
    ):
        value = float(control.get(key, -1.0))
        if not math.isfinite(value) or value < 0.0:
            raise ValueError(f"control.{key} must be finite and non-negative")
    for key in ("ik_home_posture_weight", "ik_candidate_home_weight"):
        if key in control:
            value = float(control[key])
            if not math.isfinite(value) or value < 0.0:
                raise ValueError(f"control.{key} must be finite and non-negative")
    if int(control.get("ik_iterations", 0)) <= 0:
        raise ValueError("control.ik_iterations must be positive")


def _validate_homing_and_urdf(config: dict[str, Any], config_path: Path) -> None:
    control = config["control"]
    home_threshold = float(control.get("home_gesture_threshold", 0.8))
    home_release = float(control.get("home_gesture_release_threshold", 0.35))
    if not 0.0 < home_threshold <= 1.0:
        raise ValueError("control.home_gesture_threshold must be in (0, 1]")
    if not 0.0 <= home_release < home_threshold:
        raise ValueError(
            "control.home_gesture_release_threshold must be non-negative "
            "and below home_gesture_threshold"
        )
    home_defaults = {
        "home_joint_velocity": 0.0,
        "home_joint_acceleration": 2.0 * float(control.get("home_joint_velocity", 0.0)),
        "home_tolerance": 0.0,
    }
    for key, default in home_defaults.items():
        value = float(control.get(key, default))
        if not math.isfinite(value) or value <= 0.0:
            raise ValueError(f"control.{key} must be finite and positive")
        control[key] = value
    available_homing_names = (
        [
            name
            for arm_config in config["arms"].values()
            for name in arm_config["joint_names"]
        ]
        + list(config["body"]["waist_joint_names"])
        + list(config["body"].get("neck_joint_names", []))
    )
    homing_names = control.get("home_joint_names", available_homing_names)
    if (
        not isinstance(homing_names, list)
        or not homing_names
        or len(set(homing_names)) != len(homing_names)
        or not all(
            isinstance(name, str) and name in available_homing_names
            for name in homing_names
        )
    ):
        raise ValueError(
            "control.home_joint_names must be a non-empty unique subset of controllable joints"
        )
    control["home_joint_names"] = homing_names
    missing_home = [
        name
        for name in homing_names
        if name not in config["robot"].get("initial_joints", {})
    ]
    if missing_home:
        raise ValueError(
            "robot.initial_joints lacks arm/waist home targets: "
            + ", ".join(missing_home)
        )
    urdf_path = Path(config["robot"]["urdf_path"]).expanduser()
    if not urdf_path.is_absolute():
        urdf_path = config_path.parent / urdf_path
    if not urdf_path.is_file():
        raise ValueError("robot.urdf_path does not exist")


def validate_teleop_config(
    config: Any,
    config_path: Path,
    *,
    external_ik: bool,
) -> None:
    if not isinstance(config, dict):
        raise ValueError("teleop config must be a YAML object")
    for section in ("robot", "control", "arms", "body", "grippers"):
        if section not in config:
            raise ValueError(f"missing config section: {section}")
    if set(config["arms"]) != {"left", "right"}:
        raise ValueError("arms must contain exactly left and right")
    control = config["control"]
    _validate_control_section(control)
    _validate_collision_section(control)
    _validate_body_section(config["body"])
    if not external_ik:
        _validate_legacy_ik_section(control)
    _validate_homing_and_urdf(config, config_path)


def joint_group(
    node: Any,
    names: list[str],
) -> tuple[list[int], list[int], np.ndarray, np.ndarray]:
    try:
        indices = [node.joint_by_name[name] for name in names]
    except KeyError as exc:
        raise ValueError(f"joint not found in URDF: {exc}") from exc
    lower, upper = [], []
    for index in indices:
        info = bullet.getJointInfo(
            node.robot_id, index, physicsClientId=node.physics_client
        )
        lower.append(float(info[8]))
        upper.append(float(info[9]))
    return (
        indices,
        [node.dof_by_joint[index] for index in indices],
        np.asarray(lower),
        np.asarray(upper),
    )


def create_arm_runtime(node: Any, side: str, config: dict[str, Any]) -> ArmRuntime:
    names = list(config["joint_names"])
    indices, dofs, lower, upper = node._joint_group(names)
    try:
        ee_index = node.link_by_name[config["ee_link"]]
        base_index = node.link_by_name[config["base_link"]]
    except KeyError as exc:
        raise ValueError(f"{side} link not found in URDF: {exc}") from exc
    return ArmRuntime(
        side, names, indices, dofs, lower, upper, ee_index, base_index
    )


def create_body_runtime(node: Any, config: dict[str, Any]) -> BodyRuntime:
    names = list(config["waist_joint_names"])
    indices, dofs, lower, upper = node._joint_group(names)
    if "joint_lower" in config:
        lower = np.asarray(config["joint_lower"], dtype=float)
    if "joint_upper" in config:
        upper = np.asarray(config["joint_upper"], dtype=float)
    target_link_name = config.get(
        "target_link",
        config.get("visualization_head_link", config.get("torso_link")),
    )
    target_link_index = node.link_by_name.get(
        target_link_name, node.link_by_name[config["torso_link"]]
    )
    return BodyRuntime(
        names,
        indices,
        dofs,
        lower,
        upper,
        node.link_by_name[config["torso_link"]],
        target_link_index=target_link_index,
    )


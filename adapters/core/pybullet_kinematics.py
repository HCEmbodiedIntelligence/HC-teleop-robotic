"""PyBullet forward kinematics, model synchronization, and legacy arm/waist IK."""

from __future__ import annotations

import math
from typing import Any

import numpy as np
import pybullet as bullet

from .arm_teleop_math import (
    adaptive_damping,
    clamp_step,
    joint_limit_avoidance,
    orientation_error,
)
from .teleop_state import ArmRuntime, IkCandidate


def reset_model_joint_states(node: Any) -> None:
    """Synchronize the same selected scalar joints in one PyBullet call."""
    selected = []
    for name, value in node.joint_state.items():
        index = node.joint_by_name.get(name)
        if index is not None and index in node.dof_by_joint:
            selected.append((index, value))
    if not selected:
        return
    bulk = getattr(bullet, "resetJointStatesMultiDof", None)
    if callable(bulk) and getattr(node, "_model_sync_bulk_supported", True) is not False:
        try:
            bulk(
                node.robot_id,
                [index for index, _value in selected],
                [[value] for _index, value in selected],
                targetVelocities=[[0.0]] * len(selected),
                physicsClientId=node.physics_client,
            )
            return
        except (TypeError, NotImplementedError, bullet.error):
            # Older/custom PyBullet builds may expose an unsupported batch API.
            # Reapply the entire original selection, then retain the scalar
            # path so an unsupported call never repeats on every control tick.
            node._model_sync_bulk_supported = False
    for index, value in selected:
        bullet.resetJointState(
            node.robot_id, index, value, physicsClientId=node.physics_client
        )


def reset_named_joints(
    node: Any,
    names: list[str] | tuple[str, ...] | Any,
    values_by_name: dict[str, float],
) -> None:
    """Reset a named set of joints in the PyBullet model to scalar values."""
    for name in names:
        index = node.joint_by_name.get(name)
        if index is not None:
            bullet.resetJointState(
                node.robot_id,
                index,
                float(values_by_name[name]),
                physicsClientId=node.physics_client,
            )


def set_group(node: Any, indices: list[int], values: np.ndarray) -> None:
    for index, value in zip(indices, values):
        bullet.resetJointState(
            node.robot_id,
            index,
            float(value),
            physicsClientId=node.physics_client,
        )


def root_pose(node: Any) -> tuple[np.ndarray, np.ndarray]:
    position, orientation = bullet.getBasePositionAndOrientation(
        node.robot_id, physicsClientId=node.physics_client
    )
    return np.asarray(position, dtype=float), np.asarray(orientation, dtype=float)


def link_pose(node: Any, index: int) -> tuple[np.ndarray, np.ndarray]:
    if index == -1:
        # PyBullet's base pose is the inertial frame; task frames refer to
        # the URDF root link, which may have a nonzero inertial offset.
        position, orientation = node._root_pose()
        dynamics = bullet.getDynamicsInfo(
            node.robot_id, -1, physicsClientId=node.physics_client
        )
        inverse = bullet.invertTransform(dynamics[3], dynamics[4])
        link = bullet.multiplyTransforms(position, orientation, *inverse)
        return np.asarray(link[0], dtype=float), np.asarray(link[1], dtype=float)
    state = bullet.getLinkState(
        node.robot_id,
        index,
        computeForwardKinematics=True,
        physicsClientId=node.physics_client,
    )
    return np.asarray(state[4], dtype=float), np.asarray(state[5], dtype=float)


def relative_pose(
    parent: tuple[np.ndarray, np.ndarray],
    child: tuple[np.ndarray, np.ndarray],
) -> tuple[np.ndarray, np.ndarray]:
    inverse = bullet.invertTransform(parent[0], parent[1])
    result = bullet.multiplyTransforms(inverse[0], inverse[1], child[0], child[1])
    return np.asarray(result[0]), np.asarray(result[1])


def world_pose(
    parent: tuple[np.ndarray, np.ndarray],
    local: tuple[np.ndarray, np.ndarray],
) -> tuple[np.ndarray, np.ndarray]:
    result = bullet.multiplyTransforms(parent[0], parent[1], local[0], local[1])
    return np.asarray(result[0]), np.asarray(result[1])


def all_dof_positions(node: Any) -> list[float]:
    return [
        float(
            bullet.getJointState(
                node.robot_id, index, physicsClientId=node.physics_client
            )[0]
        )
        for index in node.dof_joint_indices
    ]


def jacobian(node: Any, link_index: int) -> tuple[np.ndarray, np.ndarray]:
    value = bullet.calculateJacobian(
        node.robot_id,
        link_index,
        [0.0, 0.0, 0.0],
        node._all_dof_positions(),
        node.zeros,
        node.zeros,
        physicsClientId=node.physics_client,
    )
    return np.asarray(value[0], dtype=float), np.asarray(value[1], dtype=float)


def evaluate_arm_candidate(
    node: Any,
    arm: ArmRuntime,
    target_position: np.ndarray,
    target_orientation: np.ndarray,
    joints: np.ndarray,
    reference: np.ndarray,
    seed_name: str,
    damping: float,
) -> IkCandidate:
    q = np.clip(np.asarray(joints, dtype=float), arm.lower, arm.upper)
    node._set_group(arm.joint_indices, q)
    current_position, current_orientation = node._link_pose(arm.ee_index)
    position_error = float(np.linalg.norm(target_position - current_position))
    rotation_error = float(
        np.linalg.norm(orientation_error(target_orientation, current_orientation))
    )
    if not bool(node.control["orientation_enabled"]):
        rotation_error = 0.0
    position_tolerance = float(node.control["ik_position_tolerance"])
    orientation_tolerance = float(node.control["ik_orientation_tolerance"])
    span = arm.upper - arm.lower
    normalized_margin = np.minimum(
        (q - arm.lower) / span, (arm.upper - q) / span
    )
    minimum_margin = float(np.min(normalized_margin))
    transition = float(np.linalg.norm((q - reference) / span))
    preferred_margin = float(node.control["ik_reseed_limit_margin"])
    limit_cost = float(
        np.sum(np.maximum(preferred_margin - normalized_margin, 0.0) ** 2)
    )
    arm_home = np.asarray(
        [node.initial_joints.get(name, 0.0) for name in arm.joint_names],
        dtype=float,
    )
    home_distance = float(np.linalg.norm((q - arm_home) / span))
    candidate_home_weight = float(
        node.control.get("ik_candidate_home_weight", 0.5)
    )
    score = (
        transition
        + candidate_home_weight * home_distance
        + float(node.control.get("ik_solution_limit_weight", 4.0)) * limit_cost
    )
    return IkCandidate(
        q,
        position_error,
        rotation_error,
        position_error <= position_tolerance
        and rotation_error <= orientation_tolerance,
        position_error <= position_tolerance
        and rotation_error <= orientation_tolerance,
        minimum_margin,
        damping,
        seed_name,
        score,
    )


def solve_arm_seed(
    node: Any,
    arm: ArmRuntime,
    target_position: np.ndarray,
    target_orientation: np.ndarray,
    seed: np.ndarray,
    reference: np.ndarray,
    seed_name: str,
) -> IkCandidate:
    q = np.clip(np.asarray(seed, dtype=float), arm.lower, arm.upper)
    minimum_damping = float(node.control["ik_damping_min"])
    maximum_damping = float(node.control["ik_damping_max"])
    singularity_threshold = float(node.control["ik_singularity_threshold"])
    max_update = float(node.control["ik_max_update"])
    orientation_enabled = bool(node.control["orientation_enabled"])
    orientation_weight = float(node.control["orientation_weight"])
    position_tolerance = float(node.control["ik_position_tolerance"])
    orientation_tolerance = float(node.control["ik_orientation_tolerance"])
    limit_margin = float(node.control["ik_limit_margin"])
    limit_gain = float(node.control["ik_limit_avoidance_gain"])
    posture_gain = float(node.control["ik_posture_gain"])
    best = node._evaluate_arm_candidate(
        arm,
        target_position,
        target_orientation,
        q,
        reference,
        seed_name,
        minimum_damping,
    )
    largest_damping = minimum_damping
    for _ in range(int(node.control["ik_iterations"])):
        node._set_group(arm.joint_indices, q)
        current_position, current_orientation = node._link_pose(arm.ee_index)
        position_delta = target_position - current_position
        rotation_delta = orientation_error(target_orientation, current_orientation)
        position_norm = float(np.linalg.norm(position_delta))
        orientation_norm = (
            float(np.linalg.norm(rotation_delta)) if orientation_enabled else 0.0
        )
        if (
            position_norm <= position_tolerance
            and orientation_norm <= orientation_tolerance
        ):
            break
        error = np.concatenate(
            [
                position_delta,
                rotation_delta * orientation_weight
                if orientation_enabled
                else np.zeros(3),
            ]
        )
        linear, angular = node._jacobian(arm.ee_index)
        matrix = np.vstack(
            [
                linear[:, arm.dof_indices],
                angular[:, arm.dof_indices] * orientation_weight,
            ]
        )
        singular_values = np.linalg.svd(matrix, compute_uv=False)
        damping = adaptive_damping(
            float(np.min(singular_values)),
            minimum_damping,
            maximum_damping,
            singularity_threshold,
        )
        largest_damping = max(largest_damping, damping)
        regularized = matrix @ matrix.T + np.eye(6) * damping * damping
        solved_error = np.linalg.solve(regularized, error)
        pseudo_inverse = matrix.T @ np.linalg.solve(regularized, np.eye(6))
        update = matrix.T @ solved_error
        nullspace = np.eye(len(q)) - pseudo_inverse @ matrix
        avoidance = joint_limit_avoidance(q, arm.lower, arm.upper, limit_margin)
        arm_home = np.asarray(
            [node.initial_joints.get(name, 0.0) for name in arm.joint_names],
            dtype=float,
        )
        home_weight = float(node.control.get("ik_home_posture_weight", 0.8))
        posture_home = arm_home - q
        posture_prev = reference - q
        posture = posture_home * home_weight + posture_prev * (1.0 - home_weight)
        update += nullspace @ (avoidance * limit_gain + posture * posture_gain)
        norm = float(np.linalg.norm(update))
        if norm > max_update:
            update *= max_update / norm
        q = np.clip(q + update, arm.lower, arm.upper)
        candidate = node._evaluate_arm_candidate(
            arm,
            target_position,
            target_orientation,
            q,
            reference,
            seed_name,
            largest_damping,
        )
        candidate_error = (
            candidate.position_error / position_tolerance
            + candidate.orientation_error / orientation_tolerance
        )
        best_error = (
            best.position_error / position_tolerance
            + best.orientation_error / orientation_tolerance
        )
        if candidate.converged or candidate_error < best_error:
            best = candidate
        if candidate.converged:
            break
    return best


def solve_arm_ik(
    node: Any,
    arm: ArmRuntime,
    target_position: np.ndarray,
    target_orientation: np.ndarray,
    previous: np.ndarray,
) -> IkCandidate:
    reference = np.clip(np.asarray(previous, dtype=float), arm.lower, arm.upper)
    now = node._monotonic()
    if (
        arm.recovery_solution is None
        and arm.ik_rejections > 0
        and now - arm.last_ik_attempt < float(node.control["ik_failure_retry_interval"])
    ):
        held = node._evaluate_arm_candidate(
            arm,
            target_position,
            target_orientation,
            reference,
            reference,
            "hold_last_valid",
            max(arm.ik_damping, float(node.control["ik_damping_min"])),
        )
        held.converged = False
        held.accepted = False
        return held
    arm.last_ik_attempt = now
    primary = node._solve_arm_seed(
        arm,
        target_position,
        target_orientation,
        reference,
        reference,
        "previous",
    )
    preferred_margin = float(node.control["ik_reseed_limit_margin"])
    candidates = [primary]
    if arm.recovery_solution is not None:
        candidates.append(
            node._evaluate_arm_candidate(
                arm,
                target_position,
                target_orientation,
                arm.recovery_solution,
                reference,
                f"recovery_{arm.recovery_seed}",
                max(primary.damping, arm.ik_damping),
            )
        )
    needs_reseed = (
        not primary.converged or primary.minimum_limit_margin < preferred_margin
    )
    target_changed_since_reseed = (
        arm.reseed_target_position is None
        or arm.reseed_target_orientation is None
        or float(np.linalg.norm(target_position - arm.reseed_target_position))
        >= float(node.control["ik_reseed_target_position_delta"])
        or float(
            np.linalg.norm(
                orientation_error(target_orientation, arm.reseed_target_orientation)
            )
        )
        >= float(node.control["ik_reseed_target_orientation_delta"])
    )
    reseed_due = target_changed_since_reseed and now - arm.last_reseed_attempt >= float(
        node.control["ik_reseed_interval"]
    )
    if needs_reseed and reseed_due:
        arm.last_reseed_attempt = now
        arm.reseed_target_position = target_position.copy()
        arm.reseed_target_orientation = target_orientation.copy()
        feedback = np.asarray(
            [node.joint_state[name] for name in arm.joint_names], dtype=float
        )
        home = np.asarray(
            [node.initial_joints[name] for name in arm.joint_names], dtype=float
        )
        perturbation = float(node.control["ik_reseed_perturbation"])
        branch_positive = reference.copy()
        branch_negative = reference.copy()
        branch_positive[[0, 2, 3]] += np.asarray(
            [perturbation, -perturbation, 0.5 * perturbation]
        )
        branch_negative[[0, 2, 3]] -= np.asarray(
            [perturbation, -perturbation, 0.5 * perturbation]
        )
        seed_options = (
            ("feedback", feedback),
            ("home_blend", 0.5 * (reference + home)),
            ("home", home),
            ("branch_positive", branch_positive),
            ("branch_negative", branch_negative),
        )
        for seed_name, seed in seed_options:
            if any(
                np.linalg.norm(np.asarray(seed) - candidate.joints) < 1e-6
                for candidate in candidates
            ):
                continue
            candidates.append(
                node._solve_arm_seed(
                    arm,
                    target_position,
                    target_orientation,
                    seed,
                    reference,
                    seed_name,
                )
            )
    held = node._evaluate_arm_candidate(
        arm,
        target_position,
        target_orientation,
        reference,
        reference,
        "hold_last_valid",
        max(candidate.damping for candidate in candidates),
    )
    held.converged = bool(held.converged)

    max_step = float(node.control["max_joint_velocity"]) / float(
        node.control["rate_hz"]
    )
    position_tolerance = float(node.control["ik_position_tolerance"])
    orientation_tolerance = float(node.control["ik_orientation_tolerance"])

    def task_error(candidate: IkCandidate) -> float:
        return math.hypot(
            candidate.position_error / position_tolerance,
            candidate.orientation_error / orientation_tolerance,
        )

    held_error = task_error(held)
    minimum_ratio = float(node.control.get("ik_min_progress_ratio", 0.001))
    minimum_absolute = float(node.control.get("ik_min_progress_absolute", 0.0001))
    required_progress = max(minimum_absolute, held_error * minimum_ratio)
    safe_steps: list[tuple[IkCandidate, IkCandidate, bool]] = []
    for candidate in candidates:
        limited_joints = clamp_step(
            candidate.joints,
            reference,
            arm.lower,
            arm.upper,
            max_step,
        )
        step = limited_joints - reference
        step_norm = float(np.linalg.norm(step))
        max_norm_step = float(
            node.control.get(
                "max_joint_velocity_norm", node.control["max_joint_velocity"]
            )
        ) / float(node.control["rate_hz"])
        if step_norm > max_norm_step:
            limited_joints = reference + step * (max_norm_step / step_norm)
        limited = node._evaluate_arm_candidate(
            arm,
            target_position,
            target_orientation,
            limited_joints,
            reference,
            candidate.seed_name,
            candidate.damping,
        )
        progress = held_error - task_error(limited)
        candidate_progress = held_error - task_error(candidate)
        escape_progress = max(
            required_progress,
            held_error
            * float(node.control.get("ik_reseed_escape_progress_ratio", 0.05)),
        )
        escape_regression = float(
            node.control.get("ik_reseed_escape_max_regression", 0.25)
        )
        reseed_escape = bool(
            candidate.seed_name not in {"previous", "feedback"}
            and candidate_progress >= escape_progress
            and task_error(limited) <= held_error + escape_regression
        )
        limited.accepted = bool(
            limited.converged
            or (
                np.all(np.isfinite(limited.joints))
                and (progress >= required_progress or reseed_escape)
            )
        )
        if limited.accepted:
            limited.score = task_error(limited) + 1e-3 * candidate.score
            safe_steps.append((limited, candidate, reseed_escape))

    if safe_steps:

        def continuity_rank(item: tuple[IkCandidate, IkCandidate, bool]):
            seed_name = item[1].seed_name
            rank = (
                0
                if seed_name == "previous"
                else 1
                if seed_name.startswith("recovery_")
                else 2
            )
            return rank, item[0].score

        selected, full_candidate, _used_escape = min(safe_steps, key=continuity_rank)
        if full_candidate.seed_name == "previous":
            if selected.converged:
                arm.recovery_solution = None
                arm.recovery_seed = ""
        elif full_candidate.seed_name.startswith("recovery_"):
            if selected.converged:
                arm.recovery_solution = None
                arm.recovery_seed = ""
        elif not selected.converged:
            arm.recovery_solution = full_candidate.joints.copy()
            arm.recovery_seed = full_candidate.seed_name
        node._set_group(arm.joint_indices, selected.joints)
        return selected

    arm.recovery_solution = None
    arm.recovery_seed = ""
    held.converged = False
    held.accepted = False
    node._set_group(arm.joint_indices, reference)
    return held


def solve_waist(
    node: Any,
    target_position: np.ndarray,
    target_orientation: np.ndarray,
    seed: np.ndarray,
) -> np.ndarray:
    body = node.body
    q = np.clip(np.asarray(seed, dtype=float), body.lower, body.upper)
    q_home = np.array(
        [node.initial_joints.get(name, 0.0) for name in body.joint_names],
        dtype=float,
    )
    pitch_axis = np.asarray(node.body_config["pitch_axis_world"], dtype=float)
    pitch_axis /= np.linalg.norm(pitch_axis)
    damping = float(node.body_config["ik_damping"])
    posture_gain = float(node.body_config.get("ik_posture_gain", 0.5))
    # PyBullet Jacobians are expressed in the base frame; FK errors below
    # are world-frame quantities. Rotate both linear and angular rows.
    base_rotation = np.asarray(
        bullet.getMatrixFromQuaternion(node._root_pose()[1]), dtype=float
    ).reshape(3, 3)
    fwd_axis = base_rotation[:, 0]
    up_axis = np.array([0.0, 0.0, 1.0])
    ik_mode = str(
        node.body_config.get(
            "ik_mode",
            "2dof" if bool(node.body_config.get("unconstrained", False)) else "2dof",
        )
    )
    link_index = (
        body.target_link_index
        if body.target_link_index is not None
        else body.torso_index
    )

    for _ in range(int(node.body_config["ik_iterations"])):
        node._set_group(body.joint_indices, q)
        position, orientation = node._link_pose(link_index)
        rotation_delta = orientation_error(target_orientation, orientation)
        linear, angular = node._jacobian(link_index)
        linear_world = (base_rotation @ linear)[:, body.dof_indices]
        angular_world = (base_rotation @ angular)[:, body.dof_indices]

        if ik_mode == "3dof":
            fwd_error = float(fwd_axis @ (target_position - position))
            height_error = float(up_axis @ (target_position - position))
            pitch_error = float(pitch_axis @ rotation_delta)
            if (
                abs(fwd_error) < 0.002
                and abs(height_error) < 0.002
                and abs(pitch_error) < 0.002
            ):
                break
            fwd_weight = float(node.body_config.get("ik_forward_weight", 0.5))
            error = np.array(
                [fwd_error * fwd_weight, height_error, pitch_error], dtype=float
            )
            matrix = np.vstack(
                [
                    (fwd_axis @ linear_world).reshape(1, -1) * fwd_weight,
                    (up_axis @ linear_world).reshape(1, -1),
                    (pitch_axis @ angular_world).reshape(1, -1),
                ]
            )
            if posture_gain > 0.0:
                posture = q_home - q
                reg = matrix.T @ matrix + (damping * damping + posture_gain) * np.eye(3)
                update = np.linalg.solve(reg, matrix.T @ error + posture_gain * posture)
            else:
                update = matrix.T @ np.linalg.solve(
                    matrix @ matrix.T + np.eye(3) * damping * damping, error
                )
        elif ik_mode == "2dof":
            height_error = float(target_position[2] - position[2])
            pitch_error = float(pitch_axis @ rotation_delta)
            error = np.array([height_error, pitch_error], dtype=float)
            if abs(height_error) < 0.002 and abs(pitch_error) < 0.002:
                break
            matrix = np.vstack(
                [
                    linear_world[2:3, :],
                    (pitch_axis @ angular_world).reshape(1, -1),
                ]
            )
            reg = matrix @ matrix.T + np.eye(2) * damping * damping
            pseudo_inverse = matrix.T @ np.linalg.solve(reg, np.eye(2))
            update = pseudo_inverse @ error
            if posture_gain > 0.0:
                nullspace = np.eye(len(q)) - pseudo_inverse @ matrix
                posture = q_home - q
                update += nullspace @ (posture * posture_gain)
        else:
            error = np.r_[target_position - position, float(pitch_axis @ rotation_delta)]
            if np.linalg.norm(error) < 0.002:
                break
            matrix = np.vstack(
                [
                    linear_world,
                    (pitch_axis @ angular_world).reshape(1, -1),
                ]
            )
            update = matrix.T @ np.linalg.solve(
                matrix @ matrix.T + np.eye(4) * damping * damping, error
            )
        norm = float(np.linalg.norm(update))
        max_update = float(node.body_config["ik_max_update"])
        if norm > max_update:
            update *= max_update / norm
        q = np.clip(q + update, body.lower, body.upper)
    node._set_group(body.joint_indices, q)
    position, orientation = node._link_pose(link_index)
    body.forward_error = abs(float(fwd_axis @ (target_position - position)))
    body.lift_error = abs(float(target_position[2] - position[2]))
    body.pitch_error = abs(
        float(pitch_axis @ orientation_error(target_orientation, orientation))
    )
    return q


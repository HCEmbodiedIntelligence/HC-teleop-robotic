"""Weighted, box-constrained differential IK for controller_v2_3."""
from __future__ import annotations

from dataclasses import dataclass
from typing import Sequence

import numpy as np

try:
    from .tasks.joint_task import JointTask
except ImportError:
    try:
        from tasks.joint_task import JointTask
    except ImportError:
        JointTask = None


@dataclass
class SolveIKResult:
    velocity: np.ndarray
    residual_norm: float
    active_lower: np.ndarray
    active_upper: np.ndarray
    rows: int


def _stack_tasks(interface, q, tasks, targets):
    primary_matrix_rows, primary_target_rows = [], []
    secondary_matrix_rows, secondary_target_rows = [], []
    for task in tasks:
        linearized = task.linearize(interface, q, targets)
        if linearized is None:
            continue
        matrix, target = linearized.weighted_rows()
        keep = np.linalg.norm(matrix, axis=1) > 0.0
        if not np.any(keep):
            continue
        is_secondary = (
            (JointTask is not None and isinstance(task, JointTask))
            or getattr(task, "is_secondary", False)
        )
        if is_secondary:
            secondary_matrix_rows.append(matrix[keep])
            secondary_target_rows.append(target[keep])
        else:
            primary_matrix_rows.append(matrix[keep])
            primary_target_rows.append(target[keep])

    if not primary_matrix_rows:
        if not secondary_matrix_rows:
            return np.zeros((0, interface.nv_free)), np.zeros(0)
        return np.vstack(secondary_matrix_rows), np.concatenate(secondary_target_rows)

    j_primary = np.vstack(primary_matrix_rows)
    target_primary = np.concatenate(primary_target_rows)

    if not secondary_matrix_rows:
        return j_primary, target_primary

    # Project secondary posture tasks into the nullspace of primary tasks:
    # N = I - J_primary^+ @ J_primary
    j_secondary = np.vstack(secondary_matrix_rows)
    target_secondary = np.concatenate(secondary_target_rows)

    lam = 1e-3
    m_prim = j_primary.shape[0]
    reg = j_primary @ j_primary.T + (lam * lam) * np.eye(m_prim)
    try:
        j_pinv = j_primary.T @ np.linalg.solve(reg, np.eye(m_prim))
    except np.linalg.LinAlgError:
        j_pinv = j_primary.T @ np.linalg.lstsq(reg, np.eye(m_prim), rcond=None)[0]

    n_proj = np.eye(interface.nv_free) - j_pinv @ j_primary
    j_sec_proj = j_secondary @ n_proj

    v_des = np.zeros(interface.nv_free)
    for i in range(len(target_secondary)):
        row_norm = float(np.linalg.norm(j_secondary[i]))
        if row_norm > 1e-12:
            col = int(np.argmax(np.abs(j_secondary[i])))
            v_des[col] = float(target_secondary[i]) / row_norm
    target_sec_proj = j_secondary @ (n_proj @ v_des)

    return np.vstack([j_primary, j_sec_proj]), np.concatenate([target_primary, target_sec_proj])


def velocity_bounds(interface, q: np.ndarray, dt: float, velocity_limits=None):
    count = interface.nv_free
    lower = np.full(count, -np.inf)
    upper = np.full(count, np.inf)
    if velocity_limits is not None:
        limits = np.asarray(velocity_limits, dtype=float).reshape(-1)
        if limits.size != count:
            raise ValueError(f"velocity limit size {limits.size} != {count}")
        if np.any(limits < 0.0):
            raise ValueError("velocity limits must be non-negative")
        lower = np.maximum(lower, -limits)
        upper = np.minimum(upper, limits)
    q_lower, q_upper = interface.configuration_bounds_free()
    q_free = interface.free_configuration(q)
    if dt > 0.0:
        lower = np.maximum(
            lower, np.where(np.isfinite(q_lower), (q_lower - q_free) / dt, -np.inf)
        )
        upper = np.minimum(
            upper, np.where(np.isfinite(q_upper), (q_upper - q_free) / dt, np.inf)
        )
    if np.any(lower > upper):
        raise ValueError("inconsistent velocity/configuration bounds")
    return lower, upper


def _solve_box_lsq(matrix, target, damping, lower, upper, max_iter=None):
    """Active-set solution of min ||Av-b||²+damping||v||² with box bounds."""
    count = matrix.shape[1]
    hessian = matrix.T @ matrix + float(damping) * np.eye(count)
    gradient_target = matrix.T @ target
    active = np.zeros(count, dtype=np.int8)  # -1 lower, +1 upper
    velocity = np.zeros(count)
    max_iter = int(max_iter or (6 * count + 20))
    tolerance = 1e-10
    for _ in range(max_iter):
        free = active == 0
        fixed = ~free
        if np.any(free):
            right = gradient_target[free]
            if np.any(fixed):
                right -= hessian[np.ix_(free, fixed)] @ velocity[fixed]
            block = hessian[np.ix_(free, free)]
            try:
                velocity[free] = np.linalg.solve(block, right)
            except np.linalg.LinAlgError:
                velocity[free] = np.linalg.lstsq(block, right, rcond=None)[0]
        low_violation = np.where(free, lower - velocity, -np.inf)
        high_violation = np.where(free, velocity - upper, -np.inf)
        low_index = int(np.argmax(low_violation))
        high_index = int(np.argmax(high_violation))
        if low_violation[low_index] > tolerance or high_violation[high_index] > tolerance:
            if low_violation[low_index] >= high_violation[high_index]:
                active[low_index] = -1
                velocity[low_index] = lower[low_index]
            else:
                active[high_index] = 1
                velocity[high_index] = upper[high_index]
            continue
        gradient = hessian @ velocity - gradient_target
        release_lower = np.where(active == -1, -gradient, -np.inf)
        release_upper = np.where(active == 1, gradient, -np.inf)
        lower_index = int(np.argmax(release_lower))
        upper_index = int(np.argmax(release_upper))
        if release_lower[lower_index] > tolerance or release_upper[upper_index] > tolerance:
            release = (
                lower_index
                if release_lower[lower_index] >= release_upper[upper_index]
                else upper_index
            )
            active[release] = 0
            continue
        break
    return np.minimum(np.maximum(velocity, lower), upper)


def _compute_adaptive_damping(matrix: np.ndarray, base_damping: float) -> float:
    base = max(float(base_damping), 1.5e-3)
    if matrix.shape[0] == 0:
        return base
    try:
        s = np.linalg.svd(matrix, compute_uv=False)
        s_min = float(np.min(s)) if s.size > 0 else 0.0
        sigma_thresh = 0.08
        if s_min < sigma_thresh:
            ratio = s_min / sigma_thresh
            adaptive_extra = 0.05 * ((1.0 - ratio) ** 2)
            return float(base + adaptive_extra)
    except Exception:
        pass
    return base


def solve_ik(
    interface, q: np.ndarray, tasks: Sequence, targets, dt: float,
    velocity_limits=None, damping: float = 1e-5,
) -> SolveIKResult:
    q = np.asarray(q, dtype=float)
    matrix, target = _stack_tasks(interface, q, tasks, targets)
    lower, upper = velocity_bounds(interface, q, float(dt), velocity_limits)
    if matrix.shape[0] == 0:
        velocity = np.zeros(interface.nv_free)
        residual = 0.0
    else:
        effective_damping = _compute_adaptive_damping(matrix, damping)
        velocity = _solve_box_lsq(
            matrix, target, effective_damping, lower, upper
        )
        residual = float(np.linalg.norm(matrix @ velocity - target))
    tolerance = 1e-9
    return SolveIKResult(
        velocity=velocity,
        residual_norm=residual,
        active_lower=np.isfinite(lower) & np.isclose(velocity, lower, atol=tolerance),
        active_upper=np.isfinite(upper) & np.isclose(velocity, upper, atol=tolerance),
        rows=matrix.shape[0],
    )

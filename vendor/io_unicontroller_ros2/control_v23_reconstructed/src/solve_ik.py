"""Weighted, box-constrained differential IK for controller_v2_3."""
from __future__ import annotations

from dataclasses import dataclass
from typing import Sequence

import numpy as np


@dataclass
class SolveIKResult:
    velocity: np.ndarray
    residual_norm: float
    active_lower: np.ndarray
    active_upper: np.ndarray
    rows: int


def _stack_tasks(interface, q, tasks, targets):
    matrix_rows, target_rows = [], []
    for task in tasks:
        linearized = task.linearize(interface, q, targets)
        if linearized is None:
            continue
        matrix, target = linearized.weighted_rows()
        keep = np.linalg.norm(matrix, axis=1) > 0.0
        if np.any(keep):
            matrix_rows.append(matrix[keep])
            target_rows.append(target[keep])
    if not matrix_rows:
        return np.zeros((0, interface.nv_free)), np.zeros(0)
    return np.vstack(matrix_rows), np.concatenate(target_rows)


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
        velocity = _solve_box_lsq(
            matrix, target, damping, lower, upper
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

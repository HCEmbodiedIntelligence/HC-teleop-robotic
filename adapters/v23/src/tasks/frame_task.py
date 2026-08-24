"""Relative frame pose task."""
from __future__ import annotations

import numpy as np
import pinocchio as pin

from .task import LinearizedTask, Task


class FrameTask(Task):
    def __init__(
        self,
        root: str,
        frame: str,
        position_cost: float,
        orientation_cost: float,
        gain: float = 1.0,
    ):
        super().__init__(gain)
        self.root = root
        self.frame = frame
        self.position_cost = float(position_cost)
        self.orientation_cost = float(orientation_cost)

    def linearize(self, interface, q: np.ndarray, targets) -> LinearizedTask | None:
        target = targets.relative_target(self.root, self.frame)
        if target is None:
            return None
        current = interface.relative_pose(self.root, self.frame, q)
        jacobian = interface.relative_jacobian(self.root, self.frame, q)
        position_error = np.asarray(
            target.translation - current.translation, dtype=float
        )
        rotation_error_local = pin.log3(current.rotation.T @ target.rotation)
        rotation_error = current.rotation @ np.asarray(
            rotation_error_local, dtype=float
        )
        error = self.gain * np.concatenate([position_error, rotation_error])
        weight = np.array(
            [self.position_cost] * 3 + [self.orientation_cost] * 3, dtype=float
        )
        return LinearizedTask(
            jacobian, weight=weight, error=error,
            name=f"frame:{self.root}->{self.frame}"
        )

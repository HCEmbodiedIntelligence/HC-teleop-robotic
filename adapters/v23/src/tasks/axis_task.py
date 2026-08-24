"""Orientation-axis alignment task used by retargeting configurations."""
from __future__ import annotations

import numpy as np

from .task import LinearizedTask, Task

_AXIS = {"X": 0, "Y": 1, "Z": 2}


class AxisTask(Task):
    def __init__(
        self, root: str, frame: str, axis: str, cost: float, gain: float = 1.0
    ):
        super().__init__(gain)
        axis = str(axis).upper()
        if axis not in _AXIS:
            raise ValueError(f"axis must be X/Y/Z, got {axis!r}")
        self.root = root
        self.frame = frame
        self.axis = axis
        self.cost = float(cost)

    def linearize(self, interface, q: np.ndarray, targets) -> LinearizedTask | None:
        target = targets.relative_target(self.root, self.frame)
        if target is None:
            return None
        current = interface.relative_pose(self.root, self.frame, q)
        jacobian = interface.relative_jacobian(self.root, self.frame, q)
        index = _AXIS[self.axis]
        current_axis = np.asarray(current.rotation[:, index], dtype=float)
        target_axis = np.asarray(target.rotation[:, index], dtype=float)
        error = self.gain * np.cross(current_axis, target_axis)
        return LinearizedTask(
            jacobian[3:, :], error, np.full(3, self.cost, dtype=float),
            f"axis:{self.root}->{self.frame}:{self.axis}"
        )

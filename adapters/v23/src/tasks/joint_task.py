"""Joint posture task retained for vendor YAML compatibility."""
from __future__ import annotations

from typing import Sequence

import numpy as np

from .task import LinearizedTask, Task


class JointTask(Task):
    def __init__(
        self, joint_names: Sequence[str], cost: float, gain: float = 1.0,
        target=None
    ):
        super().__init__(gain)
        self.joint_names = list(joint_names)
        self.cost = float(cost)
        self.target = (
            None if target is None else np.asarray(target, dtype=float).reshape(-1)
        )

    def set_target(self, values: Sequence[float]) -> None:
        values = np.asarray(values, dtype=float).reshape(-1)
        if values.size != len(self.joint_names):
            raise ValueError("joint target size mismatch")
        self.target = values

    def linearize(self, interface, q: np.ndarray, targets) -> LinearizedTask | None:
        if self.target is None:
            return None
        name_to_column = {
            name: index for index, name in enumerate(interface.free_joints)
        }
        rows, errors = [], []
        for name, desired in zip(self.joint_names, self.target):
            if name not in name_to_column:
                continue
            column = name_to_column[name]
            row = np.zeros(interface.nv_free)
            row[column] = 1.0
            current = q[interface.joints[column].q_index]
            rows.append(row)
            errors.append(self.gain * (float(desired) - float(current)))
        if not rows:
            return None
        error = np.asarray(errors, dtype=float)
        return LinearizedTask(
            np.vstack(rows), error, np.full(error.size, self.cost), "joint-posture"
        )

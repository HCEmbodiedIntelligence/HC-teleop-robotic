"""Common task primitives for the reconstructed differential IK."""
from __future__ import annotations

from abc import ABC, abstractmethod
from dataclasses import dataclass
from typing import Protocol

import numpy as np


class TargetProvider(Protocol):
    def relative_target(self, root: str, frame: str): ...


@dataclass
class LinearizedTask:
    jacobian: np.ndarray
    error: np.ndarray
    weight: np.ndarray
    name: str = "task"

    def weighted_rows(self) -> tuple[np.ndarray, np.ndarray]:
        weights = np.sqrt(np.asarray(self.weight, dtype=float).reshape(-1))
        if self.jacobian.shape[0] != weights.size or self.error.size != weights.size:
            raise ValueError(f"{self.name}: inconsistent task dimensions")
        return self.jacobian * weights[:, None], self.error.reshape(-1) * weights


class Task(ABC):
    def __init__(self, gain: float = 1.0):
        self.gain = float(gain)

    @abstractmethod
    def linearize(
        self, interface, q: np.ndarray, targets: TargetProvider
    ) -> LinearizedTask | None:
        raise NotImplementedError

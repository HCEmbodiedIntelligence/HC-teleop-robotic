from __future__ import annotations

from dataclasses import dataclass

import numpy as np
from sensor_msgs.msg import Joy


@dataclass
class ArmRuntime:
    name: str
    joint_names: list[str]
    joint_indices: list[int]
    dof_indices: list[int]
    lower: np.ndarray
    upper: np.ndarray
    ee_index: int
    base_index: int
    pose: tuple[np.ndarray, np.ndarray] | None = None
    pose_stamp: float = 0.0
    joy: Joy | None = None
    joy_stamp: float = 0.0
    active: bool = False
    reference_vr: tuple[np.ndarray, np.ndarray] | None = None
    reference_local_ee: tuple[np.ndarray, np.ndarray] | None = None
    target_local: tuple[np.ndarray, np.ndarray] | None = None
    target_world: tuple[np.ndarray, np.ndarray] | None = None
    last_solution: np.ndarray | None = None
    position_error: float = 0.0
    orientation_error: float = 0.0
    ik_converged: bool = True
    ik_within_tolerance: bool = True
    ik_rejections: int = 0
    ik_rejection_total: int = 0
    ik_seed: str = "feedback"
    ik_damping: float = 0.0
    last_ik_log: float = 0.0
    last_ik_attempt: float = 0.0
    last_reseed_attempt: float = 0.0
    recovery_solution: np.ndarray | None = None
    recovery_seed: str = ""
    reseed_target_position: np.ndarray | None = None
    reseed_target_orientation: np.ndarray | None = None


@dataclass
class IkCandidate:
    joints: np.ndarray
    position_error: float
    orientation_error: float
    converged: bool
    accepted: bool
    minimum_limit_margin: float
    damping: float
    seed_name: str
    score: float


@dataclass
class BodyRuntime:
    joint_names: list[str]
    joint_indices: list[int]
    dof_indices: list[int]
    lower: np.ndarray
    upper: np.ndarray
    torso_index: int
    head_pose: tuple[np.ndarray, np.ndarray] | None = None
    head_stamp: float = 0.0
    active: bool = False
    reference_head: tuple[np.ndarray, np.ndarray] | None = None
    reference_torso: tuple[np.ndarray, np.ndarray] | None = None
    target_base: tuple[np.ndarray, np.ndarray] | None = None
    last_solution: np.ndarray | None = None
    lift_error: float = 0.0
    pitch_error: float = 0.0

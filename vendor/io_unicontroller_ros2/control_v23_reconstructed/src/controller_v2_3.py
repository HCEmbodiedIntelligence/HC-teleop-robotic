"""Behavioral reconstruction of controller_v2_3 with modular task/IK stack."""
from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Dict, Mapping, Sequence, Tuple

import numpy as np
import pinocchio as pin
import yaml

try:
    from .pinocchio_interface_v3 import PinocchioInterfaceV3
    from .solve_ik import solve_ik
    from .tasks import AxisTask, FrameTask, JointTask
except ImportError:
    from pinocchio_interface_v3 import PinocchioInterfaceV3
    from solve_ik import solve_ik
    from tasks import AxisTask, FrameTask, JointTask


@dataclass
class TargetTransform:
    translation: np.ndarray
    rotation: np.ndarray

    @classmethod
    def identity(cls):
        return cls(np.zeros(3), np.eye(3))

    def as_se3(self) -> pin.SE3:
        return pin.SE3(
            np.asarray(self.rotation, dtype=float),
            np.asarray(self.translation, dtype=float),
        )


class TargetStore:
    def __init__(self):
        self.transforms: Dict[Tuple[str, str], TargetTransform] = {}

    def update(self, values: Mapping[Tuple[str, str], TargetTransform]):
        self.transforms.update(values)

    def relative_target(self, root: str, frame: str):
        direct = self.transforms.get((root, frame))
        if direct is not None:
            return direct.as_se3()
        root_transform = self.transforms.get(("", root)) or self.transforms.get(
            ("world", root)
        )
        child_transform = self.transforms.get(("", frame)) or self.transforms.get(
            ("world", frame)
        )
        if root_transform is None or child_transform is None:
            return None
        return root_transform.as_se3().inverse() * child_transform.as_se3()


class ControllerV23:
    def __init__(self, config_path: str | Path, *, damping: float | None = None):
        self.config_path = Path(config_path).resolve()
        with self.config_path.open("r", encoding="utf-8") as stream:
            self.cfg = yaml.safe_load(stream)
        urdf = self.config_path.parent / self.cfg["model"]["urdf"]
        free_joints = list(self.cfg["model"].get("free_joints", []))
        if not free_joints:
            raise ValueError("model.free_joints must not be empty")
        self.interface = PinocchioInterfaceV3(urdf, free_joints)
        self.free_joint_names = free_joints
        self.q = self.interface.copy_q()
        self.command_q = self.q.copy()
        self.have_feedback = False
        self.targets = TargetStore()
        control = self.cfg.get("control", {})
        self.dt = float(control.get("dt", 0.0025))
        if self.dt <= 0.0:
            raise ValueError("control.dt must be positive")
        self.method = str(control.get("method", "control"))
        if self.method not in {"control", "retarget"}:
            raise ValueError("control.method must be control or retarget")
        configured_damping = control.get("damping", 1e-5)
        self.damping = float(configured_damping if damping is None else damping)
        if self.damping < 0.0:
            raise ValueError("control.damping must be non-negative")
        self.tasks = self._build_tasks(self.cfg.get("task", {}))
        self.pose_tasks = [task for task in self.tasks if isinstance(task, FrameTask)]
        self.axis_tasks = [task for task in self.tasks if isinstance(task, AxisTask)]
        self.joint_tasks = [task for task in self.tasks if isinstance(task, JointTask)]
        raw_limits = np.asarray(
            self.cfg.get("limit", {}).get("velocity", []), dtype=float
        )
        if raw_limits.size == 0:
            raw_limits = np.full(len(free_joints), np.inf)
        if raw_limits.size != len(free_joints):
            raise ValueError("velocity limit count must match free_joints")
        self.velocity_limits = np.deg2rad(raw_limits)
        self.last_result = None

    @staticmethod
    def _build_tasks(config):
        tasks = []
        for item in config.get("pose", []) or []:
            (root, frame), position_cost, orientation_cost, *rest = item
            tasks.append(
                FrameTask(
                    root, frame, position_cost, orientation_cost,
                    rest[0] if rest else 1.0,
                )
            )
        for item in config.get("axis", []) or []:
            (root, frame), axis, cost, *rest = item
            tasks.append(
                AxisTask(root, frame, axis, cost, rest[0] if rest else 1.0)
            )
        for item in config.get("joint", []) or []:
            names, cost, *rest = item
            tasks.append(JointTask(names, cost, rest[0] if rest else 1.0))
        return tasks

    def update_joint_state(
        self, names: Sequence[str], positions: Sequence[float]
    ) -> None:
        self.q = self.interface.update_named_joints(names, positions)
        if not self.have_feedback:
            self.command_q = self.q.copy()
            self.have_feedback = True
        elif self.method == "retarget":
            # If physical feedback differs significantly from command_q (e.g. during homing, e-stop, or clutch release),
            # sync command_q to feedback to prevent open-loop drift and singular IK postures.
            diff = np.linalg.norm(
                self.interface.free_configuration(self.command_q)
                - self.interface.free_configuration(self.q)
            )
            if diff > 0.15:
                self.command_q = self.q.copy()

    def update_tf_targets(
        self, transforms: Mapping[Tuple[str, str], TargetTransform]
    ) -> None:
        self.targets.update(transforms)

    def step(self, dt: float | None = None):
        if not self.have_feedback:
            raise RuntimeError("joint feedback has not been received")
        step_dt = self.dt if dt is None else float(dt)
        if step_dt <= 0.0:
            raise ValueError("step dt must be positive")
        base_q = self.q if self.method == "control" else self.command_q
        result = solve_ik(
            self.interface,
            base_q,
            self.tasks,
            self.targets,
            step_dt,
            velocity_limits=self.velocity_limits,
            damping=self.damping,
        )
        next_q = self.interface.integrate_free(base_q, result.velocity, step_dt)
        next_q = self.interface.clip_configuration(next_q)
        if not np.all(np.isfinite(next_q)):
            raise FloatingPointError("IK produced a non-finite configuration")
        self.command_q = next_q
        self.last_result = result
        return (
            self.free_joint_names.copy(),
            self.interface.free_configuration(next_q),
        )

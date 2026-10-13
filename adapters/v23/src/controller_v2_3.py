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

        # Resolve initial / home joints from config or peer arm_teleop.yaml
        joints_init_raw = control.get("joints_init")
        self.joints_init: dict[str, float] = {}
        if isinstance(joints_init_raw, dict):
            self.joints_init = {str(k): float(v) for k, v in joints_init_raw.items()}
        elif isinstance(joints_init_raw, (list, tuple)) and len(joints_init_raw) == len(free_joints):
            self.joints_init = {name: float(v) for name, v in zip(free_joints, joints_init_raw)}

        if not self.joints_init:
            arm_teleop_file = self.config_path.parent / "arm_teleop.yaml"
            if arm_teleop_file.exists():
                try:
                    with arm_teleop_file.open("r", encoding="utf-8") as stream:
                        arm_cfg = yaml.safe_load(stream)
                    initial_map = arm_cfg.get("robot", {}).get("initial_joints", {})
                    if isinstance(initial_map, dict):
                        self.joints_init = {
                            name: float(initial_map[name])
                            for name in free_joints
                            if name in initial_map
                        }
                except Exception:
                    pass

        self.tasks = self._build_tasks(
            self.cfg.get("task", {}),
            default_posture=self.joints_init,
            control_cfg=control,
            free_joints=free_joints,
        )
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

    @classmethod
    def _build_tasks(
        cls,
        config,
        default_posture=None,
        control_cfg=None,
        free_joints=None,
    ):
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
        joint_items = config.get("joint", []) or []
        for item in joint_items:
            names, cost, *rest = item
            gain = rest[0] if len(rest) > 0 else 1.0
            target = rest[1] if len(rest) > 1 else None
            if target is None and default_posture:
                if isinstance(default_posture, dict):
                    resolved = [default_posture.get(n) for n in names]
                    if all(v is not None for v in resolved):
                        target = resolved
                elif (
                    isinstance(default_posture, (list, tuple, np.ndarray))
                    and free_joints
                    and len(default_posture) == len(free_joints)
                ):
                    name_to_val = dict(zip(free_joints, default_posture))
                    resolved = [name_to_val.get(n) for n in names]
                    if all(v is not None for v in resolved):
                        target = resolved
            tasks.append(JointTask(names, cost, gain, target=target))

        if not joint_items and control_cfg and free_joints and default_posture:
            posture_cost = float(control_cfg.get("posture_cost", 0.0))
            if posture_cost > 0.0:
                posture_gain = float(control_cfg.get("posture_gain", 1.0))
                if isinstance(default_posture, dict):
                    resolved = [default_posture.get(n) for n in free_joints]
                    if all(v is not None for v in resolved):
                        tasks.append(
                            JointTask(
                                free_joints, posture_cost, posture_gain, target=resolved
                            )
                        )

        return tasks

    def update_joint_state(
        self, names: Sequence[str], positions: Sequence[float]
    ) -> None:
        self.q = self.interface.update_named_joints(names, positions)
        if not self.have_feedback:
            self.command_q = self.q.copy()
            self.have_feedback = True
        # Feedback lag must not hard-reset the retarget integrator: that would
        # bypass step()'s velocity limits and produce a discontinuous command.
        # Reinitialization is explicit via reset_to_feedback() and the ROS
        # reset acknowledgement / clutch rearm handshake.

    def update_tf_targets(
        self, transforms: Mapping[Tuple[str, str], TargetTransform]
    ) -> None:
        self.targets.update(transforms)

    def reset_to_feedback(self) -> None:
        """Discard an unsafe retarget integrator state and restart from feedback."""
        if not self.have_feedback:
            return
        self.command_q = self.q.copy()

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

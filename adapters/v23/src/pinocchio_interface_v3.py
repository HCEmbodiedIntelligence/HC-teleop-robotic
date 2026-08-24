"""Pinocchio model interface for the controller_v2_3 reconstruction."""
from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Sequence

import numpy as np
import pinocchio as pin


@dataclass(frozen=True)
class JointSlice:
    name: str
    joint_id: int
    q_index: int
    v_index: int


class PinocchioInterfaceV3:
    def __init__(self, urdf_path: str | Path, free_joints: Sequence[str]):
        self.urdf_path = Path(urdf_path).resolve()
        self.model = pin.buildModelFromUrdf(str(self.urdf_path))
        self.data = self.model.createData()
        self.q = pin.neutral(self.model)
        self.free_joints = list(free_joints)
        self.joints: list[JointSlice] = []
        for name in self.free_joints:
            joint_id = self.model.getJointId(name)
            if joint_id == 0:
                raise ValueError(f"joint not found in URDF: {name}")
            joint = self.model.joints[joint_id]
            if joint.nq != 1 or joint.nv != 1:
                raise ValueError(
                    f"expected scalar joint {name}; got nq={joint.nq}, nv={joint.nv}"
                )
            self.joints.append(
                JointSlice(name, joint_id, joint.idx_q, joint.idx_v)
            )
        self.q_indices = np.asarray(
            [joint.q_index for joint in self.joints], dtype=int
        )
        self.v_indices = np.asarray(
            [joint.v_index for joint in self.joints], dtype=int
        )
        self._name_to_joint = {joint.name: joint for joint in self.joints}
        self.forward(self.q)

    @property
    def nv_free(self) -> int:
        return len(self.joints)

    def copy_q(self) -> np.ndarray:
        return self.q.copy()

    def set_q(self, q: Sequence[float]) -> None:
        values = np.asarray(q, dtype=float)
        if values.shape != (self.model.nq,):
            raise ValueError(f"q shape must be {(self.model.nq,)}, got {values.shape}")
        self.q[:] = values
        self.forward(self.q)

    def update_named_joints(
        self, names: Sequence[str], positions: Sequence[float]
    ) -> np.ndarray:
        if len(names) != len(positions):
            raise ValueError("joint name/position size mismatch")
        for name, value in zip(names, positions):
            joint = self._name_to_joint.get(name)
            if joint is not None:
                value = float(value)
                if not np.isfinite(value):
                    raise ValueError(f"non-finite feedback for {name}")
                self.q[joint.q_index] = value
        self.forward(self.q)
        return self.q.copy()

    def free_configuration(self, q: np.ndarray | None = None) -> np.ndarray:
        source = self.q if q is None else q
        return np.asarray(source)[self.q_indices].copy()

    def inject_free_configuration(
        self, q_free: Sequence[float], base_q: np.ndarray | None = None
    ) -> np.ndarray:
        values = np.asarray(q_free, dtype=float)
        if values.shape != (self.nv_free,):
            raise ValueError(
                f"expected {self.nv_free} free joint values, got {values.shape}"
            )
        output = (self.q if base_q is None else base_q).copy()
        output[self.q_indices] = values
        return output

    def forward(self, q: np.ndarray | None = None) -> None:
        source = self.q if q is None else q
        pin.forwardKinematics(self.model, self.data, source)
        pin.updateFramePlacements(self.model, self.data)

    def frame_id(self, name: str) -> int:
        frame_id = self.model.getFrameId(name)
        if frame_id >= len(self.model.frames):
            raise ValueError(f"frame not found in URDF: {name}")
        return frame_id

    def frame_pose(self, frame: str, q: np.ndarray | None = None) -> pin.SE3:
        if q is not None:
            self.forward(q)
        return self.data.oMf[self.frame_id(frame)].copy()

    def relative_pose(
        self, root: str, child: str, q: np.ndarray | None = None
    ) -> pin.SE3:
        if q is not None:
            self.forward(q)
        world_root = self.data.oMf[self.frame_id(root)]
        world_child = self.data.oMf[self.frame_id(child)]
        return world_root.inverse() * world_child

    def relative_jacobian(
        self, root: str, child: str, q: np.ndarray | None = None
    ) -> np.ndarray:
        """Return [linear; angular] relative velocity in root coordinates.

        The archive's WORLD-Jacobian subtraction omitted both the change of
        origin and the rotating-root term. This formulation was checked against
        finite differences on all HC-TJ free joints (worst error < 4e-8).
        """
        source = self.q if q is None else q
        self.forward(source)
        root_id = self.frame_id(root)
        child_id = self.frame_id(child)
        world_root = self.data.oMf[root_id]
        root_child = world_root.inverse() * self.data.oMf[child_id]
        root_jacobian = pin.computeFrameJacobian(
            self.model, self.data, source, root_id,
            pin.ReferenceFrame.LOCAL_WORLD_ALIGNED,
        )
        child_jacobian = pin.computeFrameJacobian(
            self.model, self.data, source, child_id,
            pin.ReferenceFrame.LOCAL_WORLD_ALIGNED,
        )
        world_to_root = world_root.rotation.T
        root_angular = world_to_root @ root_jacobian[3:, :]
        linear = (
            world_to_root @ (child_jacobian[:3, :] - root_jacobian[:3, :])
            + pin.skew(root_child.translation) @ root_angular
        )
        angular = world_to_root @ (
            child_jacobian[3:, :] - root_jacobian[3:, :]
        )
        return np.asarray(
            np.vstack([linear, angular])[:, self.v_indices], dtype=float
        )

    def integrate_free(
        self, q: np.ndarray, v_free: Sequence[float], dt: float
    ) -> np.ndarray:
        velocity = np.zeros(self.model.nv)
        velocity[self.v_indices] = np.asarray(v_free, dtype=float)
        return pin.integrate(self.model, q, float(dt) * velocity)

    def configuration_bounds_free(self) -> tuple[np.ndarray, np.ndarray]:
        return (
            np.asarray(self.model.lowerPositionLimit[self.q_indices], dtype=float).copy(),
            np.asarray(self.model.upperPositionLimit[self.q_indices], dtype=float).copy(),
        )

    def clip_configuration(self, q: np.ndarray) -> np.ndarray:
        output = np.asarray(q, dtype=float).copy()
        lower, upper = self.configuration_bounds_free()
        values = output[self.q_indices]
        values = np.where(np.isfinite(lower), np.maximum(values, lower), values)
        values = np.where(np.isfinite(upper), np.minimum(values, upper), values)
        output[self.q_indices] = values
        return output

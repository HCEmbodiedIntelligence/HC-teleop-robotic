"""Validated URDF kinematics and bounded damped-least-squares IK.

This implementation uses model joint names; no PyBullet array index is part of
the public contract. Collision checking is a separate configured capability.
"""

from dataclasses import dataclass
from pathlib import Path
import math
import time
import xml.etree.ElementTree as ET

import numpy as np


def rotation(axis, angle):
    axis = np.asarray(axis, dtype=float)
    axis = axis / np.linalg.norm(axis)
    x, y, z = axis
    skew = np.array([[0, -z, y], [z, 0, -x], [-y, x, 0]])
    return np.eye(3) + math.sin(angle) * skew + (1 - math.cos(angle)) * (skew @ skew)


def quaternion_matrix(quaternion):
    q = np.asarray(quaternion, dtype=float)
    if q.shape != (4,) or not np.isfinite(q).all() or np.linalg.norm(q) < 1e-9:
        raise ValueError("invalid quaternion")
    x, y, z, w = q / np.linalg.norm(q)
    return np.array(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
        ]
    )


def matrix_quaternion(matrix):
    """Stable xyzw conversion, including rotations close to pi."""
    r = np.asarray(matrix, dtype=float)
    candidates = np.maximum(
        0,
        np.array(
            [
                1 + r[0, 0] - r[1, 1] - r[2, 2],
                1 - r[0, 0] + r[1, 1] - r[2, 2],
                1 - r[0, 0] - r[1, 1] + r[2, 2],
                1 + np.trace(r),
            ]
        ),
    )
    i = int(np.argmax(candidates))
    value = math.sqrt(candidates[i]) * 0.5
    q = np.zeros(4)
    q[i] = value
    divisor = max(4 * value, 1e-12)
    if i == 3:
        q[:3] = [
            (r[2, 1] - r[1, 2]) / divisor,
            (r[0, 2] - r[2, 0]) / divisor,
            (r[1, 0] - r[0, 1]) / divisor,
        ]
    else:
        j, k = (i + 1) % 3, (i + 2) % 3
        q[j] = (r[j, i] + r[i, j]) / divisor
        q[k] = (r[k, i] + r[i, k]) / divisor
        q[3] = (r[k, j] - r[j, k]) / divisor
    return q / np.linalg.norm(q)


def rotation_vector(matrix):
    q = matrix_quaternion(matrix)
    if q[3] < 0:
        q = -q
    size = np.linalg.norm(q[:3])
    return 2 * q[:3] if size < 1e-10 else q[:3] * (2 * math.atan2(size, q[3]) / size)


def pose_matrix(values):
    if len(values) != 7 or not np.isfinite(values).all():
        raise ValueError("pose must contain seven finite xyz/xyzw values")
    result = np.eye(4)
    result[:3, :3] = quaternion_matrix(values[3:])
    result[:3, 3] = values[:3]
    return result


def _vector(text, default):
    values = np.array(
        [float(x) for x in text.split()] if text else default, dtype=float
    )
    if values.shape != (3,) or not np.isfinite(values).all():
        raise ValueError("URDF vector must contain three finite values")
    return values


@dataclass(frozen=True)
class Joint:
    name: str
    kind: str
    parent: str
    child: str
    origin: np.ndarray
    axis: np.ndarray
    lower: float
    upper: float
    velocity: float
    effort: float
    mimic: object = None


@dataclass(frozen=True)
class IkResult:
    positions: dict
    converged: bool
    position_error: float
    orientation_error: float
    iterations: int
    elapsed_ms: float
    reason: str


class UrdfModel:
    def __init__(self, path, package_root=None, backend="auto"):
        self.path = Path(path).resolve()
        self.package_root = (
            Path(package_root).resolve() if package_root else self.path.parent.parent
        )
        raw = self.path.read_bytes()
        if (
            len(raw) > 16 * 1024**2
            or b"<!DOCTYPE" in raw.upper()
            or b"<!ENTITY" in raw.upper()
        ):
            raise ValueError("unsafe or oversized URDF")
        root = ET.fromstring(raw)
        if root.tag != "robot":
            raise ValueError("URDF root must be robot")
        self.name = root.get("name", "robot")
        self.links = tuple(x.get("name") for x in root.findall("link"))
        if (
            not self.links
            or None in self.links
            or len(set(self.links)) != len(self.links)
        ):
            raise ValueError("missing or duplicate link name")
        self.meshes = []
        for mesh in root.iter("mesh"):
            uri = mesh.get("filename", "")
            if "://" in uri:
                raise ValueError("URDF meshes must be resolved package-local paths")
            resolved = (self.path.parent / uri).resolve()
            if not resolved.is_relative_to(self.package_root) or not resolved.is_file():
                raise ValueError("missing or escaping mesh: " + uri)
            if "scale" in mesh.attrib:
                scale = _vector(mesh.get("scale"), [1, 1, 1])
                if not np.all(np.isfinite(scale)) or np.any(np.abs(scale) < 1e-12):
                    raise ValueError("mesh scale must be nonzero")
            self.meshes.append(resolved)
        self.joints = {}
        self.parents = {}
        for node in root.findall("joint"):
            name, kind = node.get("name"), node.get("type")
            if (
                not name
                or name in self.joints
                or kind not in ("fixed", "revolute", "continuous", "prismatic")
            ):
                raise ValueError(
                    "duplicate/missing joint name or unsupported joint type"
                )
            parent, child = node.find("parent"), node.find("child")
            if parent is None or child is None:
                raise ValueError("joint missing parent or child")
            parent, child = parent.get("link"), child.get("link")
            if (
                parent not in self.links
                or child not in self.links
                or child in self.parents
            ):
                raise ValueError("invalid joint link tree")
            origin = np.eye(4)
            element = node.find("origin")
            if element is not None:
                origin[:3, 3] = _vector(element.get("xyz"), [0, 0, 0])
                roll, pitch, yaw = _vector(element.get("rpy"), [0, 0, 0])
                origin[:3, :3] = (
                    rotation([0, 0, 1], yaw)
                    @ rotation([0, 1, 0], pitch)
                    @ rotation([1, 0, 0], roll)
                )
            element = node.find("axis")
            axis = _vector(None if element is None else element.get("xyz"), [1, 0, 0])
            if not np.all(np.isfinite(axis)):
                raise ValueError("joint axis must be finite")
            norm = float(np.linalg.norm(axis))
            if norm < 1e-9:
                if kind != "fixed":
                    raise ValueError("joint axis must be nonzero")
                axis = np.array([1.0, 0.0, 0.0], dtype=float)
            else:
                axis = axis / norm
            limit = node.find("limit")
            if kind in ("revolute", "prismatic") and limit is None:
                raise ValueError("bounded joint requires limits")
            lower = (
                float(limit.get("lower", "-inf")) if limit is not None else -math.inf
            )
            upper = float(limit.get("upper", "inf")) if limit is not None else math.inf
            velocity = float(limit.get("velocity", "1")) if limit is not None else 1.0
            effort = float(limit.get("effort", "100")) if limit is not None else 100.0
            if kind in ("revolute", "prismatic") and (
                not math.isfinite(lower + upper) or lower > upper
            ):
                raise ValueError("invalid joint limits")
            if (
                not math.isfinite(velocity)
                or velocity < 0
                or not math.isfinite(effort)
                or effort < 0
            ):
                raise ValueError("invalid velocity/effort")
            if kind == "continuous":
                lower, upper = -math.inf, math.inf
            m = node.find("mimic")
            mimic = (
                (
                    m.get("joint"),
                    float(m.get("multiplier", "1")),
                    float(m.get("offset", "0")),
                )
                if m is not None
                else None
            )
            if mimic and (not math.isfinite(mimic[1]) or not math.isfinite(mimic[2])):
                raise ValueError("invalid mimic parameters")
            self.joints[name] = Joint(
                name,
                kind,
                parent,
                child,
                origin,
                axis,
                lower,
                upper,
                velocity,
                effort,
                mimic,
            )
            self.parents[child] = name
        roots = set(self.links) - set(self.parents)
        if len(roots) != 1:
            raise ValueError("URDF must contain one connected root")
        self.root = roots.pop()
        self.order = []
        seen = {self.root}
        while len(self.order) < len(self.joints):
            progress = False
            for name, joint in self.joints.items():
                if name not in self.order and joint.parent in seen:
                    if joint.child in seen:
                        raise ValueError("URDF cycle")
                    self.order.append(name)
                    seen.add(joint.child)
                    progress = True
            if not progress:
                raise ValueError("URDF cycle or disconnected tree")
        self.joint_names = tuple(
            n
            for n in self.order
            if self.joints[n].kind != "fixed" and not self.joints[n].mimic
        )
        for joint in self.joints.values():
            if joint.mimic:
                self._value(joint.name, {}, set())
        if backend not in ("auto", "native", "python"):
            raise ValueError("kinematics backend must be auto, native, or python")
        self.native = None
        self.backend = "python_reference"
        self.backend_reason = None
        if backend != "python":
            from .native import NativeModel, NativeUnavailable

            try:
                self.native = NativeModel(self)
                self.backend = "eigen_native"
            except NativeUnavailable as error:
                self.backend_reason = str(error)
                if backend == "native":
                    raise

    def _value(self, name, positions, visiting):
        if name in visiting:
            raise ValueError("mimic joint cycle")
        if name not in self.joints:
            raise ValueError("unknown mimic joint")
        joint = self.joints[name]
        if joint.mimic:
            other, multiplier, offset = joint.mimic
            return (
                self._value(other, positions, visiting | {name}) * multiplier + offset
            )
        return float(positions.get(name, 0.0))

    def frames(self, positions):
        if any(name not in self.joints for name in positions) or not all(
            math.isfinite(float(x)) for x in positions.values()
        ):
            raise ValueError("unknown joint or non-finite position")
        frames = {self.root: np.eye(4)}
        joint_frames = {}
        for name in self.order:
            joint = self.joints[name]
            frame = frames[joint.parent] @ joint.origin
            joint_frames[name] = frame
            motion = np.eye(4)
            if joint.kind in ("revolute", "continuous"):
                motion[:3, :3] = rotation(
                    joint.axis, self._value(name, positions, set())
                )
            elif joint.kind == "prismatic":
                motion[:3, 3] = joint.axis * self._value(name, positions, set())
            frames[joint.child] = frame @ motion
        return frames, joint_frames

    def fk(self, positions, tip, base=None):
        if self.native is not None:
            return self.native.fk(positions, tip, base)
        frames, _ = self.frames(positions)
        if tip not in frames or (base and base not in frames):
            raise ValueError("unknown kinematic frame")
        return np.linalg.inv(frames[base]) @ frames[tip] if base else frames[tip].copy()

    def chain(self, tip, base=None):
        result = []
        while tip != (base or self.root):
            if tip not in self.parents:
                raise ValueError("base is not an ancestor of tip")
            name = self.parents[tip]
            result.append(name)
            tip = self.joints[name].parent
        return tuple(reversed(result))

    def jacobian(self, positions, tip, joint_names, base=None):
        if self.native is not None:
            return self.native.jacobian(positions, tip, joint_names, base)
        frames, joint_frames = self.frames(positions)
        chain = set(self.chain(tip, base))
        target = frames[tip][:3, 3]
        output = np.zeros((6, len(joint_names)))
        rotate = frames[base][:3, :3].T if base else np.eye(3)
        for i, name in enumerate(joint_names):
            if name not in chain or self.joints[name].mimic:
                raise ValueError(
                    "IK joints must be independent joints in the base-tip chain"
                )
            joint = self.joints[name]
            axis = joint_frames[name][:3, :3] @ joint.axis
            if joint.kind in ("revolute", "continuous"):
                output[:3, i] = rotate @ np.cross(
                    axis, target - joint_frames[name][:3, 3]
                )
                output[3:, i] = rotate @ axis
            elif joint.kind == "prismatic":
                output[:3, i] = rotate @ axis
        return output

    def ik(
        self,
        target,
        seed,
        joint_names,
        tip,
        base=None,
        *,
        max_iterations=20,
        timeout_ms=30.0,
        damping=0.03,
        max_step=0.12,
        position_tolerance=0.005,
        orientation_tolerance=0.04,
        orientation_weight=0.35,
    ):
        if self.native is not None:
            positions, converged, stats = self.native.ik(
                target,
                seed,
                joint_names,
                tip,
                base,
                max_iterations=max_iterations,
                timeout_ms=timeout_ms,
                damping=damping,
                max_step=max_step,
                position_tolerance=position_tolerance,
                orientation_tolerance=orientation_tolerance,
                orientation_weight=orientation_weight,
            )
            return IkResult(
                positions,
                converged,
                float(stats[0]),
                float(stats[1]),
                int(stats[2]),
                float(stats[3]),
                {0: "CONVERGED", 1: "ITERATION_LIMIT", 2: "DEADLINE"}[int(stats[4])],
            )
        started = time.monotonic_ns()
        target = np.asarray(target, dtype=float)
        if target.shape != (4, 4) or not np.isfinite(target).all():
            raise ValueError("IK target must be finite 4x4")
        if (
            not 1 <= max_iterations <= 200
            or timeout_ms <= 0
            or damping <= 0
            or max_step <= 0
        ):
            raise ValueError("invalid IK budget")
        q = dict(seed)
        for name in joint_names:
            if name not in q:
                raise ValueError("IK requires feedback for every active joint")
        weights = np.array(
            [1, 1, 1, orientation_weight, orientation_weight, orientation_weight]
        )
        error = np.zeros(6)
        reason, converged = "ITERATION_LIMIT", False
        for iteration in range(max_iterations + 1):
            current = self.fk(q, tip, base)
            error[:3] = target[:3, 3] - current[:3, 3]
            error[3:] = rotation_vector(target[:3, :3] @ current[:3, :3].T)
            if np.linalg.norm(error[:3]) <= position_tolerance and (
                orientation_weight == 0
                or np.linalg.norm(error[3:]) <= orientation_tolerance
            ):
                converged, reason = True, "CONVERGED"
                break
            if iteration == max_iterations:
                break
            if (time.monotonic_ns() - started) / 1e6 >= timeout_ms:
                reason = "DEADLINE"
                break
            jacobian = self.jacobian(q, tip, joint_names, base) * weights[:, None]
            delta = jacobian.T @ np.linalg.solve(
                jacobian @ jacobian.T + damping * damping * np.eye(6), error * weights
            )
            delta *= min(1.0, max_step / max(np.max(np.abs(delta)), 1e-12))
            for name, step in zip(joint_names, delta):
                joint = self.joints[name]
                q[name] = float(np.clip(q[name] + step, joint.lower, joint.upper))
        return IkResult(
            q,
            converged,
            float(np.linalg.norm(error[:3])),
            float(np.linalg.norm(error[3:])),
            iteration,
            (time.monotonic_ns() - started) / 1e6,
            reason,
        )


def relative_target(
    reference_ee,
    current_pose,
    reference_pose,
    mapping,
    position_scale=1.0,
    rotation_scale=1.0,
    max_displacement=0.9,
):
    """Map Unity relative motion; reflection belongs in basis conjugation."""
    current, reference = pose_matrix(current_pose), pose_matrix(reference_pose)
    mapping = np.asarray(mapping, dtype=float)
    if mapping.shape != (3, 3) or not np.allclose(
        mapping @ mapping.T, np.eye(3), atol=1e-6
    ):
        raise ValueError("axis_mapping must be orthogonal")
    target = np.array(reference_ee, copy=True)
    offset = mapping @ (current[:3, 3] - reference[:3, 3]) * position_scale
    distance = np.linalg.norm(offset)
    if distance > max_displacement:
        offset *= max_displacement / distance
    target[:3, 3] += offset
    delta = mapping @ (current[:3, :3] @ reference[:3, :3].T) @ mapping.T
    rotvec = rotation_vector(delta) * rotation_scale
    if np.linalg.norm(rotvec) > 1e-12:
        delta = rotation(rotvec, np.linalg.norm(rotvec))
    else:
        delta = np.eye(3)
    target[:3, :3] = delta @ reference_ee[:3, :3]
    return target

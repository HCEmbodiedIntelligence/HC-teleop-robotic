"""ctypes boundary to the native Eigen kinematics worker library (ABI v1)."""

import ctypes as C
import os
from pathlib import Path

import numpy as np


class NativeUnavailable(RuntimeError):
    pass


def _double(array):
    return array.ctypes.data_as(C.POINTER(C.c_double))


def _integer(array):
    return array.ctypes.data_as(C.POINTER(C.c_int))


class NativeModel:
    def __init__(self, model):
        path = Path(
            os.environ.get(
                "HC_KINEMATICS_LIBRARY",
                str(
                    Path(__file__).resolve().parents[3]
                    / "kinematics/build/libhc_kinematics.so"
                ),
            )
        )
        if not path.is_file():
            raise NativeUnavailable(
                "Build platform/kinematics or set HC_KINEMATICS_LIBRARY"
            )
        try:
            self.lib = C.CDLL(str(path))
        except OSError as error:
            raise NativeUnavailable(str(error)) from error
        self.lib.hc_kinematics_abi_version.restype = C.c_int
        if self.lib.hc_kinematics_abi_version() != 1:
            raise NativeUnavailable("Unsupported native kinematics ABI")
        ptr = C.c_void_p
        ip = C.POINTER(C.c_int)
        dp = C.POINTER(C.c_double)
        self.lib.hc_model_create.argtypes = [
            C.c_int,
            C.c_int,
            C.c_int,
            ip,
            ip,
            ip,
            dp,
            dp,
            dp,
            dp,
        ]
        self.lib.hc_model_create.restype = ptr
        self.lib.hc_model_destroy.argtypes = [ptr]
        self.lib.hc_fk.argtypes = [ptr, dp, C.c_int, C.c_int, dp]
        self.lib.hc_fk.restype = C.c_int
        self.lib.hc_jacobian.argtypes = [ptr, dp, C.c_int, C.c_int, ip, C.c_int, dp]
        self.lib.hc_jacobian.restype = C.c_int
        self.lib.hc_ik.argtypes = (
            [ptr, dp, C.c_int, C.c_int, ip, C.c_int, dp, C.c_int]
            + [C.c_double] * 6
            + [dp, dp]
        )
        self.lib.hc_ik.restype = C.c_int
        self.model = model
        self.names = list(model.order)
        self.indices = {name: index for index, name in enumerate(self.names)}
        self.links = {name: index for index, name in enumerate(model.links)}
        joints = [model.joints[name] for name in self.names]
        parents = np.array([self.links[j.parent] for j in joints], dtype=np.int32)
        children = np.array([self.links[j.child] for j in joints], dtype=np.int32)
        kinds = np.array(
            [
                {"fixed": 0, "revolute": 1, "continuous": 1, "prismatic": 2}[j.kind]
                for j in joints
            ],
            dtype=np.int32,
        )
        origins = np.ascontiguousarray([j.origin for j in joints], dtype=np.float64)
        axes = np.ascontiguousarray([j.axis for j in joints], dtype=np.float64)
        lower = np.array([j.lower for j in joints], dtype=np.float64)
        upper = np.array([j.upper for j in joints], dtype=np.float64)
        self.handle = self.lib.hc_model_create(
            len(joints),
            len(model.links),
            self.links[model.root],
            _integer(parents),
            _integer(children),
            _integer(kinds),
            _double(origins),
            _double(axes),
            _double(lower),
            _double(upper),
        )
        if not self.handle:
            raise ValueError("native model rejected invalid topology or capacity")

    def close(self):
        if getattr(self, "handle", None):
            self.lib.hc_model_destroy(self.handle)
            self.handle = None

    def __del__(self):
        self.close()

    def _q(self, positions):
        if any(name not in self.model.joints for name in positions):
            raise ValueError("unknown joint")
        q = np.array(
            [self.model._value(name, positions, set()) for name in self.names],
            dtype=np.float64,
        )
        if not np.isfinite(q).all():
            raise ValueError("nonfinite joint position")
        return q

    def _frames(self, tip, base):
        if tip not in self.links or (base is not None and base not in self.links):
            raise ValueError("unknown kinematic frame")
        return self.links[tip], -1 if base is None else self.links[base]

    def _active(self, names):
        if any(
            name not in self.indices or self.model.joints[name].mimic for name in names
        ):
            raise ValueError("invalid active joint")
        return np.array([self.indices[name] for name in names], dtype=np.int32)

    def fk(self, positions, tip, base):
        q = self._q(positions)
        tip, base = self._frames(tip, base)
        result = np.empty((4, 4), dtype=np.float64)
        if self.lib.hc_fk(self.handle, _double(q), tip, base, _double(result)) != 0:
            raise ValueError("native FK rejected request")
        return result

    def jacobian(self, positions, tip, names, base):
        q = self._q(positions)
        tip, base = self._frames(tip, base)
        active = self._active(names)
        result = np.empty((6, len(names)), dtype=np.float64)
        if (
            self.lib.hc_jacobian(
                self.handle,
                _double(q),
                tip,
                base,
                _integer(active),
                len(names),
                _double(result),
            )
            != 0
        ):
            raise ValueError("native Jacobian rejected request")
        return result

    def ik(self, target, seed, names, tip, base, **settings):
        q = self._q(seed)
        if any(name not in seed for name in names):
            raise ValueError("IK requires feedback for every active joint")
        tip, base = self._frames(tip, base)
        active = self._active(names)
        target = np.ascontiguousarray(target, dtype=np.float64)
        if target.shape != (4, 4) or not np.isfinite(target).all():
            raise ValueError("IK target must be finite 4x4")
        result = np.empty_like(q)
        stats = np.empty(5, dtype=np.float64)
        code = self.lib.hc_ik(
            self.handle,
            _double(q),
            tip,
            base,
            _integer(active),
            len(names),
            _double(target),
            int(settings["max_iterations"]),
            *[
                float(settings[key])
                for key in (
                    "timeout_ms",
                    "damping",
                    "max_step",
                    "position_tolerance",
                    "orientation_tolerance",
                    "orientation_weight",
                )
            ],
            _double(result),
            _double(stats),
        )
        if code < 0:
            raise ValueError("native IK rejected request")
        positions = dict(seed)
        positions.update({name: float(result[self.indices[name]]) for name in names})
        return positions, bool(code), stats

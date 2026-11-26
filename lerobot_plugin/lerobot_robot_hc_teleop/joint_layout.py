"""Translate the training coordinate system using the discovered HC contract."""

from dataclasses import dataclass
from math import isfinite


@dataclass(frozen=True)
class Axis:
    public: str
    scale: float
    offset: float


def resolve_axes(names, units, joints):
    lookup = {}
    for joint in joints:
        public = joint["public_name"]
        model = joint.get("model_name", public)
        public_axis = Axis(public, 1.0, 0.0)
        variants = {public: public_axis}
        if (
            public in names
            and model == public
            and (joint.get("scale", 1.0) != 1.0 or joint.get("offset", 0.0) != 0.0)
        ):
            raise ValueError(f"Ambiguous training coordinates for {public}")
        if model and model in names and model != public:
            scale, offset = float(joint["scale"]), float(joint["offset"])
            if not isfinite(scale) or scale == 0 or not isfinite(offset):
                raise ValueError(f"Invalid servo_j transform for {public}")
            variants[model] = Axis(public, scale, offset)
        for name, axis in variants.items():
            if name in lookup:
                raise ValueError(f"Ambiguous servo_j axis name {name}")
            lookup[name] = axis, joint["unit"]
    if isinstance(units, str):
        units = [units] * len(names)
    if units is not None and len(units) != len(names):
        raise ValueError("Training units do not match the joint layout")
    aliases = {
        "radian": "rad",
        "radians": "rad",
        "meter": "m",
        "meters": "m",
        "metre": "m",
        "metres": "m",
        "unitless": "1",
        "dimensionless": "1",
    }
    result = {}
    for i, name in enumerate(names):
        if name not in lookup:
            raise ValueError(
                f"Training joint {name} is absent from HC servo_j capabilities"
            )
        axis, unit = lookup[name]
        if units is not None and aliases.get(units[i], units[i]) != aliases.get(
            unit, unit
        ):
            raise ValueError(
                f"Training unit {units[i]} for {name} differs from servo_j unit {unit}"
            )
        result[name] = axis
    if len({axis.public for axis in result.values()}) != len(result):
        raise ValueError("Training layout names the same servo_j axis twice")
    return result


def select_groups(groups, requested, public_names):
    needed = set(public_names)
    if requested:
        return requested
    available = {
        name: set(spec["joint_names"])
        for name, spec in groups.items()
        if spec["endpoints"].get("servo_j")
    }
    arms = [
        name
        for name in ("left_arm", "right_arm")
        if available.get(name, set()) & needed
    ]
    if arms and needed <= set().union(*(available[name] for name in arms)):
        return arms
    for name, joints in sorted(available.items(), key=lambda entry: len(entry[1])):
        if needed <= joints:
            return [name]
    candidates = [
        g
        for g in ("whole_body", "left_hand", "right_hand")
        if available.get(g, set()) & needed
    ]
    if candidates and needed <= set().union(*(available[g] for g in candidates)):
        return candidates
    raise ValueError(
        "Specify disjoint servo_j groups covering all training action axes"
    )

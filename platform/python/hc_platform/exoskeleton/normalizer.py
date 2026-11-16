"""Name/unit normalization and sample freshness, independent of ROS imports."""

import math
import xml.etree.ElementTree as ET
from pathlib import Path


class ExoskeletonInputError(ValueError):
    pass


class ExoskeletonNormalizer:
    def __init__(self, config):
        self.config = config
        self.options = config.get("exoskeleton", {})
        self.names = list(config["joint_names"])
        self.index = {name: i for i, name in enumerate(self.names)}
        self.mapping = self.options.get("joint_mapping", {})
        if not isinstance(self.mapping, dict):
            raise ExoskeletonInputError("joint_mapping must be an object")
        self.max_age_ns = int(float(self.options.get("max_sample_age_ms", 100)) * 1e6)
        self.future_ns = int(float(self.options.get("future_tolerance_ms", 5)) * 1e6)
        self.ttl_ms = float(self.options.get("ttl_ms", 100))
        if (
            not 0 < self.max_age_ns <= 500_000_000
            or not 0 <= self.future_ns <= 50_000_000
            or not 10 <= self.ttl_ms <= 500
        ):
            raise ExoskeletonInputError("invalid exoskeleton timing limits")
        self.units = {name: "rad" for name in self.names}
        urdf = config.get("urdf", config.get("urdf_path"))
        if urdf:
            raw = Path(urdf).read_bytes()
            if (
                len(raw) > 16 * 1024 * 1024
                or b"<!DOCTYPE" in raw.upper()
                or b"<!ENTITY" in raw.upper()
            ):
                raise ExoskeletonInputError("unsafe URDF")
            for joint in ET.fromstring(raw).findall("joint"):
                if joint.get("name") in self.units and joint.get("type") == "prismatic":
                    self.units[joint.get("name")] = "m"
        self.last_stamp_ns = 0
        self.sequence = 0

    def normalize(self, names, positions, stamp_ns, ros_now_ns, mono_now_ns):
        names = list(names)
        positions = list(positions)
        if (
            not names
            or len(names) > 64
            or len(names) != len(positions)
            or len(set(names)) != len(names)
        ):
            raise ExoskeletonInputError("INVALID_JOINT_ARRAYS")
        if (
            not isinstance(stamp_ns, int)
            or stamp_ns <= 0
            or stamp_ns <= self.last_stamp_ns
        ):
            raise ExoskeletonInputError("NONINCREASING_OR_ZERO_HEADER_STAMP")
        age = ros_now_ns - stamp_ns
        if age > self.max_age_ns:
            raise ExoskeletonInputError("EXOSKELETON_SAMPLE_STALE")
        if age < -self.future_ns:
            raise ExoskeletonInputError("EXOSKELETON_SAMPLE_FUTURE")
        result_names = []
        result_positions = []
        for external, value in zip(names, positions):
            item = self.mapping.get(external, external)
            if isinstance(item, str):
                item = {"name": item}
            if not isinstance(item, dict) or item.get("name") not in self.index:
                raise ExoskeletonInputError("UNKNOWN_JOINT")
            name = item["name"]
            if name in result_names:
                raise ExoskeletonInputError("DUPLICATE_MAPPED_JOINT")
            target_unit = self.units[name]
            source_unit = item.get("unit", target_unit)
            factors = {
                "rad": ("rad", 1.0),
                "deg": ("rad", math.pi / 180),
                "m": ("m", 1.0),
                "mm": ("m", 0.001),
            }
            if source_unit not in factors or factors[source_unit][0] != target_unit:
                raise ExoskeletonInputError("JOINT_UNIT_DIMENSION_MISMATCH")
            value = float(value) * factors[source_unit][1] * float(
                item.get("scale", 1)
            ) + float(item.get("offset", 0))
            if not math.isfinite(value):
                raise ExoskeletonInputError("NONFINITE_JOINT")
            index = self.index[name]
            if not self.config["lower"][index] <= value <= self.config["upper"][index]:
                raise ExoskeletonInputError("JOINT_LIMIT")
            result_names.append(name)
            result_positions.append(value)
        self.last_stamp_ns = stamp_ns
        self.sequence += 1
        sample_ns = mono_now_ns - max(0, age)
        return {
            "joint_names": result_names,
            "positions": result_positions,
            "seq": self.sequence,
            "sample_mono_ns": sample_ns,
            "received_mono_ns": mono_now_ns,
            "ttl_ms": self.ttl_ms,
            "deadline_mono_ns": sample_ns + int(self.ttl_ms * 1e6),
            "ros_stamp_ns": stamp_ns,
        }

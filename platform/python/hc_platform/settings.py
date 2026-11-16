"""Typed run-time overrides. Applying them always rebuilds a stopped generation."""

from __future__ import annotations
import hashlib
import json
import math

from .configuration import ConfigurationError, finite
from .topics import TopicRegistry


EDITABLE = {
    "control_backend",
    "kinematics_backend",
    "control_hz",
    "feedback_timeout_ms",
    "command_timeout_ms",
    "lease_timeout_ms",
    "handover_timeout_ms",
    "velocity",
    "acceleration",
    "home",
    "teleop",
    "hardware",
    "camera",
    "motion",
    "exoskeleton",
}


def merge(base, patch):
    result = json.loads(json.dumps(base))
    for key, value in patch.items():
        result[key] = (
            merge(result[key], value)
            if isinstance(value, dict) and isinstance(result.get(key), dict)
            else value
        )
    return result


def apply_overrides(config, overrides):
    if not isinstance(overrides, dict) or set(overrides) - EDITABLE:
        raise ConfigurationError("Unknown runtime configuration field")
    if len(json.dumps(overrides, allow_nan=False)) > 1024 * 1024:
        raise ConfigurationError("Runtime configuration exceeds 1 MiB")
    if "teleop" in overrides and set(overrides["teleop"]) - {
        "control",
        "body",
        "camera_metadata",
    }:
        raise ConfigurationError(
            "Joint topology and calibration belong in a new robot package"
        )
    if "hardware" in overrides and set(overrides["hardware"]) - {
        "contract",
        "allowed_groups",
        "feedback_timeout_s",
        "ready_timeout_s",
    }:
        raise ConfigurationError("Unknown hardware setting")
    config = merge(config, overrides)
    if config.get("control_backend") not in ("v23", "motion"):
        raise ConfigurationError("control_backend must be v23 or motion")
    if config.get("kinematics_backend", "native") not in ("native", "python"):
        raise ConfigurationError("kinematics_backend must be native or python")
    for key, low, high in (
        ("control_hz", 20, 1000),
        ("feedback_timeout_ms", 20, 2000),
        ("command_timeout_ms", 10, 1000),
        ("lease_timeout_ms", 50, 5000),
        ("handover_timeout_ms", 50, 5000),
    ):
        value = finite(config[key], key)
        if not low <= value <= high:
            raise ConfigurationError(f"{key} must be between {low} and {high}")
    count = len(config["joint_names"])
    for key in ("velocity", "acceleration", "home"):
        values = config[key]
        if not isinstance(values, list) or len(values) != count:
            raise ConfigurationError(f"{key} must match joint_names")
        for index, value in enumerate(values):
            value = finite(value, key)
            if key == "home":
                if not config["lower"][index] <= value <= config["upper"][index]:
                    raise ConfigurationError("Home outside joint limits")
            elif not 0 < value <= 100:
                raise ConfigurationError(f"Invalid {key}")
    config["initial_positions"] = dict(zip(config["joint_names"], config["home"]))
    control = config["teleop"].get("control", {})
    body = config["teleop"].get("body", {})
    config["home_velocity"] = finite(
        control.get("home_joint_velocity", 0.5), "home velocity"
    )
    config["home_acceleration"] = finite(
        control.get("home_joint_acceleration", 1.0), "home acceleration"
    )
    config["home_tolerance"] = finite(
        control.get("home_tolerance", 0.02), "home tolerance"
    )
    config["home_joint_names"] = control.get("home_joint_names", config["joint_names"])
    if (
        not 0 < config["home_velocity"] <= 100
        or not 0 < config["home_acceleration"] <= 100
        or not 0 < config["home_tolerance"] <= 0.5
    ):
        raise ConfigurationError("Invalid home motion limits")
    if not isinstance(config["home_joint_names"], list) or set(
        config["home_joint_names"]
    ) - set(config["joint_names"]):
        raise ConfigurationError("Unknown home joints")
    body_names = body.get("waist_joint_names", [])
    for key, combine in (("joint_lower", max), ("joint_upper", min)):
        values = body.get(key, [])
        if values and len(values) != len(body_names):
            raise ConfigurationError("Body limit array differs from waist_joint_names")
        for name, value in zip(body_names, values):
            if name not in config["joint_names"]:
                continue
            index = config["joint_names"].index(name)
            field = "lower" if key == "joint_lower" else "upper"
            config[field][index] = combine(config[field][index], finite(value, key))
    for index, name in enumerate(config["joint_names"]):
        if config["velocity"][index] > config["model_velocity_limits"][index]:
            raise ConfigurationError("Joint speed exceeds the URDF hardware limit")
        if name in body_names:
            config["velocity"][index] = min(
                config["velocity"][index],
                finite(
                    body.get("max_joint_velocity", config["velocity"][index]),
                    "body velocity",
                ),
            )
        if (
            not config["lower"][index]
            <= config["home"][index]
            <= config["upper"][index]
        ):
            raise ConfigurationError("Home outside the configured body/joint limits")
    if config.get("exoskeleton", {}).get("enabled"):
        from .exoskeleton import ExoskeletonNormalizer

        try:
            ExoskeletonNormalizer(config)
        except ValueError as exc:
            raise ConfigurationError(str(exc)) from exc
    # These values are consumed in another process; reject bad values before stopping the old generation.
    for section in (control, body, body.get("head_pose_base", {})):
        for key, value in section.items():
            if isinstance(value, (int, float)) and not isinstance(value, bool):
                finite(value, key)
                if (key.endswith("_alpha") and not 0 < value <= 1) or (
                    "deadzone" in key
                    and not key.endswith("_deg")
                    and not 0 <= value < 1
                ):
                    raise ConfigurationError(f"Invalid {key}")
                if (
                    "timeout" in key
                    or key.endswith("velocity")
                    or key.endswith("_scale")
                ) and value < 0:
                    raise ConfigurationError(f"Negative {key}")
    for key in ("axis_mapping", "head_axis_mapping"):
        matrix = control.get(key)
        if matrix is not None:
            if (
                not isinstance(matrix, list)
                or len(matrix) != 3
                or any(len(row) != 3 for row in matrix)
            ):
                raise ConfigurationError(f"{key} must be 3x3")
            for i in range(3):
                for j in range(3):
                    if (
                        abs(
                            sum(
                                finite(matrix[i][k], key) * finite(matrix[j][k], key)
                                for k in range(3)
                            )
                            - (1 if i == j else 0)
                        )
                        > 1e-5
                    ):
                        raise ConfigurationError(f"{key} must be orthonormal")
    if body.get("neck_tracking_enabled"):
        neck = body.get("neck_joint_names", [])
        if not neck or any(name not in config["joint_names"] for name in neck):
            raise ConfigurationError(
                "Enabling neck joints requires a robot package with those joints in its control group"
            )
    hardware = config["hardware"]
    if hardware["contract"] not in ("legacy-model-topic", "internal-model-v2"):
        raise ConfigurationError("Unknown X1 hardware contract")
    allowed = hardware["allowed_groups"]
    known = set(config["groups"]) | {
        side + "_gripper" for side in ("left", "right") if side in config["grippers"]
    }
    if config["enable_base"]:
        known.add("base")
    if (
        not isinstance(allowed, list)
        or not allowed
        or len(set(allowed)) != len(allowed)
        or set(allowed) - known
    ):
        raise ConfigurationError("Unknown, duplicate or empty allowed_groups")
    for key in ("ready_timeout_s", "feedback_timeout_s"):
        if not 0.02 <= finite(hardware.get(key, 0.5), key) <= 5:
            raise ConfigurationError(f"Invalid {key}")
    camera = config.get("camera", {})
    if camera.get("encoder_backend", "x264") not in (
        "auto",
        "x264",
        "libx264",
        "software",
        "nvenc",
        "h264_nvenc",
        "vaapi",
        "h264_vaapi",
        "v4l2",
        "v4l2m2m",
        "h264_v4l2m2m",
    ):
        raise ConfigurationError("Unknown camera encoder backend")
    streams = camera.get("streams", [])
    identifiers = []
    if not isinstance(streams, list) or len(streams) > 16:
        raise ConfigurationError("At most 16 camera sources may be configured")
    for stream in streams:
        identifier = stream.get("id", "")
        if (
            not isinstance(identifier, str)
            or not identifier
            or not identifier.replace("_", "").replace("-", "").isalnum()
            or identifier in identifiers
        ):
            raise ConfigurationError("Camera IDs must be unique names")
        identifiers.append(identifier)
        TopicRegistry.validate(
            {
                "topic": stream.get("topic", ""),
                "type": stream.get("type", "sensor_msgs/msg/CompressedImage"),
            }
        )
        if stream.get("type", "sensor_msgs/msg/CompressedImage") not in (
            "sensor_msgs/msg/Image",
            "sensor_msgs/msg/CompressedImage",
        ):
            raise ConfigurationError("Unsupported camera message type")
        if str(stream.get("codec", "H264")).upper() != "H264":
            raise ConfigurationError("The negotiated headset transport requires H264")
        for key, low, high in (
            ("width", 16, 4096),
            ("height", 16, 2160),
            ("fps", 1, 90),
        ):
            if (
                not low
                <= finite(
                    stream.get(key, camera.get(key, 30 if key == "fps" else 640)), key
                )
                <= high
            ):
                raise ConfigurationError(f"Invalid camera {key}")
        width, height = (
            int(stream.get("width", camera.get("width", 640))),
            int(stream.get("height", camera.get("height", 400))),
        )
        fps = float(stream.get("fps", camera.get("fps", 30)))
        blocks = math.ceil(width / 16) * math.ceil(height / 16)
        if width % 2 or height % 2 or blocks > 3600 or blocks * fps > 108000:
            raise ConfigurationError(
                "Camera dimensions/rate exceed negotiated H264 level 3.1 (even dimensions required)"
            )
    return config


def seal_config(config):
    """Hash effective control/recording semantics, independent of machine file paths."""
    config["package_hash"] = config.get("package_hash", config["config_hash"])
    excluded = {
        "config_hash",
        "package_root",
        "urdf",
        "urdf_path",
        "runtime_state_dir",
        "ros_domain_id",
        "vr",
        "vr_udp_port",
        "discovery_port",
        "status_port",
        "topics",
        "verification",
    }
    canonical = {key: value for key, value in config.items() if key not in excluded}
    config["config_hash"] = hashlib.sha256(
        json.dumps(
            canonical, sort_keys=True, separators=(",", ":"), allow_nan=False
        ).encode()
    ).hexdigest()
    return config

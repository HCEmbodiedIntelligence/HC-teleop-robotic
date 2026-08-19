from __future__ import annotations

import copy
import os
import tempfile
from pathlib import Path
from typing import Any

import yaml


DEFAULT_CONFIG: dict[str, Any] = {
    "server": {"host": "0.0.0.0", "port": 7876},
    "robot_profiles": {"root": "robot_configs", "active": "hc_tj_description"},
    "ros": {
        "enabled": True,
        "domain_id": 0,
        "node_name": "hc_teleop_middleware",
        "subscriptions": [],
        "recording": {
            "directory": "runtime/topic_recordings",
        },
    },
    "vr": {
        "enabled": True,
        "listen_host": "0.0.0.0",
        "pose_port": 5005,
        "discovery_port": 5006,
        "outbound_host": "",
        "outbound_port": 5007,
        "pose_timeout_ms": 200,
        "publish_to_ros": True,
        "data_topic": "/vrdata",
    },
    "safety": {
        "enabled": True,
        "stop_on_startup": True,
        "stop_topic": "/teleop/emergency_stop",
    },
    "camera": {
        "enabled": False,
        "source": "ros",
        "topic": "/hc_teleop/camera_head/color/compressed",
        "custom_topic": "",
        "width": 640,
        "height": 400,
        "fps": 30,
        "codec": "H264",
    },
}


class ConfigError(ValueError):
    pass


def _merge(base: dict[str, Any], override: dict[str, Any]) -> dict[str, Any]:
    result = copy.deepcopy(base)
    for key, value in override.items():
        if isinstance(value, dict) and isinstance(result.get(key), dict):
            result[key] = _merge(result[key], value)
        else:
            result[key] = copy.deepcopy(value)
    return result


def _port(value: Any, path: str) -> int:
    if isinstance(value, bool) or not isinstance(value, int) or not 0 < value < 65536:
        raise ConfigError(f"{path} must be an integer between 1 and 65535")
    return value


def validate_config(config: dict[str, Any]) -> dict[str, Any]:
    if not isinstance(config, dict):
        raise ConfigError("configuration root must be an object")
    value = _merge(DEFAULT_CONFIG, config)

    profile_config = value.get("robot_profiles")
    if not isinstance(profile_config, dict):
        raise ConfigError("robot_profiles must be an object")
    profile_root = profile_config.get("root")
    if not isinstance(profile_root, str) or not profile_root.strip():
        raise ConfigError("robot_profiles.root must be a non-empty path")
    active_profile = profile_config.get("active", "")
    if not isinstance(active_profile, str):
        raise ConfigError("robot_profiles.active must be a string")
    if active_profile and (
        len(active_profile) > 64
        or not active_profile[0].isalnum()
        or any(
            not (character.isascii() and (character.isalnum() or character in "_.-"))
            for character in active_profile
        )
    ):
        raise ConfigError("robot_profiles.active contains invalid characters")

    domain_id = value["ros"].get("domain_id", 0)
    if isinstance(domain_id, bool) or not isinstance(domain_id, int) or not (0 <= domain_id <= 232):
        raise ConfigError("ros.domain_id must be an integer between 0 and 232")
    value["ros"]["domain_id"] = domain_id

    _port(value["server"]["port"], "server.port")
    _port(value["vr"]["pose_port"], "vr.pose_port")
    _port(value["vr"]["discovery_port"], "vr.discovery_port")
    _port(value["vr"]["outbound_port"], "vr.outbound_port")
    timeout = value["vr"]["pose_timeout_ms"]
    if isinstance(timeout, bool) or not isinstance(timeout, (int, float)) or timeout < 50:
        raise ConfigError("vr.pose_timeout_ms must be at least 50")
    value["vr"]["data_topic"] = "/vrdata"
    for obsolete in (
        "publish_pose_to_ros",
        "publish_input_to_ros",
        "pose_topics",
        "input_topics",
        "event_topic",
    ):
        value["vr"].pop(obsolete, None)
    value["safety"]["stop_topic"] = "/teleop/emergency_stop"

    subscriptions = value["ros"].get("subscriptions", [])
    if not isinstance(subscriptions, list):
        raise ConfigError("ros.subscriptions must be an array")
    seen: set[str] = set()
    normalized = []
    for index, item in enumerate(subscriptions):
        if not isinstance(item, dict):
            raise ConfigError(f"ros.subscriptions[{index}] must be an object")
        topic = item.get("topic", "")
        msg_type = item.get("type", "")
        if not isinstance(topic, str) or not topic.startswith("/"):
            raise ConfigError(f"ros.subscriptions[{index}].topic must start with /")
        if not isinstance(msg_type, str) or msg_type.count("/") != 2:
            raise ConfigError(
                f"ros.subscriptions[{index}].type must look like package/msg/Type"
            )
        if topic in seen:
            raise ConfigError(f"duplicate ROS subscription: {topic}")
        seen.add(topic)
        outputs = item.get("outputs", ["websocket"])
        if not isinstance(outputs, list) or any(
            output not in ("websocket", "udp", "record") for output in outputs
        ):
            raise ConfigError(
                f"ros.subscriptions[{index}].outputs may only contain websocket, udp or record"
            )
        max_hz = item.get("max_hz", 0)
        if isinstance(max_hz, bool) or not isinstance(max_hz, (int, float)) or max_hz < 0:
            raise ConfigError(f"ros.subscriptions[{index}].max_hz must be >= 0")
        normalized.append(
            {
                "topic": topic,
                "type": msg_type,
                "enabled": bool(item.get("enabled", True)),
                "outputs": outputs,
                "max_hz": float(max_hz),
            }
        )
    value["ros"]["subscriptions"] = normalized
    recording = value["ros"].get("recording")
    if not isinstance(recording, dict):
        raise ConfigError("ros.recording must be an object")
    directory = recording.get("directory")
    if not isinstance(directory, str) or not directory.strip():
        raise ConfigError("ros.recording.directory must be a non-empty path")
    cam_config = value.get("camera", {})
    if isinstance(cam_config, dict):
        cam_config["enabled"] = bool(cam_config.get("enabled", False))
        cam_config["source"] = str(cam_config.get("source", "ros"))
        cam_topic = str(cam_config.get("topic", "/hc_teleop/camera_head/color/compressed")).strip()
        if cam_topic and not cam_topic.startswith("/"):
            cam_topic = "/" + cam_topic
        cam_config["topic"] = cam_topic or "/hc_teleop/camera_head/color/compressed"
        custom_topic = str(cam_config.get("custom_topic", "")).strip()
        if custom_topic and not custom_topic.startswith("/"):
            custom_topic = "/" + custom_topic
        cam_config["custom_topic"] = custom_topic
        cam_config["width"] = int(cam_config.get("width", 640))
        cam_config["height"] = int(cam_config.get("height", 400))
        cam_config["fps"] = int(cam_config.get("fps", 30))
        cam_config["codec"] = str(cam_config.get("codec", "H264"))
    value["camera"] = cam_config
    return value


class ConfigStore:
    def __init__(self, path: str | Path):
        self.path = Path(path).expanduser().resolve()
        self.value = copy.deepcopy(DEFAULT_CONFIG)

    def load(self) -> dict[str, Any]:
        if self.path.exists():
            with self.path.open("r", encoding="utf-8") as stream:
                loaded = yaml.safe_load(stream) or {}
            self.value = validate_config(loaded)
        else:
            self.value = validate_config({})
            self.save(self.value)
        return copy.deepcopy(self.value)

    def save(self, value: dict[str, Any]) -> dict[str, Any]:
        validated = validate_config(value)
        self.path.parent.mkdir(parents=True, exist_ok=True)
        fd, temp_name = tempfile.mkstemp(
            prefix=f".{self.path.name}.", dir=self.path.parent, text=True
        )
        try:
            with os.fdopen(fd, "w", encoding="utf-8") as stream:
                yaml.safe_dump(
                    validated, stream, allow_unicode=True, sort_keys=False
                )
                stream.flush()
                os.fsync(stream.fileno())
            os.replace(temp_name, self.path)
        finally:
            if os.path.exists(temp_name):
                os.unlink(temp_name)
        self.value = validated
        return copy.deepcopy(validated)

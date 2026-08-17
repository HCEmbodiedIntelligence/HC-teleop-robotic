from __future__ import annotations

import copy
import os
import tempfile
from pathlib import Path
from typing import Any

import yaml


DEFAULT_CONFIG: dict[str, Any] = {
    "server": {"host": "0.0.0.0", "port": 7876},
    "ros": {
        "enabled": True,
        "node_name": "hc_teleop_middleware",
        "subscriptions": [],
    },
    "vr": {
        "enabled": True,
        "listen_host": "0.0.0.0",
        "pose_port": 5005,
        "discovery_port": 5006,
        "outbound_host": "",
        "outbound_port": 5007,
        "pose_timeout_ms": 200,
        "publish_pose_to_ros": True,
        "publish_input_to_ros": True,
        "pose_topics": {
            "head": "/vr/head_pose",
            "left": "/vr/left_controller_pose",
            "right": "/vr/right_controller_pose",
        },
        "input_topics": {
            "left": "/vr/left_controller/input",
            "right": "/vr/right_controller/input",
        },
        "event_topic": "/vr/controller_events",
    },
    "safety": {
        "enabled": True,
        "stop_on_startup": True,
        "stop_topic": "/teleop/emergency_stop",
    },
    "camera": {
        "enabled": False,
        "width": 1280,
        "height": 720,
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

    _port(value["server"]["port"], "server.port")
    _port(value["vr"]["pose_port"], "vr.pose_port")
    _port(value["vr"]["discovery_port"], "vr.discovery_port")
    _port(value["vr"]["outbound_port"], "vr.outbound_port")
    timeout = value["vr"]["pose_timeout_ms"]
    if isinstance(timeout, bool) or not isinstance(timeout, (int, float)) or timeout < 50:
        raise ConfigError("vr.pose_timeout_ms must be at least 50")
    for name in ("head", "left", "right"):
        topic = value["vr"]["pose_topics"].get(name)
        if not isinstance(topic, str) or not topic.startswith("/"):
            raise ConfigError(f"vr.pose_topics.{name} must start with /")
    for name in ("left", "right"):
        topic = value["vr"]["input_topics"].get(name)
        if not isinstance(topic, str) or not topic.startswith("/"):
            raise ConfigError(f"vr.input_topics.{name} must start with /")
    event_topic = value["vr"].get("event_topic")
    if not isinstance(event_topic, str) or not event_topic.startswith("/"):
        raise ConfigError("vr.event_topic must start with /")
    stop_topic = value["safety"].get("stop_topic")
    if not isinstance(stop_topic, str) or not stop_topic.startswith("/"):
        raise ConfigError("safety.stop_topic must start with /")

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
            output not in ("websocket", "udp") for output in outputs
        ):
            raise ConfigError(
                f"ros.subscriptions[{index}].outputs may only contain websocket or udp"
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

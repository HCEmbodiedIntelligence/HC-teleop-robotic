from __future__ import annotations

import copy
import os
import tempfile
from pathlib import Path
from typing import Any

import yaml


DEFAULT_CONFIG: dict[str, Any] = {
    "server": {"host": "0.0.0.0", "port": 7876},
    "robot_profiles": {"root": "../adapters/robots", "active": "x1"},
    "simulation": {"headless": False, "with_control": True, "debug_joints": False},
    "ros": {
        "enabled": True,
        "domain_id": 14,
        "node_name": "hc_teleop_middleware",
        "command_mux": {
            "enabled": True,
            "source": "vr",
            "vr_topic": "/control/vr/joint_states",
            "exoskeleton_topic": "/control/exoskeleton/joint_states",
            "output_topic": "/action/joint_states",
            "control_source_topic": "/control/source",
            "hardware_enable_topic": "/control/hardware/enabled",
        },
        "subscriptions": [],
        "recording": {
            "directory": "runtime/topic_recordings",
            "format": "mcap",
            "lerobot_fps": 15,
            "lerobot_task": "遥操作",
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
        "stop_topic": "/control/emergency_stop",
    },
    "camera": {
        "enabled": False,
        "source": "ros",
        "topic": "/observation/cameras/head/color/compressed",
        "custom_topic": "",
        "streams": [],
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

    simulation = value.get("simulation")
    if not isinstance(simulation, dict):
        raise ConfigError("simulation must be an object")
    for key in ("headless", "with_control", "debug_joints"):
        if not isinstance(simulation.get(key), bool):
            raise ConfigError(f"simulation.{key} must be boolean")

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

    command_mux = value["ros"].get("command_mux")
    if not isinstance(command_mux, dict):
        raise ConfigError("ros.command_mux must be an object")
    source = str(command_mux.get("source", "vr")).strip().lower()
    if source not in {"vr", "exoskeleton", "motion"}:
        raise ConfigError("ros.command_mux.source must be vr, exoskeleton or motion")
    command_mux["source"] = source
    command_mux["enabled"] = bool(command_mux.get("enabled", True))
    command_mux.setdefault("motion_topic", "/control/motion/joint_targets")
    command_mux.setdefault("motion_permit_topic", "/control/motion/permit")
    mux_topics = []
    for key in (
        "vr_topic",
        "motion_topic",
        "motion_permit_topic",
        "exoskeleton_topic",
        "output_topic",
        "control_source_topic",
        "hardware_enable_topic",
    ):
        topic = str(command_mux.get(key, "")).strip()
        if not topic.startswith("/"):
            raise ConfigError(f"ros.command_mux.{key} must start with /")
        command_mux[key] = topic
        mux_topics.append(topic)
    if len(set(mux_topics)) != len(mux_topics):
        raise ConfigError("ros.command_mux topics must be different")
    if command_mux["hardware_enable_topic"] in {
        "/control/teleop/enabled", value.get("safety", {}).get("stop_topic", "/control/emergency_stop")
    }:
        raise ConfigError("hardware_enable_topic must differ from VR enable and emergency stop")

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
    value["safety"]["stop_topic"] = "/control/emergency_stop"

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
        event_max_hz = item.get("event_max_hz", 0)
        if (
            isinstance(event_max_hz, bool)
            or not isinstance(event_max_hz, (int, float))
            or event_max_hz < 0
        ):
            raise ConfigError(
                f"ros.subscriptions[{index}].event_max_hz must be >= 0"
            )
        qos_reliability = item.get("qos_reliability")
        if qos_reliability is not None and qos_reliability not in ("best_effort", "reliable"):
            raise ConfigError(f"ros.subscriptions[{index}].qos_reliability must be best_effort or reliable")
        description = item.get("description", "")
        if not isinstance(description, str) or len(description) > 500:
            raise ConfigError(f"ros.subscriptions[{index}].description must be a string of at most 500 characters")
        mode = item.get("recording_mode", "event" if topic.startswith(("/action/", "/control/", "/status/")) else "continuous")
        if mode not in ("continuous", "event") or not isinstance(item.get("required", True), bool):
            raise ConfigError("invalid recording_mode/required")
        normalized.append(
            {
                "topic": topic,
                "recording_mode": mode,
                "required": item.get("required", True),
                "description": description,
                "type": msg_type,
                "enabled": bool(item.get("enabled", True)),
                "outputs": outputs,
                "max_hz": float(max_hz),
                "event_max_hz": float(event_max_hz),
            }
        )
        if qos_reliability is not None:
            normalized[-1]["qos_reliability"] = qos_reliability
    value["ros"]["subscriptions"] = normalized
    mux = value["ros"].setdefault("command_mux", {})
    mux.setdefault("motion_topic", "/control/motion/joint_targets")
    timeout = mux.setdefault("hardware_ready_timeout_sec", 0.5)
    import math
    if isinstance(timeout, bool) or not isinstance(timeout, (int, float)) or not math.isfinite(timeout) or timeout <= 0:
        raise ConfigError("hardware_ready_timeout_sec must be positive and finite")
    if not isinstance(mux["motion_topic"], str) or not mux["motion_topic"].startswith("/"):
        raise ConfigError("motion_topic must be an absolute ROS topic")
    recording = value["ros"].get("recording")
    if not isinstance(recording, dict):
        raise ConfigError("ros.recording must be an object")
    directory = recording.get("directory")
    if not isinstance(directory, str) or not directory.strip():
        raise ConfigError("ros.recording.directory must be a non-empty path")
    workflow = recording.setdefault("workflow", "hc")
    if workflow not in ("hc", "platform_raw"):
        raise ConfigError("ros.recording.workflow must be hc or platform_raw")
    if type(recording.setdefault("derive_action_from_next_state", False)) is not bool:
        raise ConfigError("ros.recording.derive_action_from_next_state must be boolean")
    if workflow != "platform_raw" and recording["derive_action_from_next_state"]:
        raise ConfigError("测试 action 补齐仅支持平台 raw 录制工作流")
    for key, default in (("capture_config_file", ""), ("job_state_dir", "../runtime/recording_jobs"),
                         ("artifact_state_dir", "../runtime/artifact_ready")):
        value_path = recording.setdefault(key, default)
        if not isinstance(value_path, str) or (key != "capture_config_file" and not value_path.strip()):
            raise ConfigError("ros.recording." + key + " must be a path")
    if workflow == "platform_raw" and not recording["capture_config_file"].strip():
        raise ConfigError("platform_raw requires capture_config_file")
    for key, default in (("max_pending_jobs", 8), ("min_free_bytes", 268435456)):
        number = recording.setdefault(key, default)
        if isinstance(number, bool) or not isinstance(number, int) or number < 1:
            raise ConfigError("ros.recording." + key + " must be a positive integer")
    gap_rules = recording.setdefault("gap_rules", [])
    if not isinstance(gap_rules, list):
        raise ConfigError("recording.gap_rules must be a list")
    for rule in gap_rules:
        if not isinstance(rule, dict) or set(rule) != {"topic", "max_gap_seconds"}:
            raise ConfigError("gap_rules requires topic and max_gap_seconds")
        gap = rule["max_gap_seconds"]
        if not isinstance(rule["topic"], str) or not rule["topic"].startswith("/") or isinstance(gap, bool) or not isinstance(gap, (int, float)) or not math.isfinite(gap) or gap <= 0:
            raise ConfigError("gap_rules requires an absolute topic and positive finite gap")
    sources = recording.setdefault("action_sources", [])
    if not isinstance(sources, list):
        raise ConfigError("action_sources must be a list")
    used_topics, used_names = set(), set()
    for item in sources:
        if not isinstance(item, dict) or set(item) != {"topic", "names", "units", "max_age_sec"}:
            raise ConfigError("action_sources requires topic, names, units, max_age_sec")
        if not isinstance(item['topic'], str) or not item['topic'].startswith('/') or item['topic'] in used_topics | {'/action/joint_states', '/observation/joint_states'}:
            raise ConfigError("invalid or duplicate action topic")
        if not isinstance(item['names'], list) or not item['names'] or any(not isinstance(n,str) or not n for n in item['names']):
            raise ConfigError("action source names must be nonempty strings")
        if len(set(item['names'])) != len(item['names']) or used_names.intersection(item['names']):
            raise ConfigError("duplicate action joint names")
        if not isinstance(item['units'], list) or len(item['units']) != len(item['names']) or any(u not in {'rad', 'm', 'unitless'} for u in item['units']):
            raise ConfigError("action source units must match names")
        age = item['max_age_sec']
        if isinstance(age, bool) or not isinstance(age, (int,float)) or not math.isfinite(age) or age <= 0:
            raise ConfigError("action source max_age_sec must be positive and finite")
        used_topics.add(item['topic'])
        used_names.update(item['names'])
    recording_format = recording.get("format", "mcap")
    if recording_format not in ("mcap", "lerobot", "both"):
        raise ConfigError("ros.recording.format must be mcap, lerobot or both")
    lerobot_fps = recording.get("lerobot_fps", 15)
    if isinstance(lerobot_fps, bool) or not isinstance(lerobot_fps, int) or not 1 <= lerobot_fps <= 60:
        raise ConfigError("ros.recording.lerobot_fps must be an integer from 1 to 60")
    lerobot_task = recording.get("lerobot_task", "遥操作")
    if not isinstance(lerobot_task, str) or not lerobot_task.strip() or len(lerobot_task) > 500:
        raise ConfigError("ros.recording.lerobot_task must be a non-empty string of at most 500 characters")
    recording.update(format=recording_format, lerobot_fps=lerobot_fps, lerobot_task=lerobot_task.strip())
    cam_config = value.get("camera", {})
    if isinstance(cam_config, dict):
        cam_config["enabled"] = bool(cam_config.get("enabled", False))
        cam_config["source"] = str(cam_config.get("source", "ros"))
        cam_topic = str(cam_config.get("topic", "/observation/cameras/head/color/compressed")).strip()
        if cam_topic and not cam_topic.startswith("/"):
            cam_topic = "/" + cam_topic
        cam_config["topic"] = cam_topic or "/observation/cameras/head/color/compressed"
        custom_topic = str(cam_config.get("custom_topic", "")).strip()
        if custom_topic and not custom_topic.startswith("/"):
            custom_topic = "/" + custom_topic
        cam_config["custom_topic"] = custom_topic
        cam_config["width"] = int(cam_config.get("width", 640))
        cam_config["height"] = int(cam_config.get("height", 400))
        cam_config["fps"] = int(cam_config.get("fps", 30))
        if cam_config["width"] < 2 or cam_config["height"] < 2:
            raise ConfigError("camera width and height must be at least 2")
        if cam_config["fps"] < 1 or cam_config["fps"] > 60:
            raise ConfigError("camera fps must be between 1 and 60")
        cam_config["codec"] = str(cam_config.get("codec", "H264")).upper()
        if cam_config["codec"] not in {"H264", "VP8"}:
            raise ConfigError("camera codec must be H264 or VP8")
        streams = cam_config.get("streams", [])
        if not isinstance(streams, list):
            raise ConfigError("camera.streams must be a list")
        normalized_streams = []
        stream_ids = set()
        for index, item in enumerate(streams):
            if not isinstance(item, dict):
                raise ConfigError(f"camera.streams[{index}] must be an object")
            stream_id = str(item.get("id", f"camera_{index + 1}")).strip()
            if not stream_id or any(
                not (character.isalnum() or character in "_-")
                for character in stream_id
            ):
                raise ConfigError(
                    f"camera.streams[{index}].id may only contain letters, numbers, _ and -"
                )
            if stream_id in stream_ids:
                raise ConfigError(f"duplicate camera stream id: {stream_id}")
            stream_ids.add(stream_id)
            stream_topic = str(item.get("topic", "")).strip()
            if stream_topic and not stream_topic.startswith("/"):
                stream_topic = "/" + stream_topic
            if not stream_topic:
                raise ConfigError(f"camera.streams[{index}].topic is required")
            subscription = next((s for s in normalized if s["topic"] == stream_topic), None)
            message_type = str(item.get("type") or (subscription or {}).get("type") or (
                "sensor_msgs/msg/CompressedImage" if stream_topic.endswith("/compressed")
                else "sensor_msgs/msg/Image"
            ))
            if message_type not in {"sensor_msgs/msg/Image", "sensor_msgs/msg/CompressedImage"}:
                raise ConfigError(f"camera.streams[{index}].type must be Image or CompressedImage")
            if subscription and subscription["type"] != message_type:
                raise ConfigError(f"camera topic type conflicts with ROS subscription: {stream_topic}")
            stream_width = int(item.get("width", cam_config["width"]))
            stream_height = int(item.get("height", cam_config["height"]))
            stream_fps = int(item.get("fps", cam_config["fps"]))
            stream_codec = str(item.get("codec", cam_config["codec"])).upper()
            if stream_width < 2 or stream_height < 2:
                raise ConfigError(
                    f"camera.streams[{index}] width and height must be at least 2"
                )
            if stream_fps < 1 or stream_fps > 60:
                raise ConfigError(
                    f"camera.streams[{index}] fps must be between 1 and 60"
                )
            if stream_codec not in {"H264", "VP8"}:
                raise ConfigError(
                    f"camera.streams[{index}] codec must be H264 or VP8"
                )
            normalized_streams.append(
                {
                    **item,
                    "id": stream_id,
                    "name": str(item.get("name", stream_id)).strip() or stream_id,
                    "topic": stream_topic,
                    "type": message_type,
                    "custom_topic": "",
                    "enabled": bool(item.get("enabled", True)),
                    "width": stream_width,
                    "height": stream_height,
                    "fps": stream_fps,
                    "codec": stream_codec,
                }
            )
        cam_config["streams"] = normalized_streams
        # Projection does not require recording or JSON/WebSocket image events.
        # Ensure a newly selected image topic actually reaches CameraService.
        if cam_config["enabled"] and cam_config["source"] == "ros":
            for stream in normalized_streams:
                if not stream["enabled"]:
                    continue
                subscription = next((s for s in normalized if s["topic"] == stream["topic"]), None)
                if subscription is None:
                    subscription = {"topic": stream["topic"], "type": stream["type"],
                                    "enabled": True, "outputs": [], "max_hz": 0.0, "event_max_hz": 0.0, "description": ""}
                    normalized.append(subscription)
                elif subscription["type"] != stream["type"]:
                    raise ConfigError(f"conflicting camera types for topic: {stream['topic']}")
    uploads = value.setdefault('uploads', {})
    if not isinstance(uploads, dict):
        raise ConfigError('uploads must be an object')
    for key in ('enabled', 'auto_enqueue'):
        if not isinstance(uploads.setdefault(key, False), bool):
            raise ConfigError('uploads.' + key + ' must be boolean')
    for key, default in (('queue_directory','../runtime/upload_queue'), ('platform_base_url',''), ('credential_ref',''), ('robot_id',''), ('local_robot_id',''), ('task_id','')):
        if not isinstance(uploads.setdefault(key, default), str):
            raise ConfigError('uploads.' + key + ' must be string')
    if uploads['credential_ref'] and not uploads['credential_ref'].startswith(('env:', 'file:')):
        raise ConfigError('uploads credentials must use env: or file: reference')
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

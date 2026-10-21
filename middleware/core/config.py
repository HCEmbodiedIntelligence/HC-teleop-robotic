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
    "simulation": {"headless": False, "with_control": True, "debug_joints": False, "startup_timeout_sec": 120},
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
            "hardware_ready_topic": "/status/hardware/ready",
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


ROBOT_MIDDLEWARE_DEFAULTS: dict[str, Any] = {
    "camera": copy.deepcopy(DEFAULT_CONFIG["camera"]),
    "safety": copy.deepcopy(DEFAULT_CONFIG["safety"]),
    "ros": {
        "subscriptions": [
            {"topic": topic, "type": "sensor_msgs/msg/JointState", "enabled": True,
             "outputs": ["websocket", "record"], "event_max_hz": 20.0}
            for topic in ("/observation/joint_states", "/action/joint_states")
        ],
        "command_mux": copy.deepcopy(DEFAULT_CONFIG["ros"]["command_mux"]),
    },
    "vr": {"pose_timeout_ms": 600},
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

    timeout = simulation.get("startup_timeout_sec")
    if isinstance(timeout, bool) or not isinstance(timeout, (int, float)) or not 0 < timeout <= 600:
        raise ConfigError("simulation.startup_timeout_sec must be between 0 and 600 seconds")

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
    if source not in {"vr", "exoskeleton"}:
        raise ConfigError("ros.command_mux.source must be vr or exoskeleton")
    command_mux["source"] = source
    command_mux["enabled"] = bool(command_mux.get("enabled", True))
    mux_topics = []
    for key in (
        "vr_topic",
        "exoskeleton_topic",
        "output_topic",
        "control_source_topic",
    ):
        topic = str(command_mux.get(key, "")).strip()
        if not topic.startswith("/"):
            raise ConfigError(f"ros.command_mux.{key} must start with /")
        command_mux[key] = topic
        mux_topics.append(topic)
    if len(set(mux_topics)) != len(mux_topics):
        raise ConfigError("ros.command_mux topics must be different")

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
        normalized.append(
            {
                "topic": topic,
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
    recording = value["ros"].get("recording")
    if not isinstance(recording, dict):
        raise ConfigError("ros.recording must be an object")
    directory = recording.get("directory")
    if not isinstance(directory, str) or not directory.strip():
        raise ConfigError("ros.recording.directory must be a non-empty path")
    recording_format = recording.get("format", "mcap")
    if recording_format not in ("mcap", "lerobot", "both", "mcap_mp4"):
        raise ConfigError("ros.recording.format must be mcap, lerobot, both or mcap_mp4")
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
    uploads = value.setdefault("uploads", {})
    if not isinstance(uploads, dict):
        raise ConfigError("uploads must be an object")
    for key in ("enabled", "auto_enqueue"):
        if not isinstance(uploads.setdefault(key, False), bool):
            raise ConfigError("uploads." + key + " must be boolean")
    for key, default in (("queue_directory", "../runtime/upload_queue"),
                         ("platform_base_url", ""), ("credential_ref", ""),
                         ("robot_id", ""), ("local_robot_id", ""), ("task_id", "")):
        if not isinstance(uploads.setdefault(key, default), str):
            raise ConfigError("uploads." + key + " must be string")
    if not uploads["queue_directory"].strip():
        raise ConfigError("uploads.queue_directory must be a non-empty path")
    if uploads["credential_ref"] and not uploads["credential_ref"].startswith(("env:", "file:")):
        raise ConfigError("uploads credentials must use env: or file: reference")
    for key, default in (("artifact_state_dir", "../runtime/artifact_state"),
                         ("upload_job_state_dir", "../runtime/recording_uploads")):
        if not isinstance(recording.setdefault(key, default), str) or not recording[key].strip():
            raise ConfigError("ros.recording." + key + " must be a non-empty path")
    value["camera"] = cam_config
    return value


def _robot_middleware_fields(value: dict[str, Any]) -> dict[str, Any]:
    return {
        section: {key: copy.deepcopy(value[section][key]) for key in defaults}
        for section, defaults in ROBOT_MIDDLEWARE_DEFAULTS.items()
    }


def validate_robot_middleware(value: dict[str, Any]) -> dict[str, Any]:
    """Validate a portable robot configuration, excluding host settings."""
    if not isinstance(value, dict):
        raise ConfigError("robot middleware configuration must be an object")
    for section, settings in value.items():
        if section not in ROBOT_MIDDLEWARE_DEFAULTS:
            raise ConfigError(f"robot middleware cannot configure {section}")
        if not isinstance(settings, dict):
            raise ConfigError(f"robot middleware {section} must be an object")
        unknown = set(settings) - set(ROBOT_MIDDLEWARE_DEFAULTS[section])
        if unknown:
            raise ConfigError(f"robot middleware cannot configure {section}.{sorted(unknown)[0]}")
    mux = value.get("ros", {}).get("command_mux", {})
    if not isinstance(mux, dict) or set(mux) - set(DEFAULT_CONFIG["ros"]["command_mux"]):
        raise ConfigError("robot middleware ros.command_mux contains unsupported fields")
    for key, setting in value.get("safety", {}).items():
        if key != "stop_topic" and not isinstance(setting, bool):
            raise ConfigError(f"safety.{key} must be boolean")
    try:
        normalized = validate_config(_merge(ROBOT_MIDDLEWARE_DEFAULTS, value))
    except (TypeError, ValueError, OverflowError) as exc:
        raise ConfigError(str(exc)) from exc
    return _robot_middleware_fields(normalized)


def _replace_files(payloads: dict[Path, bytes]) -> None:
    """Stage every write before replacing files; restore prior files on failure."""
    staged: dict[Path, str] = {}
    previous: dict[Path, bytes | None] = {}
    replaced: list[Path] = []
    try:
        for path, payload in payloads.items():
            path.parent.mkdir(parents=True, exist_ok=True)
            previous[path] = path.read_bytes() if path.exists() else None
            fd, name = tempfile.mkstemp(prefix=f".{path.name}.", dir=path.parent)
            staged[path] = name
            with os.fdopen(fd, "wb") as stream:
                stream.write(payload)
                stream.flush()
                os.fsync(stream.fileno())
        for path, name in staged.items():
            os.replace(name, path)
            replaced.append(path)
    except Exception:
        for path in reversed(replaced):
            original = previous[path]
            if original is None:
                path.unlink(missing_ok=True)
            else:
                fd, name = tempfile.mkstemp(prefix=f".{path.name}.rollback-", dir=path.parent)
                try:
                    with os.fdopen(fd, "wb") as stream:
                        stream.write(original)
                        stream.flush()
                        os.fsync(stream.fileno())
                    os.replace(name, path)
                finally:
                    if os.path.exists(name):
                        os.unlink(name)
        raise
    finally:
        for name in staged.values():
            if os.path.exists(name):
                os.unlink(name)


class ConfigStore:
    def __init__(self, path: str | Path):
        self.path = Path(path).expanduser().resolve()
        self.value = copy.deepcopy(DEFAULT_CONFIG)

    def _profile_middleware_path(self, value: dict[str, Any]) -> Path | None:
        selection = value["robot_profiles"]
        active = selection.get("active", "")
        if not active:
            return None
        root = Path(selection["root"]).expanduser()
        if not root.is_absolute():
            root = self.path.parent / root
        return root.resolve() / active / "middleware.yaml"

    def apply_profile(self, value: dict[str, Any]) -> dict[str, Any]:
        """Load the selected robot's settings without modifying either file."""
        validated = validate_config(value)
        path = self._profile_middleware_path(validated)
        if path is None or not path.is_file():
            return validated
        try:
            with path.open(encoding="utf-8") as stream:
                robot = validate_robot_middleware(yaml.safe_load(stream))
        except (OSError, yaml.YAMLError) as exc:
            raise ConfigError(f"unable to read robot middleware {path}: {exc}") from exc
        # Replace each robot-owned field, including complete lists and camera
        # settings, so no settings from the previously selected robot survive.
        for section, settings in robot.items():
            if section in {"camera", "safety"}:
                validated[section] = copy.deepcopy(settings)
            else:
                validated[section].update(copy.deepcopy(settings))
        return validate_config(validated)

    def load(self) -> dict[str, Any]:
        if self.path.exists():
            with self.path.open("r", encoding="utf-8") as stream:
                loaded = yaml.safe_load(stream) or {}
            self.value = self.apply_profile(loaded)
        else:
            self.value = self.apply_profile({})
            self.save(self.value)
        return copy.deepcopy(self.value)

    def save(self, value: dict[str, Any]) -> dict[str, Any]:
        validated = validate_config(value)
        path = self._profile_middleware_path(validated)
        previous_path = self._profile_middleware_path(self.value)
        package_exists = path is not None and path.is_file()
        switching = path != previous_path
        if switching:
            validated = self.apply_profile(validated)
        host = copy.deepcopy(validated)
        payloads: dict[Path, bytes] = {}
        if package_exists:
            robot = validate_robot_middleware(_robot_middleware_fields(validated))
            if not switching:
                payloads[path] = yaml.safe_dump(robot, allow_unicode=True, sort_keys=False).encode("utf-8")
            for section, defaults in ROBOT_MIDDLEWARE_DEFAULTS.items():
                for key in defaults:
                    host[section].pop(key, None)
        payloads[self.path] = yaml.safe_dump(host, allow_unicode=True, sort_keys=False).encode("utf-8")
        _replace_files(payloads)
        self.value = validated
        return copy.deepcopy(validated)

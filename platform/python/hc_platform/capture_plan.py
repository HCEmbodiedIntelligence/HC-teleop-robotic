"""Match a frozen platform contract to actual local capture/export capabilities."""

from __future__ import annotations

import math

from .configuration import ConfigurationError

STATE = "/platform/control/state"
ACTION = "/platform/control/commands"
GENERATED = {"timestamp", "frame_index", "episode_index", "index", "task_index"}


def validate_capture_plan(snapshot, config, selected, preflight, spec, control_state):
    """Return explicit converter options; never fabricate an unavailable feature.

    Camera shape is checked against the *exported* image, not WebRTC's resized
    preview. MCAP keeps the original source data and its frozen server adapter.
    """
    output = snapshot.get("output", {})
    features = output.get("features")
    if not isinstance(features, dict) or not features:
        raise ConfigurationError("Task capture plan has no feature definitions")
    lerobot = snapshot.get("source_format") == "LEROBOT_V3" or spec.get(
        "local_format"
    ) in ("lerobot", "mcap_lerobot")
    adapter = snapshot.get("effective_adapter", {})
    raw_source_map = adapter.get("feature_sources", {})
    source_map = raw_source_map if isinstance(raw_source_map, dict) else {}
    mapping = adapter.get("configuration", {})
    selected_names = {item["topic"] for item in selected}
    available = selected_names | {STATE, ACTION}
    unhealthy = {
        item["topic"]: item["reason"]
        for item in preflight.get("warnings", [])
        if item.get("topic")
    }
    joint_units = dict(zip(config["joint_names"], config["joint_units"]))
    streams = config.get("camera", {}).get("streams", [])
    disabled_cameras = {
        "observation.images." + item["id"]
        for item in streams
        if not item.get("enabled", True)
    }
    disabled_topics = {
        item["topic"] for item in streams if not item.get("enabled", True)
    }
    cameras = {
        "observation.images." + item["id"]: item["topic"]
        for item in streams
        if item.get("enabled", True)
    }
    # MCAP wire contract uses rgb/rgb_topic and depth/depth_topic mappings.
    for camera in mapping.get("cameras", []):
        for kind in ("rgb", "depth"):
            cam_key, cam_topic = camera.get(kind), camera.get(kind + "_topic")
            if (
                cam_key
                and cam_topic
                and cam_key not in disabled_cameras
                and cam_topic not in disabled_topics
            ):
                cameras[cam_key] = cam_topic
    options, vectors, exported_cameras = {}, {}, {}

    def require_topic(topic, feature, *, idle_allowed=False):
        if topic not in available:
            raise ConfigurationError(
                f"Task {feature} requires unselected topic: {topic}"
            )
        reason = unhealthy.get(topic)
        if reason and not (
            idle_allowed and reason in {"TOPIC_UNDISCOVERED", "NO_FRESH_DATA"}
        ):
            raise ConfigurationError(
                f"Task {feature} source unavailable: {topic}: {reason}"
            )

    for key, wanted in features.items():
        if not isinstance(wanted, dict):
            raise ConfigurationError(f"Task feature must be a mapping: {key}")
        required = wanted.get("required", True)
        source = source_map.get(key, key)
        if not isinstance(source, str):
            raise ConfigurationError(f"Task feature source must be a name: {key}")
        if source in {"observation.state", "action"}:
            topic = mapping.get(
                "state_topic" if source == "observation.state" else "action_topic",
                STATE if source == "observation.state" else ACTION,
            )
            if required or topic in available:
                require_topic(topic, key, idle_allowed=source == "action")
                names = (
                    wanted["names"]
                    if "names" in wanted and wanted["names"] is not None
                    else [
                        name
                        for name in config["joint_names"]
                        if any(
                            name in config["groups"].get(group, [])
                            for group in config.get("allowed_groups", [])
                        )
                    ]
                )
                if (
                    not isinstance(names, list)
                    or not names
                    or any(not isinstance(name, str) for name in names)
                    or len(set(names)) != len(names)
                    or set(names) - joint_units.keys()
                ):
                    raise ConfigurationError(
                        f"Task joint names do not match the applied robot: {key}"
                    )
                if wanted.get("shape") not in (None, [len(names)]):
                    raise ConfigurationError(
                        f"Task joint shape does not match its names: {key}"
                    )
                if wanted.get("units") and wanted["units"] != [
                    joint_units[name] for name in names
                ]:
                    raise ConfigurationError(
                        f"Task joint units do not match the applied robot: {key}"
                    )
                if lerobot and wanted.get("dtype", "float32") != "float32":
                    raise ConfigurationError(
                        f"Unsupported training vector dtype: {key}"
                    )
                if source == "observation.state" and topic == STATE and required:
                    if set(names) - set(control_state.get("fresh_joint_names", [])):
                        raise ConfigurationError(
                            "Task requires fresh feedback for every selected state joint"
                        )
                if source == "action" and topic == ACTION:
                    controllable = {
                        name
                        for group in config.get("allowed_groups", [])
                        for name in config["groups"].get(group, [])
                    }
                    if set(names) - controllable:
                        raise ConfigurationError(
                            "Task action joints exceed the configured control authorization"
                        )
                vectors[source] = names
                options[
                    "state_topic" if source == "observation.state" else "action_topic"
                ] = topic
        elif source in cameras:
            topic = cameras[source]
            if required or topic in selected_names:
                require_topic(topic, key)
                if lerobot and (
                    wanted.get("dtype") != "video"
                    or wanted.get("role", "rgb") != "rgb"
                    or "/depth/" in topic
                ):
                    raise ConfigurationError(
                        f"Only RGB video training export is supported: {key}"
                    )
                exported_cameras[source] = topic
        elif lerobot and source in GENERATED | {
            "observation.source_valid",
            "observation.source_age_s",
        }:
            continue
        elif required:
            raise ConfigurationError(
                f"Task requires an unsupported or unmapped feature: {key} (source {source})"
            )

    if not lerobot:
        return {}
    if len(vectors) == 2 and vectors["observation.state"] != vectors["action"]:
        raise ConfigurationError(
            "This exporter requires the same named joint order for state and action"
        )
    names = next(iter(vectors.values()), None)
    if names:
        options.update(
            joint_names=names, joint_units=[joint_units[name] for name in names]
        )
    options["cameras"] = exported_cameras
    frequency = output.get("frequency_hz", spec.get("dataset_fps", 30))
    if (
        isinstance(frequency, bool)
        or not isinstance(frequency, (int, float))
        or not math.isfinite(frequency)
        or int(frequency) != frequency
        or not 1 <= frequency <= 120
    ):
        raise ConfigurationError(
            "Task LeRobot frequency must be an integer from 1 to 120 Hz"
        )
    if spec.get("dataset_fps", int(frequency)) != int(frequency):
        raise ConfigurationError(f"Task requires dataset_fps={int(frequency)}")
    options["fps"] = int(frequency)
    supplied = spec.get("conversion", {})
    if not isinstance(supplied, dict):
        raise ConfigurationError("conversion must be a mapping")
    for key in options:
        if key in supplied and supplied[key] != options[key]:
            raise ConfigurationError(
                f"Conversion option conflicts with frozen task: {key}"
            )
    return {**supplied, **options}


def validate_lerobot_features(snapshot, info):
    """Verify actual artifact definitions before publishing a derived artifact."""
    if not snapshot or snapshot.get("source_format") != "LEROBOT_V3":
        return
    adapter, output = snapshot["effective_adapter"], snapshot["output"]
    if info.get("codebase_version") != adapter.get("codebase_version", "v3.0"):
        raise ConfigurationError(
            "Converted LeRobot version differs from the frozen task"
        )
    if (
        output.get("frequency_hz") is not None
        and info.get("fps") != output["frequency_hz"]
    ):
        raise ConfigurationError(
            "Converted sampling frequency differs from the frozen task"
        )
    for key, wanted in output["features"].items():
        source = adapter.get("feature_sources", {}).get(key, key)
        actual = info.get("features", {}).get(source)
        if actual is None:
            if wanted.get("required", True):
                raise ConfigurationError(
                    f"Converted artifact lacks required feature: {key}"
                )
            continue
        for field in ("dtype", "shape", "names", "units"):
            expected = wanted.get(field)
            if (
                expected is not None
                and (field != "units" or expected)
                and actual.get(field) != expected
            ):
                raise ConfigurationError(
                    f"Converted {key}.{field} differs from the frozen task"
                )

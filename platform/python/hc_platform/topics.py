"""Topic configuration is independent of recording and UI sampling rates."""

from __future__ import annotations
from pathlib import Path
import json
import re
from .configuration import atomic_json, ConfigurationError


class TopicRegistry:
    def __init__(self, path: Path, defaults=()):
        self.path = path
        self.items = json.loads(path.read_text()) if path.exists() else list(defaults)
        if len(self.items) > 256:
            raise ConfigurationError("At most 256 configured topics are supported")
        for item in self.items:
            self.validate(item)

    @staticmethod
    def validate(item):
        if not re.fullmatch(r"/[A-Za-z0-9_/]+", item.get("topic", "")):
            raise ConfigurationError("Invalid ROS topic name")
        if not re.fullmatch(
            r"[A-Za-z][A-Za-z0-9_]*/msg/[A-Za-z][A-Za-z0-9_]*", item.get("type", "")
        ):
            raise ConfigurationError("Expected package/msg/Type")
        if any(
            output not in ("record", "websocket", "udp")
            for output in item.get("outputs", [])
        ):
            raise ConfigurationError("Unknown topic output")
        for key in ("max_hz", "event_max_hz", "min_hz", "target_hz"):
            value = float(item.get(key, 0))
            if not 0 <= value <= 10000:
                raise ConfigurationError(f"Invalid {key}")
        if item.get("qos_reliability", "best_effort") not in (
            "best_effort",
            "reliable",
        ):
            raise ConfigurationError("Invalid reliability")
        if item.get("qos_durability", "volatile") not in (
            "volatile",
            "transient_local",
        ):
            raise ConfigurationError("Invalid durability")
        if "udp" in item.get("outputs", []):
            target = item.get("udp_target", {})
            if (
                not isinstance(target.get("host"), str)
                or not 1 <= int(target.get("port", 0)) <= 65535
            ):
                raise ConfigurationError("UDP forwarding needs host and port")

    def put(self, item):
        self.validate(item)
        value = dict(item)
        updated = [i for i in self.items if i["topic"] != item["topic"]] + [value]
        if len(updated) > 256:
            raise ConfigurationError("At most 256 configured topics are supported")
        atomic_json(self.path, updated)
        self.items = updated
        return value

    def delete(self, name):
        updated = [i for i in self.items if i["topic"] != name]
        atomic_json(self.path, updated)
        self.items = updated

    def selected(self):
        return [
            dict(i)
            for i in self.items
            if i.get("enabled", True) and "record" in i.get("outputs", [])
        ]


def public_publish_allowed(topic: str) -> bool:
    # Arbitrary ROS publication must never become a second actuator path.
    return topic.startswith("/external/") and not any(
        segment in {"action", "control", "internal", "io_teleop"}
        for segment in topic.split("/")
    )

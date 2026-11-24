"""URDF XML 解析与数值/关节索引校验工具。"""
from __future__ import annotations

import math
from typing import Any
from xml.etree import ElementTree

MAX_URDF_BYTES = 16 * 1024 * 1024


class RobotProfileError(ValueError):
    pass


def _number_list(value: Any, length: int, path: str) -> list[float]:
    if not isinstance(value, list) or len(value) != length:
        raise RobotProfileError(f"{path} must contain {length} numbers")
    result = []
    for item in value:
        if isinstance(item, bool) or not isinstance(item, (int, float)):
            raise RobotProfileError(f"{path} must contain {length} numbers")
        number = float(item)
        if not math.isfinite(number):
            raise RobotProfileError(f"{path} must contain finite numbers")
        result.append(number)
    return result


def _index(value: Any, count: int, path: str, *, root_allowed: bool = False) -> int:
    if isinstance(value, bool) or not isinstance(value, int):
        raise RobotProfileError(f"{path} must be an integer")
    minimum = -1 if root_allowed else 0
    if value < minimum or value >= count:
        raise RobotProfileError(f"{path}={value} is outside [{minimum}, {count - 1}]")
    return value


def _index_list(
    value: Any,
    count: int,
    path: str,
    *,
    root_allowed: bool = False,
) -> list[int]:
    if not isinstance(value, list):
        raise RobotProfileError(f"{path} must be an integer list")
    return [
        _index(item, count, f"{path}[{index}]", root_allowed=root_allowed)
        for index, item in enumerate(value)
    ]


class UrdfModel:
    """解析并校验机器人 URDF 的连杆、关节及限位结构。"""

    def __init__(self, payload: bytes):
        if not payload or len(payload) > MAX_URDF_BYTES:
            raise RobotProfileError(
                f"URDF must be between 1 byte and {MAX_URDF_BYTES // 1024 // 1024} MiB"
            )
        try:
            text = payload.decode("utf-8-sig")
        except UnicodeDecodeError as exc:
            raise RobotProfileError("URDF must be UTF-8 text") from exc
        if "<!DOCTYPE" in text.upper() or "<!ENTITY" in text.upper():
            raise RobotProfileError("URDF document types and entities are not allowed")
        try:
            root = ElementTree.fromstring(text)
        except ElementTree.ParseError as exc:
            raise RobotProfileError(f"invalid URDF XML: {exc}") from exc
        if root.tag.split("}")[-1] != "robot":
            raise RobotProfileError("URDF root element must be <robot>")

        links = [item.get("name", "").strip() for item in root.findall("link")]
        if not links or any(not name for name in links) or len(set(links)) != len(links):
            raise RobotProfileError("URDF link names must be present and unique")
        link_set = set(links)
        joints = []
        child_links = set()
        for item_index, item in enumerate(root.findall("joint")):
            name = item.get("name", "").strip()
            joint_type = item.get("type", "").strip()
            parent = item.find("parent")
            child = item.find("child")
            parent_name = "" if parent is None else str(parent.get("link", "")).strip()
            child_name = "" if child is None else str(child.get("link", "")).strip()
            if (
                not name
                or not joint_type
                or parent_name not in link_set
                or child_name not in link_set
            ):
                raise RobotProfileError(
                    f"URDF joint[{item_index}] requires a name, type and valid parent/child links"
                )
            limit = item.find("limit")
            velocity = None
            lower = None
            upper = None
            if limit is not None and limit.get("velocity") not in (None, ""):
                try:
                    velocity = float(str(limit.get("velocity")))
                except ValueError as exc:
                    raise RobotProfileError(
                        f"URDF joint {name} has an invalid velocity limit"
                    ) from exc
                if not math.isfinite(velocity) or velocity < 0.0:
                    raise RobotProfileError(
                        f"URDF joint {name} velocity limit must be non-negative"
                    )
            if limit is not None:
                try:
                    lower = (
                        float(str(limit.get("lower")))
                        if limit.get("lower") not in (None, "")
                        else None
                    )
                    upper = (
                        float(str(limit.get("upper")))
                        if limit.get("upper") not in (None, "")
                        else None
                    )
                except ValueError as exc:
                    raise RobotProfileError(
                        f"URDF joint {name} has invalid position limits"
                    ) from exc
                if any(
                    value is not None and not math.isfinite(value)
                    for value in (lower, upper)
                ):
                    raise RobotProfileError(
                        f"URDF joint {name} position limits must be finite"
                    )
            joints.append(
                {
                    "name": name,
                    "type": joint_type,
                    "parent": parent_name,
                    "child": child_name,
                    "velocity": velocity,
                    "lower": lower,
                    "upper": upper,
                }
            )
            child_links.add(child_name)
        if len({joint["name"] for joint in joints}) != len(joints):
            raise RobotProfileError("URDF joint names must be unique")

        roots = [name for name in links if name not in child_links]
        if len(roots) != 1:
            raise RobotProfileError("URDF must contain exactly one root link")
        self.robot_name = str(root.get("name", "")).strip() or "unnamed"
        self.links = links
        self.joints = joints
        self.root_link = roots[0]
        self.movable_joint_names = {
            joint["name"] for joint in joints if joint["type"] != "fixed"
        }

    def link_at_joint_index(self, index: int) -> str:
        return self.root_link if index == -1 else str(self.joints[index]["child"])


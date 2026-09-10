from __future__ import annotations

import copy
import io
import math
import os
import re
import shutil
import tempfile
import zipfile
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Iterable
from xml.etree import ElementTree

import yaml


STANDARD_TOPICS = {
    "vr_frame": "/vrdata",
    "joint_state": "/hc_teleop/joint_states",
    "joint_target": "/hc_teleop/joint_cmd_arm",
    "joint_command": "/hc_teleop/joint_cmd",
    "ee_target": "/hc_teleop/controller_target_ee_poses",
    "ee_visual_target": "/hc_teleop/target_ee_poses",
    "ee_actual": "/hc_teleop/actual_ee_poses",
    "solver_state": "/hc_teleop/sol_q",
    "solver_reset": "/hc_teleop/solver_reset",
    "solver_reset_ack": "/hc_teleop/solver_reset_ack",
    "base_move": "/hc_teleop/target_base_move",
}

PROFILE_ID_PATTERN = re.compile(r"^[A-Za-z0-9][A-Za-z0-9_.-]{0,63}$")
MAX_URDF_BYTES = 16 * 1024 * 1024
MAX_CONFIG_BYTES = 2 * 1024 * 1024
MAX_ARCHIVE_BYTES = 100 * 1024 * 1024
MAX_UNCOMPRESSED_BYTES = 300 * 1024 * 1024


class RobotProfileError(ValueError):
    pass


def validate_profile_id(value: str) -> str:
    value = str(value).strip()
    if not PROFILE_ID_PATTERN.fullmatch(value):
        raise RobotProfileError(
            "profile id must be 1-64 characters using letters, numbers, '.', '_' or '-'"
        )
    return value


def _safe_filename(value: str, suffixes: set[str]) -> str:
    name = str(value).strip()
    if not name or Path(name).name != name or "\\" in name:
        raise RobotProfileError("uploaded filename must not contain a directory")
    if Path(name).suffix.lower() not in suffixes:
        allowed = ", ".join(sorted(suffixes))
        raise RobotProfileError(f"unsupported file extension; expected {allowed}")
    return name


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
            if not name or not joint_type or parent_name not in link_set or child_name not in link_set:
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


def _load_yaml(payload: bytes) -> dict[str, Any]:
    if not payload or len(payload) > MAX_CONFIG_BYTES:
        raise RobotProfileError(
            f"YAML must be between 1 byte and {MAX_CONFIG_BYTES // 1024 // 1024} MiB"
        )
    try:
        value = yaml.safe_load(payload.decode("utf-8-sig"))
    except UnicodeDecodeError as exc:
        raise RobotProfileError("YAML must be UTF-8 text") from exc
    except yaml.YAMLError as exc:
        raise RobotProfileError(f"invalid YAML: {exc}") from exc
    if not isinstance(value, dict):
        raise RobotProfileError("YAML root must be an object")
    return value


def _standard_ros_interface(profile_id: str) -> dict[str, Any]:
    return {
        "node_name": f"controller_v2_3_{profile_id.replace('-', '_').replace('.', '_')}",
        "rate": 100,
        "sub_topic": {
            "joint_state": STANDARD_TOPICS["joint_state"],
            "ee_target": STANDARD_TOPICS["ee_target"],
            "reset": STANDARD_TOPICS["solver_reset"],
        },
        "pub_topic": {
            "joint_target": STANDARD_TOPICS["joint_target"],
            "solver_state": STANDARD_TOPICS["solver_state"],
            "reset_ack": STANDARD_TOPICS["solver_reset_ack"],
        },
    }


def _validate_task_frames(config: dict[str, Any], model: UrdfModel) -> int:
    task = config.get("task", {})
    if not isinstance(task, dict):
        raise RobotProfileError("task must be an object")
    count = 0
    for kind in ("pose", "axis"):
        values = task.get(kind, []) or []
        if not isinstance(values, list):
            raise RobotProfileError(f"task.{kind} must be a list")
        for index, item in enumerate(values):
            if not isinstance(item, list) or not item or not isinstance(item[0], list):
                raise RobotProfileError(f"task.{kind}[{index}] has an invalid structure")
            frames = item[0]
            if len(frames) != 2 or any(frame not in model.links for frame in frames):
                raise RobotProfileError(
                    f"task.{kind}[{index}] must reference two existing URDF links"
                )
            count += 1
    joint_tasks = task.get("joint", []) or []
    if not isinstance(joint_tasks, list):
        raise RobotProfileError("task.joint must be a list")
    return count + len(joint_tasks)


def _normalize_v23(
    profile_id: str,
    config: dict[str, Any],
    model: UrdfModel,
    urdf_name: str,
) -> tuple[dict[str, Any], dict[str, Any]]:
    normalized = copy.deepcopy(config)
    model_config = normalized.get("model")
    if not isinstance(model_config, dict):
        raise RobotProfileError("controller YAML requires model object")
    free_joints = model_config.get("free_joints")
    if not isinstance(free_joints, list) or not free_joints:
        raise RobotProfileError("model.free_joints must be a non-empty list")
    if len(set(free_joints)) != len(free_joints):
        raise RobotProfileError("model.free_joints must not contain duplicates")
    unknown = [name for name in free_joints if name not in model.movable_joint_names]
    if unknown:
        raise RobotProfileError(
            "model.free_joints are missing from movable URDF joints: " + ", ".join(unknown)
        )
    task_count = _validate_task_frames(normalized, model)
    if task_count == 0:
        raise RobotProfileError("controller YAML must define at least one task")
    limit_config = normalized.get("limit", {})
    if not isinstance(limit_config, dict):
        raise RobotProfileError("limit must be an object")
    velocity = limit_config.get("velocity", [])
    if velocity:
        velocity = _number_list(
            velocity, len(free_joints), "limit.velocity"
        )
        if any(value < 0.0 for value in velocity):
            raise RobotProfileError("limit.velocity values must be non-negative")
        limit_config["velocity"] = velocity

    model_config["urdf"] = urdf_name if "/" in urdf_name else f"urdf/{urdf_name}"
    normalized["ros_interface"] = _standard_ros_interface(profile_id)
    control = normalized.setdefault("control", {})
    if not isinstance(control, dict):
        raise RobotProfileError("control must be an object")
    control.setdefault("method", "control")
    control.setdefault("dt", 0.01)
    control.setdefault("damping", 1.0e-5)
    return normalized, {
        "schema": "controller-v23",
        "free_joint_count": len(free_joints),
        "task_count": task_count,
        "arm_count": len(normalized.get("task", {}).get("pose", []) or []),
        "warnings": [],
    }


def _component_joint_indices(
    config: dict[str, Any], model: UrdfModel
) -> Iterable[int]:
    for key in ("folding_waist", "waist", "dorsal", "head"):
        component = config.get(key)
        if not isinstance(component, dict) or "joint_index" not in component:
            continue
        for index in _index_list(
            component["joint_index"],
            len(model.joints),
            f"{key}.joint_index",
            root_allowed=True,
        ):
            if index >= 0:
                yield index


def _generate_arm_teleop_config(
    normalized: dict[str, Any],
    model: UrdfModel,
    urdf_name: str,
    arm_joint_indices: list[list[int]],
    arm_ee_indices: list[int],
    bases: list[int],
) -> dict[str, Any] | None:
    if len(arm_joint_indices) != 2:
        return None
    root_dir = Path(__file__).resolve().parents[2]
    template_path = root_dir / "adapters" / "robots" / "x1" / "arm_teleop.yaml"
    if not template_path.is_file():
        template_path = root_dir / "arm_teleop.yaml"
    try:
        template = yaml.safe_load(template_path.read_text(encoding="utf-8"))
    except (OSError, yaml.YAMLError) as exc:
        raise RobotProfileError(f"unable to load arm teleop template: {exc}") from exc
    if not isinstance(template, dict):
        raise RobotProfileError("arm teleop template is invalid")
    result = copy.deepcopy(template)
    robot = result["robot"]
    robot["urdf_path"] = f"urdf/{urdf_name}"
    base_pose = normalized.get("base_pose", {})
    robot["base_position"] = list(base_pose.get("position", [0.0, 0.0, 0.0]))
    robot["base_orientation"] = list(
        base_pose.get("orientation", [0.0, 0.0, 0.0, 1.0])
    )
    robot["initial_joints"] = {}

    for side, arm_index in (("right", 0), ("left", 1)):
        source = normalized["arms"][arm_index]
        joint_indices = arm_joint_indices[arm_index]
        joint_names = [str(model.joints[index]["name"]) for index in joint_indices]
        rest = list(source.get("rest_j_pos", [0.0] * len(joint_names)))
        robot["initial_joints"].update(zip(joint_names, rest))
        base_link = model.link_at_joint_index(bases[arm_index])
        ee_link = model.link_at_joint_index(arm_ee_indices[arm_index])
        result["arms"][side].update(
            {
                "base_link": base_link,
                "generic_task_base_link": base_link,
                "ee_link": ee_link,
                "joint_names": joint_names,
            }
        )

    body_source = None
    for key in ("folding_waist", "waist", "dorsal"):
        if isinstance(normalized.get(key), dict):
            body_source = normalized[key]
            break
    body_indices = []
    body_rest = []
    if body_source is not None and "joint_index" in body_source:
        all_body_indices = _index_list(
            body_source["joint_index"],
            len(model.joints),
            "body.joint_index",
            root_allowed=True,
        )
        all_body_rest = _number_list(
            body_source.get("rest_j_pos", [0.0] * len(all_body_indices)),
            len(all_body_indices),
            "body.rest_j_pos",
        )
        movable_pairs = [
            (index, rest)
            for index, rest in zip(all_body_indices, all_body_rest)
            if index >= 0 and model.joints[index]["name"] in model.movable_joint_names
        ]
        body_indices = [index for index, _rest in movable_pairs]
        body_rest = [rest for _index, rest in movable_pairs]
    body_names = [str(model.joints[index]["name"]) for index in body_indices]
    robot["initial_joints"].update(zip(body_names, body_rest))
    torso_index = (
        int(body_source["cmd_ee"])
        if body_source is not None and "cmd_ee" in body_source
        else bases[0]
    )
    body = result["body"]
    body["enabled"] = bool(body_names)
    body["waist_joint_names"] = body_names
    body["torso_link"] = model.link_at_joint_index(torso_index)
    body["joint_lower"] = [
        model.joints[index]["lower"]
        if model.joints[index]["lower"] is not None
        else -math.pi
        for index in body_indices
    ]
    body["joint_upper"] = [
        model.joints[index]["upper"]
        if model.joints[index]["upper"] is not None
        else math.pi
        for index in body_indices
    ]

    grippers = result["grippers"]
    grippers["include_sim_joints"] = False
    for side in ("right", "left"):
        grippers[side]["finger_joint_names"] = []
        grippers[side]["finger_open"] = []
        grippers[side]["finger_closed"] = []
    result["control"]["backend"] = "v23"
    return result


def _normalize_io(
    profile_id: str,
    config: dict[str, Any],
    model: UrdfModel,
    urdf_name: str,
) -> tuple[dict[str, Any], dict[str, Any], dict[str, Any] | None, dict[str, Any]]:
    normalized = copy.deepcopy(config)
    arms = normalized.get("arms")
    if not isinstance(arms, list) or not arms:
        raise RobotProfileError("HC robot YAML requires a non-empty arms list")
    joint_count = len(model.joints)
    arm_joint_indices: list[list[int]] = []
    arm_ee_indices: list[int] = []
    for arm_index, arm in enumerate(arms):
        if not isinstance(arm, dict):
            raise RobotProfileError(f"arms[{arm_index}] must be an object")
        indices = _index_list(
            arm.get("joint_index"), joint_count, f"arms[{arm_index}].joint_index"
        )
        if not indices:
            raise RobotProfileError(f"arms[{arm_index}].joint_index must not be empty")
        fixed = [
            model.joints[index]["name"]
            for index in indices
            if model.joints[index]["name"] not in model.movable_joint_names
        ]
        if fixed:
            raise RobotProfileError(
                f"arms[{arm_index}].joint_index contains fixed joints: "
                + ", ".join(fixed)
            )
        arm_joint_indices.append(indices)
        arm_ee_indices.append(
            _index(arm.get("ee_index"), joint_count, f"arms[{arm_index}].ee_index")
        )
        if "rest_j_pos" in arm:
            _number_list(arm["rest_j_pos"], len(indices), f"arms[{arm_index}].rest_j_pos")
    if "base_pose" in normalized:
        base_pose = normalized["base_pose"]
        if not isinstance(base_pose, dict):
            raise RobotProfileError("base_pose must be an object")
        _number_list(base_pose.get("position"), 3, "base_pose.position")
        _number_list(base_pose.get("orientation"), 4, "base_pose.orientation")

    controller_indices = normalized.get("controller_indices", {})
    if not isinstance(controller_indices, dict):
        raise RobotProfileError("controller_indices must be an object")
    command_ee = controller_indices.get("cmd_ee", arm_ee_indices)
    command_ee = _index_list(command_ee, joint_count, "controller_indices.cmd_ee")
    bases = controller_indices.get("base", [-1] * len(arms))
    bases = _index_list(
        bases, joint_count, "controller_indices.base", root_allowed=True
    )
    if len(command_ee) != len(arms) or len(bases) != len(arms):
        raise RobotProfileError(
            "controller_indices.cmd_ee/base counts must match the arms count"
        )
    normalized["controller_indices"] = {**controller_indices, "cmd_ee": command_ee, "base": bases}
    normalized["urdf_path"] = urdf_name if "/" in urdf_name else f"urdf/{urdf_name}"
    normalized["robot_name"] = profile_id

    free_indices = []
    for indices in arm_joint_indices:
        free_indices.extend(indices)
    free_indices.extend(_component_joint_indices(normalized, model))
    free_indices = list(dict.fromkeys(free_indices))
    free_indices = [
        index
        for index in free_indices
        if model.joints[index]["name"] in model.movable_joint_names
    ]
    free_joints = [str(model.joints[index]["name"]) for index in free_indices]
    if not free_joints:
        raise RobotProfileError("HC robot YAML does not select any movable URDF joints")

    pose_tasks = [
        [
            [model.link_at_joint_index(base), model.link_at_joint_index(ee)],
            5.0,
            1.0,
            3.0,
        ]
        for base, ee in zip(bases, command_ee)
    ]
    folding = normalized.get("folding_waist")
    if isinstance(folding, dict) and "base" in folding and "cmd_ee" in folding:
        body_base = _index(
            folding["base"], joint_count, "folding_waist.base", root_allowed=True
        )
        body_ee = _index(folding["cmd_ee"], joint_count, "folding_waist.cmd_ee")
        pose_tasks.append(
            [
                [
                    model.link_at_joint_index(body_base),
                    model.link_at_joint_index(body_ee),
                ],
                5.0,
                1.0,
                1.0,
            ]
        )

    velocity = []
    for index in free_indices:
        limit = model.joints[index]["velocity"]
        velocity.append(round(math.degrees(limit), 6) if limit is not None else 180.0)
    controller = {
        "ros_interface": _standard_ros_interface(profile_id),
        "control": {"method": "control", "dt": 0.01, "damping": 1.0e-5, "joints_init": []},
        "model": {"urdf": urdf_name if "/" in urdf_name else f"urdf/{urdf_name}", "free_joints": free_joints},
        "task": {"pose": pose_tasks, "axis": [], "joint": []},
        "limit": {"velocity": velocity},
    }
    warnings = []
    if len(arms) != 2:
        warnings.append(
            "the current VR adapter expects two arms; this profile can be stored but needs adapter configuration"
        )
    if not isinstance(folding, dict):
        warnings.append(
            "no folding_waist section; generated controller contains arm tasks only"
        )
    teleop = _generate_arm_teleop_config(
        normalized,
        model,
        urdf_name,
        arm_joint_indices,
        command_ee,
        bases,
    )
    return normalized, controller, teleop, {
        "schema": "hc-robot-config-v1",
        "free_joint_count": len(free_joints),
        "task_count": len(pose_tasks),
        "arm_count": len(arms),
        "warnings": warnings,
    }


def _normalize_teleop(
    profile_id: str,
    config: dict[str, Any],
    model: UrdfModel,
    urdf_name: str,
) -> tuple[dict[str, Any], dict[str, Any], dict[str, Any], dict[str, Any]]:
    teleop = copy.deepcopy(config)
    arms_config = teleop.get("arms")
    if not isinstance(arms_config, (dict, list)) or not arms_config:
        raise RobotProfileError("arm_teleop.yaml requires a non-empty arms section")

    robot_config = teleop.get("robot", {})
    if not isinstance(robot_config, dict):
        robot_config = {}
        teleop["robot"] = robot_config
    base_pos = list(robot_config.get("base_position", [0.0, 0.0, 0.0]))
    base_ori = list(robot_config.get("base_orientation", [0.0, 0.0, 0.0, 1.0]))
    initial_joints = robot_config.get("initial_joints")
    if not isinstance(initial_joints, dict):
        initial_joints = {}

    arm_items = []
    if isinstance(arms_config, dict):
        for side in ("right", "left"):
            if side in arms_config:
                arm_items.append((side, arms_config[side]))
        for side, arm in arms_config.items():
            if side not in ("right", "left"):
                arm_items.append((side, arm))
    else:
        for index, arm in enumerate(arms_config):
            side = "right" if index == 0 else "left" if index == 1 else f"arm_{index}"
            arm_items.append((side, arm))

    free_joints: list[str] = []
    tasks = []
    vr_arms = []
    cmd_ee_list = []
    base_list = []
    rel_urdf_path = urdf_name if "/" in urdf_name else f"urdf/{urdf_name}"
    joint_set = {j["name"] for j in model.joints}
    link_set = set(model.links)

    for side, arm in arm_items:
        if not isinstance(arm, dict):
            raise RobotProfileError(f"arms.{side} must be an object")
        joint_names = arm.get("joint_names", [])
        if not isinstance(joint_names, list) or not joint_names:
            raise RobotProfileError(f"arms.{side}.joint_names must not be empty")
        for jname in joint_names:
            if str(jname) not in joint_set:
                raise RobotProfileError(f"arms.{side} references missing URDF joint: '{jname}'")
        joint_names = [str(j) for j in joint_names]
        free_joints.extend(joint_names)

        ee_link = str(arm.get("ee_link", "")).strip()
        if not ee_link or ee_link not in link_set:
            raise RobotProfileError(f"arms.{side}.ee_link '{ee_link}' is missing from URDF")

        raw_task_base = str(arm.get("generic_task_base_link") or arm.get("base_link") or "").strip()
        task_base = raw_task_base if raw_task_base in link_set else model.root_link
        tasks.append([[task_base, ee_link], 5.0, 1.0, 3.0])

        rest_pos = [float(initial_joints.get(j, 0.0)) for j in joint_names]
        vr_arms.append({
            "joint_index": joint_names,
            "ee_index": ee_link,
            "rest_j_pos": rest_pos,
        })
        cmd_ee_list.append(ee_link)
        base_link = str(arm.get("base_link", "base")).strip()
        base_list.append(-1 if base_link in ("-1", "base", "root", model.root_link) else base_link)

    body = teleop.get("body", {})
    if isinstance(body, dict) and body.get("enabled", False):
        for wj in body.get("waist_joint_names", []):
            if wj in model.movable_joint_names and wj not in free_joints:
                free_joints.append(wj)

    free_joints = list(dict.fromkeys(free_joints))

    velocity_limits = []
    joint_map = {j["name"]: j for j in model.joints}
    for jname in free_joints:
        jinfo = joint_map.get(jname)
        if jinfo and jinfo.get("velocity") is not None:
            velocity_limits.append(round(math.degrees(float(jinfo["velocity"])), 2))
        else:
            velocity_limits.append(60.0)

    controller = {
        "ros_interface": _standard_ros_interface(profile_id),
        "control": {"method": "control", "dt": 0.01, "damping": 1.0e-5, "joints_init": []},
        "model": {"urdf": rel_urdf_path, "free_joints": free_joints},
        "task": {"pose": tasks, "axis": [], "joint": []},
        "limit": {"velocity": velocity_limits},
    }

    grippers_config = teleop.get("grippers", {})
    vr_grippers = []
    finger_topics = []
    if isinstance(grippers_config, dict):
        for side, _arm in arm_items:
            gripper = grippers_config.get(side, {})
            if isinstance(gripper, dict):
                finger_joints = gripper.get("finger_joint_names", [])
                if finger_joints:
                    vr_grippers.append({"joint_index": list(finger_joints)})
                topic = gripper.get("finger_topic")
                if topic:
                    finger_topics.append(str(topic))
    elif isinstance(grippers_config, list):
        for item in grippers_config:
            if isinstance(item, dict) and "joint_index" in item:
                vr_grippers.append({"joint_index": item["joint_index"]})

    sim_settings = teleop.get("simulation", {})
    if not isinstance(sim_settings, dict):
        sim_settings = {}
    io_config = {
        "urdf_path": rel_urdf_path,
        "robot_name": profile_id,
        "wo_controller": False,
        "joint_index_order": "pybullet",
        "base_pose": {
            "position": base_pos,
            "orientation": base_ori,
        },
        "sim_camera": sim_settings.get("sim_camera", {
            "distance": 2.5,
            "yaw": 45,
            "pitch": -25,
            "target_position": [0.0, 0.0, 0.5],
        }),
        "sim_opengl2": True,
        "arms": vr_arms,
        "grippers": vr_grippers,
        "controller_indices": {
            "cmd_ee": cmd_ee_list,
            "base": base_list,
        },
        "vibration_thresholds": {
            "ee_dist": [0.1, 0.15],
        },
        "arm_pos_scale": 1.0,
        "ee_wrt_head": False,
    }
    if finger_topics:
        io_config["finger_command_topics"] = finger_topics

    summary = {
        "schema": "arm-teleop-v1",
        "free_joint_count": len(free_joints),
        "task_count": len(tasks),
        "arm_count": len(arm_items),
        "warnings": [],
    }
    return teleop, controller, io_config, summary


class RobotProfileManager:
    def __init__(self, root: str | Path):
        self.root = Path(root).expanduser().resolve()
        self.root.mkdir(parents=True, exist_ok=True)

    def _path(self, profile_id: str) -> Path:
        return self.root / validate_profile_id(profile_id)

    def _ensure_profile_files(self, path: Path, profile_id: str) -> None:
        teleop_path = path / "arm_teleop.yaml"
        controller_path = path / "controller_v23.yml"
        vr_path = path / "vr_configs.yml"
        metadata_path = path / "profile.yaml"

        urdf_files = sorted(path.glob("**/*.urdf"), key=lambda p: len(p.parts))
        if not urdf_files:
            return
        urdf_file = urdf_files[0]
        rel_urdf = urdf_file.relative_to(path).as_posix()

        try:
            model = UrdfModel(urdf_file.read_bytes())
        except Exception:
            return

        if teleop_path.is_file():
            try:
                teleop_data = _load_yaml(teleop_path.read_bytes())
                if isinstance(teleop_data, dict) and isinstance(teleop_data.get("arms"), (dict, list)):
                    _teleop, controller, io_config, summary = _normalize_teleop(
                        profile_id, teleop_data, model, rel_urdf
                    )
                    if not controller_path.is_file():
                        controller_path.write_text(
                            yaml.safe_dump(controller, allow_unicode=True, sort_keys=False),
                            encoding="utf-8",
                        )
                    if not vr_path.is_file():
                        vr_path.write_text(
                            yaml.safe_dump(io_config, allow_unicode=True, sort_keys=False),
                            encoding="utf-8",
                        )
                    if not metadata_path.is_file():
                        metadata = {
                            "version": 1,
                            "id": profile_id,
                            "display_name": profile_id,
                            "schema": summary["schema"],
                            "managed": True,
                            "imported_at": datetime.now(timezone.utc).isoformat(),
                            "robot_name": model.robot_name,
                            "urdf": rel_urdf,
                            "primary_config": "controller_v23.yml" if controller_path.is_file() else "arm_teleop.yaml",
                            "config_files": [
                                f for f in ("controller_v23.yml", "vr_configs.yml", "arm_teleop.yaml")
                                if (path / f).is_file()
                            ],
                            "link_count": len(model.links),
                            "joint_count": len(model.joints),
                            "movable_joint_count": len(model.movable_joint_names),
                            "free_joint_count": summary["free_joint_count"],
                            "task_count": summary["task_count"],
                            "arm_count": summary["arm_count"],
                            "teleop_compatible": True,
                            "warnings": summary["warnings"],
                        }
                        metadata_path.write_text(
                            yaml.safe_dump(metadata, allow_unicode=True, sort_keys=False),
                            encoding="utf-8",
                        )
                    return
            except Exception:
                pass

        if vr_path.is_file() and not metadata_path.is_file():
            try:
                io_data = _load_yaml(vr_path.read_bytes())
                if isinstance(io_data, dict) and "arms" in io_data:
                    io_config, controller, teleop, summary = _normalize_io(
                        profile_id, io_data, model, rel_urdf
                    )
                    if not controller_path.is_file():
                        controller_path.write_text(
                            yaml.safe_dump(controller, allow_unicode=True, sort_keys=False),
                            encoding="utf-8",
                        )
                    metadata = {
                        "version": 1,
                        "id": profile_id,
                        "display_name": profile_id,
                        "schema": summary["schema"],
                        "managed": True,
                        "imported_at": datetime.now(timezone.utc).isoformat(),
                        "robot_name": model.robot_name,
                        "urdf": rel_urdf,
                        "primary_config": "controller_v23.yml" if controller_path.is_file() else "vr_configs.yml",
                        "config_files": [
                            f for f in ("controller_v23.yml", "vr_configs.yml", "arm_teleop.yaml")
                            if (path / f).is_file()
                        ],
                        "link_count": len(model.links),
                        "joint_count": len(model.joints),
                        "movable_joint_count": len(model.movable_joint_names),
                        "free_joint_count": summary["free_joint_count"],
                        "task_count": summary["task_count"],
                        "arm_count": summary["arm_count"],
                        "teleop_compatible": (path / "arm_teleop.yaml").is_file(),
                        "warnings": summary["warnings"],
                    }
                    metadata_path.write_text(
                        yaml.safe_dump(metadata, allow_unicode=True, sort_keys=False),
                        encoding="utf-8",
                    )
            except Exception:
                pass

    def get(self, profile_id: str) -> dict[str, Any]:
        path = self._path(profile_id)
        if not path.is_dir():
            raise RobotProfileError(f"robot profile does not exist: {profile_id}")
        self._ensure_profile_files(path, profile_id)
        metadata_path = path / "profile.yaml"
        if metadata_path.is_file():
            try:
                metadata = yaml.safe_load(metadata_path.read_text(encoding="utf-8"))
            except (OSError, yaml.YAMLError) as exc:
                raise RobotProfileError(f"unable to read {metadata_path}: {exc}") from exc
            if not isinstance(metadata, dict):
                raise RobotProfileError(f"invalid profile metadata: {metadata_path}")
        else:
            config_name = (
                "controller_v23.yml"
                if (path / "controller_v23.yml").is_file()
                else "arm_teleop.yaml"
                if (path / "arm_teleop.yaml").is_file()
                else "vr_configs.yml"
                if (path / "vr_configs.yml").is_file()
                else ""
            )
            if not config_name:
                raise RobotProfileError(f"profile has no supported YAML: {profile_id}")
            metadata = {
                "version": 1,
                "id": profile_id,
                "display_name": profile_id,
                "schema": "controller-v23" if config_name.startswith("controller") else "hc-robot-config-v1",
                "primary_config": config_name,
                "managed": False,
                "warnings": ["legacy profile without profile.yaml metadata"],
            }
        if "link_count" not in metadata:
            urdf_files = sorted(path.glob("**/*.urdf"), key=lambda p: len(p.parts))
            if urdf_files:
                try:
                    m = UrdfModel(urdf_files[0].read_bytes())
                    metadata.setdefault("link_count", len(m.links))
                    metadata.setdefault("joint_count", len(m.joints))
                    metadata.setdefault("movable_joint_count", len(m.movable_joint_names))
                    metadata.setdefault("robot_name", m.robot_name)
                    metadata.setdefault("urdf", urdf_files[0].relative_to(path).as_posix())
                except Exception:
                    pass
        metadata = copy.deepcopy(metadata)
        metadata["id"] = profile_id
        metadata["managed"] = bool(metadata.get("managed", metadata_path.is_file()))
        metadata["path"] = str(path)
        metadata["standard_topics"] = dict(STANDARD_TOPICS)
        return metadata

    def list(self) -> list[dict[str, Any]]:
        profiles = []
        for path in sorted(self.root.iterdir(), key=lambda item: item.name.lower()):
            if not path.is_dir() or path.name.startswith("."):
                continue
            try:
                profiles.append(self.get(path.name))
            except RobotProfileError as exc:
                profiles.append(
                    {
                        "id": path.name,
                        "display_name": path.name,
                        "schema": "invalid",
                        "managed": False,
                        "path": str(path),
                        "warnings": [str(exc)],
                        "standard_topics": dict(STANDARD_TOPICS),
                    }
                )
        return profiles

    def delete_profile(self, profile_id: str) -> None:
        profile_id = validate_profile_id(profile_id)
        target = self._path(profile_id)
        if not target.exists() or not target.is_dir():
            raise RobotProfileError(f"robot profile does not exist: {profile_id}")
        shutil.rmtree(target)

    def import_profile(
        self,
        profile_id: str,
        display_name: str,
        urdf_filename: str,
        urdf_payload: bytes,
        config_filename: str,
        config_payload: bytes,
        overwrite: bool = True,
    ) -> dict[str, Any]:
        profile_id = validate_profile_id(profile_id)
        display_name = str(display_name).strip() or profile_id
        if len(display_name) > 80:
            raise RobotProfileError("display name must not exceed 80 characters")
        urdf_filename = _safe_filename(urdf_filename, {".urdf", ".xml"})
        _safe_filename(config_filename, {".yaml", ".yml"})
        target = self._path(profile_id)
        if target.exists() and not overwrite:
            raise RobotProfileError(f"robot profile already exists: {profile_id}")

        model = UrdfModel(urdf_payload)
        source_config = _load_yaml(config_payload)
        files: dict[str, dict[str, Any]] = {}
        if isinstance(source_config.get("arms"), (dict, list)) and (
            "robot" in source_config or "grippers" in source_config or "control" in source_config
        ):
            teleop, controller, io_config, summary = _normalize_teleop(
                profile_id, source_config, model, urdf_filename
            )
            files["arm_teleop.yaml"] = teleop
            files["controller_v23.yml"] = controller
            files["vr_configs.yml"] = io_config
            primary_config = "controller_v23.yml"
        elif isinstance(source_config.get("model"), dict) and "task" in source_config:
            controller, summary = _normalize_v23(
                profile_id, source_config, model, urdf_filename
            )
            files["controller_v23.yml"] = controller
            primary_config = "controller_v23.yml"
        elif "urdf_path" in source_config and "arms" in source_config:
            io_config, controller, teleop, summary = _normalize_io(
                profile_id, source_config, model, urdf_filename
            )
            files["vr_configs.yml"] = io_config
            files["controller_v23.yml"] = controller
            if teleop is not None:
                files["arm_teleop.yaml"] = teleop
            primary_config = "vr_configs.yml"
        else:
            raise RobotProfileError("unsupported YAML schema in profile")

        metadata = {
            "version": 1,
            "id": profile_id,
            "display_name": display_name,
            "schema": summary["schema"],
            "managed": True,
            "imported_at": datetime.now(timezone.utc).isoformat(),
            "robot_name": model.robot_name,
            "urdf": urdf_filename,
            "primary_config": primary_config,
            "config_files": list(files),
            "link_count": len(model.links),
            "joint_count": len(model.joints),
            "movable_joint_count": len(model.movable_joint_names),
            "free_joint_count": summary["free_joint_count"],
            "task_count": summary["task_count"],
            "arm_count": summary["arm_count"],
            "teleop_compatible": "arm_teleop.yaml" in files,
            "warnings": summary["warnings"],
        }

        temp_path = Path(tempfile.mkdtemp(prefix=f".{profile_id}-", dir=self.root))
        try:
            (temp_path / "urdf").mkdir()
            (temp_path / "urdf" / urdf_filename).write_bytes(urdf_payload)
            for filename, value in files.items():
                (temp_path / filename).write_text(
                    yaml.safe_dump(value, allow_unicode=True, sort_keys=False),
                    encoding="utf-8",
                )
            (temp_path / "profile.yaml").write_text(
                yaml.safe_dump(metadata, allow_unicode=True, sort_keys=False),
                encoding="utf-8",
            )
            if target.exists():
                shutil.rmtree(target)
            os.rename(temp_path, target)
        except Exception:
            shutil.rmtree(temp_path, ignore_errors=True)
            raise
        return self.get(profile_id)

    def import_archive(
        self,
        profile_id: str,
        display_name: str,
        archive_payload: bytes,
        archive_filename: str = "",
        overwrite: bool = True,
    ) -> dict[str, Any]:
        if not archive_payload or len(archive_payload) > MAX_ARCHIVE_BYTES:
            raise RobotProfileError(
                f"archive must be between 1 byte and {MAX_ARCHIVE_BYTES // 1024 // 1024} MiB"
            )
        if not zipfile.is_zipfile(io.BytesIO(archive_payload)):
            raise RobotProfileError("uploaded file must be a valid .zip archive")

        with zipfile.ZipFile(io.BytesIO(archive_payload), "r") as zf:
            infolist = zf.infolist()
            if not infolist:
                raise RobotProfileError("zip archive is empty")

            total_size = 0
            cleaned_entries: list[tuple[zipfile.ZipInfo, Path]] = []
            for info in infolist:
                name = info.filename.replace("\\", "/").strip()
                if not name or name.startswith("/") or ".." in name.split("/"):
                    raise RobotProfileError(f"invalid path in archive: {info.filename}")
                parts = name.split("/")
                if any(p.startswith(".") and p not in {".", ".."} for p in parts) or "__MACOSX" in parts:
                    continue
                total_size += info.file_size
                if total_size > MAX_UNCOMPRESSED_BYTES:
                    raise RobotProfileError("uncompressed archive size exceeds limit")
                cleaned_entries.append((info, Path(name)))

            if not cleaned_entries:
                raise RobotProfileError("no valid files found in archive")

            top_dirs = {entry[1].parts[0] for entry in cleaned_entries if len(entry[1].parts) > 1}
            has_root_files = any(len(entry[1].parts) == 1 and not entry[0].is_dir() for entry in cleaned_entries)
            single_prefix = top_dirs.pop() if len(top_dirs) == 1 and not has_root_files else None

            if not str(profile_id).strip():
                candidate = single_prefix or Path(archive_filename).stem or "robot_profile"
                candidate = re.sub(r"[^A-Za-z0-9_.-]", "_", candidate).strip("_")
                if not candidate or not candidate[0].isalnum():
                    candidate = "robot_" + candidate
                profile_id = candidate[:64]

            profile_id = validate_profile_id(profile_id)
            display_name = str(display_name).strip() or profile_id
            if len(display_name) > 80:
                raise RobotProfileError("display name must not exceed 80 characters")

            target = self._path(profile_id)
            if target.exists() and not overwrite:
                raise RobotProfileError(f"robot profile already exists: {profile_id}")

            temp_path = Path(tempfile.mkdtemp(prefix=f".{profile_id}-", dir=self.root))
            try:
                for info, rel_path in cleaned_entries:
                    if info.is_dir():
                        continue
                    if single_prefix and rel_path.parts[0] == single_prefix:
                        dest_rel = Path(*rel_path.parts[1:])
                    else:
                        dest_rel = rel_path
                    dest_file = temp_path / dest_rel
                    dest_file.parent.mkdir(parents=True, exist_ok=True)
                    dest_file.write_bytes(zf.read(info.filename))

                urdf_files = list(temp_path.glob("**/*.urdf"))
                if not urdf_files:
                    for xml_file in temp_path.glob("**/*.xml"):
                        try:
                            text = xml_file.read_text(encoding="utf-8-sig", errors="ignore")
                            if "<robot" in text:
                                urdf_files.append(xml_file)
                        except Exception:
                            pass
                if not urdf_files:
                    raise RobotProfileError("no .urdf file found in the archive")

                urdf_file = urdf_files[0]
                for candidate in urdf_files:
                    if candidate.parent.name == "urdf" or candidate.stem == profile_id:
                        urdf_file = candidate
                        break

                urdf_payload = urdf_file.read_bytes()
                rel_urdf_path = urdf_file.relative_to(temp_path).as_posix()

                yaml_files = [
                    f for f in (list(temp_path.glob("**/*.yml")) + list(temp_path.glob("**/*.yaml")))
                    if f.name != "profile.yaml"
                ]

                chosen_source_config = None
                source_kind = None
                for candidate in sorted(
                    yaml_files,
                    key=lambda p: (
                        {"arm_teleop.yaml": 0, "controller_v23.yml": 1, "vr_configs.yml": 2}.get(
                            p.name, 3
                        ),
                        len(p.parts),
                    ),
                ):
                    try:
                        loaded = _load_yaml(candidate.read_bytes())
                        if isinstance(loaded, dict):
                            if isinstance(loaded.get("arms"), (dict, list)) and (
                                "robot" in loaded or "grippers" in loaded or "control" in loaded
                            ):
                                chosen_source_config = loaded
                                source_kind = "teleop"
                                break
                            elif "model" in loaded and "task" in loaded:
                                chosen_source_config = loaded
                                source_kind = "v23"
                                break
                            elif "urdf_path" in loaded and "arms" in loaded:
                                chosen_source_config = loaded
                                source_kind = "io"
                                break
                    except Exception:
                        continue

                if not chosen_source_config:
                    raise RobotProfileError(
                        "no supported YAML found in archive (expected arm_teleop.yaml, vr_configs.yml, or controller_v23.yml schema)"
                    )

                model = UrdfModel(urdf_payload)
                files: dict[str, dict[str, Any]] = {}
                if source_kind == "teleop":
                    teleop, controller, io_config, summary = _normalize_teleop(
                        profile_id, chosen_source_config, model, rel_urdf_path
                    )
                    files["arm_teleop.yaml"] = teleop
                    files["controller_v23.yml"] = controller
                    sim_file = next((f for f in yaml_files if f.name == "vr_configs.yml"), None)
                    if sim_file is not None:
                        try:
                            files["vr_configs.yml"] = _load_yaml(sim_file.read_bytes())
                        except Exception:
                            files["vr_configs.yml"] = io_config
                    else:
                        files["vr_configs.yml"] = io_config
                    primary_config = "controller_v23.yml"
                elif source_kind == "v23":
                    controller, summary = _normalize_v23(
                        profile_id, chosen_source_config, model, rel_urdf_path
                    )
                    files["controller_v23.yml"] = controller
                    primary_config = "controller_v23.yml"
                    simulation_file = next(
                        (f for f in yaml_files if f.name == "vr_configs.yml"), None
                    )
                    if simulation_file is not None:
                        simulation_config = _load_yaml(simulation_file.read_bytes())
                        io_config, _generated_controller, _generated_teleop, _io_summary = (
                            _normalize_io(
                                profile_id,
                                simulation_config,
                                model,
                                rel_urdf_path,
                            )
                        )
                        files["vr_configs.yml"] = io_config
                elif source_kind == "io":
                    io_config, controller, teleop, summary = _normalize_io(
                        profile_id, chosen_source_config, model, rel_urdf_path
                    )
                    files["vr_configs.yml"] = io_config
                    files["controller_v23.yml"] = controller
                    if teleop is not None:
                        files["arm_teleop.yaml"] = teleop
                    primary_config = "vr_configs.yml"
                else:
                    raise RobotProfileError("unsupported YAML schema in archive")

                teleop_file = next((f for f in yaml_files if f.name == "arm_teleop.yaml"), None)
                if teleop_file is not None and "arm_teleop.yaml" not in files:
                    try:
                        teleop_data = _load_yaml(teleop_file.read_bytes())
                        if isinstance(teleop_data, dict) and "control" in teleop_data:
                            files["arm_teleop.yaml"] = teleop_data
                    except Exception:
                        pass

                metadata = {
                    "version": 1,
                    "id": profile_id,
                    "display_name": display_name,
                    "schema": summary["schema"],
                    "managed": True,
                    "imported_at": datetime.now(timezone.utc).isoformat(),
                    "robot_name": model.robot_name,
                    "urdf": rel_urdf_path,
                    "primary_config": primary_config,
                    "config_files": list(files),
                    "link_count": len(model.links),
                    "joint_count": len(model.joints),
                    "movable_joint_count": len(model.movable_joint_names),
                    "free_joint_count": summary["free_joint_count"],
                    "task_count": summary["task_count"],
                    "arm_count": summary["arm_count"],
                    "teleop_compatible": "arm_teleop.yaml" in files,
                    "warnings": summary["warnings"],
                }

                for filename, value in files.items():
                    (temp_path / filename).write_text(
                        yaml.safe_dump(value, allow_unicode=True, sort_keys=False),
                        encoding="utf-8",
                    )
                (temp_path / "profile.yaml").write_text(
                    yaml.safe_dump(metadata, allow_unicode=True, sort_keys=False),
                    encoding="utf-8",
                )
                if target.exists():
                    shutil.rmtree(target)
                os.rename(temp_path, target)
            except Exception:
                shutil.rmtree(temp_path, ignore_errors=True)
                raise
            return self.get(profile_id)

    def get_profile_topics(self, profile_id: str) -> dict[str, str]:
        profile_dir = (self.root / profile_id).resolve()
        arm_teleop = profile_dir / "arm_teleop.yaml"
        if arm_teleop.is_file():
            with arm_teleop.open("r", encoding="utf-8") as stream:
                cfg = yaml.safe_load(stream) or {}
            control = cfg.get("control", {})
            body = cfg.get("body", {})
            return {
                "vr_frame": STANDARD_TOPICS["vr_frame"],
                "joint_state": control.get("joint_state_topic", STANDARD_TOPICS["joint_state"]),
                "joint_target": control.get("generic_command_topic", STANDARD_TOPICS["joint_target"]),
                "joint_command": STANDARD_TOPICS["joint_command"],
                "ee_target": control.get("controller_target_pose_topic", STANDARD_TOPICS["ee_target"]),
                "ee_visual_target": control.get("target_pose_topic", STANDARD_TOPICS["ee_visual_target"]),
                "ee_actual": control.get("actual_pose_topic", STANDARD_TOPICS["ee_actual"]),
                "solver_state": control.get("solver_topic", STANDARD_TOPICS["solver_state"]),
                "solver_reset": control.get("solver_reset_topic", STANDARD_TOPICS["solver_reset"]),
                "solver_reset_ack": control.get(
                    "solver_reset_ack_topic", STANDARD_TOPICS["solver_reset_ack"]
                ),
                "base_move": body.get("base_command_topic", STANDARD_TOPICS["base_move"]),
            }
        return dict(STANDARD_TOPICS)

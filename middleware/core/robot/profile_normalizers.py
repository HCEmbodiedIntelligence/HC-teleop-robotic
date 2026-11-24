"""机器人 Profile 多格式 YAML 配置归一化与 arm_teleop 配置生成。"""
from __future__ import annotations

import copy
import math
from pathlib import Path
from typing import Any, Iterable

import yaml

from .urdf_model import RobotProfileError, UrdfModel, _index, _index_list, _number_list

MAX_CONFIG_BYTES = 2 * 1024 * 1024

STANDARD_TOPICS = {
    "vr_frame": "/vrdata",
    "joint_state": "/observation/joint_states",
    "joint_target": "/control/solver/joint_targets",
    "joint_command": "/action/joint_states",
    "ee_target": "/control/solver/ee_targets",
    "ee_visual_target": "/action/ee_poses",
    "ee_actual": "/observation/ee_poses",
    "solver_state": "/status/solver/joint_states",
    "solver_reset": "/control/solver/reset",
    "solver_reset_ack": "/status/solver/reset_ack",
    "base_move": "/action/base/command",
}


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
            "model.free_joints are missing from movable URDF joints: "
            + ", ".join(unknown)
        )
    task_count = _validate_task_frames(normalized, model)
    if task_count == 0:
        raise RobotProfileError("controller YAML must define at least one task")
    limit_config = normalized.get("limit", {})
    if not isinstance(limit_config, dict):
        raise RobotProfileError("limit must be an object")
    velocity = limit_config.get("velocity", [])
    if velocity:
        velocity = _number_list(velocity, len(free_joints), "limit.velocity")
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
    root_dir = Path(__file__).resolve().parents[3]
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
    # 相机坐标系与内参来源仅属于模板源机器人。
    result.pop("camera_metadata", None)
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
    if isinstance(arms, dict):
        arms_list = []
        for side in ("right", "left"):
            if side in arms and isinstance(arms[side], dict):
                arms_list.append(arms[side])
        for k, v in arms.items():
            if k not in ("right", "left") and isinstance(v, dict):
                arms_list.append(v)
        arms = arms_list
        normalized["arms"] = arms
    if not isinstance(arms, list) or not arms:
        raise RobotProfileError("HC robot YAML requires a non-empty arms list")
    joint_count = len(model.joints)
    arm_joint_indices: list[list[int]] = []
    arm_ee_indices: list[int] = []
    joint_name_map = {j["name"]: i for i, j in enumerate(model.joints)}
    link_child_map = {j["child"]: i for i, j in enumerate(model.joints)}
    for arm_index, arm in enumerate(arms):
        if not isinstance(arm, dict):
            raise RobotProfileError(f"arms[{arm_index}] must be an object")
        raw_joints = arm.get("joint_index") or arm.get("joint_names")
        if not raw_joints:
            raise RobotProfileError(
                f"arms[{arm_index}] must define joint_index or joint_names"
            )
        resolved_joints = []
        for item in raw_joints:
            if isinstance(item, int):
                resolved_joints.append(item)
            elif isinstance(item, str) and item.strip() in joint_name_map:
                resolved_joints.append(joint_name_map[item.strip()])
            else:
                raise RobotProfileError(f"arms[{arm_index}] invalid joint: {item}")
        indices = _index_list(
            resolved_joints, joint_count, f"arms[{arm_index}].joint_index"
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
        raw_ee = arm.get("ee_index") if "ee_index" in arm else arm.get("ee_link")
        if isinstance(raw_ee, int):
            resolved_ee = raw_ee
        elif isinstance(raw_ee, str) and raw_ee.strip() in link_child_map:
            resolved_ee = link_child_map[raw_ee.strip()]
        elif isinstance(raw_ee, str) and raw_ee.strip() == model.root_link:
            resolved_ee = -1
        else:
            raise RobotProfileError(
                f"arms[{arm_index}] invalid ee_index or ee_link: {raw_ee}"
            )
        arm_ee_indices.append(
            _index(
                resolved_ee,
                joint_count,
                f"arms[{arm_index}].ee_index",
                root_allowed=True,
            )
        )
        if "rest_j_pos" in arm:
            _number_list(
                arm["rest_j_pos"], len(indices), f"arms[{arm_index}].rest_j_pos"
            )
    if "base_pose" in normalized:
        base_pose = normalized["base_pose"]
        if not isinstance(base_pose, dict):
            raise RobotProfileError("base_pose must be an object")
        _number_list(base_pose.get("position"), 3, "base_pose.position")
        _number_list(base_pose.get("orientation"), 4, "base_pose.orientation")

    controller_indices = normalized.get("controller_indices", {})
    if not isinstance(controller_indices, dict):
        controller_indices = {}
    command_ee = _index_list(
        controller_indices.get("cmd_ee", arm_ee_indices),
        joint_count,
        "controller_indices.cmd_ee",
        root_allowed=True,
    )
    bases = _index_list(
        controller_indices.get("base", [-1] * len(arms)),
        joint_count,
        "controller_indices.base",
        root_allowed=True,
    )
    if len(command_ee) != len(arms) or len(bases) != len(arms):
        raise RobotProfileError(
            "controller_indices.cmd_ee/base counts must match the arms count"
        )
    normalized["controller_indices"] = {
        **controller_indices,
        "cmd_ee": command_ee,
        "base": bases,
    }
    normalized["urdf_path"] = urdf_name if "/" in urdf_name else f"urdf/{urdf_name}"
    normalized["robot_name"] = profile_id

    free_indices = []
    for indices in arm_joint_indices:
        free_indices.extend(indices)
    free_indices.extend(_component_joint_indices(normalized, model))
    free_indices = [
        index
        for index in dict.fromkeys(free_indices)
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
        "control": {
            "method": "control",
            "dt": 0.01,
            "damping": 1.0e-5,
            "joints_init": [],
        },
        "model": {
            "urdf": urdf_name if "/" in urdf_name else f"urdf/{urdf_name}",
            "free_joints": free_joints,
        },
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
    return (
        normalized,
        controller,
        teleop,
        {
            "schema": "hc-robot-config-v1",
            "free_joint_count": len(free_joints),
            "task_count": len(pose_tasks),
            "arm_count": len(arms),
            "warnings": warnings,
        },
    )


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
                raise RobotProfileError(
                    f"arms.{side} references missing URDF joint: '{jname}'"
                )
        joint_names = [str(j) for j in joint_names]
        free_joints.extend(joint_names)

        ee_link = str(arm.get("ee_link", "")).strip()
        if not ee_link or ee_link not in link_set:
            raise RobotProfileError(
                f"arms.{side}.ee_link '{ee_link}' is missing from URDF"
            )

        raw_task_base = str(
            arm.get("generic_task_base_link") or arm.get("base_link") or ""
        ).strip()
        task_base = raw_task_base if raw_task_base in link_set else model.root_link
        tasks.append([[task_base, ee_link], 5.0, 1.0, 3.0])

        cmd_ee_list.append(ee_link)
        base_link = str(arm.get("base_link", "base")).strip()
        base_list.append(
            -1 if base_link in ("-1", "base", "root", model.root_link) else base_link
        )

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
        "control": {
            "method": "control",
            "dt": 0.01,
            "damping": 1.0e-5,
            "joints_init": [],
        },
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
        "sim_camera": sim_settings.get(
            "sim_camera",
            {
                "distance": 2.5,
                "yaw": 45,
                "pitch": -25,
                "target_position": [0.0, 0.0, 0.5],
            },
        ),
        "sim_opengl2": True,
        "arms": copy.deepcopy(arms_config),
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


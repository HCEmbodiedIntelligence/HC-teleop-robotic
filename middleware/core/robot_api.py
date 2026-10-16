"""Single profile resolver for the HC public/model SI boundary.

This module has no ROS dependencies. Launchers export its resolved document for
the C++ adapter; Python consumers use the same object. It never opens hardware.
"""
from __future__ import annotations

import copy
import hashlib
import json
import math
import os
import re
from pathlib import Path
from xml.etree import ElementTree

import yaml

from .robot_profiles import RobotProfileError, UrdfModel, validate_profile_id

SCHEMA = "hc-robot-api/v1"
INTERNAL_STATE = "/internal/robot/joint_states"
INTERNAL_COMMAND = "/internal/robot/joint_commands"
ENDPOINTS = {
    "joint_states": "/observation/joint_states",
    "joint_commands": "/action/joint_states",
    "info": "/status/robot/info",
    "ready": "/status/hardware/ready",
    "source": "/control/source",
    "permit": "/control/motion/permit",
}


class _RuntimeYamlDumper(yaml.SafeDumper):
    def ignore_aliases(self, _data):
        # ROS 2 parameter YAML does not accept anchors/aliases.
        return True


def selected_profile_root(config, config_dir, environ=None):
    env = os.environ if environ is None else environ
    settings = config.get("robot_profiles", {})
    root = Path(env.get("HC_ROBOT_CONFIG_ROOT") or settings.get("root", "../adapters/robots")).expanduser()
    if not root.is_absolute():
        root = Path(config_dir) / root
    return root.resolve()


def selected_profile(config, config_dir, environ=None):
    """Environment > middleware selection; relative roots use config_dir."""
    env = os.environ if environ is None else environ
    name = validate_profile_id(env.get("HC_ROBOT_NAME") or config.get("robot_profiles", {}).get("active", ""))
    return selected_profile_root(config, config_dir, env) / name


def _mapping(value, label):
    if not isinstance(value, dict):
        raise RobotProfileError(f"{label} must be an object")
    return value


def _finite(value, label, positive=False):
    if isinstance(value, bool) or not isinstance(value, (float, int)) or not math.isfinite(value):
        raise RobotProfileError(f"{label} must be finite")
    if positive and value <= 0:
        raise RobotProfileError(f"{label} must be positive")
    return float(value)


def _names(value, label):
    if (not isinstance(value, list) or not value or
            any(not isinstance(v, str) or not v for v in value) or len(set(value)) != len(value)):
        raise RobotProfileError(f"{label} must be a nonempty list of unique names")
    return value


class RobotApiProfile:
    def __init__(self, directory):
        try:
            self._load(directory)
        except RobotProfileError:
            raise
        except (KeyError, TypeError, ValueError, AttributeError, ElementTree.ParseError) as exc:
            raise RobotProfileError(f"invalid unified profile {directory}: {exc}") from exc

    def _load(self, directory):
        self.directory = Path(directory).resolve()
        self._inputs = {}
        self.metadata = self._yaml("profile.yaml")
        self.profile_id = validate_profile_id(self.metadata.get("id", self.directory.name))
        self.teleop = self._yaml("arm_teleop.yaml")
        self.config = self._yaml("robot_api.yaml")
        if self.config.get("schema") != SCHEMA:
            raise RobotProfileError(f"robot_api.schema must be {SCHEMA}")
        self.urdf_path = self._path(self.teleop["robot"]["urdf_path"])
        payload = self.urdf_path.read_bytes()
        self._inputs[str(self.urdf_path.relative_to(self.directory))] = payload
        model = UrdfModel(payload)
        self.model = {j["name"]: j for j in model.joints if j["type"] != "fixed"}
        xml = ElementTree.fromstring(payload)
        for item in xml.findall("joint"):
            limit = item.find("limit")
            if item.get("name") in self.model and limit is not None and limit.get("effort"):
                self.model[item.get("name")]["effort"] = _finite(float(limit.get("effort")), "URDF effort", True)
        self.motion = self._yaml("motion/motion.yaml")["humanoid_motion_control"]["ros__parameters"]
        self.sdk = self._yaml("motion/sdk.yaml")
        self.channels = self._yaml("motion/channels.yaml")["channels"]
        self.tools = self._yaml("motion/tools.yaml")
        self.joints = {}
        self.public = {}
        self.groups = {}
        self._resolve_groups()
        self._validate_motion(model.links)
        self.poses = self._resolve_poses()
        self.driver = self._resolve_driver()
        self.revision = hashlib.sha256(b"".join(
            name.encode() + b"\0" + data for name, data in sorted(self._inputs.items())
        )).hexdigest()

    def _path(self, name):
        path = (self.directory / name).resolve()
        if not path.is_relative_to(self.directory):
            raise RobotProfileError(f"profile path escapes its directory: {name}")
        return path

    def _yaml(self, name):
        try:
            payload = self._path(name).read_bytes()
            self._inputs[name] = payload
            return _mapping(yaml.safe_load(payload), name)
        except (OSError, yaml.YAMLError) as exc:
            raise RobotProfileError(f"cannot read {name}: {exc}") from exc

    def _reference(self, reference):
        allowed = {"robot.initial_joints", "body.waist_joint_names", "body.neck_joint_names"}
        allowed.update(f"arms.{side}.joint_names" for side in self.teleop.get("arms", {}))
        allowed.update(f"grippers.{side}.finger_joint_names" for side in ("left", "right"))
        if reference not in allowed:
            raise RobotProfileError(f"unknown profile field reference: {reference}")
        value = self.teleop
        try:
            for key in reference.split("."):
                value = value[key]
        except (KeyError, TypeError) as exc:
            raise RobotProfileError(f"missing profile field: {reference}") from exc
        return value

    def _add_joint(self, model_name, public_name):
        if model_name not in self.model:
            raise RobotProfileError(f"unknown movable model joint: {model_name}")
        if not isinstance(public_name, str) or not re.fullmatch(r"[a-z][a-z0-9_]*", public_name):
            raise RobotProfileError(f"invalid public joint name: {public_name}")
        if model_name in self.joints:
            if self.joints[model_name]["public_name"] != public_name:
                raise RobotProfileError(f"conflicting public mapping for {model_name}")
            return
        if public_name in self.public:
            raise RobotProfileError(f"duplicate public name: {public_name}")
        model = self.model[model_name]
        override = _mapping(self.config.get("joint_overrides", {}).get(model_name, {}), "joint override")
        unit = "m" if model["type"] == "prismatic" else "rad"
        offset_key = "offset_m" if unit == "m" else "offset_rad"
        if set(override) - {"scale", offset_key}:
            raise RobotProfileError(f"invalid override/offset unit for {model_name}")
        scale = _finite(override.get("scale", 1.0), f"{model_name}.scale")
        if abs(scale) < 1e-12:
            raise RobotProfileError(f"{model_name}.scale must be nonzero")
        offset = _finite(override.get(offset_key, 0.0), f"{model_name}.{offset_key}")
        lower, upper = model["lower"], model["upper"]
        if model["type"] == "continuous":
            lower = upper = None
        elif lower is None or upper is None or lower > upper:
            raise RobotProfileError(f"invalid URDF position limits for {model_name}")
        # Motion may deliberately use tighter position limits than the URDF.
        for group in self.motion.get("joint_group_names", []):
            names = self.motion.get(f"groups.{group}", [])
            if model_name not in names:
                continue
            for bound in ("lower", "upper"):
                limits = self.motion.get(f"group_{bound}_limits.{group}")
                if limits is None:
                    continue
                if not isinstance(limits, list) or len(limits) != len(names):
                    raise RobotProfileError(f"invalid Motion {bound} limits for {group}")
                value = _finite(limits[names.index(model_name)], f"{group}.{bound}")
                if bound == "lower":
                    lower = value if lower is None else max(lower, value)
                else:
                    upper = value if upper is None else min(upper, value)
        if lower is not None and upper is not None and lower > upper:
            raise RobotProfileError(f"inconsistent position limits for {model_name}")
        limits = {"lower": None, "upper": None}
        if lower is not None and upper is not None:
            limits["lower"], limits["upper"] = sorted(((lower-offset)/scale, (upper-offset)/scale))
        for key, param in (("velocity", "joint_max_velocity_rad_s"),
                           ("acceleration", "joint_max_acceleration_rad_s2"),
                           ("jerk", "joint_max_jerk_rad_s3")):
            value = self.motion.get(param) if unit == "rad" else None
            if key == "velocity" and model.get("velocity") is not None:
                value = min(value, model["velocity"]) if value is not None else model["velocity"]
            limits[key] = None if value is None else _finite(value, param, True) / abs(scale)
        limits["effort"] = None if "effort" not in model else model["effort"] * abs(scale)
        joint = dict(model_name=model_name, public_name=public_name, scale=scale, offset=offset,
                     unit=unit, limits=limits, model_lower=lower, model_upper=upper)
        self.joints[model_name] = self.public[public_name] = joint

    def _resolve_groups(self):
        groups = _mapping(self.config.get("groups"), "robot_api.groups")
        for name, spec in groups.items():
            spec = _mapping(spec, f"groups.{name}")
            if "combine" in spec:
                continue
            names = _names(self._reference(spec.get("model_joints_from")), name)
            prefix = spec.get("public_name_prefix", "")
            if name in ("left_arm", "right_arm"):
                side = name.removesuffix("_arm")
                if prefix != side + "_joint_" or spec.get("model_joints_from") != f"arms.{side}.joint_names":
                    raise RobotProfileError(f"{name} requires ordered {side}_joint_1…N names")
            for index, model_name in enumerate(names, 1):
                self._add_joint(model_name, prefix + str(index))
            self.groups[name] = list(names)
        for model_name, public_name in _mapping(self.config.get("auxiliary_joints", {}), "auxiliary_joints").items():
            self._add_joint(model_name, public_name)
        for name, spec in groups.items():
            if "combine" not in spec:
                continue
            children = _names(spec["combine"], f"groups.{name}.combine")
            if any(child not in self.groups for child in children):
                raise RobotProfileError(f"unknown/cyclic combined group: {name}")
            self.groups[name] = list(dict.fromkeys(j for child in children for j in self.groups[child]))
        missing = self.model.keys() - self.joints.keys()
        if missing:
            raise RobotProfileError("unmapped model joints (declare auxiliaries): " + ", ".join(sorted(missing)))
        unknown = self.config.get("joint_overrides", {}).keys() - self.joints.keys()
        if unknown:
            raise RobotProfileError("unknown override joints: " + ", ".join(sorted(unknown)))

    def _validate_motion(self, links):
        for field in ("control_frequency_hz", "servo_lease_ms", "feedback_max_age_ms",
                      "input_stamp_max_age_s", "input_stamp_future_tolerance_s"):
            _finite(self.motion.get(field), field, True)
        for name in _names(self.motion.get("joint_group_names"), "Motion groups"):
            if name not in self.groups:
                raise RobotProfileError(f"Motion group {name} has no public mapping")
            if self.motion.get(f"groups.{name}") != self.groups[name] or self.sdk.get("joint_groups", {}).get(name) != self.groups[name]:
                raise RobotProfileError(f"Motion/SDK group order differs from profile: {name}")
            if any(self.joints[n]['unit'] != 'rad' or self.joints[n]['model_lower'] is None or
                   self.joints[n]['model_upper'] is None for n in self.groups[name]):
                raise RobotProfileError(f"Motion SDK requires bounded angular joints: {name}")
        if set(self.sdk.get("joint_groups", {})) != set(self.motion["joint_group_names"]):
            raise RobotProfileError("SDK and Motion group sets differ")
        seen = set()
        for channel in self.channels:
            name, kind = channel["group"], channel["kind"]
            endpoint = f"/control/motion/{name}/{kind}"
            if name not in self.groups or channel["endpoint"] != endpoint or endpoint in seen:
                raise RobotProfileError(f"invalid/duplicate public channel: {channel}")
            seen.add(endpoint)
            if kind not in {"servo_j", "servo_p", "move_j", "move_l", "move_p"}:
                raise RobotProfileError(f"invalid channel kind: {kind}")
            if channel.get("base_frame") not in links or channel.get("tip_frame") not in links:
                raise RobotProfileError(f"unknown channel frame: {endpoint}")
            if name.endswith("_arm"):
                arm = self.teleop["arms"][name.removesuffix("_arm")]
                if channel["base_frame"] != arm["base_link"] or channel["tip_frame"] != arm["ee_link"]:
                    raise RobotProfileError(f"Motion frames differ from arm_teleop: {name}")
        for side in self.teleop["arms"]:
            group = side + "_arm"
            if any(f"/control/motion/{group}/{kind}" not in seen for kind in ("servo_j", "servo_p", "move_j")):
                raise RobotProfileError(f"unified arm {group} requires ServoJ, ServoP and MoveJ")

    def _resolve_poses(self):
        poses = {}
        for name, spec in _mapping(self.config.get("poses"), "poses").items():
            if "model_positions_from" in spec:
                values = self._reference(spec["model_positions_from"])
            else:
                values = spec.get("model_positions", {})
            _mapping(values, f"poses.{name}")
            if set(values) - self.model.keys():
                raise RobotProfileError(f"unknown joints in pose {name}")
            groups = spec.get("groups", [s + "_arm" for s in self.teleop["arms"]])
            poses[name] = {}
            for group in _names(groups, f"poses.{name}.groups"):
                if group not in self.groups:
                    raise RobotProfileError(f"unknown pose group: {group}")
                target = {}
                for model_name in self.groups[group]:
                    if model_name not in values:
                        raise RobotProfileError(f"pose {name} lacks {model_name}")
                    joint = self.joints[model_name]
                    q = (_finite(values[model_name], name) - joint["offset"]) / joint["scale"]
                    lo, hi = joint["limits"]["lower"], joint["limits"]["upper"]
                    if (lo is not None and q < lo) or (hi is not None and q > hi):
                        raise RobotProfileError(f"pose {name} exceeds limits: {model_name}")
                    target[joint["public_name"]] = q
                poses[name][group] = target
        if "home" not in poses or self.config["poses"]["home"].get("model_positions_from") != "robot.initial_joints":
            raise RobotProfileError("home must reference robot.initial_joints")
        return poses

    def _resolve_driver(self):
        driver = copy.deepcopy(_mapping(self.config.get("driver"), "driver"))
        kind = driver.get("kind")
        params = _mapping(driver.get("parameters"), "driver.parameters")
        if kind == "openarmx_hc_robot_adapter":
            for key in ("robot_feedback_topic", "left_controller_topic", "right_controller_topic"):
                if not str(params.get(key, "")).startswith("/"):
                    raise RobotProfileError(f"driver requires {key}")
            params.update(hc_feedback_topic=INTERNAL_STATE, hc_command_topic=INTERNAL_COMMAND,
                          enable_topic="/control/hardware/enabled")
        elif kind == "humanoid_driver_runtime":
            names, groups = [], []
            for group in _names(driver.get("groups"), "driver.groups"):
                if group not in self.groups:
                    raise RobotProfileError(f"unknown driver group {group}")
                for name in self.groups[group]:
                    if name in names:
                        raise RobotProfileError("driver groups must not execute one physical joint twice")
                    names.append(name)
                    groups.append(group)
            params.update(joint_names=names, vendor_joint_groups=groups,
                          platform_joint_state_topic=INTERNAL_STATE,
                          platform_joint_command_topic=INTERNAL_COMMAND,
                          hc_safety_enabled=True, output_enable_topic="/control/hardware/enabled")
            params.setdefault("vendor_joint_names", names)
            for field, default in (("vendor_to_logical_scales", 1.0), ("vendor_to_logical_offsets_rad", 0.0)):
                params.setdefault(field, [default] * len(names))
                if len(params[field]) != len(names):
                    raise RobotProfileError(f"{field} length differs from driver groups")
                for value in params[field]:
                    _finite(value, field)
                    if field.endswith("scales") and abs(value) < 1e-12:
                        raise RobotProfileError("vendor scale must be nonzero")
            _names(params["vendor_joint_names"], "vendor_joint_names")
            if len(params["vendor_joint_names"]) != len(names) or not params.get("plugin_class"):
                raise RobotProfileError("invalid driver vendor mapping/plugin")
        else:
            raise RobotProfileError(f"unsupported driver kind: {kind}")
        # Prevent nonfinite nested connection values from reaching ROS parameters.
        try:
            json.dumps(driver, allow_nan=False)
        except (ValueError, TypeError) as exc:
            raise RobotProfileError(f"invalid driver parameters: {exc}") from exc
        return driver

    def convert(self, names, position, velocity=(), effort=(), *, to_model=False, group=None):
        """Return converted arrays in canonical order, without filling missing fields/joints."""
        _names(list(names), "JointState.name")
        if len(position) != len(names) or any(len(v) not in (0, len(names)) for v in (velocity, effort)):
            raise RobotProfileError("JointState array lengths differ")
        lookup = self.public if to_model else self.joints
        if any(name not in lookup for name in names):
            raise RobotProfileError("JointState contains unknown names")
        allowed = self.groups.get(group) if group is not None else list(self.joints)
        if allowed is None or any(lookup[n]["model_name"] not in allowed for n in names):
            raise RobotProfileError("JointState contains joints outside the requested group")
        indices = {lookup[n]["model_name"]: i for i, n in enumerate(names)}
        output = dict(name=[], position=[], velocity=[], effort=[])
        for model in allowed:
            if model not in indices:
                continue
            i, joint = indices[model], self.joints[model]
            s, b = joint["scale"], joint["offset"]
            output["name"].append(model if to_model else joint["public_name"])
            for key, values in (("position", position), ("velocity", velocity), ("effort", effort)):
                if not len(values):
                    continue
                value = _finite(values[i], f"JointState.{key}")
                if key == "position":
                    value = s*value+b if to_model else (value-b)/s
                elif key == "velocity":
                    value = s*value if to_model else value/s
                else:
                    value = value/s if to_model else s*value
                output[key].append(value)
        return output

    def convert_message(self, message, *, to_model=False, group=None):
        result = copy.deepcopy(message)
        for field, values in self.convert(message.name, message.position, message.velocity,
                                          message.effort, to_model=to_model, group=group).items():
            setattr(result, field, values)
        return result

    def describe(self):
        groups = {}
        for name, models in self.groups.items():
            channels = [c for c in self.channels if c["group"] == name]
            groups[name] = dict(
                joint_names=[self.joints[m]["public_name"] for m in models], count=len(models),
                units=[self.joints[m]["unit"] for m in models],
                limits=[copy.deepcopy(self.joints[m]["limits"]) for m in models],
                commands=[c["kind"] for c in channels],
                endpoints={c["kind"]: c["endpoint"] for c in channels},
                joint_states=None if name == "whole_body" else f"/observation/{name}/joint_states",
                base_frame=channels[0]["base_frame"] if channels else None,
                tip_frame=channels[0]["tip_frame"] if channels else None,
                fk_pose_topic=next((c["fk_pose_topic"] for c in channels if c.get("fk_pose_topic")), None))
        return dict(schema=SCHEMA, profile_id=self.profile_id, revision=self.revision,
                    groups=groups, joints=[{k: copy.deepcopy(j[k]) for k in ("public_name", "unit", "limits")}
                                          for j in self.joints.values()],
                    poses=copy.deepcopy(self.poses), endpoints=dict(ENDPOINTS),
                    servo={k: self.motion[k] for k in ("control_frequency_hz", "servo_lease_ms",
                           "feedback_max_age_ms", "input_stamp_max_age_s", "input_stamp_future_tolerance_s")})

    def write_runtime(self, directory):
        """Generate inspectable, immutable-per-launch parameters. No live activation."""
        output = Path(directory).resolve()
        output.mkdir(parents=True, exist_ok=True)
        resolved = dict(schema=SCHEMA, profile_id=self.profile_id, revision=self.revision,
                        joints=list(self.joints.values()), groups=self.groups, info=self.describe())
        params = copy.deepcopy(self.motion)
        params.update(joint_state_endpoint=INTERNAL_STATE,
                      joint_command_endpoint="/control/motion/joint_targets",
                      robot_api_file=str(output / "robot_api.resolved.yaml"),
                      urdf_file=str(self.urdf_path),
                      sdk_config_file=str(output / "sdk.yaml"),
                      channel_config_file=str(output / "channels.yaml"),
                      tool_config_file=str(self.directory / "motion/tools.yaml"))
        for group in params["joint_group_names"]:
            params[f"group_lower_limits.{group}"] = [self.joints[n]["model_lower"] for n in self.groups[group]]
            params[f"group_upper_limits.{group}"] = [self.joints[n]["model_upper"] for n in self.groups[group]]
        sdk = copy.deepcopy(self.sdk)
        sdk["model_path"] = str(self.urdf_path)
        # Motion SI limits are also the source of SDK degree/mm RTC limits.
        rtc = sdk.setdefault("rtc", {})
        for field, suffix in (("velocity", "rad_s"), ("acceleration", "rad_s2"), ("jerk", "rad_s3")):
            rtc[f"max_{field}"] = math.degrees(self.motion[f"joint_max_{field}_{suffix}"])
        for field, suffix in (("velocity", "m_s"), ("acceleration", "m_s2"), ("jerk", "m_s3")):
            rtc[f"cartesian_max_linear_{field}"] = 1000. * self.motion[f"cartesian_max_linear_{field}_{suffix}"]
        for field, suffix in (("velocity", "rad_s"), ("acceleration", "rad_s2"), ("jerk", "rad_s3")):
            rtc[f"cartesian_max_angular_{field}"] = self.motion[f"cartesian_max_angular_{field}_{suffix}"]
        for group, settings in sdk.get("rmp", {}).get("groups", {}).items():
            for field in ("velocity", "acceleration"):
                settings[f"joint_max_{field}"] = [math.degrees(
                    self.joints[n]["limits"][field] * abs(self.joints[n]["scale"])) for n in self.groups[group]]
        # The SDK boundary is degrees/mm; never apply public scale here.
        sdk.setdefault("execution", {})["initial_state"] = {"joint_groups": {
            group: [math.degrees(self.teleop["robot"]["initial_joints"][n]) for n in self.groups[group]]
            for group in params["joint_group_names"]}}
        channels = copy.deepcopy(self.channels)
        move_priority = max(c["priority"] for c in channels) + 1
        for channel in channels:
            channel["source"] = "motion"
            if channel["kind"].startswith("move_"):
                channel["priority"] = move_priority
        for channel in list(channels):
            if channel["kind"] not in {"servo_p", "move_j"}:
                continue
            vr = copy.deepcopy(channel)
            vr.update(name="vr_" + vr["name"], source="vr",
                      endpoint=f'/internal/vr/{vr["group"]}/{vr["kind"]}')
            vr.pop("fk_pose_topic", None)
            channels.append(vr)
        files = {"robot_api.resolved.yaml": resolved,
                 "motion.yaml": {"humanoid_motion_control": {"ros__parameters": params}},
                 "sdk.yaml": sdk, "channels.yaml": {"channels": channels},
                 "driver.yaml": {self.driver["kind"]: {"ros__parameters": self.driver["parameters"]}}}
        for name, value in files.items():
            (output / name).write_text(yaml.dump(value, Dumper=_RuntimeYamlDumper, sort_keys=False), encoding="utf-8")
        (output / "robot_info.json").write_text(json.dumps(self.describe(), indent=2, allow_nan=False) + "\n")
        return output

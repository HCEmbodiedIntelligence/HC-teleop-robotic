from __future__ import annotations

import json
import math
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import numpy as np
import pybullet as bullet
import yaml
from geometry_msgs.msg import Pose, PoseArray, PoseStamped
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState, Joy
from std_msgs.msg import Bool, Float64MultiArray, String
from std_srvs.srv import Trigger

from .arm_teleop_math import (
    adaptive_damping,
    clamp_step,
    joystick_base_velocity,
    joint_limit_avoidance,
    mapped_relative_yaw,
    orientation_error,
    quaternion_from_axis_angle,
    quaternion_multiply,
    relative_target,
    stabilize_pose,
    sticks_outward,
)


@dataclass
class ArmRuntime:
    name: str
    joint_names: list[str]
    joint_indices: list[int]
    dof_indices: list[int]
    lower: np.ndarray
    upper: np.ndarray
    ee_index: int
    base_index: int
    pose: tuple[np.ndarray, np.ndarray] | None = None
    pose_stamp: float = 0.0
    joy: Joy | None = None
    joy_stamp: float = 0.0
    active: bool = False
    reference_vr: tuple[np.ndarray, np.ndarray] | None = None
    reference_local_ee: tuple[np.ndarray, np.ndarray] | None = None
    target_local: tuple[np.ndarray, np.ndarray] | None = None
    target_world: tuple[np.ndarray, np.ndarray] | None = None
    last_solution: np.ndarray | None = None
    position_error: float = 0.0
    orientation_error: float = 0.0
    ik_converged: bool = True
    ik_within_tolerance: bool = True
    ik_rejections: int = 0
    ik_rejection_total: int = 0
    ik_seed: str = "feedback"
    ik_damping: float = 0.0
    last_ik_log: float = 0.0
    last_ik_attempt: float = 0.0
    last_reseed_attempt: float = 0.0
    recovery_solution: np.ndarray | None = None
    recovery_seed: str = ""
    reseed_target_position: np.ndarray | None = None
    reseed_target_orientation: np.ndarray | None = None


@dataclass
class IkCandidate:
    joints: np.ndarray
    position_error: float
    orientation_error: float
    converged: bool
    accepted: bool
    minimum_limit_margin: float
    damping: float
    seed_name: str
    score: float


@dataclass
class BodyRuntime:
    joint_names: list[str]
    joint_indices: list[int]
    dof_indices: list[int]
    lower: np.ndarray
    upper: np.ndarray
    torso_index: int
    head_pose: tuple[np.ndarray, np.ndarray] | None = None
    head_stamp: float = 0.0
    active: bool = False
    reference_head: tuple[np.ndarray, np.ndarray] | None = None
    reference_torso: tuple[np.ndarray, np.ndarray] | None = None
    target_base: tuple[np.ndarray, np.ndarray] | None = None
    last_solution: np.ndarray | None = None
    lift_error: float = 0.0
    pitch_error: float = 0.0


class HcTjArmTeleopNode(Node):
    """HC-TJ simulation teleop using the requested two-clutch state machine."""

    def __init__(self, config_path: str, backend: str | None = None):
        super().__init__("hc_tj_vr_teleop")
        self.config_path = Path(config_path).expanduser().resolve()
        with self.config_path.open(encoding="utf-8") as stream:
            self.config = yaml.safe_load(stream)
        self._validate_config()
        self.control = self.config["control"]
        self.body_config = self.config["body"]
        self.backend = str(backend or self.control.get("backend", "legacy")).lower()
        if self.backend not in {"v23", "generic", "legacy"}:
            raise ValueError("control.backend must be v23, generic or legacy")
        self.external_ik = self.backend in {"v23", "generic"}
        self.enabled = bool(self.control.get("enabled_on_start", True))
        self.stop_reason = ""
        self.joint_state: dict[str, float] = {}
        self.joint_state_stamp = 0.0
        self.last_command: dict[str, float] = {}
        self.last_status_publish = 0.0
        self.base_zero_pending = False
        self.home_gesture_latched = False
        self.homing = False
        self.solver_stamp = 0.0
        self.solver_message_count = 0
        self.generic_command_stamp = 0.0
        self.generic_command_count = 0
        self.generic_aux_command: dict[str, float] = {}

        self.physics_client = bullet.connect(bullet.DIRECT)
        if self.physics_client < 0:
            raise RuntimeError("unable to start the PyBullet IK model")
        robot = self.config["robot"]
        self.initial_joints = {
            name: float(value) for name, value in robot.get("initial_joints", {}).items()
        }
        urdf_path = Path(robot["urdf_path"]).expanduser()
        if not urdf_path.is_absolute():
            urdf_path = self.config_path.parent / urdf_path
        urdf_path = urdf_path.resolve()
        self.robot_id = bullet.loadURDF(
            str(urdf_path),
            robot["base_position"],
            robot["base_orientation"],
            useFixedBase=True,
            flags=bullet.URDF_USE_INERTIA_FROM_FILE,
            physicsClientId=self.physics_client,
        )
        self.joint_by_name: dict[str, int] = {}
        self.link_by_name: dict[str, int] = {}
        self.dof_by_joint: dict[int, int] = {}
        self.dof_joint_indices: list[int] = []
        dof = 0
        base_info = bullet.getBodyInfo(
            self.robot_id, physicsClientId=self.physics_client
        )
        if base_info:
            self.link_by_name[base_info[0].decode()] = -1
        for index in range(
            bullet.getNumJoints(self.robot_id, physicsClientId=self.physics_client)
        ):
            info = bullet.getJointInfo(
                self.robot_id, index, physicsClientId=self.physics_client
            )
            self.joint_by_name[info[1].decode()] = index
            self.link_by_name[info[12].decode()] = index
            if info[2] != bullet.JOINT_FIXED:
                self.dof_by_joint[index] = dof
                self.dof_joint_indices.append(index)
                dof += 1
        for name, position in self.initial_joints.items():
            index = self.joint_by_name.get(name)
            if index is not None:
                bullet.resetJointState(
                    self.robot_id,
                    index,
                    position,
                    physicsClientId=self.physics_client,
                )

        self.arms = {
            side: self._create_arm(side, arm_config)
            for side, arm_config in self.config["arms"].items()
        }
        self.generic_task_base_indices = {
            side: self.link_by_name[arm_config["generic_task_base_link"]]
            for side, arm_config in self.config["arms"].items()
        }
        self.body = self._create_body(self.body_config)
        self.zeros = [0.0] * len(self.dof_joint_indices)
        self.controlled_names = (
            self.arms["right"].joint_names
            + self.arms["left"].joint_names
            + self.body.joint_names
        )
        if self.config["grippers"].get("include_sim_joints", True):
            self.controlled_names += [
                self.config["grippers"]["right"]["sim_joint"],
                self.config["grippers"]["left"]["sim_joint"],
            ]

        self.command_pub = self.create_publisher(
            JointState, self.control["command_topic"], 10
        )
        self.target_pub = self.create_publisher(
            PoseArray, self.control["target_pose_topic"], 10
        )
        self.controller_target_pub = self.create_publisher(
            PoseArray,
            self.control.get(
                "controller_target_pose_topic",
                "/hc_teleop/controller_target_ee_poses",
            ),
            10,
        )
        self.actual_pub = self.create_publisher(
            PoseArray,
            self.control.get("actual_pose_topic", "/hc_teleop/actual_ee_poses"),
            10,
        )
        self.base_pub = self.create_publisher(
            Float64MultiArray, self.body_config["base_command_topic"], 10
        )
        self.status_pub = self.create_publisher(
            String, self.control["status_topic"], 10
        )
        self.finger_pubs = {
            side: self.create_publisher(
                JointState, self.config["grippers"][side]["finger_topic"], 10
            )
            for side in ("left", "right")
        }
        self.create_subscription(
            JointState,
            self.control["joint_state_topic"],
            self._joint_state_callback,
            10,
        )
        if self.external_ik:
            self.create_subscription(
                JointState,
                self.control.get("solver_topic", "/hc_teleop/sol_q"),
                self._solver_callback,
                10,
            )
            self.create_subscription(
                JointState,
                self.control.get(
                    "generic_command_topic", "/hc_teleop/joint_cmd_arm"
                ),
                self._generic_command_callback,
                10,
            )
        self.create_subscription(
            Bool,
            self.control["safety_stop_topic"],
            self._stop_callback,
            10,
        )
        self.create_subscription(
            Bool,
            self.control["enable_topic"],
            self._enabled_callback,
            10,
        )
        self.create_subscription(
            String,
            self.control.get("vr_data_topic", "/vrdata"),
            self._vr_data_callback,
            qos_profile_sensor_data,
        )
        self.create_service(Trigger, "/teleop/arm/enable", self._enable_service)
        self.create_service(Trigger, "/teleop/arm/disable", self._disable_service)
        self.create_service(Trigger, "/teleop/arm/home", self._home_service)
        self.create_subscription(Bool, "/teleop/arm/home", self._home_topic_callback, 10)
        self.create_service(
            Trigger, "/teleop/arm/reset_reference", self._reset_reference_service
        )
        self.timer = self.create_timer(
            1.0 / float(self.control["rate_hz"]), self._control_tick
        )
        self.get_logger().info(
            "HC-TJ teleop ready: left Grip=base+waist, right Grip=both arms+grippers, "
            f"backend={self.backend}, command={self.control['command_topic']}, "
            f"enabled={self.enabled}"
        )

    def _validate_config(self) -> None:
        if not isinstance(self.config, dict):
            raise ValueError("teleop config must be a YAML object")
        for section in ("robot", "control", "arms", "body", "grippers"):
            if section not in self.config:
                raise ValueError(f"missing config section: {section}")
        if set(self.config["arms"]) != {"left", "right"}:
            raise ValueError("arms must contain exactly left and right")
        rate = float(self.config["control"].get("rate_hz", 0.0))
        if not math.isfinite(rate) or rate <= 0.0:
            raise ValueError("control.rate_hz must be positive")
        vr_data_topic = self.config["control"].get("vr_data_topic", "/vrdata")
        if not isinstance(vr_data_topic, str) or not vr_data_topic.startswith("/"):
            raise ValueError("control.vr_data_topic must start with /")
        for key in ("axis_mapping", "head_axis_mapping"):
            mapping = np.asarray(
                self.config["control"].get(
                    key, self.config["control"]["axis_mapping"]
                ),
                dtype=float,
            )
            if mapping.shape != (3, 3) or not np.all(np.isfinite(mapping)):
                raise ValueError(f"control.{key} must be finite 3x3")
        for key in (
            "target_position_deadband",
            "target_orientation_deadband",
            "command_joint_deadband",
        ):
            value = float(self.config["control"].get(key, 0.0))
            if not math.isfinite(value) or value < 0.0:
                raise ValueError(f"control.{key} must be finite and non-negative")
        filter_alpha = float(self.config["control"].get("target_filter_alpha", 1.0))
        if not math.isfinite(filter_alpha) or not 0.0 < filter_alpha <= 1.0:
            raise ValueError("control.target_filter_alpha must be in (0, 1]")
        for key in (
            "stick_forward_direction",
            "stick_lateral_direction",
            "yaw_direction",
            "height_direction",
            "pitch_direction",
        ):
            value = float(self.config["body"].get(key, 1.0))
            if value not in (-1.0, 1.0):
                raise ValueError(f"body.{key} must be -1 or 1")
        for key in ("stick_x_axis", "stick_y_axis"):
            value = self.config["body"].get(key)
            if isinstance(value, bool) or not isinstance(value, int) or value < 0:
                raise ValueError(f"body.{key} must be a non-negative integer")
        stick_deadzone = float(self.config["body"].get("stick_deadzone", -1.0))
        if not math.isfinite(stick_deadzone) or not 0.0 <= stick_deadzone < 1.0:
            raise ValueError("body.stick_deadzone must be finite and in [0, 1)")
        for key in ("max_linear_velocity", "max_lateral_velocity"):
            value = float(self.config["body"].get(key, -1.0))
            if not math.isfinite(value) or value < 0.0:
                raise ValueError(f"body.{key} must be finite and non-negative")
        if self.config["body"].get("base_command_mode", "delta") not in {
            "delta",
            "velocity",
        }:
            raise ValueError("body.base_command_mode must be delta or velocity")
        for key in (
            "ik_position_tolerance",
            "ik_orientation_tolerance",
            "ik_damping_min",
            "ik_damping_max",
            "ik_singularity_threshold",
            "ik_max_update",
            "ik_reseed_interval",
            "ik_reseed_perturbation",
            "ik_failure_retry_interval",
            "max_joint_velocity",
            "max_joint_velocity_norm",
            "ik_reseed_target_position_delta",
            "ik_reseed_target_orientation_delta",
        ):
            value = float(self.config["control"].get(key, 0.0))
            if not math.isfinite(value) or value <= 0.0:
                raise ValueError(f"control.{key} must be finite and positive")
        if float(self.config["control"]["ik_damping_max"]) < float(
            self.config["control"]["ik_damping_min"]
        ):
            raise ValueError("control.ik_damping_max must be >= ik_damping_min")
        for key in ("ik_limit_margin", "ik_reseed_limit_margin"):
            value = float(self.config["control"].get(key, 0.0))
            if not math.isfinite(value) or not 0.0 < value < 0.5:
                raise ValueError(f"control.{key} must be in (0, 0.5)")
        for key in (
            "ik_limit_avoidance_gain",
            "ik_posture_gain",
            "ik_solution_limit_weight",
            "ik_min_progress_ratio",
            "ik_min_progress_absolute",
            "ik_reseed_escape_progress_ratio",
            "ik_reseed_escape_max_regression",
        ):
            value = float(self.config["control"].get(key, -1.0))
            if not math.isfinite(value) or value < 0.0:
                raise ValueError(f"control.{key} must be finite and non-negative")
        if int(self.config["control"].get("ik_iterations", 0)) <= 0:
            raise ValueError("control.ik_iterations must be positive")
        home_threshold = float(self.config["control"].get("home_gesture_threshold", 0.8))
        home_release = float(
            self.config["control"].get("home_gesture_release_threshold", 0.35)
        )
        if not 0.0 < home_threshold <= 1.0:
            raise ValueError("control.home_gesture_threshold must be in (0, 1]")
        if not 0.0 <= home_release < home_threshold:
            raise ValueError(
                "control.home_gesture_release_threshold must be non-negative "
                "and below home_gesture_threshold"
            )
        for key in ("home_joint_velocity", "home_tolerance"):
            value = float(self.config["control"].get(key, 0.0))
            if not math.isfinite(value) or value <= 0.0:
                raise ValueError(f"control.{key} must be finite and positive")
        arm_names = [
            name
            for arm_config in self.config["arms"].values()
            for name in arm_config["joint_names"]
        ]
        missing_home = [name for name in arm_names if name not in self.config["robot"].get("initial_joints", {})]
        if missing_home:
            raise ValueError(
                "robot.initial_joints lacks arm home targets: " + ", ".join(missing_home)
            )
        urdf_path = Path(self.config["robot"]["urdf_path"]).expanduser()
        if not urdf_path.is_absolute():
            urdf_path = self.config_path.parent / urdf_path
        if not urdf_path.is_file():
            raise ValueError("robot.urdf_path does not exist")

    def _joint_group(self, names: list[str]) -> tuple[list[int], list[int], np.ndarray, np.ndarray]:
        try:
            indices = [self.joint_by_name[name] for name in names]
        except KeyError as exc:
            raise ValueError(f"joint not found in URDF: {exc}") from exc
        lower, upper = [], []
        for index in indices:
            info = bullet.getJointInfo(
                self.robot_id, index, physicsClientId=self.physics_client
            )
            lower.append(float(info[8]))
            upper.append(float(info[9]))
        return (
            indices,
            [self.dof_by_joint[index] for index in indices],
            np.asarray(lower),
            np.asarray(upper),
        )

    def _create_arm(self, side: str, config: dict[str, Any]) -> ArmRuntime:
        names = list(config["joint_names"])
        indices, dofs, lower, upper = self._joint_group(names)
        try:
            ee_index = self.link_by_name[config["ee_link"]]
            base_index = self.link_by_name[config["base_link"]]
        except KeyError as exc:
            raise ValueError(f"{side} link not found in URDF: {exc}") from exc
        return ArmRuntime(
            side, names, indices, dofs, lower, upper, ee_index, base_index
        )

    def _create_body(self, config: dict[str, Any]) -> BodyRuntime:
        names = list(config["waist_joint_names"])
        indices, dofs, lower, upper = self._joint_group(names)
        if "joint_lower" in config:
            lower = np.asarray(config["joint_lower"], dtype=float)
        if "joint_upper" in config:
            upper = np.asarray(config["joint_upper"], dtype=float)
        return BodyRuntime(
            names,
            indices,
            dofs,
            lower,
            upper,
            self.link_by_name[config["torso_link"]],
        )

    @staticmethod
    def _monotonic() -> float:
        return time.monotonic()

    @staticmethod
    def _read_pose(message: PoseStamped) -> tuple[np.ndarray, np.ndarray] | None:
        pose = message.pose
        values = np.asarray(
            [
                pose.position.x,
                pose.position.y,
                pose.position.z,
                pose.orientation.x,
                pose.orientation.y,
                pose.orientation.z,
                pose.orientation.w,
            ],
            dtype=float,
        )
        norm = float(np.linalg.norm(values[3:]))
        if not np.all(np.isfinite(values)) or norm < 1e-8:
            return None
        return values[:3], values[3:] / norm

    def _arm_pose_callback(self, arm: ArmRuntime, message: PoseStamped) -> None:
        pose = self._read_pose(message)
        if pose is not None:
            arm.pose = pose
            arm.pose_stamp = self._monotonic()

    def _head_pose_callback(self, message: PoseStamped) -> None:
        pose = self._read_pose(message)
        if pose is not None:
            self.body.head_pose = pose
            self.body.head_stamp = self._monotonic()

    def _joy_callback(self, arm: ArmRuntime, message: Joy) -> None:
        if all(math.isfinite(float(value)) for value in message.axes):
            arm.joy = message
            arm.joy_stamp = self._monotonic()

    def _vr_data_callback(self, message: String) -> None:
        try:
            value = json.loads(message.data)
            if not isinstance(value, dict):
                return
            tracking = value.get("tracking", {})
            poses = value.get("poses", {})
            inputs = value.get("inputs", {})
            if not all(isinstance(item, dict) for item in (tracking, poses, inputs)):
                return

            def pose_message(name: str) -> PoseStamped | None:
                source = poses.get(name)
                if not tracking.get(name) or not isinstance(source, dict):
                    return None
                position = source.get("position")
                quaternion = source.get("quaternion")
                if not isinstance(position, list) or not isinstance(quaternion, list):
                    return None
                if len(position) != 3 or len(quaternion) != 4:
                    return None
                result = PoseStamped()
                (
                    result.pose.position.x,
                    result.pose.position.y,
                    result.pose.position.z,
                ) = [float(item) for item in position]
                (
                    result.pose.orientation.x,
                    result.pose.orientation.y,
                    result.pose.orientation.z,
                    result.pose.orientation.w,
                ) = [float(item) for item in quaternion]
                return result

            head = pose_message("head")
            if head is not None:
                self._head_pose_callback(head)
            for side in ("left", "right"):
                pose = pose_message(side)
                if pose is not None:
                    self._arm_pose_callback(self.arms[side], pose)
                source = inputs.get(side)
                if not isinstance(source, dict):
                    continue
                primary = source.get("primary_axis", [0.0, 0.0])
                secondary = source.get("secondary_axis", [0.0, 0.0])
                if not isinstance(primary, list) or len(primary) != 2:
                    continue
                if not isinstance(secondary, list) or len(secondary) != 2:
                    continue
                held_mask = int(source.get("held_mask", 0))
                pressed_mask = int(source.get("pressed_mask", 0))
                if side == "right" and ((pressed_mask & 1) or (held_mask & 1)):
                    if not self.enabled or self.stop_reason:
                        self.enabled = True
                        self.stop_reason = ""
                        self.get_logger().info("VR controller A button pressed: teleop re-enabled")
                joy = Joy()
                joy.axes = [
                    float(source.get("trigger", 0.0)),
                    float(source.get("grip", 0.0)),
                    float(primary[0]),
                    float(primary[1]),
                    float(secondary[0]),
                    float(secondary[1]),
                ]
                joy.buttons = [int(bool(held_mask & (1 << index))) for index in range(11)]
                self._joy_callback(self.arms[side], joy)
        except (TypeError, ValueError, json.JSONDecodeError):
            return

    def _joint_state_callback(self, message: JointState) -> None:
        if len(message.name) != len(message.position):
            return
        values = dict(zip(message.name, message.position))
        if not all(math.isfinite(float(value)) for value in values.values()):
            return
        self.joint_state = {name: float(value) for name, value in values.items()}
        self.joint_state_stamp = self._monotonic()
        if not self.last_command:
            self.last_command = dict(self.joint_state)

    def _solver_callback(self, message: JointState) -> None:
        if len(message.name) != len(message.position):
            return
        if not all(math.isfinite(float(value)) for value in message.position):
            return
        self.solver_stamp = self._monotonic()
        self.solver_message_count += 1

    def _generic_command_callback(self, message: JointState) -> None:
        if len(message.name) != len(message.position):
            return
        if not all(math.isfinite(float(value)) for value in message.position):
            return
        now = self._monotonic()
        previous_stamp = self.generic_command_stamp
        self.generic_command_stamp = now
        self.generic_command_count += 1
        if not self.enabled:
            return
        merged = dict(zip(message.name, (float(value) for value in message.position)))
        if self.homing:
            nominal_period = 1.0 / float(self.control["rate_hz"])
            elapsed = nominal_period if previous_stamp <= 0.0 else now - previous_stamp
            elapsed = float(np.clip(elapsed, 0.001, 0.05))
            max_step = float(self.control.get("home_joint_velocity", 0.7)) * elapsed
            for arm in self.arms.values():
                for name in arm.joint_names:
                    target = float(self.initial_joints[name])
                    previous = float(
                        self.last_command.get(
                            name, self.joint_state.get(name, target)
                        )
                    )
                    step = float(np.clip(target - previous, -max_step, max_step))
                    merged[name] = previous + step
        else:
            arms_active = any(self.arms[side].active for side in ("right", "left"))
            if not arms_active:
                for arm in self.arms.values():
                    for name in arm.joint_names:
                        merged[name] = float(
                            self.last_command.get(
                                name, self.joint_state.get(name, self.initial_joints.get(name, 0.0))
                            )
                        )
        merged.update(self.generic_aux_command)
        output = JointState()
        output.header = message.header
        output.name = list(merged)
        output.position = [merged[name] for name in output.name]
        self.command_pub.publish(output)
        self.last_command.update(merged)

    def _stop_callback(self, message: Bool) -> None:
        if message.data:
            self.enabled = False
            self.stop_reason = "emergency stop topic"
            self._release_all(send_base_zero=True)
            self.get_logger().error("Teleop disabled by emergency stop")

    def _enabled_callback(self, message: Bool) -> None:
        self.enabled = bool(message.data)
        self.stop_reason = "" if self.enabled else "disabled by command topic"
        self._release_all(send_base_zero=True)
        self.get_logger().info(
            f"Teleop {'enabled' if self.enabled else 'disabled'} by command topic"
        )

    def _enable_service(self, _request: Trigger.Request, response: Trigger.Response):
        self.enabled = True
        self.stop_reason = ""
        self._release_all(send_base_zero=True)
        response.success = True
        response.message = "teleop enabled; use left/right Grip clutches"
        return response

    def _disable_service(self, _request: Trigger.Request, response: Trigger.Response):
        self.enabled = False
        self.stop_reason = "disabled by service"
        self._release_all(send_base_zero=True)
        response.success = True
        response.message = "teleop disabled"
        return response

    def _home_service(self, _request: Trigger.Request, response: Trigger.Response):
        self.enabled = True
        self.stop_reason = ""
        if self.backend in {"generic", "v23"}:
            self._set_generic_home_targets()
        else:
            self._start_arm_homing()
        response.success = True
        response.message = "homing sequence started"
        return response

    def _home_topic_callback(self, message: Bool):
        if message.data:
            self.enabled = True
            self.stop_reason = ""
            if self.backend in {"generic", "v23"}:
                self._set_generic_home_targets()
            else:
                self._start_arm_homing()

    def _reset_reference_service(
        self, _request: Trigger.Request, response: Trigger.Response
    ):
        self._release_all(send_base_zero=True)
        response.success = True
        response.message = "references cleared; release both Grip controls"
        return response

    def _release_arm(self, arm: ArmRuntime) -> None:
        arm.active = False
        arm.reference_vr = None
        arm.reference_local_ee = None
        arm.target_local = None
        arm.target_world = None
        arm.recovery_solution = None
        arm.recovery_seed = ""
        arm.reseed_target_position = None
        arm.reseed_target_orientation = None

    def _release_all(self, *, send_base_zero: bool) -> None:
        for arm in self.arms.values():
            self._release_arm(arm)
        self.body.active = False
        self.body.reference_head = None
        self.body.reference_torso = None
        if send_base_zero:
            self.base_pub.publish(Float64MultiArray(data=[0.0, 0.0, 0.0]))

    def _axis(self, arm: ArmRuntime, index: int) -> float:
        if arm.joy is None or len(arm.joy.axes) <= index:
            return 0.0
        return float(arm.joy.axes[index])

    def _home_gesture_triggered(self, now: float) -> bool:
        if not all(self._input_fresh(self.arms[side], now) for side in ("left", "right")):
            return False
        axis = int(self.control.get("home_gesture_axis", 2))
        left_x = self._axis(self.arms["left"], axis)
        right_x = self._axis(self.arms["right"], axis)
        threshold = float(self.control.get("home_gesture_threshold", 0.8))
        release = float(self.control.get("home_gesture_release_threshold", 0.35))
        if self.home_gesture_latched:
            if abs(left_x) <= release and abs(right_x) <= release:
                self.home_gesture_latched = False
            return False
        if sticks_outward(left_x, right_x, threshold):
            self.home_gesture_latched = True
            return True
        return False

    def _start_arm_homing(self) -> None:
        self._release_all(send_base_zero=True)
        self.base_zero_pending = False
        for arm in self.arms.values():
            for name in arm.joint_names:
                if name in self.joint_state:
                    self.last_command[name] = self.joint_state[name]
        self.homing = True
        self.get_logger().info(
            "both sticks outward: homing both arms to initial_joints"
        )

    def _update_arm_homing(self, command: dict[str, float]) -> None:
        names = self.arms["right"].joint_names + self.arms["left"].joint_names
        target = np.asarray([self.initial_joints[name] for name in names], dtype=float)
        previous = np.asarray(
            [self.last_command.get(name, self.joint_state[name]) for name in names],
            dtype=float,
        )
        max_step = float(self.control["home_joint_velocity"]) / float(
            self.control["rate_hz"]
        )
        next_values = previous + np.clip(target - previous, -max_step, max_step)
        for name, value in zip(names, next_values):
            command[name] = float(value)

        feedback_error = max(
            abs(self.joint_state[name] - self.initial_joints[name]) for name in names
        )
        if (
            np.max(np.abs(next_values - target))
            <= float(self.control["home_tolerance"])
            and feedback_error <= float(self.control["home_tolerance"])
        ):
            for name, value in zip(names, target):
                command[name] = float(value)
            self.homing = False
            self.get_logger().info("both arms homing complete")

    def _input_fresh(self, arm: ArmRuntime, now: float) -> bool:
        return arm.joy is not None and now - arm.joy_stamp <= float(
            self.control["input_timeout"]
        )

    def _clutch_pressed(self, arm: ArmRuntime) -> bool:
        if arm.joy is None:
            return False
        index = int(self.control["clutch_axis"])
        return len(arm.joy.axes) > index and arm.joy.axes[index] >= float(
            self.control["clutch_threshold"]
        )

    def _sync_model(self) -> None:
        for name, value in self.joint_state.items():
            index = self.joint_by_name.get(name)
            if index is not None and index in self.dof_by_joint:
                bullet.resetJointState(
                    self.robot_id,
                    index,
                    value,
                    physicsClientId=self.physics_client,
                )

    def _set_group(self, indices: list[int], values: np.ndarray) -> None:
        for index, value in zip(indices, values):
            bullet.resetJointState(
                self.robot_id,
                index,
                float(value),
                physicsClientId=self.physics_client,
            )

    def _link_pose(self, index: int) -> tuple[np.ndarray, np.ndarray]:
        state = bullet.getLinkState(
            self.robot_id,
            index,
            computeForwardKinematics=True,
            physicsClientId=self.physics_client,
        )
        return np.asarray(state[4], dtype=float), np.asarray(state[5], dtype=float)

    def _root_pose(self) -> tuple[np.ndarray, np.ndarray]:
        position, orientation = bullet.getBasePositionAndOrientation(
            self.robot_id, physicsClientId=self.physics_client
        )
        return np.asarray(position, dtype=float), np.asarray(orientation, dtype=float)

    def _relative_pose(
        self, parent: tuple[np.ndarray, np.ndarray], child: tuple[np.ndarray, np.ndarray]
    ) -> tuple[np.ndarray, np.ndarray]:
        inverse = bullet.invertTransform(parent[0], parent[1])
        result = bullet.multiplyTransforms(inverse[0], inverse[1], child[0], child[1])
        return np.asarray(result[0]), np.asarray(result[1])

    def _world_pose(
        self, parent: tuple[np.ndarray, np.ndarray], local: tuple[np.ndarray, np.ndarray]
    ) -> tuple[np.ndarray, np.ndarray]:
        result = bullet.multiplyTransforms(parent[0], parent[1], local[0], local[1])
        return np.asarray(result[0]), np.asarray(result[1])

    def _all_dof_positions(self) -> list[float]:
        return [
            float(
                bullet.getJointState(
                    self.robot_id, index, physicsClientId=self.physics_client
                )[0]
            )
            for index in self.dof_joint_indices
        ]

    def _jacobian(self, link_index: int) -> tuple[np.ndarray, np.ndarray]:
        value = bullet.calculateJacobian(
            self.robot_id,
            link_index,
            [0.0, 0.0, 0.0],
            self._all_dof_positions(),
            self.zeros,
            self.zeros,
            physicsClientId=self.physics_client,
        )
        return np.asarray(value[0], dtype=float), np.asarray(value[1], dtype=float)

    def _evaluate_arm_candidate(
        self,
        arm: ArmRuntime,
        target_position: np.ndarray,
        target_orientation: np.ndarray,
        joints: np.ndarray,
        reference: np.ndarray,
        seed_name: str,
        damping: float,
    ) -> IkCandidate:
        q = np.clip(np.asarray(joints, dtype=float), arm.lower, arm.upper)
        self._set_group(arm.joint_indices, q)
        current_position, current_orientation = self._link_pose(arm.ee_index)
        position_error = float(np.linalg.norm(target_position - current_position))
        rotation_error = float(
            np.linalg.norm(orientation_error(target_orientation, current_orientation))
        )
        if not bool(self.control["orientation_enabled"]):
            rotation_error = 0.0
        position_tolerance = float(self.control["ik_position_tolerance"])
        orientation_tolerance = float(self.control["ik_orientation_tolerance"])
        span = arm.upper - arm.lower
        normalized_margin = np.minimum(
            (q - arm.lower) / span, (arm.upper - q) / span
        )
        minimum_margin = float(np.min(normalized_margin))
        transition = float(np.linalg.norm((q - reference) / span))
        preferred_margin = float(self.control["ik_reseed_limit_margin"])
        limit_cost = float(
            np.sum(np.maximum(preferred_margin - normalized_margin, 0.0) ** 2)
        )
        score = transition + float(
            self.control.get("ik_solution_limit_weight", 4.0)
        ) * limit_cost
        return IkCandidate(
            q,
            position_error,
            rotation_error,
            position_error <= position_tolerance
            and rotation_error <= orientation_tolerance,
            position_error <= position_tolerance
            and rotation_error <= orientation_tolerance,
            minimum_margin,
            damping,
            seed_name,
            score,
        )

    def _solve_arm_seed(
        self,
        arm: ArmRuntime,
        target_position: np.ndarray,
        target_orientation: np.ndarray,
        seed: np.ndarray,
        reference: np.ndarray,
        seed_name: str,
    ) -> IkCandidate:
        q = np.clip(np.asarray(seed, dtype=float), arm.lower, arm.upper)
        minimum_damping = float(self.control["ik_damping_min"])
        maximum_damping = float(self.control["ik_damping_max"])
        singularity_threshold = float(self.control["ik_singularity_threshold"])
        max_update = float(self.control["ik_max_update"])
        orientation_enabled = bool(self.control["orientation_enabled"])
        orientation_weight = float(self.control["orientation_weight"])
        position_tolerance = float(self.control["ik_position_tolerance"])
        orientation_tolerance = float(self.control["ik_orientation_tolerance"])
        limit_margin = float(self.control["ik_limit_margin"])
        limit_gain = float(self.control["ik_limit_avoidance_gain"])
        posture_gain = float(self.control["ik_posture_gain"])
        best = self._evaluate_arm_candidate(
            arm,
            target_position,
            target_orientation,
            q,
            reference,
            seed_name,
            minimum_damping,
        )
        largest_damping = minimum_damping
        for _ in range(int(self.control["ik_iterations"])):
            self._set_group(arm.joint_indices, q)
            current_position, current_orientation = self._link_pose(arm.ee_index)
            position_delta = target_position - current_position
            rotation_delta = orientation_error(target_orientation, current_orientation)
            position_norm = float(np.linalg.norm(position_delta))
            orientation_norm = (
                float(np.linalg.norm(rotation_delta)) if orientation_enabled else 0.0
            )
            if (
                position_norm <= position_tolerance
                and orientation_norm <= orientation_tolerance
            ):
                break
            error = np.concatenate(
                [
                    position_delta,
                    rotation_delta * orientation_weight
                    if orientation_enabled
                    else np.zeros(3),
                ]
            )
            linear, angular = self._jacobian(arm.ee_index)
            matrix = np.vstack(
                [
                    linear[:, arm.dof_indices],
                    angular[:, arm.dof_indices] * orientation_weight,
                ]
            )
            singular_values = np.linalg.svd(matrix, compute_uv=False)
            damping = adaptive_damping(
                float(np.min(singular_values)),
                minimum_damping,
                maximum_damping,
                singularity_threshold,
            )
            largest_damping = max(largest_damping, damping)
            regularized = matrix @ matrix.T + np.eye(6) * damping * damping
            solved_error = np.linalg.solve(regularized, error)
            pseudo_inverse = matrix.T @ np.linalg.solve(regularized, np.eye(6))
            update = matrix.T @ solved_error
            nullspace = np.eye(len(q)) - pseudo_inverse @ matrix
            avoidance = joint_limit_avoidance(
                q, arm.lower, arm.upper, limit_margin
            )
            posture = reference - q
            update += nullspace @ (
                avoidance * limit_gain + posture * posture_gain
            )
            norm = float(np.linalg.norm(update))
            if norm > max_update:
                update *= max_update / norm
            q = np.clip(q + update, arm.lower, arm.upper)
            candidate = self._evaluate_arm_candidate(
                arm,
                target_position,
                target_orientation,
                q,
                reference,
                seed_name,
                largest_damping,
            )
            candidate_error = (
                candidate.position_error / position_tolerance
                + candidate.orientation_error / orientation_tolerance
            )
            best_error = (
                best.position_error / position_tolerance
                + best.orientation_error / orientation_tolerance
            )
            if candidate.converged or candidate_error < best_error:
                best = candidate
            if candidate.converged:
                break
        return best

    def _solve_arm_ik(
        self,
        arm: ArmRuntime,
        target_position: np.ndarray,
        target_orientation: np.ndarray,
        previous: np.ndarray,
    ) -> IkCandidate:
        reference = np.clip(np.asarray(previous, dtype=float), arm.lower, arm.upper)
        now = self._monotonic()
        if (
            arm.recovery_solution is None
            and arm.ik_rejections > 0
            and now - arm.last_ik_attempt < float(
                self.control["ik_failure_retry_interval"]
            )
        ):
            held = self._evaluate_arm_candidate(
                arm,
                target_position,
                target_orientation,
                reference,
                reference,
                "hold_last_valid",
                max(arm.ik_damping, float(self.control["ik_damping_min"])),
            )
            held.converged = False
            held.accepted = False
            return held
        arm.last_ik_attempt = now
        primary = self._solve_arm_seed(
            arm,
            target_position,
            target_orientation,
            reference,
            reference,
            "previous",
        )
        preferred_margin = float(self.control["ik_reseed_limit_margin"])
        candidates = [primary]
        if arm.recovery_solution is not None:
            candidates.append(
                self._evaluate_arm_candidate(
                    arm,
                    target_position,
                    target_orientation,
                    arm.recovery_solution,
                    reference,
                    f"recovery_{arm.recovery_seed}",
                    max(primary.damping, arm.ik_damping),
                )
            )
        needs_reseed = (
            not primary.converged
            or primary.minimum_limit_margin < preferred_margin
        )
        target_changed_since_reseed = (
            arm.reseed_target_position is None
            or arm.reseed_target_orientation is None
            or float(np.linalg.norm(target_position - arm.reseed_target_position))
            >= float(self.control["ik_reseed_target_position_delta"])
            or float(
                np.linalg.norm(
                    orientation_error(
                        target_orientation, arm.reseed_target_orientation
                    )
                )
            )
            >= float(self.control["ik_reseed_target_orientation_delta"])
        )
        reseed_due = target_changed_since_reseed and now - arm.last_reseed_attempt >= float(
            self.control["ik_reseed_interval"]
        )
        if needs_reseed and reseed_due:
            arm.last_reseed_attempt = now
            arm.reseed_target_position = target_position.copy()
            arm.reseed_target_orientation = target_orientation.copy()
            feedback = np.asarray(
                [self.joint_state[name] for name in arm.joint_names], dtype=float
            )
            home = np.asarray(
                [self.initial_joints[name] for name in arm.joint_names], dtype=float
            )
            perturbation = float(self.control["ik_reseed_perturbation"])
            branch_positive = reference.copy()
            branch_negative = reference.copy()
            branch_positive[[0, 2, 3]] += np.asarray(
                [perturbation, -perturbation, 0.5 * perturbation]
            )
            branch_negative[[0, 2, 3]] -= np.asarray(
                [perturbation, -perturbation, 0.5 * perturbation]
            )
            seed_options = (
                ("feedback", feedback),
                ("home_blend", 0.5 * (reference + home)),
                ("home", home),
                ("branch_positive", branch_positive),
                ("branch_negative", branch_negative),
            )
            for seed_name, seed in seed_options:
                if any(
                    np.linalg.norm(np.asarray(seed) - candidate.joints) < 1e-6
                    for candidate in candidates
                ):
                    continue
                candidates.append(
                    self._solve_arm_seed(
                        arm,
                        target_position,
                        target_orientation,
                        seed,
                        reference,
                        seed_name,
                    )
                )
        held = self._evaluate_arm_candidate(
            arm,
            target_position,
            target_orientation,
            reference,
            reference,
            "hold_last_valid",
            max(candidate.damping for candidate in candidates),
        )
        held.converged = bool(held.converged)

        # The hardware controller solves a velocity-level tracking problem every
        # 10 ms.  Requiring an absolute pose solution to be fully converged in a
        # single tick turns normal tracking lag into a permanent rejection: the
        # target keeps moving while the last command is held.  Evaluate the
        # velocity-limited *next command* instead.  A partial solution is safe to
        # publish when it measurably reduces the normalized task error; genuine
        # failures still fall through to hold_last_valid.
        max_step = float(self.control["max_joint_velocity"]) / float(
            self.control["rate_hz"]
        )
        position_tolerance = float(self.control["ik_position_tolerance"])
        orientation_tolerance = float(self.control["ik_orientation_tolerance"])

        def task_error(candidate: IkCandidate) -> float:
            return math.hypot(
                candidate.position_error / position_tolerance,
                candidate.orientation_error / orientation_tolerance,
            )

        held_error = task_error(held)
        minimum_ratio = float(self.control.get("ik_min_progress_ratio", 0.001))
        minimum_absolute = float(
            self.control.get("ik_min_progress_absolute", 0.0001)
        )
        required_progress = max(minimum_absolute, held_error * minimum_ratio)
        safe_steps: list[tuple[IkCandidate, IkCandidate, bool]] = []
        for candidate in candidates:
            limited_joints = clamp_step(
                candidate.joints,
                reference,
                arm.lower,
                arm.upper,
                max_step,
            )
            step = limited_joints - reference
            step_norm = float(np.linalg.norm(step))
            max_norm_step = float(
                self.control.get(
                    "max_joint_velocity_norm", self.control["max_joint_velocity"]
                )
            ) / float(self.control["rate_hz"])
            if step_norm > max_norm_step:
                limited_joints = reference + step * (max_norm_step / step_norm)
            limited = self._evaluate_arm_candidate(
                arm,
                target_position,
                target_orientation,
                limited_joints,
                reference,
                candidate.seed_name,
                candidate.damping,
            )
            progress = held_error - task_error(limited)
            candidate_progress = held_error - task_error(candidate)
            escape_progress = max(
                required_progress,
                held_error
                * float(
                    self.control.get("ik_reseed_escape_progress_ratio", 0.05)
                ),
            )
            escape_regression = float(
                self.control.get("ik_reseed_escape_max_regression", 0.25)
            )
            reseed_escape = bool(
                candidate.seed_name not in {"previous", "feedback"}
                and candidate_progress >= escape_progress
                and task_error(limited) <= held_error + escape_regression
            )
            limited.accepted = bool(
                limited.converged
                or (
                    np.all(np.isfinite(limited.joints))
                    and (progress >= required_progress or reseed_escape)
                )
            )
            if limited.accepted:
                # Prefer actual tracking progress first.  The transition/limit
                # score remains a deterministic tie-breaker between IK branches.
                limited.score = task_error(limited) + 1e-3 * candidate.score
                safe_steps.append((limited, candidate, reseed_escape))

        if safe_steps:
            def continuity_rank(item: tuple[IkCandidate, IkCandidate, bool]):
                seed_name = item[1].seed_name
                rank = (
                    0
                    if seed_name == "previous"
                    else 1
                    if seed_name.startswith("recovery_")
                    else 2
                )
                return rank, item[0].score

            selected, full_candidate, _used_escape = min(
                safe_steps, key=continuity_rank
            )
            if full_candidate.seed_name == "previous":
                # A local DLS step can be temporarily better than the cached
                # nonlinear branch.  Keep that branch available until the task
                # is actually reached, otherwise the arm stalls again as soon
                # as the local Jacobian loses the axial direction.
                if selected.converged:
                    arm.recovery_solution = None
                    arm.recovery_seed = ""
            elif full_candidate.seed_name.startswith("recovery_"):
                if selected.converged:
                    arm.recovery_solution = None
                    arm.recovery_seed = ""
            elif not selected.converged:
                # Follow the selected nonlinear branch at the control rate.  The
                # expensive multi-seed search only refreshes this waypoint at
                # ik_reseed_interval, matching the hardware split between its
                # optimizer (sol_q) and smooth joint controller.
                arm.recovery_solution = full_candidate.joints.copy()
                arm.recovery_seed = full_candidate.seed_name
            self._set_group(arm.joint_indices, selected.joints)
            return selected

        arm.recovery_solution = None
        arm.recovery_seed = ""
        held.converged = False
        held.accepted = False
        self._set_group(arm.joint_indices, reference)
        return held

    def _solve_waist(
        self,
        target_height: float,
        target_orientation: np.ndarray,
        seed: np.ndarray,
    ) -> np.ndarray:
        body = self.body
        q = np.clip(np.asarray(seed, dtype=float), body.lower, body.upper)
        pitch_axis = np.asarray(self.body_config["pitch_axis_world"], dtype=float)
        pitch_axis /= np.linalg.norm(pitch_axis)
        damping = float(self.body_config["ik_damping"])
        for _ in range(int(self.body_config["ik_iterations"])):
            self._set_group(body.joint_indices, q)
            position, orientation = self._link_pose(body.torso_index)
            rotation_delta = orientation_error(target_orientation, orientation)
            error = np.asarray(
                [target_height - position[2], float(pitch_axis @ rotation_delta)]
            )
            if np.linalg.norm(error) < 0.002:
                break
            linear, angular = self._jacobian(body.torso_index)
            matrix = np.vstack(
                [
                    linear[2, body.dof_indices],
                    pitch_axis @ angular[:, body.dof_indices],
                ]
            )
            update = matrix.T @ np.linalg.solve(
                matrix @ matrix.T + np.eye(2) * damping * damping, error
            )
            norm = float(np.linalg.norm(update))
            max_update = float(self.body_config["ik_max_update"])
            if norm > max_update:
                update *= max_update / norm
            q = np.clip(q + update, body.lower, body.upper)
        self._set_group(body.joint_indices, q)
        position, orientation = self._link_pose(body.torso_index)
        body.lift_error = abs(target_height - float(position[2]))
        body.pitch_error = abs(float(pitch_axis @ orientation_error(target_orientation, orientation)))
        return q

    @staticmethod
    def _yaw(quaternion: np.ndarray) -> float:
        x, y, z, w = quaternion
        return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))

    @staticmethod
    def _wrap_angle(value: float) -> float:
        return (value + math.pi) % (2.0 * math.pi) - math.pi

    def _engage_body(self) -> None:
        body = self.body
        body.active = True
        body.reference_head = body.head_pose
        body.reference_torso = self._link_pose(body.torso_index)
        body.last_solution = np.asarray(
            [self.joint_state[name] for name in body.joint_names], dtype=float
        )
        self.get_logger().info("left clutch engaged: base + waist")

    def _update_body(self, command: dict[str, float]) -> None:
        body = self.body
        assert body.head_pose is not None
        assert body.reference_head is not None
        assert body.reference_torso is not None
        delta_height = float(
            body.head_pose[0][int(self.body_config["head_height_axis"])]
            - body.reference_head[0][int(self.body_config["head_height_axis"])]
        )
        delta_pitch = float(
            body.head_pose[0][int(self.body_config["head_pitch_position_axis"])]
            - body.reference_head[0][int(self.body_config["head_pitch_position_axis"])]
        )
        lift = np.clip(
            delta_height
            * float(self.body_config.get("height_direction", 1.0))
            * float(self.body_config["height_scale"]),
            -float(self.body_config["max_height_delta"]),
            float(self.body_config["max_height_delta"]),
        )
        pitch = np.clip(
            delta_pitch
            * float(self.body_config.get("pitch_direction", 1.0))
            * float(self.body_config["pitch_scale"]),
            -float(self.body_config["max_pitch_delta"]),
            float(self.body_config["max_pitch_delta"]),
        )
        target_height = float(body.reference_torso[0][2] + lift)
        target_orientation = quaternion_multiply(
            quaternion_from_axis_angle(self.body_config["pitch_axis_world"], pitch),
            body.reference_torso[1],
        )
        solution = self._solve_waist(
            target_height, target_orientation, body.last_solution
        )
        previous = np.asarray(
            [self.last_command.get(name, self.joint_state[name]) for name in body.joint_names]
        )
        solution = clamp_step(
            solution,
            previous,
            body.lower,
            body.upper,
            float(self.body_config["max_joint_velocity"])
            / float(self.control["rate_hz"]),
        )
        body.last_solution = solution
        self._set_group(body.joint_indices, solution)
        for name, value in zip(body.joint_names, solution):
            command[name] = float(value)

    def _update_body_target(self) -> None:
        """Update the torso task without solving joints in the VR adapter."""
        body = self.body
        assert body.head_pose is not None
        assert body.reference_head is not None
        assert body.reference_torso is not None
        delta_height = float(
            body.head_pose[0][int(self.body_config["head_height_axis"])]
            - body.reference_head[0][int(self.body_config["head_height_axis"])]
        )
        delta_pitch = float(
            body.head_pose[0][int(self.body_config["head_pitch_position_axis"])]
            - body.reference_head[0][int(self.body_config["head_pitch_position_axis"])]
        )
        lift = float(
            np.clip(
                delta_height
                * float(self.body_config.get("height_direction", 1.0))
                * float(self.body_config["height_scale"]),
                -float(self.body_config["max_height_delta"]),
                float(self.body_config["max_height_delta"]),
            )
        )
        pitch = float(
            np.clip(
                delta_pitch
                * float(self.body_config.get("pitch_direction", 1.0))
                * float(self.body_config["pitch_scale"]),
                -float(self.body_config["max_pitch_delta"]),
                float(self.body_config["max_pitch_delta"]),
            )
        )
        target_position = np.array(body.reference_torso[0], dtype=float, copy=True)
        target_position[2] += lift
        target_orientation = quaternion_multiply(
            quaternion_from_axis_angle(self.body_config["pitch_axis_world"], pitch),
            body.reference_torso[1],
        )
        body.target_base = self._relative_pose(
            self._root_pose(), (target_position, target_orientation)
        )

    def _publish_base_command(self, *, track_head_yaw: bool) -> None:
        left = self.arms["left"]
        body = self.body
        assert left.joy is not None
        axes = left.joy.axes
        x_index = int(self.body_config["stick_x_axis"])
        y_index = int(self.body_config["stick_y_axis"])
        stick_x = axes[x_index] if len(axes) > x_index else 0.0
        stick_y = axes[y_index] if len(axes) > y_index else 0.0
        forward, lateral = joystick_base_velocity(
            stick_x,
            stick_y,
            float(self.body_config["stick_deadzone"]),
            float(self.body_config["max_linear_velocity"]),
            float(self.body_config["max_lateral_velocity"]),
            float(self.body_config.get("stick_forward_direction", 1.0)),
            float(self.body_config.get("stick_lateral_direction", -1.0)),
        )
        yaw = 0.0
        if track_head_yaw:
            assert body.head_pose is not None and body.reference_head is not None
            relative_yaw = self._wrap_angle(
                mapped_relative_yaw(
                    body.head_pose[1],
                    body.reference_head[1],
                    self.control.get(
                        "head_axis_mapping", self.control["axis_mapping"]
                    ),
                )
            )
            yaw_deadzone = float(self.body_config["yaw_deadzone"])
            yaw_error = 0.0 if abs(relative_yaw) <= yaw_deadzone else relative_yaw
            yaw = float(
                np.clip(
                    yaw_error
                    * float(self.body_config.get("yaw_direction", 1.0))
                    * float(self.body_config["yaw_gain"]),
                    -float(self.body_config["max_angular_velocity"]),
                    float(self.body_config["max_angular_velocity"]),
                )
            )
        if self.body_config.get("base_command_mode", "delta") == "delta":
            period = 1.0 / float(self.control["rate_hz"])
            yaw, forward, lateral = yaw * period, forward * period, lateral * period
        self.base_pub.publish(Float64MultiArray(data=[yaw, forward, lateral]))
        self.base_zero_pending = True

    def _stop_base_if_pending(self) -> None:
        if self.base_zero_pending:
            self.base_pub.publish(Float64MultiArray(data=[0.0, 0.0, 0.0]))
            self.base_zero_pending = False

    def _engage_arm(self, arm: ArmRuntime) -> None:
        arm.active = True
        arm.reference_vr = arm.pose
        arm.target_local = self._relative_pose(
            self._link_pose(arm.base_index), self._link_pose(arm.ee_index)
        )
        arm.reference_local_ee = arm.target_local
        arm.last_solution = np.asarray(
            [self.joint_state.get(name, self.initial_joints.get(name, 0.0)) for name in arm.joint_names], dtype=float
        )
        arm.ik_converged = True
        arm.ik_within_tolerance = True
        arm.ik_rejections = 0
        arm.ik_seed = "feedback"
        arm.last_ik_attempt = 0.0
        arm.last_reseed_attempt = 0.0
        arm.recovery_solution = None
        arm.recovery_seed = ""
        arm.reseed_target_position = None
        arm.reseed_target_orientation = None

    def _update_arm(self, arm: ArmRuntime, command: dict[str, float]) -> None:
        assert arm.pose is not None
        assert arm.reference_vr is not None
        assert arm.reference_local_ee is not None
        raw_target_local = relative_target(
            *arm.pose,
            *arm.reference_vr,
            *arm.reference_local_ee,
            self.control["axis_mapping"],
            float(self.control["position_scale"]),
            float(self.control["max_displacement"]),
            bool(self.control["orientation_enabled"]),
        )
        if arm.target_local is None:
            arm.target_local = raw_target_local
        else:
            arm.target_local = stabilize_pose(
                *raw_target_local,
                *arm.target_local,
                float(self.control.get("target_position_deadband", 0.0)),
                float(self.control.get("target_orientation_deadband", 0.0)),
                float(self.control.get("target_filter_alpha", 1.0)),
            )
        arm.target_world = self._world_pose(
            self._link_pose(arm.base_index), arm.target_local
        )
        previous = np.asarray(
            [self.last_command.get(name, self.joint_state[name]) for name in arm.joint_names]
        )
        result = self._solve_arm_ik(arm, *arm.target_world, previous)
        was_converged = arm.ik_converged
        arm.position_error = result.position_error
        arm.orientation_error = result.orientation_error
        arm.ik_converged = result.accepted
        arm.ik_within_tolerance = result.converged
        arm.ik_seed = result.seed_name
        arm.ik_damping = result.damping
        if not result.accepted:
            arm.ik_rejections += 1
            arm.ik_rejection_total += 1
            self._set_group(arm.joint_indices, previous)
            for name, value in zip(arm.joint_names, previous):
                command[name] = float(value)
            now = self._monotonic()
            if now - arm.last_ik_log >= 1.0:
                arm.last_ik_log = now
                self.get_logger().warning(
                    f"{arm.name} IK rejected; holding last valid command "
                    f"(position_error={result.position_error:.4f} m, "
                    f"orientation_error={result.orientation_error:.3f} rad, "
                    f"damping={result.damping:.3f})"
                )
            return

        if not was_converged:
            self.get_logger().info(
                f"{arm.name} IK recovered using {result.seed_name} seed"
            )
        arm.ik_rejections = 0
        # _solve_arm_ik has already evaluated and returned the velocity-limited
        # command.  Clamping it again is harmless but would hide solver mistakes,
        # so publish that exact validated step.
        solution = result.joints
        joint_deadband = float(self.control.get("command_joint_deadband", 0.0))
        solution = np.where(np.abs(solution - previous) <= joint_deadband, previous, solution)
        arm.last_solution = solution
        self._set_group(arm.joint_indices, solution)
        for name, value in zip(arm.joint_names, solution):
            command[name] = float(value)

    def _update_arm_target(self, arm: ArmRuntime) -> None:
        """Convert VR motion into the chest-relative task consumed by sol_q."""
        assert arm.pose is not None
        assert arm.reference_vr is not None
        assert arm.reference_local_ee is not None
        raw_target_local = relative_target(
            *arm.pose,
            *arm.reference_vr,
            *arm.reference_local_ee,
            self.control["axis_mapping"],
            float(self.control["position_scale"]),
            float(self.control["max_displacement"]),
            bool(self.control["orientation_enabled"]),
        )
        if arm.target_local is None:
            arm.target_local = raw_target_local
        else:
            arm.target_local = stabilize_pose(
                *raw_target_local,
                *arm.target_local,
                float(self.control.get("target_position_deadband", 0.0)),
                float(self.control.get("target_orientation_deadband", 0.0)),
                float(self.control.get("target_filter_alpha", 1.0)),
            )

    def _trigger(self, side: str) -> float:
        joy = self.arms[side].joy
        index = int(self.config["grippers"]["trigger_axis"])
        if joy is None or len(joy.axes) <= index:
            return 0.0
        return float(np.clip(joy.axes[index], 0.0, 1.0))

    def _update_grippers(self, command: dict[str, float], now: float) -> None:
        for side in ("right", "left"):
            cfg = self.config["grippers"][side]
            trigger = self._trigger(side) if self._input_fresh(self.arms[side], now) else 0.0
            if self.config["grippers"].get("include_sim_joints", True):
                name = cfg["sim_joint"]
                target = trigger * float(cfg["sim_closed_position"])
                previous = self.last_command.get(name, self.joint_state.get(name, 0.0))
                max_step = float(self.config["grippers"]["max_velocity"]) / float(
                    self.control["rate_hz"]
                )
                command[name] = float(
                    np.clip(target, previous - max_step, previous + max_step)
                )
            finger = JointState()
            finger.header.stamp = self.get_clock().now().to_msg()
            finger.name = list(cfg["finger_joint_names"])
            opened = np.asarray(cfg["finger_open"], dtype=float)
            closed = np.asarray(cfg["finger_closed"], dtype=float)
            finger.position = list(opened + trigger * (closed - opened))
            self.finger_pubs[side].publish(finger)

    def _command_seed(self) -> dict[str, float] | None:
        required = self.arms["right"].joint_names + self.arms["left"].joint_names + self.body.joint_names
        if any(name not in self.joint_state for name in required):
            return None
        return {
            name: self.joint_state.get(
                name, self.last_command.get(name, self.initial_joints.get(name, 0.0))
            )
            for name in self.controlled_names
        }

    def _ensure_generic_targets(self) -> None:
        for arm in self.arms.values():
            if arm.target_local is None:
                arm.target_local = self._relative_pose(
                    self._link_pose(arm.base_index), self._link_pose(arm.ee_index)
                )
        if self.body.target_base is None:
            self.body.target_base = self._relative_pose(
                self._root_pose(), self._link_pose(self.body.torso_index)
            )

    def _set_generic_home_targets(self) -> None:
        """Capture the configured arm home as chest-relative Cartesian tasks."""
        self._release_all(send_base_zero=True)
        self.base_zero_pending = False
        for arm in self.arms.values():
            for name in arm.joint_names:
                bullet.resetJointState(
                    self.robot_id,
                    self.joint_by_name[name],
                    self.initial_joints[name],
                    physicsClientId=self.physics_client,
                )
            arm.target_local = self._relative_pose(
                self._link_pose(arm.base_index), self._link_pose(arm.ee_index)
            )
            arm.reference_local_ee = arm.target_local
        self._sync_model()
        self.body.active = False
        self.body.reference_head = None
        self.body.reference_torso = None
        self.homing = True
        self.homing_start_time = self._monotonic()
        for name in self.controlled_names:
            if name in self.joint_state:
                self.last_command[name] = float(self.joint_state[name])
        self.get_logger().info(
            f"homing triggered: {self.backend} controller homing both arms to initial_joints"
        )

    def _generic_homing_complete(self) -> bool:
        names = self.arms["right"].joint_names + self.arms["left"].joint_names
        cmd_reached = all(
            abs(self.last_command.get(name, 0.0) - self.initial_joints[name]) <= 0.01
            for name in names
        )
        feedback_close = max(
            abs(self.joint_state.get(name, self.initial_joints[name]) - self.initial_joints[name])
            for name in names
        ) <= float(self.control.get("home_tolerance", 0.08))
        elapsed = self._monotonic() - getattr(self, "homing_start_time", 0.0)
        done = cmd_reached and (feedback_close or elapsed >= 4.0)
        if done:
            for arm in self.arms.values():
                for name in arm.joint_names:
                    bullet.resetJointState(
                        self.robot_id,
                        self.joint_by_name[name],
                        self.initial_joints[name],
                        physicsClientId=self.physics_client,
                    )
                arm.target_local = self._relative_pose(
                    self._link_pose(arm.base_index), self._link_pose(arm.ee_index)
                )
                arm.reference_local_ee = arm.target_local
                arm.active = False
        return done

    def _publish_generic_grippers(self, now: float) -> None:
        command: dict[str, float] = {}
        self._update_grippers(command, now)
        self.generic_aux_command.update(command)
        self.last_command.update(command)

    def _generic_control_tick(
        self, now: float, feedback_fresh: bool, command: dict[str, float] | None
    ) -> None:
        if not self.enabled or command is None:
            self.homing = False
            self._release_all(send_base_zero=self.base_zero_pending)
            self.base_zero_pending = False
            self._publish_status(now, feedback_fresh)
            return

        self._ensure_generic_targets()
        if self._home_gesture_triggered(now):
            self._set_generic_home_targets()
        if self.homing:
            self._publish_target_poses()
            if self._generic_homing_complete():
                self.homing = False
                self.get_logger().info(
                    f"{self.backend} controller arm homing complete"
                )
            self._publish_status(now, feedback_fresh)
            return
        if self.home_gesture_latched:
            self._stop_base_if_pending()
            self._publish_status(now, feedback_fresh)
            return

        left_input_fresh = self._input_fresh(self.arms["left"], now)
        right_input_fresh = self._input_fresh(self.arms["right"], now)
        base_requested = left_input_fresh and self._clutch_pressed(
            self.arms["left"]
        )
        body_requested = (
            base_requested
            and bool(self.body_config.get("enabled", True))
            and now - self.body.head_stamp <= float(self.control["pose_timeout"])
        )
        arms_requested = right_input_fresh and self._clutch_pressed(
            self.arms["right"]
        )

        body_command: dict[str, float] = {}
        if body_requested:
            if not self.body.active:
                self._engage_body()
            self._update_body(body_command)
        elif self.body.active:
            self.body.active = False
            self.body.reference_head = None
            self.body.reference_torso = None
            self.get_logger().info("head/left clutch released: waist + yaw hold")
            for name in self.body.joint_names:
                body_command[name] = float(
                    self.last_command.get(
                        name, self.joint_state.get(name, self.initial_joints.get(name, 0.0))
                    )
                )
        else:
            for name in self.body.joint_names:
                body_command[name] = float(
                    self.last_command.get(
                        name, self.joint_state.get(name, self.initial_joints.get(name, 0.0))
                    )
                )
        self.generic_aux_command.update(body_command)
        self.last_command.update(body_command)

        if base_requested:
            self._publish_base_command(track_head_yaw=body_requested)
        else:
            self._stop_base_if_pending()

        arm_was_active = any(arm.active for arm in self.arms.values())
        for side in ("right", "left"):
            arm = self.arms[side]
            pose_fresh = now - arm.pose_stamp <= float(self.control["pose_timeout"])
            if arms_requested and pose_fresh:
                if not arm.active:
                    self._engage_arm(arm)
                self._update_arm_target(arm)
            elif arm.active:
                arm.active = False
                arm.reference_vr = None
                arm.reference_local_ee = None
                arm.target_world = None
            elif not arms_requested:
                if arm.target_local is None:
                    arm.target_local = self._relative_pose(
                        self._link_pose(arm.base_index), self._link_pose(arm.ee_index)
                    )
        if arms_requested and not arm_was_active and any(
            arm.active for arm in self.arms.values()
        ):
            self.get_logger().info(
                f"right clutch engaged: {self.backend} controller owns both arms"
            )
        if not arms_requested and arm_was_active:
            self.get_logger().info(
                f"right clutch released: {self.backend} controller holds last EE targets"
            )
        if arms_requested:
            self._publish_generic_grippers(now)

        # Always publish all three finite tasks. The copied solver otherwise sees
        # an uninitialised target at startup and CasADi can produce NaN.
        self._publish_target_poses()
        self._publish_status(now, feedback_fresh)

    def _control_tick(self) -> None:
        now = self._monotonic()
        feedback_fresh = now - self.joint_state_stamp <= float(
            self.control["joint_state_timeout"]
        )
        command = self._command_seed() if feedback_fresh else None
        if command is not None:
            self._sync_model()
            self._publish_actual_poses()
        if self.external_ik:
            self._generic_control_tick(now, feedback_fresh, command)
            return
        if not self.enabled or command is None:
            self.homing = False
            self._release_all(send_base_zero=self.base_zero_pending)
            self.base_zero_pending = False
            self._publish_status(now, feedback_fresh)
            return
        if self._home_gesture_triggered(now):
            self._start_arm_homing()
        if self.homing:
            self._update_arm_homing(command)
            message = JointState()
            message.header.stamp = self.get_clock().now().to_msg()
            message.name = self.controlled_names
            message.position = [float(command[name]) for name in message.name]
            self.command_pub.publish(message)
            self.last_command.update(command)
            self._publish_status(now, feedback_fresh)
            return
        if self.home_gesture_latched:
            self._publish_status(now, feedback_fresh)
            return
        left_input_fresh = self._input_fresh(self.arms["left"], now)
        right_input_fresh = self._input_fresh(self.arms["right"], now)
        base_requested = left_input_fresh and self._clutch_pressed(
            self.arms["left"]
        )
        body_requested = (
            base_requested
            and bool(self.body_config.get("enabled", True))
            and now - self.body.head_stamp <= float(self.control["pose_timeout"])
        )
        arms_requested = right_input_fresh and self._clutch_pressed(self.arms["right"])
        any_active = False

        if body_requested:
            if not self.body.active:
                self._engage_body()
            self._update_body(command)
            any_active = True
        elif self.body.active:
            self.body.active = False
            self.body.reference_head = None
            self.body.reference_torso = None
            self.get_logger().info("head/left clutch released: waist + yaw hold")

        if base_requested:
            self._publish_base_command(track_head_yaw=body_requested)
        else:
            self._stop_base_if_pending()

        arm_was_active = any(arm.active for arm in self.arms.values())
        for side in ("right", "left"):
            arm = self.arms[side]
            pose_fresh = now - arm.pose_stamp <= float(self.control["pose_timeout"])
            if arms_requested and pose_fresh:
                if not arm.active:
                    self._engage_arm(arm)
                self._update_arm(arm, command)
                any_active = True
            elif arm.active:
                self._release_arm(arm)
        if arms_requested and not arm_was_active and any(arm.active for arm in self.arms.values()):
            self.get_logger().info("right clutch engaged: both arms + grippers")
        if not arms_requested and arm_was_active:
            right_joy = self.arms["right"].joy
            grip_index = int(self.control["clutch_axis"])
            grip_value = (
                right_joy.axes[grip_index]
                if right_joy is not None and len(right_joy.axes) > grip_index
                else float("nan")
            )
            self.get_logger().info(
                "right clutch released: both arms + grippers hold "
                f"(grip={grip_value:.3f}, input_age={now - self.arms['right'].joy_stamp:.3f}s)"
            )
        if arms_requested:
            self._update_grippers(command, now)

        if any_active:
            message = JointState()
            message.header.stamp = self.get_clock().now().to_msg()
            message.name = self.controlled_names
            message.position = [float(command[name]) for name in message.name]
            self.command_pub.publish(message)
            self.last_command.update(command)
            if arms_requested:
                self._publish_target_poses()
        self._publish_status(now, feedback_fresh)

    def _publish_target_poses(self) -> None:
        stamp = self.get_clock().now().to_msg()
        message = PoseArray()
        message.header.stamp = stamp
        base_links = {config["base_link"] for config in self.config["arms"].values()}
        message.header.frame_id = (
            next(iter(base_links))
            if len(base_links) == 1
            else "hc_tj_arm_bases"
        )
        controller_message = PoseArray()
        controller_message.header.stamp = stamp
        controller_message.header.frame_id = (
            "generic_task_bases" if self.external_ik else message.header.frame_id
        )
        for side in ("right", "left"):
            arm = self.arms[side]
            local = arm.target_local or self._relative_pose(
                self._link_pose(arm.base_index), self._link_pose(arm.ee_index)
            )
            pose = Pose()
            pose.position.x, pose.position.y, pose.position.z = local[0]
            (
                pose.orientation.x,
                pose.orientation.y,
                pose.orientation.z,
                pose.orientation.w,
            ) = local[1]
            message.poses.append(pose)
            controller_local = local
            if self.external_ik:
                target_world = self._world_pose(
                    self._link_pose(arm.base_index), local
                )
                controller_local = self._relative_pose(
                    self._link_pose(self.generic_task_base_indices[side]),
                    target_world,
                )
            controller_pose = Pose()
            (
                controller_pose.position.x,
                controller_pose.position.y,
                controller_pose.position.z,
            ) = controller_local[0]
            (
                controller_pose.orientation.x,
                controller_pose.orientation.y,
                controller_pose.orientation.z,
                controller_pose.orientation.w,
            ) = controller_local[1]
            controller_message.poses.append(controller_pose)
        if self.external_ik and bool(self.body_config.get("include_in_controller_tasks", False)):
            torso = self.body.target_base or self._relative_pose(
                self._root_pose(), self._link_pose(self.body.torso_index)
            )
            pose = Pose()
            pose.position.x, pose.position.y, pose.position.z = torso[0]
            (
                pose.orientation.x,
                pose.orientation.y,
                pose.orientation.z,
                pose.orientation.w,
            ) = torso[1]
            controller_message.poses.append(pose)
        self.target_pub.publish(message)
        self.controller_target_pub.publish(controller_message)

    def _publish_actual_poses(self) -> None:
        message = PoseArray()
        message.header.stamp = self.get_clock().now().to_msg()
        base_links = {config["base_link"] for config in self.config["arms"].values()}
        message.header.frame_id = (
            next(iter(base_links)) if len(base_links) == 1 else "hc_tj_arm_bases"
        )
        for side in ("right", "left"):
            arm = self.arms[side]
            local = self._relative_pose(
                self._link_pose(arm.base_index), self._link_pose(arm.ee_index)
            )
            pose = Pose()
            pose.position.x, pose.position.y, pose.position.z = local[0]
            (
                pose.orientation.x,
                pose.orientation.y,
                pose.orientation.z,
                pose.orientation.w,
            ) = local[1]
            message.poses.append(pose)
        self.actual_pub.publish(message)

    def _publish_status(self, now: float, feedback_fresh: bool) -> None:
        if now - self.last_status_publish < 0.2:
            return
        self.last_status_publish = now
        left_clutch = self._input_fresh(self.arms["left"], now) and self._clutch_pressed(
            self.arms["left"]
        )
        right_clutch = self._input_fresh(self.arms["right"], now) and self._clutch_pressed(
            self.arms["right"]
        )
        payload = {
            "enabled": self.enabled,
            "backend": self.backend,
            "generic_controller": {
                "solver_fresh": not self.external_ik
                or now - self.solver_stamp
                <= float(self.control["joint_state_timeout"]),
                "solver_messages": self.solver_message_count,
                "command_fresh": not self.external_ik
                or now - self.generic_command_stamp
                <= float(self.control["joint_state_timeout"]),
                "command_messages": self.generic_command_count,
            },
            "stop_reason": self.stop_reason,
            "feedback_fresh": feedback_fresh,
            "mode": (
                "homing"
                if self.homing
                else "both"
                if left_clutch and right_clutch
                else "base_waist"
                if left_clutch
                else "arms_grippers"
                if right_clutch
                else "hold"
            ),
            "home_gesture_latched": self.home_gesture_latched,
            "left_clutch": left_clutch,
            "right_clutch": right_clutch,
            "body": {
                "active": self.body.active,
                "lift_error": round(self.body.lift_error, 5),
                "pitch_error": round(self.body.pitch_error, 5),
            },
            "arms": {
                side: {
                    "active": arm.active,
                    "pose_fresh": now - arm.pose_stamp
                    <= float(self.control["pose_timeout"]),
                    "position_error": round(arm.position_error, 5),
                    "orientation_error": round(arm.orientation_error, 5),
                    "ik_converged": arm.ik_converged
                    if self.backend == "legacy"
                    else None,
                    "ik_within_tolerance": arm.ik_within_tolerance,
                    "ik_rejections": arm.ik_rejections,
                    "ik_rejection_total": arm.ik_rejection_total,
                    "ik_seed": arm.ik_seed,
                    "ik_damping": round(arm.ik_damping, 5),
                }
                for side, arm in self.arms.items()
            },
        }
        self.status_pub.publish(
            String(data=json.dumps(payload, separators=(",", ":")))
        )

    def destroy_node(self):
        try:
            self.base_pub.publish(Float64MultiArray(data=[0.0, 0.0, 0.0]))
        except Exception:
            pass
        if bullet.isConnected(self.physics_client):
            bullet.disconnect(self.physics_client)
        return super().destroy_node()

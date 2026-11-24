from __future__ import annotations

from contextlib import nullcontext
import json
import math
import os
from pathlib import Path
import threading
import time
from typing import Any

from geometry_msgs.msg import Pose, PoseArray, PoseStamped
import numpy as np
import pybullet as bullet
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
    qos_profile_sensor_data,
)
from sensor_msgs.msg import JointState, Joy
from std_msgs.msg import Bool, Float64MultiArray, Header, String
from std_srvs.srv import Trigger
import yaml

from .arm_teleop_config import (
    LEGACY_IK_DEFAULTS,
    _generic_pipeline_metrics,
    _generic_pipeline_snapshot,
    _measure_clocks,
    _record_elapsed,
    _timing_metric,
    create_arm_runtime,
    create_body_runtime,
    joint_group,
    validate_teleop_config,
)
from .body_base_controller import (
    camera_heading_yaw,
    clear_body_reference,
    compute_hands_reach_assist_velocity,
    compute_head_orientation_velocity,
    compute_user_base_velocity,
    engage_body,
    left_stick_active,
    publish_base_command,
    publish_head_base_pose,
    publish_head_planar_velocity,
    right_stick_yaw_active,
    stop_base_if_pending,
    update_body,
    update_body_target,
    update_neck_joints,
    wrap_angle,
    yaw_from_quaternion,
)
from .collision_guard import (
    apply_collision_latch,
    block_colliding_command,
    clear_collision_block,
    collision_command_allowed,
    collision_for_command,
    initialize_collision_safety,
)
from .head_base_pose import HeadBasePoseFollower, HeadBaseVelocityFollower
from .homing_controller import (
    begin_joint_home_trajectory,
    check_initial_homing,
    home_allowed,
    home_gesture_triggered,
    homing_joint_names,
    motion_home_result,
    motion_home_tick,
    request_motion_home,
    set_generic_home_targets,
    start_homing,
    update_homing,
)
from .pybullet_kinematics import (
    all_dof_positions,
    evaluate_arm_candidate,
    jacobian,
    link_pose,
    relative_pose,
    reset_model_joint_states as _reset_model_joint_states,
    reset_named_joints,
    root_pose,
    set_group,
    solve_arm_ik,
    solve_arm_seed,
    solve_waist,
    world_pose,
)
from .solver_rearm import SolverRearmGate
from .teleop_publishers import (
    arm_base_frame,
    pose_from_tuple,
    publish_actual_poses,
    publish_hand_triggers,
    publish_head_tracking_poses,
    publish_status,
    publish_target_poses,
)
from .teleop_state import ArmRuntime, BodyRuntime, IkCandidate
from .teleop_tick_pipeline import (
    command_seed,
    control_tick,
    control_tick_body,
    engage_arm,
    ensure_generic_targets,
    forward_generic_command,
    generic_command_callback,
    generic_control_tick,
    gripper_position,
    publish_generic_grippers,
    publish_motion_aux,
    trigger_value,
    update_arm,
    update_arm_target,
    update_grippers,
)
from .vr_input import decode_vr_frame


class RobotArmTeleopNode(Node):
    """Profile-driven bimanual teleop node with a two-clutch state machine."""

    def __init__(
        self,
        config_path: str,
        backend: str | None = None,
        *,
        command_node: Node | None = None,
    ):
        super().__init__("hc_robot_vr_teleop")
        self._command_node = command_node
        self._command_state_lock = threading.RLock()
        self._model_lock = threading.RLock()
        self.config_path = Path(config_path).expanduser().resolve()
        with self.config_path.open(encoding="utf-8") as stream:
            self.config = yaml.safe_load(stream)
        # Simulation and external drivers share the same body parameters.
        self.control = self.config.get("control", {})
        self.body_config = self.config.get("body", {})
        self.backend = str(backend or self.control.get("backend", "v23")).lower()
        if self.backend not in {"v23", "generic", "motion", "legacy"}:
            raise ValueError("control.backend must be v23, generic, motion or legacy")
        self.external_ik = self.backend in {"v23", "generic", "motion"}
        self.robot_api = None
        if self.backend == "motion":
            from middleware.core.robot.robot_api import INTERNAL_STATE, RobotApiProfile

            self.robot_api = RobotApiProfile(self.config_path.parent)
            self.control["joint_state_topic"] = INTERNAL_STATE
            self.control["command_topic"] = "/internal/vr/aux_joint_targets"
        if not self.external_ik:
            for key, value in LEGACY_IK_DEFAULTS.items():
                self.control.setdefault(key, value)
        self._validate_config()

        self._init_runtime_state()
        self._init_pybullet_model()
        pose_base_config = self._init_head_base_followers()
        if self.collision_enabled and self.backend != "motion":
            self._initialize_collision_safety()
        self._init_ros_interfaces(pose_base_config)
        self._init_intervention_and_diagnostics()

    def _init_runtime_state(self) -> None:
        self.enabled = bool(self.control.get("enabled_on_start", True))
        if os.environ.get("HC_CONTROL_START_DISABLED") == "1":
            self.enabled = False
            self.control["home_on_start"] = False
        self.stop_reason = ""
        self.control_source = "vr"
        self.source_feedback_pending = False
        self.source_cutoff_ns = 0
        self.joint_state: dict[str, float] = {}
        self.joint_state_stamp = 0.0
        self.joint_state_source_ns = 0
        self.last_command: dict[str, float] = {}
        self.last_status_publish = 0.0
        self.base_zero_pending = False
        self.home_gesture_latched = False
        self.homing = False
        self.homing_velocity: dict[str, float] = {}
        self.homing_last_time = 0.0
        self.homing_target_reached_time: float | None = None
        self.motion_home_request_ns = 0
        self.motion_home_done = False
        self.initial_homing_done = False
        self.solver_stamp = 0.0
        self.solver_message_count = 0
        self.generic_command_stamp = 0.0
        self.generic_command_count = 0
        self.generic_command_pipeline = _generic_pipeline_metrics()
        self.control_tick_metrics = dict(
            ticks=0,
            sampled_ticks=0,
            sample_every_ticks=16,
            stages={
                name: _timing_metric()
                for name in ("sync_model", "actual_head_fk", "remainder", "total")
            },
        )
        self._control_tick_sample = None
        self.generic_aux_command: dict[str, float] = {}
        self.waist_output_enabled = False
        self.waist_clutch_rearm_required = False
        self._prev_left_grip = False
        self._prev_resume_requested = False
        self.solver_rearm = SolverRearmGate()
        self._clutch_solver_held = False
        self._clutch_resume_pending = False
        self.hardware_ready = False
        self.collision_enabled = bool(
            self.control.get("collision_avoidance_enabled", True)
        )
        self.collision_min_distance = float(
            self.control.get("collision_min_distance", 0.02)
        )
        self.collision_home_tolerance = float(
            self.control.get("collision_home_tolerance", 0.008)
        )
        self.collision_latch_until_release = bool(
            self.control.get("collision_latch_until_release", True)
        )
        self.collision_check_intra_arm = bool(
            self.control.get("collision_check_intra_arm", False)
        )
        self.collision_check_inter_arm = bool(
            self.control.get("collision_check_inter_arm", True)
        )
        self.collision_check_arm_body = bool(
            self.control.get("collision_check_arm_body", True)
        )
        self.collision_escape_epsilon = float(
            self.control.get("collision_escape_epsilon", 1.0e-5)
        )
        self.collision_pairs: list[tuple[int, int]] = []
        self.collision_clearances: dict[tuple[int, int], float] = {}
        self.collision_links: set[int] = set()
        self.collision_blocked = False
        self.collision_pair = ""
        self.collision_distance = math.inf
        self.collision_rejections = 0
        self.collision_escape_mode = False
        self.collision_wait_right_release = False
        self.collision_wait_left_release = False
        self.last_collision_log = 0.0
        self.last_collision_reset = 0.0

    def _init_pybullet_model(self) -> None:
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
        self.link_name_by_index: dict[int, str] = {}
        self.parent_by_link: dict[int, int] = {}
        self.dof_by_joint: dict[int, int] = {}
        self.dof_joint_indices: list[int] = []
        dof = 0
        base_info = bullet.getBodyInfo(
            self.robot_id, physicsClientId=self.physics_client
        )
        if base_info:
            base_name = base_info[0].decode()
            self.link_by_name[base_name] = -1
            self.link_name_by_index[-1] = base_name
        for index in range(
            bullet.getNumJoints(self.robot_id, physicsClientId=self.physics_client)
        ):
            info = bullet.getJointInfo(
                self.robot_id, index, physicsClientId=self.physics_client
            )
            self.joint_by_name[info[1].decode()] = index
            self.link_by_name[info[12].decode()] = index
            self.link_name_by_index[index] = info[12].decode()
            self.parent_by_link[index] = int(info[16])
            if info[2] != bullet.JOINT_FIXED:
                self.dof_by_joint[index] = dof
                self.dof_joint_indices.append(index)
                dof += 1
        reset_named_joints(self, self.initial_joints.keys(), self.initial_joints)

        self.arms = {
            side: self._create_arm(side, arm_config)
            for side, arm_config in self.config["arms"].items()
        }
        self.generic_task_base_indices = {
            side: self.link_by_name[arm_config["generic_task_base_link"]]
            for side, arm_config in self.config["arms"].items()
        }
        base_links = {config["base_link"] for config in self.config["arms"].values()}
        self._arm_base_frame_id = (
            next(iter(base_links)) if len(base_links) == 1 else "hc_tj_arm_bases"
        )
        self.body = self._create_body(self.body_config)
        self.neck_enabled = bool(self.body_config.get("neck_tracking_enabled", True))
        raw_neck_names = self.body_config.get("neck_joint_names", ["head_1", "head_2"])
        self.neck_joint_names = [
            str(name) for name in raw_neck_names if str(name) in self.joint_by_name
        ]
        self.neck_indices = [self.joint_by_name[name] for name in self.neck_joint_names]
        for name in self.neck_joint_names:
            self.initial_joints.setdefault(name, 0.0)
        self.neck_reference_head: (
            tuple[tuple[float, float, float], tuple[float, float, float, float]] | None
        ) = None
        self.base_yaw_alignment_offset: float = 0.0
        self.zeros = [0.0] * len(self.dof_joint_indices)
        self.controlled_names = (
            self.arms["right"].joint_names
            + self.arms["left"].joint_names
            + self.body.joint_names
            + self.neck_joint_names
        )
        if self.config["grippers"].get("include_sim_joints", True):
            self.controlled_names += [
                self.config["grippers"]["right"]["sim_joint"],
                self.config["grippers"]["left"]["sim_joint"],
            ]

    def _init_head_base_followers(self) -> dict[str, Any]:
        pose_base_config = self.body_config.get("head_pose_base", {})
        teleop_head_base_mode = pose_base_config.get("teleop_mode", "pose")
        if teleop_head_base_mode not in {"pose", "yaw_velocity", "planar_velocity"}:
            raise ValueError(
                "body.head_pose_base.teleop_mode must be pose, yaw_velocity or planar_velocity"
            )
        self.head_base_mode = (
            teleop_head_base_mode
            if os.environ.get("HC_TELEOP_MODE") == "teleop"
            and pose_base_config.get("enabled", False)
            else "pose"
        )
        self.head_velocity_max_angular = float(
            pose_base_config.get("velocity_max_angular_velocity", 0.2)
        )
        if (
            not math.isfinite(self.head_velocity_max_angular)
            or self.head_velocity_max_angular <= 0
        ):
            raise ValueError(
                "head_pose_base.velocity_max_angular_velocity must be positive and finite"
            )
        self.head_velocity_reference = None
        self.last_head_velocity_time = None
        head_axis_mapping = self.control.get(
            "head_axis_mapping", self.control["axis_mapping"]
        )
        self.head_base_follower = (
            HeadBasePoseFollower(
                pose_base_config,
                head_axis_mapping,
                self.body_config.get("yaw_direction", 1.0),
            )
            if pose_base_config.get("enabled", False) and self.head_base_mode == "pose"
            else None
        )
        self.head_planar_follower = (
            HeadBaseVelocityFollower(
                pose_base_config,
                head_axis_mapping,
                self.body_config.get("yaw_direction", 1.0),
            )
            if self.head_base_mode == "planar_velocity"
            else None
        )
        self.head_velocity_max_linear = float(
            pose_base_config.get("velocity_max_linear_velocity", 0.05)
        )
        self.head_velocity_max_lateral = float(
            pose_base_config.get("velocity_max_lateral_velocity", 0.05)
        )
        for value in (self.head_velocity_max_linear, self.head_velocity_max_lateral):
            if not math.isfinite(value) or value <= 0:
                raise ValueError("head planar velocity limits must be positive and finite")
        self.base_pose_feedback = None
        self.base_pose_stamp = float("-inf")
        self.base_pose_frame = ""
        self.head_base_state = "idle"
        return pose_base_config

    def _init_ros_interfaces(self, pose_base_config: dict[str, Any]) -> None:
        latest_reliable_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        command_pub_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=4,
        )
        solver_sub_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=4,
        )
        cmd_node = self._command_node if self._command_node is not None else self
        self.command_pub = cmd_node.create_publisher(
            JointState, self.control["command_topic"], command_pub_qos
        )
        if self.backend == "motion":
            self.motion_home_pub = self.create_publisher(Header, "/internal/vr/home", 1)
            self.motion_home_cancel_pub = self.create_publisher(
                Bool, "/internal/vr/home_cancel", 1
            )
            self.create_subscription(
                String, "/internal/vr/home_result", self._motion_home_result, 1
            )
            self.create_subscription(
                String, "/status/robot/collision", self._shared_collision_status, 1
            )
        self.solver_reset_pub = self.create_publisher(
            Bool,
            self.control.get("solver_reset_topic", "/control/solver/reset"),
            solver_sub_qos,
        )
        self.create_subscription(
            String,
            self.control.get("control_source_topic", "/control/source"),
            self._control_source_callback,
            QoSProfile(
                depth=1,
                reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.TRANSIENT_LOCAL,
            ),
        )
        self.solver_clutch_pub = self.create_publisher(
            Header, "/control/solver/clutch", solver_sub_qos
        )
        self.solver_reset_ack_sub = None
        self.target_pub = self.create_publisher(
            PoseArray, self.control["target_pose_topic"], command_pub_qos
        )
        self.controller_target_pub = self.create_publisher(
            PoseArray,
            self.control.get(
                "controller_target_pose_topic",
                "/control/solver/ee_targets",
            ),
            command_pub_qos,
        )
        self.actual_pub = None
        self.record_actual_pub = None
        if self.config_path.parent.name == "x1" and self.control.get(
            "legacy_intervention"
        ):
            self.record_actual_pub = self.create_publisher(
                PoseArray, "/internal/recording/observation/ee_poses", command_pub_qos
            )
        if not (
            self.control.get("actual_pose_from_hardware", False)
            and os.environ.get("HC_RUNTIME_MODE") == "teleop"
        ):
            self.actual_pub = self.create_publisher(
                PoseArray,
                self.control.get("actual_pose_topic", "/observation/ee_poses"),
                command_pub_qos,
            )
        self.base_pub = self.create_publisher(
            Float64MultiArray, self.body_config["base_command_topic"], 10
        )
        if self.head_base_follower is not None:
            self.base_pose_sub = self.create_subscription(
                PoseStamped,
                pose_base_config.get("feedback_topic", "/observation/base/pose"),
                self._base_pose_callback,
                qos_profile_sensor_data,
            )
        self.head_visualization_pub = self.create_publisher(
            PoseArray, "/status/head/tracking_poses", command_pub_qos
        )
        self.status_pub = self.create_publisher(
            String, self.control["status_topic"], 10
        )
        self._gripper_open_closed = {
            side: (
                np.asarray(self.config["grippers"][side]["finger_open"], dtype=float),
                np.asarray(self.config["grippers"][side]["finger_closed"], dtype=float),
            )
            for side in ("left", "right")
        }
        self.finger_pubs = {
            side: self.create_publisher(
                JointState,
                (
                    f"/control/vr/{side}_hand/joint_states"
                    if self.backend == "v23"
                    and self.control.get("legacy_intervention", False)
                    else self.config["grippers"][side]["finger_topic"]
                ),
                10,
            )
            for side in ("left", "right")
        }
        self.hand_trigger_pub = self.create_publisher(
            PoseArray, "/observation/hands/triggers", 10
        )
        self.create_subscription(
            JointState,
            self.control["joint_state_topic"],
            self._joint_state_callback,
            qos_profile_sensor_data
            if self.backend == "motion"
            else (
                solver_sub_qos
                if self.backend in {"v23", "generic"}
                else latest_reliable_qos
            ),
        )
        if self.external_ik:
            self.create_subscription(
                JointState,
                self.control.get("solver_topic", "/status/solver/joint_states"),
                self._solver_callback,
                solver_sub_qos,
            )
            self.solver_reset_ack_sub = self.create_subscription(
                Header,
                self.control.get(
                    "solver_reset_ack_topic", "/status/solver/reset_ack"
                ),
                self._solver_reset_ack_callback,
                solver_sub_qos,
            )
            cmd_node.create_subscription(
                JointState,
                self.control.get(
                    "generic_command_topic", "/control/solver/joint_targets"
                ),
                self._generic_command_callback,
                solver_sub_qos,
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
        self.hardware_ready_topic = self.control.get(
            "hardware_ready_topic", "/status/hardware/ready"
        )
        self.create_subscription(
            Bool,
            self.hardware_ready_topic,
            self._hardware_ready_callback,
            latest_reliable_qos,
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
        self.create_subscription(
            Bool, "/teleop/arm/home", self._home_topic_callback, 10
        )
        self.create_service(
            Trigger, "/teleop/arm/reset_reference", self._reset_reference_service
        )
        self.timer = self.create_timer(
            1.0 / float(self.control["rate_hz"]), self._control_tick
        )

    def _init_intervention_and_diagnostics(self) -> None:
        self.intervention = None
        if self.backend == "motion" or (
            self.backend == "v23" and self.control.get("legacy_intervention", False)
        ):
            from .intervention import InterventionTeleop

            self.intervention = InterventionTeleop(self)
        self.get_logger().info(
            "HC-TJ teleop ready: left Grip=base+waist, right Grip=both arms+grippers, "
            f"backend={self.backend}, command={self.control['command_topic']}, "
            f"enabled={self.enabled}"
        )
        from middleware.core.control.qos_event_observer import QosEventObserver

        observed_nodes = (
            [self, self._command_node] if self._command_node is not None else [self]
        )
        self._qos_event_observer = QosEventObserver(
            observed_nodes,
            enabled=(
                None
                if self.backend == "v23" and self.config_path.parent.name == "x1"
                else False
            ),
        )
        self._qos_event_observer.start()

    # ── Config & runtime builders ────────────────────────────────────────────

    def _validate_config(self) -> None:
        validate_teleop_config(
            self.config, self.config_path, external_ik=self.external_ik
        )

    def _joint_group(
        self, names: list[str]
    ) -> tuple[list[int], list[int], np.ndarray, np.ndarray]:
        return joint_group(self, names)

    def _create_arm(self, side: str, config: dict[str, Any]) -> ArmRuntime:
        return create_arm_runtime(self, side, config)

    def _create_body(self, config: dict[str, Any]) -> BodyRuntime:
        return create_body_runtime(self, config)

    # ── Input parsing & ROS callbacks ────────────────────────────────────────

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

    def _input_source_stamp(self, message, timeout: float) -> float | None:
        now = self._monotonic()
        stamp = message.header.stamp
        source_ns = int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)
        if not source_ns:  # Typed legacy ROS inputs without a source timestamp.
            return now
        age = (self.get_clock().now().nanoseconds - source_ns) / 1e9
        if not -0.05 <= age <= timeout:
            return None
        return now - max(0.0, age)

    def _arm_pose_callback(
        self, arm: ArmRuntime, message: PoseStamped, *, sample_time=None
    ) -> None:
        pose = self._read_pose(message)
        stamp = self._input_source_stamp(message, float(self.control["pose_timeout"]))
        source_ns = (
            message.header.stamp.sec * 1_000_000_000 + message.header.stamp.nanosec
        )
        if pose is None:
            arm.pose = None
            arm.pose_stamp = float("-inf")
            arm.predictor.reset()
            return
        if stamp is None:
            return
        if source_ns and source_ns <= arm.pose_source_ns:
            return
        arm.pose_source_ns = source_ns
        arm.pose = pose
        arm.pose_stamp = stamp
        arm.predictor.observe(pose, stamp, sample_time)

    def _base_pose_callback(self, message: PoseStamped) -> None:
        pose = self._read_pose(message)
        if pose is None or not message.header.frame_id:
            self.base_pose_stamp = float("-inf")
            return
        if self.base_pose_frame and self.base_pose_frame != message.header.frame_id:
            self.head_base_follower.reset()
        self.base_pose_frame = message.header.frame_id
        position, q = pose
        yaw = yaw_from_quaternion(q)
        self.base_pose_feedback = (float(position[0]), float(position[1]), yaw)
        self.base_pose_stamp = self._monotonic()

    def _head_pose_callback(self, message: PoseStamped) -> None:
        pose = self._read_pose(message)
        if pose is not None:
            self.body.head_pose = pose
            self.body.head_stamp = self._monotonic()
            if self.neck_reference_head is None:
                self.neck_reference_head = pose
            if self.body.reference_head is None and not self.body.active:
                self.body.reference_head = pose

    def _joy_callback(self, arm: ArmRuntime, message: Joy) -> None:
        if all(math.isfinite(float(value)) for value in message.axes):
            stamp = self._input_source_stamp(
                message, float(self.control["input_timeout"])
            )
            if stamp is None:
                return
            old_ns = (
                arm.joy.header.stamp.sec * 1_000_000_000 + arm.joy.header.stamp.nanosec
                if arm.joy
                else 0
            )
            new_ns = (
                message.header.stamp.sec * 1_000_000_000 + message.header.stamp.nanosec
            )
            if new_ns and new_ns <= old_ns:
                return
            arm.joy = message
            arm.joy_stamp = stamp

    def _vr_data_callback(self, message: String) -> None:
        if getattr(self, "intervention", None):
            self.intervention.observe(message.data)
        frame = decode_vr_frame(message.data)
        if frame is None:
            return
        head = frame.poses.get("head")
        if getattr(self, "head_planar_follower", None) is not None and (
            head is None or self._read_pose(head) is None
        ):
            # Unified frames explicitly report tracking loss: do not retain a
            # previous moving target until the timestamp timeout expires.
            self.body.head_pose = None
            self.body.head_stamp = float("-inf")
            self.head_planar_follower.reset()
            self.head_base_state = "waiting_head_pose"
            self._stop_base_if_pending()
        if head is not None:
            self._head_pose_callback(head)
        for side in ("left", "right"):
            pose = frame.poses.get(side)
            if pose is not None:
                self._arm_pose_callback(
                    self.arms[side], pose, sample_time=frame.sample_time
                )
            else:
                self.arms[side].pose = None
                self.arms[side].pose_stamp = float("-inf")
                self.arms[side].predictor.reset()
            joy = frame.inputs.get(side)
            if joy is not None:
                self._joy_callback(self.arms[side], joy)
        self._publish_hand_triggers(frame.inputs)
        resume_rising = frame.resume_requested and not self._prev_resume_requested
        self._prev_resume_requested = frame.resume_requested
        if (
            frame.resume_requested
            and not (getattr(self, "intervention", None) and self.intervention.active)
            and (not self.enabled or self.stop_reason or resume_rising)
        ):
            self._resume_teleop("VR controller A button pressed")

    def _resume_teleop(self, reason: str = "VR controller A button pressed") -> None:
        if self.control_source == "vr" and (not self.enabled or not self.homing):
            # Replay can move the robot without changing the selected VR source.
            # Resume at the next measured pose, even if both A and Grip are down.
            self._reset_control_references()
            if not self.enabled:
                self.initial_homing_done = True
            self.home_gesture_latched = True
        self.enabled = True
        self.stop_reason = ""
        self.waist_output_enabled = False
        self.waist_clutch_rearm_required = True
        if self.body.active:
            clear_body_reference(self, clear_smoothed=True)
        with (getattr(self, "_command_state_lock", None) or nullcontext()):
            for name in self.body.joint_names:
                if name in self.joint_state:
                    self.last_command[name] = float(self.joint_state[name])
                self.generic_aux_command.pop(name, None)
        now = self._monotonic()
        left_input_fresh = self._input_fresh(self.arms["left"], now)
        self._prev_left_grip = bool(
            left_input_fresh and self._clutch_pressed(self.arms["left"])
        )
        self.get_logger().info(
            f"{reason}: teleop re-enabled, waist output disabled until new left Grip edge"
        )

    def _control_source_callback(self, message: String) -> None:
        source = message.data.strip().lower()
        if source not in {"vr", "motion", "exoskeleton"} or source == self.control_source:
            return
        self.control_source = source
        if getattr(self, "intervention", None) and self.intervention.active:
            return  # The ownership permit handles DAgger resets and references.
        self._reset_control_references()
        self.get_logger().info(
            f"control source changed to {source}; old VR state discarded"
        )

    def _reset_control_references(self) -> None:
        self._cancel_solver_clutch()
        with (getattr(self, "_command_state_lock", None) or nullcontext()):
            self._release_all(send_base_zero=True)
            self.arms["right"].joy = None  # A cached release cannot rearm the solver.
            self.body.smoothed_head_pose = None
            self.homing = False
            self.generic_aux_command.clear()
            self.body.target_base = None
            self.waist_output_enabled = False
            self.waist_clutch_rearm_required = True
            self.source_feedback_pending = True
            self.source_cutoff_ns = int(self.get_clock().now().nanoseconds)
            if self.external_ik:
                self.solver_rearm.start_homing()
                if self.control_source == "vr":
                    self.solver_rearm.homing_complete()

    def _joint_state_callback(self, message: JointState) -> None:
        if len(message.name) != len(message.position) or len(set(message.name)) != len(
            message.name
        ):
            return
        stamp_ns = (
            message.header.stamp.sec * 1_000_000_000 + message.header.stamp.nanosec
        )
        if self.backend in {"v23", "generic"}:
            now_ns = self.get_clock().now().nanoseconds
            if (
                not -50_000_000 <= now_ns - stamp_ns <= 250_000_000
                or stamp_ns <= self.joint_state_source_ns <= now_ns + 50_000_000
            ):
                return
        values = dict(zip(message.name, message.position))
        if not all(math.isfinite(float(value)) for value in values.values()):
            return
        state_lock = getattr(self, "_command_state_lock", None) or nullcontext()
        joint_state_next = {name: float(value) for name, value in values.items()}
        with state_lock:
            self.joint_state = joint_state_next
            self.joint_state_stamp = self._monotonic()
            self.joint_state_source_ns = stamp_ns
            pending_check = self.source_feedback_pending
            if pending_check:
                if (
                    not self.hardware_ready
                    or self.joint_state_source_ns <= self.source_cutoff_ns
                    or not self._complete_feedback_fresh()
                ):
                    return
                self.last_command = dict(self.joint_state)
                self.generic_aux_command.clear()
            if not self.last_command:
                self.last_command = dict(self.joint_state)
        if pending_check:
            for arm in self.arms.values():
                arm.target_local = None
            self.body.target_base = None
            self._sync_model()
            self._ensure_generic_targets()
            with state_lock:
                self.source_feedback_pending = False
            if (
                self.enabled
                and self.control_source == "vr"
                and self.backend in {"v23", "generic"}
                and not (self.intervention and self.intervention.active)
            ):
                # Hold the arms at measured positions while waiting for the IK reset/Grip.
                names = self.arms["right"].joint_names + self.arms["left"].joint_names
                hold = JointState(
                    name=list(names),
                    position=[joint_state_next[name] for name in names],
                )
                hold.header.stamp = self.get_clock().now().to_msg()
                self.command_pub.publish(hold)
        if (
            not self.initial_homing_done
            and self.hardware_ready
            and self.control_source == "vr"
        ):
            self._check_initial_homing()
        if self.external_ik and self.solver_rearm.observe_feedback():
            cutoff_ns = int(self.get_clock().now().nanoseconds)
            self.solver_reset_pub.publish(Bool(data=True))
            self.solver_rearm.reset_published(cutoff_ns)
            self._last_solver_reset_retry_ns = cutoff_ns
            self.get_logger().info(
                "post-home joint feedback received; solver reset requested"
            )
        elif (
            self.external_ik
            and self.solver_rearm.state == self.solver_rearm.WAIT_ACK
            and self._complete_feedback_fresh()
        ):
            now_ns = int(self.get_clock().now().nanoseconds)
            last_retry_ns = getattr(
                self, "_last_solver_reset_retry_ns", self.solver_rearm.reset_cutoff_ns
            )
            if now_ns - last_retry_ns >= 500_000_000:
                self._last_solver_reset_retry_ns = now_ns
                if self._clutch_resume_pending:
                    self.solver_clutch_pub.publish(
                        Header(stamp=self.get_clock().now().to_msg(), frame_id="resume")
                    )
                else:
                    self.solver_reset_pub.publish(Bool(data=True))

    def _complete_feedback_fresh(self) -> bool:
        age_ns = self.get_clock().now().nanoseconds - self.joint_state_source_ns
        return (
            -50_000_000 <= age_ns <= 250_000_000
            and self._monotonic() - self.joint_state_stamp
            <= float(self.control["joint_state_timeout"])
            and all(name in self.joint_state for name in self._homing_joint_names())
        )

    def _ensure_vr_feedback(self) -> bool:
        if self._complete_feedback_fresh():
            return True
        self._reset_control_references()
        return False

    def _solver_callback(self, message: JointState) -> None:
        if len(message.name) != len(message.position):
            return
        if not all(math.isfinite(float(value)) for value in message.position):
            return
        self.solver_stamp = self._monotonic()
        self.solver_message_count += 1
        if self.backend == "motion" and self.hardware_ready:
            stamp_ns = (
                message.header.stamp.sec * 1_000_000_000 + message.header.stamp.nanosec
            )
            self.solver_rearm.observe_solver_output(stamp_ns)

    def _solver_reset_ack_callback(self, message: Header) -> None:
        if (message.frame_id == "clutch") != self._clutch_resume_pending:
            return
        stamp_ns = int(message.stamp.sec) * 1_000_000_000 + int(message.stamp.nanosec)
        if self.solver_rearm.observe_reset_ack(stamp_ns):
            self.get_logger().info(
                "solver reset acknowledged; waiting for fresh IK output"
            )

    def _hardware_ready_callback(self, message: Bool) -> None:
        ready = bool(message.data)
        was_ready = self.hardware_ready
        self.hardware_ready = ready

        if not ready:
            self._cancel_solver_clutch()
            if any(arm.active for arm in self.arms.values()) or self.body.active:
                self._release_all(send_base_zero=True)
            if self.external_ik:
                self.solver_rearm.start_homing()
            return

        if ready and not was_ready:
            if self.source_feedback_pending:
                if self.external_ik and self.control_source == "vr":
                    self.solver_rearm.homing_complete()
                return
            self.get_logger().info(
                "hardware ready confirmed: resetting IK, recapturing pose, and enabling joint output"
            )
            self._release_all(send_base_zero=True)
            if self.joint_state:
                for name in self.controlled_names:
                    if name in self.joint_state:
                        self.last_command[name] = float(self.joint_state[name])
            self._sync_model()
            self._ensure_generic_targets()
            for arm in self.arms.values():
                arm.target_local = self._relative_pose(
                    self._link_pose(arm.base_index), self._link_pose(arm.ee_index)
                )
                arm.reference_local_ee = arm.target_local
                arm.reference_vr = None

            if self.external_ik:
                self.solver_rearm.homing_complete()
                if self.joint_state:
                    cutoff_ns = int(self.get_clock().now().nanoseconds)
                    if self.solver_rearm.observe_feedback():
                        self.solver_reset_pub.publish(Bool(data=True))
                        self.solver_rearm.reset_published(cutoff_ns)
                        self.get_logger().info(
                            "hardware ready: solver reset published to feedback"
                        )
            if not self.initial_homing_done:
                self._check_initial_homing()

    def _shared_collision_status(self, message: String) -> None:
        try:
            status = json.loads(message.data)
            if status.get("blocked") and self.enabled:
                self._block_colliding_command(
                    tuple(status["collision"]),
                    self._monotonic(),
                    escape_mode=bool(status.get("escape_mode")),
                )
        except (ValueError, KeyError, TypeError):
            return

    def _disable_teleop(self, reason: str) -> None:
        with (getattr(self, "_command_state_lock", None) or nullcontext()):
            self.enabled = False
            self.stop_reason = reason
            self.waist_output_enabled = False
            self.waist_clutch_rearm_required = True
            for name in self.body.joint_names:
                self.generic_aux_command.pop(name, None)
            self._release_all(send_base_zero=True)

    def _stop_callback(self, message: Bool) -> None:
        if message.data:
            self._disable_teleop("emergency stop topic")
            self.get_logger().error("Teleop disabled by emergency stop")

    def _enabled_callback(self, message: Bool) -> None:
        if getattr(self, "intervention", None) and self.intervention.active:
            return
        if message.data and not self.enabled:
            self._resume_teleop("teleoperation enabled by command topic")
        elif not message.data:
            self._disable_teleop("disabled by command topic")
            self.get_logger().info("Teleop disabled by command topic")

    def _enable_service(self, _request: Trigger.Request, response: Trigger.Response):
        if getattr(self, "intervention", None) and self.intervention.active:
            response.success, response.message = (
                False,
                "intervention session owns enable and reference changes",
            )
            return response
        self._resume_teleop("teleoperation enabled by service")
        response.success = True
        response.message = "teleop enabled; use left/right Grip clutches"
        return response

    def _disable_service(self, _request: Trigger.Request, response: Trigger.Response):
        if getattr(self, "intervention", None) and self.intervention.active:
            response.success, response.message = (
                False,
                "use intervention pause or emergency stop",
            )
            return response
        self.enabled = False
        self.stop_reason = "disabled by service"
        self._release_all(send_base_zero=True)
        response.success = True
        response.message = "teleop disabled"
        return response

    def _home_service(self, _request: Trigger.Request, response: Trigger.Response):
        if getattr(self, "intervention", None) and self.intervention.active:
            response.success, response.message = (
                False,
                "use the DAgger home button during takeover",
            )
            return response
        if not self._home_allowed():
            response.success = False
            response.message = (
                "home requires ordinary VR mode and fresh post-switch feedback"
            )
            return response
        if self.backend == "motion":
            response.success = self._request_motion_home()
            response.message = (
                "MoveJ home requested"
                if response.success
                else "enable VR and hardware before home"
            )
            return response
        self.enabled = True
        self.stop_reason = ""
        if self.backend in {"generic", "v23"}:
            self._set_generic_home_targets()
        else:
            self._start_homing()
        response.success = True
        response.message = "arms and waist homing sequence started"
        return response

    def _home_topic_callback(self, message: Bool):
        if getattr(self, "intervention", None) and self.intervention.active:
            return  # Session home arrives with an immutable ownership token.
        if message.data and not self._home_allowed():
            self.get_logger().warning(
                "home ignored: ordinary VR mode and fresh post-switch feedback required"
            )
            return
        if message.data:
            if self.backend == "motion":
                self._request_motion_home()
                return
            self.enabled = True
            self.stop_reason = ""
            if self.backend in {"generic", "v23"}:
                self._set_generic_home_targets()
            else:
                self._start_homing()

    def _reset_reference_service(
        self, _request: Trigger.Request, response: Trigger.Response
    ):
        if getattr(self, "intervention", None) and self.intervention.active:
            response.success, response.message = (
                False,
                "references are anchored by the next Grip transaction",
            )
            return response
        self._release_all(send_base_zero=True)
        response.success = True
        response.message = "references cleared; release both Grip controls"
        return response

    # ── Clutch & release primitives ──────────────────────────────────────────

    def _release_arm(self, arm: ArmRuntime) -> None:
        arm.active = False
        arm.clutch_latched = False
        arm.predictor.reset()
        arm.reference_vr = None
        arm.reference_local_ee = None
        arm.target_local = None
        arm.target_world = None
        arm.recovery_solution = None
        arm.recovery_seed = ""
        arm.reseed_target_position = None
        arm.reseed_target_orientation = None

    def _release_all(self, *, send_base_zero: bool) -> None:
        self.head_velocity_reference = None
        self.last_head_velocity_time = None
        self.head_base_state = "idle"
        if getattr(self, "head_base_follower", None) is not None:
            self.head_base_follower.reset()
            self.head_base_state = "idle"
        if getattr(self, "head_planar_follower", None) is not None:
            self.head_planar_follower.reset()
        for arm in self.arms.values():
            self._release_arm(arm)
        clear_body_reference(self)
        self.neck_reference_head = None
        self.base_yaw_alignment_offset = 0.0
        self.waist_output_enabled = False
        self.waist_clutch_rearm_required = True
        with (getattr(self, "_command_state_lock", None) or nullcontext()):
            for name in self.body.joint_names:
                self.generic_aux_command.pop(name, None)
        if send_base_zero:
            self.base_pub.publish(Float64MultiArray(data=[0.0, 0.0, 0.0]))

    def _axis(self, arm: ArmRuntime, index: int) -> float:
        if arm.joy is None or len(arm.joy.axes) <= index:
            return 0.0
        return float(arm.joy.axes[index])

    def _input_fresh(self, arm: ArmRuntime, now: float) -> bool:
        fresh = (
            arm.joy is not None
            and now - arm.joy_stamp <= float(self.control["input_timeout"])
        )
        if not fresh:
            arm.clutch_latched = False
            arm.predictor.reset()
        return fresh

    def _clutch_pressed(self, arm: ArmRuntime) -> bool:
        index = int(self.control["clutch_axis"])
        if arm.joy is None or len(arm.joy.axes) <= index:
            arm.clutch_latched = False
            return False
        press = float(self.control["clutch_threshold"])
        release = float(self.control.get("clutch_release_threshold", press))
        value = float(arm.joy.axes[index])
        if not math.isfinite(value):
            arm.clutch_latched = False
        elif arm.clutch_latched and release < press:
            arm.clutch_latched = value > float(np.float32(release))
        else:
            arm.clutch_latched = value >= press
        return arm.clutch_latched

    def _cancel_solver_clutch(self) -> None:
        if getattr(self, "_clutch_solver_held", False) or getattr(
            self, "_clutch_resume_pending", False
        ):
            self.solver_clutch_pub.publish(
                Header(stamp=self.get_clock().now().to_msg(), frame_id="clear")
            )
        self._clutch_solver_held = False
        self._clutch_resume_pending = False

    def _update_solver_clutch(self, requested: bool, *, input_fresh: bool) -> None:
        if self.backend != "v23" or not self.control.get(
            "clutch_reset_on_release", False
        ):
            return
        if not requested and (
            any(a.active for a in self.arms.values()) or self._clutch_resume_pending
        ):
            with self._command_state_lock:
                for arm in self.arms.values():
                    self._release_arm(arm)
                self.solver_rearm.start_homing()
                if input_fresh:
                    self.solver_rearm.observe_clutch(False)
                self._clutch_solver_held = True
                self._clutch_resume_pending = False
            self.solver_clutch_pub.publish(
                Header(stamp=self.get_clock().now().to_msg(), frame_id="hold")
            )
            self.get_logger().info(
                "right Grip released: arm integrators held; awaiting new press and feedback reset"
            )
        elif (
            requested
            and self._clutch_solver_held
            and self.solver_rearm.release_observed
            and self._complete_feedback_fresh()
        ):
            for arm in self.arms.values():
                arm.target_local = self._relative_pose(
                    self._link_pose(arm.base_index), self._link_pose(arm.ee_index)
                )
                arm.predictor.reset()
            with self._command_state_lock:
                self.solver_rearm.homing_complete()
                self.solver_rearm.observe_feedback()
                cutoff = int(self.get_clock().now().nanoseconds)
                self.solver_rearm.reset_published(cutoff)
                self._last_solver_reset_retry_ns = cutoff
                self._clutch_solver_held = False
                self._clutch_resume_pending = True
            self.solver_clutch_pub.publish(
                Header(stamp=self.get_clock().now().to_msg(), frame_id="resume")
            )
            self.get_logger().info(
                "right Grip re-pressed: awaiting fresh arm feedback, reset ACK and new IK output"
            )
        if self._clutch_resume_pending and not self.solver_rearm.blocked:
            self._clutch_resume_pending = False

    def _update_waist_clutch(self, now: float) -> bool:
        left_input_fresh = self._input_fresh(self.arms["left"], now)
        left_grip = left_input_fresh and self._clutch_pressed(self.arms["left"])
        rising_edge = left_grip and not self._prev_left_grip
        self._prev_left_grip = left_grip

        if self.waist_clutch_rearm_required:
            if rising_edge:
                self.waist_clutch_rearm_required = False
                self.waist_output_enabled = True
                self.get_logger().info(
                    "new left Grip rising edge detected: waist output rearmed and enabled"
                )
            else:
                self.waist_output_enabled = False
        else:
            self.waist_output_enabled = left_grip

        return self.waist_output_enabled

    # ── Delegating facade methods (preserving SimpleNamespace/test contracts) ─

    def _sync_model(self) -> None:
        with (getattr(self, "_model_lock", None) or nullcontext()):
            _reset_model_joint_states(self)

    def _initialize_collision_safety(self) -> None:
        initialize_collision_safety(self)

    def _collision_for_command(
        self, command: dict[str, float]
    ) -> tuple[str, str, float, float] | None:
        return collision_for_command(self, command)

    def _block_colliding_command(
        self,
        collision: tuple[str, str, float, float],
        now: float,
        *,
        escape_mode: bool = False,
    ) -> None:
        block_colliding_command(self, collision, now, escape_mode=escape_mode)

    def _collision_command_allowed(self, command: dict[str, float], now: float) -> bool:
        return collision_command_allowed(self, command, now)

    def _clear_collision_block(self) -> None:
        clear_collision_block(self)

    def _apply_collision_latch(
        self, base_requested: bool, arms_requested: bool
    ) -> tuple[bool, bool]:
        return apply_collision_latch(self, base_requested, arms_requested)

    def _set_group(self, indices: list[int], values: np.ndarray) -> None:
        set_group(self, indices, values)

    def _link_pose(self, index: int) -> tuple[np.ndarray, np.ndarray]:
        return link_pose(self, index)

    def _root_pose(self) -> tuple[np.ndarray, np.ndarray]:
        return root_pose(self)

    def _relative_pose(
        self, parent: tuple[np.ndarray, np.ndarray], child: tuple[np.ndarray, np.ndarray]
    ) -> tuple[np.ndarray, np.ndarray]:
        return relative_pose(parent, child)

    def _world_pose(
        self, parent: tuple[np.ndarray, np.ndarray], local: tuple[np.ndarray, np.ndarray]
    ) -> tuple[np.ndarray, np.ndarray]:
        return world_pose(parent, local)

    def _all_dof_positions(self) -> list[float]:
        return all_dof_positions(self)

    def _jacobian(self, link_index: int) -> tuple[np.ndarray, np.ndarray]:
        return jacobian(self, link_index)

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
        return evaluate_arm_candidate(
            self,
            arm,
            target_position,
            target_orientation,
            joints,
            reference,
            seed_name,
            damping,
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
        return solve_arm_seed(
            self, arm, target_position, target_orientation, seed, reference, seed_name
        )

    def _solve_arm_ik(
        self,
        arm: ArmRuntime,
        target_position: np.ndarray,
        target_orientation: np.ndarray,
        previous: np.ndarray,
    ) -> IkCandidate:
        return solve_arm_ik(self, arm, target_position, target_orientation, previous)

    def _solve_waist(
        self,
        target_position: np.ndarray,
        target_orientation: np.ndarray,
        seed: np.ndarray,
    ) -> np.ndarray:
        return solve_waist(self, target_position, target_orientation, seed)

    @staticmethod
    def _yaw(quaternion: np.ndarray) -> float:
        return yaw_from_quaternion(quaternion)

    @staticmethod
    def _wrap_angle(value: float) -> float:
        return wrap_angle(value)

    def _engage_body(self) -> None:
        engage_body(self)

    def _update_body(self, command: dict[str, float]) -> None:
        update_body(self, command)

    def _update_body_target(self) -> None:
        update_body_target(self)

    def _camera_heading_yaw(self) -> float:
        return camera_heading_yaw(self)

    def _update_neck_joints(self, command: dict[str, float], now: float) -> None:
        update_neck_joints(self, command, now)

    def _left_stick_active(self) -> bool:
        return left_stick_active(self)

    def _right_stick_yaw_active(self) -> bool:
        return right_stick_yaw_active(self)

    def _compute_user_base_velocity(
        self, now: float, left_clutch_active: bool = False
    ) -> tuple[float, float, float]:
        return compute_user_base_velocity(
            self, now, left_clutch_active=left_clutch_active
        )

    def _compute_head_orientation_velocity(
        self,
        now: float,
        *,
        forward_velocity: float = 0.0,
        lateral_velocity: float = 0.0,
        track_head_yaw: bool = True,
    ) -> float:
        return compute_head_orientation_velocity(
            self,
            now,
            forward_velocity=forward_velocity,
            lateral_velocity=lateral_velocity,
            track_head_yaw=track_head_yaw,
        )

    def _compute_hands_reach_assist_velocity(self, now: float) -> tuple[float, float]:
        return compute_hands_reach_assist_velocity(self, now)

    def _publish_head_base_pose(self, now: float, engaged: bool) -> None:
        publish_head_base_pose(self, now, engaged)

    def _publish_head_planar_velocity(self, now: float, engaged: bool) -> None:
        publish_head_planar_velocity(self, now, engaged)

    def _publish_base_command(
        self,
        now: float,
        *,
        track_head_yaw: bool = True,
        left_clutch_active: bool = False,
    ) -> None:
        publish_base_command(
            self,
            now,
            track_head_yaw=track_head_yaw,
            left_clutch_active=left_clutch_active,
        )

    def _stop_base_if_pending(self) -> None:
        stop_base_if_pending(self)

    def _home_allowed(self) -> bool:
        return home_allowed(self)

    def _home_gesture_triggered(self, now: float) -> bool:
        return home_gesture_triggered(self, now)

    def _homing_joint_names(self) -> list[str]:
        return homing_joint_names(self)

    def _begin_joint_home_trajectory(self) -> None:
        begin_joint_home_trajectory(self)

    def _check_initial_homing(self) -> None:
        check_initial_homing(self)

    def _start_homing(self) -> None:
        start_homing(self)

    def _update_homing(self, command: dict[str, float]) -> None:
        update_homing(self, command)

    def _set_generic_home_targets(self) -> None:
        set_generic_home_targets(self)

    def _request_motion_home(self) -> bool:
        return request_motion_home(self)

    def _motion_home_result(self, message: String) -> None:
        motion_home_result(self, message)

    def _motion_home_tick(self, now: float, feedback_fresh: bool) -> None:
        motion_home_tick(self, now, feedback_fresh)

    def _engage_arm(self, arm: ArmRuntime) -> None:
        engage_arm(self, arm)

    def _update_arm(self, arm: ArmRuntime, command: dict[str, float]) -> None:
        update_arm(self, arm, command)

    def _update_arm_target(self, arm: ArmRuntime) -> None:
        update_arm_target(self, arm)

    def _trigger(self, side: str) -> float:
        return trigger_value(self, side)

    def _gripper_position(self, side: str, trigger: float) -> float:
        return gripper_position(self, side, trigger)

    def _update_grippers(self, command: dict[str, float], now: float) -> None:
        update_grippers(self, command, now)

    def _command_seed(self) -> dict[str, float] | None:
        return command_seed(self)

    def _ensure_generic_targets(self) -> None:
        ensure_generic_targets(self)

    def _publish_motion_aux(self) -> None:
        publish_motion_aux(self)

    def _publish_generic_grippers(self, now: float) -> None:
        publish_generic_grippers(self, now)

    def _generic_command_callback(self, message: JointState) -> None:
        generic_command_callback(self, message, measure_clocks=_measure_clocks)

    def _forward_generic_command(self, message: JointState) -> str | None:
        return forward_generic_command(self, message, measure_clocks=_measure_clocks)

    def _generic_control_tick(
        self, now: float, feedback_fresh: bool, command: dict[str, float] | None
    ) -> None:
        generic_control_tick(self, now, feedback_fresh, command)

    def _control_tick(self) -> None:
        control_tick(self, measure_clocks=_measure_clocks)

    def _control_tick_body(self) -> None:
        control_tick_body(self, measure_clocks=_measure_clocks)

    def _arm_base_frame(self) -> str:
        return arm_base_frame(self)

    @staticmethod
    def _pose_from_tuple(position: Any, orientation: Any) -> Pose:
        return pose_from_tuple(position, orientation)

    def _publish_hand_triggers(self, inputs: dict[str, Joy]) -> None:
        publish_hand_triggers(self, inputs)

    def _publish_target_poses(self) -> None:
        publish_target_poses(self)

    def _publish_actual_poses(self) -> None:
        publish_actual_poses(self)

    def _publish_head_tracking_poses(self, stamp: Any = None) -> None:
        publish_head_tracking_poses(self, stamp=stamp)

    def _publish_status(self, now: float, feedback_fresh: bool) -> None:
        publish_status(self, now, feedback_fresh)

    def destroy_node(self):
        try:
            self.base_pub.publish(Float64MultiArray(data=[0.0, 0.0, 0.0]))
        except Exception:
            pass
        if getattr(self, "_qos_event_observer", None) is not None:
            self._qos_event_observer.close()
        if bullet.isConnected(self.physics_client):
            bullet.disconnect(self.physics_client)
        return super().destroy_node()


# Compatibility alias for downstream imports made before the generic rename.
HcTjArmTeleopNode = RobotArmTeleopNode

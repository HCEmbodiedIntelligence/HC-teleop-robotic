"""Config-driven PyBullet robot model used by TeleXperience simulations.

This is a ROS-independent adaptation of the utility module in
``ioai-tech/TeleXperience_robot_ros1_ws``.  Keeping ROS out of this module makes
the robot description and its home pose testable with PyBullet in DIRECT mode.
"""

from __future__ import annotations

import math
import os
import threading
from dataclasses import dataclass
from typing import Iterable, Optional, Sequence
from xml.etree import ElementTree

import numpy as np
import pybullet as p

from .utils import invert_transform, multiply_transforms


def _as_index_list(value) -> list[int]:
    if isinstance(value, int):
        return [value]
    return list(value)


def _as_position_list(value) -> list[float]:
    if isinstance(value, (int, float)):
        return [float(value)]
    return [float(item) for item in value]


class RobotJointGroup:
    """A named group of scalar joints with a configured home position."""

    def __init__(self, robot, joint_index, home_j_pos):
        self.robot = robot
        self.joint_index = _as_index_list(joint_index)
        self.home_j_pos = _as_position_list(home_j_pos)
        if len(self.joint_index) != len(self.home_j_pos):
            raise ValueError(
                "joint_index and rest_j_pos must have the same number of entries"
            )

    @property
    def joint_pos(self) -> list[float]:
        return self.robot.get_joint_positions(self.joint_index)

    def reset(self) -> None:
        self.robot.reset_j(self.joint_index, self.home_j_pos)

    def move(self, positions: Sequence[float]) -> None:
        self.robot.move_j(self.joint_index, positions)


class RobotDorsal(RobotJointGroup):
    def __init__(self, robot, joint_index, home_j_pos):
        super().__init__(robot, joint_index, home_j_pos)
        self.dorsal_lift_joint_index = (
            self.joint_index[0] if len(self.joint_index) == 1 else self.joint_index
        )

    @property
    def height(self):
        values = self.joint_pos
        return values[0] if len(values) == 1 else values


class RobotHead(RobotJointGroup):
    def __init__(self, robot, joint_index, home_j_pos):
        valid_pairs = [
            (index, position)
            for index, position in zip(
                _as_index_list(joint_index), _as_position_list(home_j_pos)
            )
            if index != -1
        ]
        self.head_joint_index = _as_index_list(joint_index)
        self._full_home_j_pos = _as_position_list(home_j_pos)
        if len(self.head_joint_index) != len(self._full_home_j_pos):
            raise ValueError("head joint_index and rest_j_pos lengths do not match")
        super().__init__(
            robot,
            [pair[0] for pair in valid_pairs],
            [pair[1] for pair in valid_pairs],
        )

    @property
    def rotation(self) -> list[float]:
        current = dict(zip(self.joint_index, self.joint_pos))
        return [current.get(index, -1.0) for index in self.head_joint_index]


class RobotBase:
    def __init__(self, robot):
        self.robot = robot
        with self.robot.bullet_lock:
            base_pose = p.getBasePositionAndOrientation(
                self.robot.robot_id,
                physicsClientId=self.robot.physics_clent_id,
            )
            inertial_pose = p.getDynamicsInfo(
                self.robot.robot_id,
                -1,
                physicsClientId=self.robot.physics_clent_id,
            )[3:5]
        self.trans = multiply_transforms(invert_transform(base_pose), inertial_pose)

    @property
    def pose(self):
        with self.robot.bullet_lock:
            base_pose = p.getBasePositionAndOrientation(
                self.robot.robot_id,
                physicsClientId=self.robot.physics_clent_id,
            )
        return multiply_transforms(base_pose, self.trans)

    def reset_base(self, pose) -> None:
        bullet_pose = multiply_transforms(pose, invert_transform(self.trans))
        with self.robot.bullet_lock:
            p.resetBasePositionAndOrientation(
                self.robot.robot_id,
                bullet_pose[0],
                bullet_pose[1],
                physicsClientId=self.robot.physics_clent_id,
            )


class RobotArm(RobotJointGroup):
    def __init__(
        self,
        robot,
        arm_joint_index,
        ee_index,
        home_j_pos,
        with_ee_constraint=False,
    ):
        super().__init__(robot, arm_joint_index, home_j_pos)
        self.arm_joint_index = self.joint_index
        self.ee_index = int(ee_index)
        self.with_ee_constraint = bool(with_ee_constraint)
        self.reset()
        self.home_ee_pose = self.rel_ee_pose
        self._target_body_id = None
        self._target_base_constraint = None
        self._target_ee_constraint = None
        if self.with_ee_constraint:
            self._create_ee_constraint()

    @property
    def abs_ee_pose(self):
        with self.robot.bullet_lock:
            return p.getLinkState(
                self.robot.robot_id,
                self.ee_index,
                computeForwardKinematics=True,
                physicsClientId=self.robot.physics_clent_id,
            )[4:6]

    @property
    def rel_ee_pose(self):
        return multiply_transforms(invert_transform(self.robot.base.pose), self.abs_ee_pose)

    def _create_ee_constraint(self) -> None:
        client_id = self.robot.physics_clent_id
        with self.robot.bullet_lock:
            visual_shape_id = p.createVisualShape(
                shapeType=p.GEOM_BOX,
                halfExtents=[0.0001, 0.0001, 0.0002],
                rgbaColor=[1.0, 0.0, 0.0, 1.0],
                physicsClientId=client_id,
            )
            self._target_body_id = p.createMultiBody(
                baseMass=0.1,
                baseInertialFramePosition=[0.0, 0.0, 0.0],
                baseVisualShapeIndex=visual_shape_id,
                basePosition=self.abs_ee_pose[0],
                baseOrientation=self.abs_ee_pose[1],
                physicsClientId=client_id,
            )
            self._target_base_constraint = p.createConstraint(
                self._target_body_id,
                -1,
                self.robot.robot_id,
                -1,
                p.JOINT_FIXED,
                [0.0, 0.0, 0.0],
                [0.0, 0.0, 0.0],
                self.rel_ee_pose[0],
                [0.0, 0.0, 0.0, 1.0],
                self.rel_ee_pose[1],
                physicsClientId=client_id,
            )
            self._target_ee_constraint = p.createConstraint(
                self._target_body_id,
                -1,
                self.robot.robot_id,
                self.ee_index,
                p.JOINT_FIXED,
                [0.0, 0.0, 0.0],
                [0.0, 0.0, 0.0],
                [0.0, 0.0, 0.0],
                physicsClientId=client_id,
            )

    def move_ee(self, ee_pose) -> None:
        if not self.with_ee_constraint:
            return
        relative_pose = multiply_transforms(invert_transform(self.robot.base.pose), ee_pose)
        self.robot.move_j(self.arm_joint_index, self.home_j_pos)
        delta_ee_pose = multiply_transforms(
            invert_transform(self.home_ee_pose), relative_pose
        )
        delta_base_pose = multiply_transforms(
            invert_transform(
                [
                    self.robot.configs["base_pose"]["position"],
                    self.robot.configs["base_pose"]["orientation"],
                ]
            ),
            self.robot.base.pose,
        )
        euler = -np.asarray(p.getEulerFromQuaternion(delta_ee_pose[1]))
        delta_ee_pose = [
            delta_ee_pose[0],
            p.getQuaternionFromEuler(euler),
        ]
        delta_pose = multiply_transforms(delta_ee_pose, delta_base_pose)
        with self.robot.bullet_lock:
            p.changeConstraint(
                self._target_base_constraint,
                relative_pose[0],
                physicsClientId=self.robot.physics_clent_id,
            )
            p.changeConstraint(
                self._target_ee_constraint,
                [0.0, 0.0, 0.0],
                delta_pose[1],
                physicsClientId=self.robot.physics_clent_id,
            )


class RobotGripper:
    def __init__(self, robot, finger_index):
        self.robot = robot
        self.finger_index = _as_index_list(finger_index)
        self.joint_range = [self.robot.joint_limits[index] for index in self.finger_index]

    @property
    def joint_pos(self) -> list[float]:
        return self.robot.get_joint_positions(self.finger_index)

    def joint_pos_from_status(self, status: float) -> list[float]:
        status = float(np.clip(status, 0.0, 1.0))
        return [
            lower + (upper - lower) * status
            for lower, upper in self.joint_range
        ]

    def reset(self, status: float) -> None:
        self.robot.reset_j(self.finger_index, self.joint_pos_from_status(status))

    def move(self, status: float) -> None:
        self.robot.move_j(self.finger_index, self.joint_pos_from_status(status))

    @property
    def status(self) -> float:
        minimum = sum(bounds[0] for bounds in self.joint_range)
        maximum = sum(bounds[1] for bounds in self.joint_range)
        if math.isclose(minimum, maximum):
            return 0.0
        return float(np.clip((sum(self.joint_pos) - minimum) / (maximum - minimum), 0, 1))


@dataclass
class RobotCamera:
    robot: object
    camera_name: str
    camera_quat: Sequence[float]
    camera_height: float
    pos_offset: Sequence[float]
    target_offset: Sequence[float]
    near_val: float
    far_val: float
    up_axis: Sequence[float]
    img_width: int
    img_height: int

    def get_cam_base_pose(self):
        if self.camera_name == "first_person":
            return self.robot.base.pose
        if "hand_eye" in self.camera_name:
            arm_index = 0 if "right" in self.camera_name else 1
            return self.robot.arms[arm_index].abs_ee_pose
        raise ValueError(f"Unsupported camera name: {self.camera_name}")

    def get_img(self):
        eye_pos, eye_ori = multiply_transforms(
            self.get_cam_base_pose(), [[0.0, 0.0, 0.0], self.camera_quat]
        )
        rotation = p.getMatrixFromQuaternion(eye_ori)
        y_axis = np.asarray([rotation[1], rotation[4], rotation[7]])
        z_axis = np.asarray([rotation[2], rotation[5], rotation[8]])
        camera_position = (
            np.asarray(eye_pos)
            + np.asarray([0.0, 0.0, self.camera_height])
            + np.asarray(self.pos_offset) * z_axis
        )
        target_position = np.asarray(eye_pos) + np.asarray(self.target_offset) * z_axis
        view = p.computeViewMatrix(
            cameraEyePosition=camera_position,
            cameraTargetPosition=target_position,
            cameraUpVector=np.asarray(self.up_axis) * y_axis,
        )
        projection = p.computeProjectionMatrixFOV(
            fov=100,
            aspect=float(self.img_width) / float(self.img_height),
            nearVal=self.near_val,
            farVal=self.far_val,
        )
        with self.robot.bullet_lock:
            return p.getCameraImage(
                self.img_width,
                self.img_height,
                view,
                projection,
                shadow=1,
                renderer=p.ER_BULLET_HARDWARE_OPENGL,
                flags=p.ER_NO_SEGMENTATION_MASK,
                physicsClientId=self.robot.physics_clent_id,
            )[2]


class AssembledRobot:
    """Load a URDF and expose components declared in ``vr_configs.yml``."""

    def __init__(
        self,
        robot_urdf_path,
        configs,
        debug=False,
        with_ee_constraint=False,
        physics_clent_id=0,
    ):
        self.configs = configs
        self.physics_clent_id = physics_clent_id
        if not hasattr(self, "_bullet_lock"):
            self._bullet_lock = threading.RLock()
        self._debug_joint_parameters: dict[int, int] = {}
        self.fixed_link_constraint_id: Optional[int] = None
        self.load_urdf(robot_urdf_path, debug)
        self.assemble_robot(with_ee_constraint)

    @property
    def bullet_lock(self):
        return self._bullet_lock

    def _warning(self, message: str) -> None:
        get_logger = getattr(self, "get_logger", None)
        if callable(get_logger):
            get_logger().warning(message)
        else:
            print(f"[sim warning] {message}")

    def load_urdf(self, robot_urdf_path, debug=False):
        robot_urdf_path = os.path.abspath(robot_urdf_path)
        if not os.path.isfile(robot_urdf_path):
            raise FileNotFoundError(f"Robot URDF does not exist: {robot_urdf_path}")
        base_pose = self.configs.get("base_pose")
        if not isinstance(base_pose, dict):
            raise ValueError("vr_configs.yml must define base_pose")

        use_fixed_base = "fixed_link" not in self.configs
        # The numeric indices in existing vr_configs.yml files were generated
        # with PyBullet's default traversal order.  MAINTAIN_LINK_ORDER changes
        # those IDs for branched robots such as HC-TJ.
        flags = p.URDF_ENABLE_CACHED_GRAPHICS_SHAPES
        with self.bullet_lock:
            self.robot_id = p.loadURDF(
                fileName=robot_urdf_path,
                basePosition=base_pose["position"],
                baseOrientation=base_pose["orientation"],
                useFixedBase=use_fixed_base,
                flags=flags,
                physicsClientId=self.physics_clent_id,
            )
        if self.robot_id < 0:
            raise RuntimeError(f"PyBullet failed to load URDF: {robot_urdf_path}")

        self.joint_name2id_dict: dict[str, int] = {}
        self.joint_id2name_dict: dict[int, str] = {}
        self.link_name2id_dict: dict[str, int] = {}
        self.joint_limits: dict[int, tuple[float, float]] = {}
        self.joint_types: dict[int, int] = {}

        with self.bullet_lock:
            joint_count = p.getNumJoints(
                self.robot_id, physicsClientId=self.physics_clent_id
            )
            for index in range(joint_count):
                info = p.getJointInfo(
                    self.robot_id,
                    index,
                    physicsClientId=self.physics_clent_id,
                )
                joint_name = info[1].decode("utf-8")
                link_name = info[12].decode("utf-8")
                self.joint_name2id_dict[joint_name] = index
                self.joint_id2name_dict[index] = joint_name
                self.link_name2id_dict[link_name] = index
                self.joint_limits[index] = (float(info[8]), float(info[9]))
                self.joint_types[index] = info[2]

        self._resolve_config_indices(robot_urdf_path)

        if not use_fixed_base:
            fixed_link = self.configs["fixed_link"]
            if fixed_link not in self.link_name2id_dict:
                raise ValueError(f"fixed_link is not present in URDF: {fixed_link}")
            with self.bullet_lock:
                self.fixed_link_constraint_id = p.createConstraint(
                    parentBodyUniqueId=self.robot_id,
                    parentLinkIndex=self.link_name2id_dict[fixed_link],
                    childBodyUniqueId=-1,
                    childLinkIndex=-1,
                    jointType=p.JOINT_FIXED,
                    jointAxis=[0.0, 0.0, 0.0],
                    parentFramePosition=[0.0, 0.0, 0.0],
                    childFramePosition=base_pose["position"],
                    childFrameOrientation=base_pose["orientation"],
                    physicsClientId=self.physics_clent_id,
                )

        if debug:
            self._create_debug_parameters()
        return self.robot_id

    def _resolve_config_indices(self, robot_urdf_path: str) -> None:
        """Translate stable URDF-order indices to PyBullet traversal indices."""
        order = str(self.configs.get("joint_index_order", "pybullet")).lower()
        if order == "pybullet":
            return
        if order != "urdf":
            raise ValueError("joint_index_order must be 'pybullet' or 'urdf'")

        document = ElementTree.parse(robot_urdf_path)
        urdf_joints = document.getroot().findall("joint")
        index_map: dict[int, int] = {}
        for index, joint in enumerate(urdf_joints):
            name = str(joint.get("name", "")).strip()
            if name not in self.joint_name2id_dict:
                raise ValueError(
                    f"URDF joint {name or index} is missing from the PyBullet model"
                )
            index_map[index] = self.joint_name2id_dict[name]

        def resolve(value, label: str):
            if isinstance(value, bool) or not isinstance(value, int):
                raise ValueError(f"{label} must be an integer")
            if value == -1:
                return -1
            if value not in index_map:
                raise ValueError(f"{label} references missing URDF joint index {value}")
            return index_map[value]

        def resolve_list(value, label: str):
            if not isinstance(value, list):
                raise ValueError(f"{label} must be an integer list")
            return [resolve(item, f"{label}[{offset}]") for offset, item in enumerate(value)]

        for group_name in ("arms", "grippers"):
            for offset, group in enumerate(self.configs.get(group_name, [])):
                if "joint_index" in group:
                    group["joint_index"] = resolve_list(
                        group["joint_index"], f"{group_name}[{offset}].joint_index"
                    )
                if "ee_index" in group:
                    group["ee_index"] = resolve(
                        group["ee_index"], f"{group_name}[{offset}].ee_index"
                    )

        for group_name in ("folding_waist", "waist", "dorsal", "head"):
            group = self.configs.get(group_name)
            if not isinstance(group, dict):
                continue
            if "joint_index" in group:
                group["joint_index"] = resolve_list(
                    group["joint_index"], f"{group_name}.joint_index"
                )
            for key in ("cmd_ee", "base"):
                if key in group:
                    group[key] = resolve(group[key], f"{group_name}.{key}")

        controller = self.configs.get("controller_indices")
        if isinstance(controller, dict):
            for key in ("cmd_ee", "base"):
                if key in controller:
                    controller[key] = resolve_list(
                        controller[key], f"controller_indices.{key}"
                    )
        self.configs["joint_index_order"] = "pybullet"

    def _create_debug_parameters(self) -> None:
        type_names = {
            p.JOINT_REVOLUTE: "REVOLUTE",
            p.JOINT_PRISMATIC: "PRISMATIC",
            p.JOINT_SPHERICAL: "SPHERICAL",
            p.JOINT_PLANAR: "PLANAR",
            p.JOINT_FIXED: "FIXED",
        }
        with self.bullet_lock:
            for index, name in self.joint_id2name_dict.items():
                joint_type = self.joint_types[index]
                link_name = next(
                    link for link, link_index in self.link_name2id_dict.items()
                    if link_index == index
                )
                print(
                    f"joint&link_id: {index:<5} "
                    f"joint_type:{type_names.get(joint_type, str(joint_type)):<10} "
                    f"joint_name: {name:<28} link_name: {link_name:<28}"
                )
                if joint_type not in (p.JOINT_REVOLUTE, p.JOINT_PRISMATIC):
                    continue
                lower, upper = self.joint_limits[index]
                if lower >= upper:
                    lower, upper = -math.pi, math.pi
                current = p.getJointState(
                    self.robot_id,
                    index,
                    physicsClientId=self.physics_clent_id,
                )[0]
                parameter_id = p.addUserDebugParameter(
                    f"{index:3d} {name}",
                    lower,
                    upper,
                    current,
                    physicsClientId=self.physics_clent_id,
                )
                if parameter_id >= 0:
                    self._debug_joint_parameters[index] = parameter_id

    def update_debug_joints(self) -> None:
        if not self._debug_joint_parameters:
            return
        with self.bullet_lock:
            positions = [
                p.readUserDebugParameter(
                    parameter_id, physicsClientId=self.physics_clent_id
                )
                for parameter_id in self._debug_joint_parameters.values()
            ]
        self.reset_j(list(self._debug_joint_parameters), positions)

    def _validate_link_index(self, index: int, label: str) -> None:
        if index != -1 and index not in self.joint_id2name_dict:
            raise ValueError(f"{label} references missing URDF link index {index}")

    def _make_group(self, config_name: str):
        config = self.configs[config_name]
        group = RobotJointGroup(
            self,
            config["joint_index"],
            config.get("rest_j_pos", 0.0),
        )
        self._validate_joint_group(group.joint_index, config_name)
        return group

    def assemble_robot(self, with_ee_constraint=False) -> None:
        self.base = RobotBase(self)
        self.arms: list[RobotArm] = []
        for arm_number, config in enumerate(self.configs.get("arms", [])):
            self._validate_link_index(config["ee_index"], f"arms[{arm_number}].ee_index")
            arm = RobotArm(
                self,
                config["joint_index"],
                config["ee_index"],
                config["rest_j_pos"],
                with_ee_constraint,
            )
            self._validate_joint_group(arm.joint_index, f"arms[{arm_number}]")
            self.arms.append(arm)

        if "grippers" in self.configs:
            self.grippers: list[RobotGripper] = []
            for gripper_number, config in enumerate(self.configs["grippers"]):
                indices = _as_index_list(config["joint_index"])
                self._validate_joint_group(indices, f"grippers[{gripper_number}]")
                gripper = RobotGripper(self, indices)
                self.grippers.append(gripper)
                with self.bullet_lock:
                    for index in gripper.finger_index:
                        p.changeDynamics(
                            self.robot_id,
                            index,
                            mass=0.1,
                            lateralFriction=1000,
                            physicsClientId=self.physics_clent_id,
                        )
                    if len(gripper.finger_index) == 2:
                        constraint_id = p.createConstraint(
                            self.robot_id,
                            gripper.finger_index[0],
                            self.robot_id,
                            gripper.finger_index[1],
                            jointType=p.JOINT_GEAR,
                            jointAxis=[1.0, 0.0, 0.0],
                            parentFramePosition=[0.0, 0.0, 0.0],
                            childFramePosition=[0.0, 0.0, 0.0],
                            physicsClientId=self.physics_clent_id,
                        )
                        p.changeConstraint(
                            constraint_id,
                            gearRatio=-1,
                            erp=0.1,
                            maxForce=50,
                            physicsClientId=self.physics_clent_id,
                        )

        if "head" in self.configs:
            config = self.configs["head"]
            self.head = RobotHead(
                self, config["joint_index"], config.get("rest_j_pos", 0.0)
            )
            self._validate_joint_group(self.head.joint_index, "head")
        if "dorsal" in self.configs:
            config = self.configs["dorsal"]
            self.dorsal = RobotDorsal(
                self, config["joint_index"], config.get("rest_j_pos", 0.0)
            )
            self._validate_joint_group(self.dorsal.joint_index, "dorsal")
        if "waist" in self.configs:
            self.waist = self._make_group("waist")
        if "folding_waist" in self.configs:
            self.folding_waist = self._make_group("folding_waist")

        if "cameras" in self.configs:
            self.cameras = [
                RobotCamera(
                    self,
                    config["name"],
                    config["quat"],
                    config["height"],
                    config["pos_offset"],
                    config["target_offset"],
                    config["near_val"],
                    config["far_val"],
                    config["up_axis"],
                    config["img_width"],
                    config["img_height"],
                )
                for config in self.configs["cameras"]
            ]

    def _validate_joint_group(self, indices: Iterable[int], label: str) -> None:
        missing = [index for index in indices if index not in self.joint_id2name_dict]
        if missing:
            raise ValueError(f"{label} references missing URDF joint indices: {missing}")
        fixed = [
            index for index in indices if self.joint_types[index] == p.JOINT_FIXED
        ]
        if fixed:
            raise ValueError(f"{label} references fixed URDF joints: {fixed}")

    def _bounded_position(self, joint_index: int, position: float) -> float:
        if not math.isfinite(position):
            raise ValueError(
                f"Joint {self.joint_id2name_dict[joint_index]} received non-finite position"
            )
        lower, upper = self.joint_limits[joint_index]
        if lower < upper and position < lower:
            self._warning(
                f"Clamped {self.joint_id2name_dict[joint_index]} from {position:.4f} "
                f"to lower limit {lower:.4f}"
            )
            return lower
        if lower < upper and position > upper:
            self._warning(
                f"Clamped {self.joint_id2name_dict[joint_index]} from {position:.4f} "
                f"to upper limit {upper:.4f}"
            )
            return upper
        return position

    def reset_j(self, j_index, j_pos) -> None:
        indices = _as_index_list(j_index)
        positions = _as_position_list(j_pos)
        if len(indices) != len(positions):
            raise ValueError("Joint index and position lengths do not match")
        self._validate_joint_group(indices, "joint command")
        positions = [
            self._bounded_position(index, position)
            for index, position in zip(indices, positions)
        ]
        with self.bullet_lock:
            for index, position in zip(indices, positions):
                p.resetJointState(
                    self.robot_id,
                    index,
                    targetValue=position,
                    targetVelocity=0.0,
                    physicsClientId=self.physics_clent_id,
                )

    def get_joint_positions(self, indices: Sequence[int]) -> list[float]:
        with self.bullet_lock:
            return [
                p.getJointState(
                    self.robot_id,
                    index,
                    physicsClientId=self.physics_clent_id,
                )[0]
                for index in indices
            ]

    def move_j(
        self,
        j_index,
        j_pos,
        max_vel: Optional[Sequence[float]] = None,
        force: Optional[Sequence[float]] = None,
    ) -> None:
        indices = _as_index_list(j_index)
        positions = _as_position_list(j_pos)
        if len(indices) != len(positions):
            raise ValueError("Joint index and position lengths do not match")
        self._validate_joint_group(indices, "joint command")
        if max_vel is not None and len(max_vel) != len(indices):
            raise ValueError("max_vel length does not match joint count")
        if force is not None and len(force) != len(indices):
            raise ValueError("force length does not match joint count")

        with self.bullet_lock:
            for offset, (index, position) in enumerate(zip(indices, positions)):
                info = p.getJointInfo(
                    self.robot_id,
                    index,
                    physicsClientId=self.physics_clent_id,
                )
                kwargs = {
                    "bodyUniqueId": self.robot_id,
                    "jointIndex": index,
                    "controlMode": p.POSITION_CONTROL,
                    "targetPosition": self._bounded_position(index, position),
                    "force": float(force[offset]) if force is not None else float(info[10]),
                    "physicsClientId": self.physics_clent_id,
                }
                velocity = float(max_vel[offset]) if max_vel is not None else float(info[11])
                if velocity > 0.0:
                    kwargs["maxVelocity"] = velocity
                p.setJointMotorControl2(**kwargs)

"""PyBullet backend with named joints, measured simulated feedback and fencing.

Arm joints use position motors. The mobile base is a separately declared
kinematic chassis model, not a claim to model X1 wheel/contact dynamics.
"""

import math
import time

from .backend import OutputGate, allowed_joint_names, map_gripper
from .model import UrdfModel


class SimulationAdapter:
    physical_stop_ack = False

    def __init__(self, config, on_feedback):
        self.config, self.on_feedback = config, on_feedback
        self.options = config.get("sim", {})
        self.dt = 1.0 / float(self.options.get("physics_hz", 240))
        self.model = UrdfModel(
            config.get("urdf_path", config.get("urdf")), config.get("package_root")
        )
        self.gate = OutputGate(allowed_joint_names(config))
        self._mimics = {
            name: joint.mimic
            for name, joint in self.model.joints.items()
            if joint.mimic is not None
        }
        self.client = self.robot = None
        self.pending = None
        self.active = None
        self.targets = {}
        self.seq = 0
        self._feedback_elapsed = 0.0
        self.last_error = None
        self.base_velocity = (0.0, 0.0, 0.0)
        self.on_output = None
        self._holding = True

    def start(self):
        try:
            import pybullet as bullet
        except ImportError as error:
            raise RuntimeError(
                "PyBullet simulation dependency is not installed"
            ) from error
        self.bullet = bullet
        self.client = bullet.connect(
            bullet.GUI if self.options.get("gui", False) else bullet.DIRECT
        )
        if self.client < 0:
            raise RuntimeError("PyBullet connection failed")
        try:
            bullet.setTimeStep(self.dt, physicsClientId=self.client)
            bullet.setGravity(0, 0, -9.81, physicsClientId=self.client)
            pose = self.options.get("base_pose", {})
            self.base_position = list(
                pose.get("position", self.options.get("base_position", [0, 0, 0.07]))
            )
            self.base_orientation = list(
                pose.get(
                    "orientation",
                    self.options.get(
                        "base_orientation", [0, 0, 0.70710678, 0.70710678]
                    ),
                )
            )
            self.robot = bullet.loadURDF(
                str(self.model.path),
                self.base_position,
                self.base_orientation,
                useFixedBase=True,
                physicsClientId=self.client,
            )
            self.joint_indices = {}
            for index in range(
                bullet.getNumJoints(self.robot, physicsClientId=self.client)
            ):
                info = bullet.getJointInfo(
                    self.robot, index, physicsClientId=self.client
                )
                self.joint_indices[info[1].decode()] = index
            initial = dict(zip(self.config["joint_names"], self.config["home"]))
            active_joints = [
                name
                for name, joint in self.model.joints.items()
                if joint.kind != "fixed" and name in self.joint_indices
            ]
            for name in active_joints:
                self.targets[name] = float(initial.get(name, 0.0))
            self._set_gripper_targets(self._grippers(), [0.0, 0.0], [True, True])
            self._apply_mimics(set())
            for name in active_joints:
                bullet.resetJointState(
                    self.robot,
                    self.joint_indices[name],
                    self.targets[name],
                    physicsClientId=self.client,
                )
            self._apply_motors()
            self._feedback()
        except BaseException:
            self.close()
            raise

    def submit(self, output):
        self.pending = output

    def _grippers(self) -> dict:
        return self.config.get(
            "grippers", self.config.get("teleop", {}).get("grippers", {})
        )

    def _apply_mimics(self, explicit_joints) -> None:
        for _ in range(len(self._mimics)):
            changed = False
            for child, (parent, multiplier, offset) in self._mimics.items():
                if (
                    child in self.targets
                    and child not in explicit_joints
                    and parent in self.targets
                ):
                    value = self.targets[parent] * multiplier + offset
                    if self.targets[child] != value:
                        self.targets[child] = value
                        changed = True
            if not changed:
                break

    def _set_gripper_targets(self, grippers: dict, amounts, valid) -> None:
        if len(valid) != 2 or len(amounts) != 2:
            return
        applied = set()
        for i, side in enumerate(("left", "right")):
            if not valid[i]:
                continue
            calibration = grippers.get(side, {})
            amount = max(0.0, min(1.0, float(amounts[i])))
            names = calibration.get("finger_joint_names", ())
            opened = calibration.get("finger_open", ())
            closed = calibration.get("finger_closed", ())
            if names and len(names) == len(opened) == len(closed):
                mapped_names, mapped_values = map_gripper(calibration, amount)
                for name, value in zip(mapped_names, mapped_values):
                    if name in self.targets:
                        self.targets[name] = float(value)
                        applied.add(name)
            sim_joint = calibration.get("sim_joint")
            if sim_joint in self.targets and sim_joint not in applied:
                open_pos = float(calibration.get("sim_open_position", 0.0))
                closed_pos = float(calibration.get("sim_closed_position", 1.5707))
                self.targets[sim_joint] = open_pos + (closed_pos - open_pos) * amount
                applied.add(sim_joint)
        self._apply_mimics(applied)

    def fence(self, boot_id, epoch, timeout=0.5):
        self.pending = None
        self.gate.fence(boot_id, epoch)
        self.stop(force=True)

    def _apply_motors(self):
        names = list(self.targets)
        self.bullet.setJointMotorControlArray(
            self.robot,
            [self.joint_indices[n] for n in names],
            self.bullet.POSITION_CONTROL,
            targetPositions=[self.targets[n] for n in names],
            forces=[max(1.0, self.model.joints[n].effort) for n in names],
            positionGains=[0.3] * len(names),
            velocityGains=[1.0] * len(names),
            physicsClientId=self.client,
        )

    def step(self):
        if self.client is None:
            raise RuntimeError("simulation not started")
        now = time.monotonic_ns()
        command, self.pending = self.pending, None
        if command is not None:
            if self.gate.validate(command, now) and command["mode"] != "stop":
                self.targets.update(zip(command["joint_names"], command["positions"]))
                self.active = command
                self._holding = False
                self.base_velocity = tuple(command.get("base_velocity", [0, 0, 0]))
                valid = command.get(
                    "grippers_valid", command.get("gripper_valid", [False, False])
                )
                self._set_gripper_targets(
                    self._grippers(), command.get("grippers", [0, 0]), valid
                )
                self._apply_motors()
                if self.on_output:
                    self.on_output(command["seq"], True)
            else:
                self.stop()
                if self.on_output:
                    self.on_output(
                        command.get("seq", 0),
                        command.get("mode") == "stop" and self.gate.reason == "OK",
                        self.gate.reason,
                    )
        if self.active is not None and self.active["deadline_mono_ns"] <= now:
            self.stop()
        if any(self.base_velocity):
            position, orientation = self.bullet.getBasePositionAndOrientation(
                self.robot, physicsClientId=self.client
            )
            yaw = self.bullet.getEulerFromQuaternion(orientation)[2]
            vx, vy, wz = self.base_velocity
            position = [
                position[0] + (math.cos(yaw) * vx - math.sin(yaw) * vy) * self.dt,
                position[1] + (math.sin(yaw) * vx + math.cos(yaw) * vy) * self.dt,
                position[2],
            ]
            orientation = self.bullet.getQuaternionFromEuler([0, 0, yaw + wz * self.dt])
            self.bullet.resetBasePositionAndOrientation(
                self.robot, position, orientation, physicsClientId=self.client
            )
        self.bullet.stepSimulation(physicsClientId=self.client)
        self._feedback_elapsed += self.dt
        if self._feedback_elapsed >= 0.01:
            self._feedback_elapsed %= 0.01
            self._feedback()

    def _feedback(self):
        names = [n for n in self.config["joint_names"] if n in self.joint_indices]
        states = self.bullet.getJointStates(
            self.robot,
            [self.joint_indices[n] for n in names],
            physicsClientId=self.client,
        )
        self.seq += 1
        all_names = list(self.model.joint_names)
        all_states = self.bullet.getJointStates(
            self.robot,
            [self.joint_indices[n] for n in all_names],
            physicsClientId=self.client,
        )
        base_position, base_orientation = self.bullet.getBasePositionAndOrientation(
            self.robot, physicsClientId=self.client
        )
        self.on_feedback(
            {
                "op": "feedback",
                "seq": self.seq,
                "joint_names": names,
                "positions": [x[0] for x in states],
                "velocities": [x[1] for x in states],
                "ready": True,
                "sample_mono_ns": time.monotonic_ns(),
                "sample_unix_ns": time.time_ns(),
                "model_joint_names": all_names,
                "model_positions": [x[0] for x in all_states],
                "base_pose": list(base_position) + list(base_orientation),
                "source": "pybullet_state",
            }
        )

    def stop(self, force=False):
        self.pending = self.active = None
        self.base_velocity = (0.0, 0.0, 0.0)
        if self._holding and not force:
            return
        self._holding = True
        if self.client is not None and self.robot is not None:
            names = list(self.targets)
            states = self.bullet.getJointStates(
                self.robot,
                [self.joint_indices[n] for n in names],
                physicsClientId=self.client,
            )
            self.targets = {name: state[0] for name, state in zip(names, states)}
            self._apply_motors()

    def status(self):
        return {
            "backend": "pybullet",
            "ready": self.client is not None,
            "physical_stop_ack": False,
            "joint_model": "position_motors",
            "chassis_model": "kinematic_velocity",
            "reject_reason": self.gate.reason,
        }

    def close(self):
        if self.client is not None:
            self.bullet.disconnect(physicsClientId=self.client)
            self.client = self.robot = None

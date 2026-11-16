"""ROS boundary to the independently deployed HC_X1 hardware repository.

One ROS executor thread owns all ROS publishers. Async IPC uses a one-command
mailbox; a fence is acknowledged only after that thread has cleared the mailbox
and invoked the software hold/zero procedure. This is NOT physical stop ACK.
"""

from collections import deque
import math
import threading
import time

from .backend import OutputGate, X1Topics, allowed_joint_names, map_gripper


class X1RosAdapter:
    physical_stop_ack = False

    def __init__(self, config, on_feedback):
        self.config, self.on_feedback = config, on_feedback
        hardware = config.get("hardware", {})
        self.allowed_groups = set(
            hardware.get(
                "allowed_groups",
                config.get("allowed_groups", ["right_arm", "right_gripper"]),
            )
        )
        self.allowed_names = allowed_joint_names(config, self.allowed_groups)
        self.topics = X1Topics.for_contract(
            hardware.get("contract", "legacy-model-topic")
        )
        self.gate = OutputGate(self.allowed_names)
        self.feedback_timeout = float(hardware.get("feedback_timeout_s", 0.2))
        self.ready_timeout = float(hardware.get("ready_timeout_s", 1.0))
        self.domain_id = int(
            hardware.get("ros_domain_id", config.get("ros_domain_id", 14))
        )
        self.grippers = config.get(
            "grippers", config.get("teleop", {}).get("grippers", {})
        )
        self.feedback = {}
        self.feedback_at = {}
        self.ready = False
        self.ready_at = 0.0
        self.seq = 0
        self.on_output = None
        self.hand_feedback = {}
        self.hand_feedback_at = {}
        self.last_error = None
        self.conflict = False
        self._mailbox = None
        self._fences = deque()
        self._lock = threading.Lock()
        self._quit = threading.Event()
        self._started = threading.Event()
        self._thread = None
        self._last_applied = None
        self._moving = False
        self.node = self.context = self.executor = None

    def start(self):
        if self._thread:
            raise RuntimeError("adapter already started")
        self._thread = threading.Thread(
            target=self._run, name="x1-ros-boundary", daemon=False
        )
        self._thread.start()
        if not self._started.wait(10):
            self._quit.set()
            raise RuntimeError("ROS adapter startup timed out")
        if self.last_error:
            raise RuntimeError(self.last_error)

    def _run(self):
        try:
            import rclpy
            from rclpy.context import Context
            from rclpy.executors import SingleThreadedExecutor
            from rclpy.qos import (
                QoSProfile,
                HistoryPolicy,
                ReliabilityPolicy,
                DurabilityPolicy,
            )
            from sensor_msgs.msg import JointState
            from std_msgs.msg import Bool, Float64MultiArray

            self._JointState, self._Base = JointState, Float64MultiArray
            self.context = Context()
            rclpy.init(context=self.context, domain_id=self.domain_id)
            self.node = rclpy.create_node(
                "hc_platform_x1_boundary", context=self.context
            )
            qos = QoSProfile(
                history=HistoryPolicy.KEEP_LAST,
                depth=1,
                reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.VOLATILE,
            )
            self.joint_pub = self.node.create_publisher(
                JointState, self.topics.command, qos
            )
            self.base_pub = (
                self.node.create_publisher(Float64MultiArray, self.topics.base, qos)
                if "base" in self.allowed_groups
                else None
            )
            self.hand_pubs = {}
            for side in ("left", "right"):
                if side + "_gripper" in self.allowed_groups and side in self.grippers:
                    map_gripper(
                        self.grippers[side], 0.0
                    )  # validate before publishers are used
                    self.hand_pubs[side] = self.node.create_publisher(
                        JointState, getattr(self.topics, side + "_hand"), qos
                    )
            self.node.create_subscription(
                JointState, self.topics.feedback, self._feedback, qos
            )
            self.node.create_subscription(Bool, self.topics.ready, self._ready, qos)
            self.node.create_subscription(
                JointState, "/observation/hands/joint_states", self._hands, qos
            )
            self.node.create_timer(0.01, self._tick)
            self.node.create_timer(0.5, self._check_conflict)
            self.executor = SingleThreadedExecutor(context=self.context)
            self.executor.add_node(self.node)
            self._started.set()
            while not self._quit.is_set():
                self.executor.spin_once(timeout_sec=0.02)
            self._halt()
        except Exception as error:
            self.last_error = str(error)
            self._started.set()
        finally:
            if self.executor:
                self.executor.shutdown(timeout_sec=1.0)
            if self.node:
                self.node.destroy_node()
            if self.context and self.context.ok():
                self.context.shutdown()

    def _check_conflict(self):
        conflicts = self.node.count_publishers(self.topics.command) > 1
        if conflicts:
            self.conflict = True
            self.last_error = "MULTIPLE_COMMAND_PUBLISHERS"
            self._halt()
        else:
            self.conflict = False

    def _ready(self, message):
        self.ready, self.ready_at = bool(message.data), time.monotonic()
        if not self.ready:
            self._halt()

    def _feedback(self, message):
        if len(message.name) != len(message.position) or len(set(message.name)) != len(
            message.name
        ):
            self.last_error = "INVALID_FEEDBACK_NAMES"
            return
        if not all(math.isfinite(x) for x in message.position):
            self.last_error = "NONFINITE_FEEDBACK"
            return
        now = time.monotonic()
        known = set(self.config["joint_names"])
        pairs = [
            (name, float(position))
            for name, position in zip(message.name, message.position)
            if name in known
        ]
        if not pairs:
            self.last_error = "FEEDBACK_CONTRACT_MISMATCH"
            return
        for name, position in pairs:
            self.feedback[name], self.feedback_at[name] = position, now
        self.seq += 1
        self.on_feedback(
            {
                "op": "feedback",
                "seq": self.seq,
                "joint_names": [p[0] for p in pairs],
                "positions": [p[1] for p in pairs],
                "sample_mono_ns": time.monotonic_ns(),
                "ready": self.ready
                and now - self.ready_at < self.ready_timeout
                and not self.conflict,
            }
        )

    def _hands(self, message):
        if (
            len(message.name) != len(message.position)
            or len(set(message.name)) != len(message.name)
            or not all(math.isfinite(x) for x in message.position)
        ):
            return
        now = time.monotonic()
        self.hand_feedback.update(zip(message.name, map(float, message.position)))
        for name in message.name:
            self.hand_feedback_at[name] = now

    def _ack(self, command, accepted, reason=""):
        if self.on_output:
            self.on_output(command.get("seq", 0), accepted, reason)

    def submit(self, output):
        # IPC reader overwrites pending setpoints; it never publishes ROS itself.
        with self._lock:
            self._mailbox = output

    def fence(self, boot_id, epoch, timeout=0.5):
        event = threading.Event()
        with self._lock:
            if len(self._fences) >= 8:
                self._mailbox = None
                raise RuntimeError("backend fence queue overflow")
            self._mailbox = None
            self._fences.append((boot_id, epoch, event))
        if not event.wait(timeout):
            raise TimeoutError("backend software stop was not acknowledged")

    def stop(self):
        # Local disconnect path uses the same executor-owned fence boundary.
        self.fence(self.gate.boot_id or "disconnected", max(0, self.gate.epoch))

    def _tick(self):
        with self._lock:
            barriers = list(self._fences)
            self._fences.clear()
            command, self._mailbox = self._mailbox, None
        if barriers:
            command = None
            self._halt()
            for boot, epoch, event in barriers:
                try:
                    self.gate.fence(boot, epoch)
                finally:
                    event.set()
        now, now_ns = time.monotonic(), time.monotonic_ns()
        if command is None:
            if self._moving and (
                not self._last_applied
                or now_ns > self._last_applied["deadline_mono_ns"]
            ):
                self._halt()
            return
        if not self.gate.validate(command, now_ns):
            self._halt()
            self._ack(command, False, self.gate.reason)
            return
        if command["mode"] == "stop":
            self._halt()
            self._ack(command, True, "SOFTWARE_STOP")
            return
        if not self.ready or now - self.ready_at > self.ready_timeout or self.conflict:
            self.last_error = "HARDWARE_NOT_READY"
            self._halt()
            self._ack(command, False, self.last_error)
            return
        names = command["joint_names"]
        if any(
            now - self.feedback_at.get(name, -math.inf) > self.feedback_timeout
            for name in names
        ):
            self.last_error = "FEEDBACK_STALE"
            self._halt()
            self._ack(command, False, self.last_error)
            return
        # Synthetic URDF gripper joints are simulation geometry only. Physical
        # fingers are sent through the calibrated dedicated standard channels.
        sim_names = {
            entry.get("sim_joint")
            for entry in self.grippers.values()
            if isinstance(entry, dict)
        }
        joints = [
            (n, p) for n, p in zip(names, command["positions"]) if n not in sim_names
        ]
        if joints:
            self._publish_joints(
                self.joint_pub, [n for n, p in joints], [p for n, p in joints]
            )
        if self.base_pub:
            vx, vy, wz = command.get("base_velocity", [0, 0, 0])
            message = self._Base()
            message.data = [float(wz), float(vx), float(vy)]  # HC_X1 wire order
            self.base_pub.publish(message)
        valid = command.get(
            "grippers_valid", command.get("gripper_valid", [False, False])
        )
        dropped = []
        for i, side in enumerate(("left", "right")):
            if side in self.hand_pubs and len(valid) == 2 and valid[i]:
                hand_names, positions = map_gripper(
                    self.grippers[side], command.get("grippers", [0, 0])[i]
                )
                if all(
                    now - self.hand_feedback_at.get(name, -math.inf) < 0.5
                    for name in hand_names
                ):
                    self._publish_joints(self.hand_pubs[side], hand_names, positions)
                else:
                    dropped.append(side)
        self._last_applied, self._moving = command, True
        self._ack(
            command,
            not dropped,
            "PARTIAL_GRIPPER_FEEDBACK_STALE:" + ",".join(dropped) if dropped else "",
        )

    def _publish_joints(self, publisher, names, positions):
        message = self._JointState()
        message.header.stamp = self.node.get_clock().now().to_msg()
        message.name, message.position = list(names), list(map(float, positions))
        publisher.publish(message)

    def _halt(self):
        self._moving, self._last_applied = False, None
        with self._lock:
            self._mailbox = None
        if self.node is None:
            return
        now = time.monotonic()
        pairs = [
            (name, value)
            for name, value in self.feedback.items()
            if name in self.allowed_names
            and now - self.feedback_at[name] < self.feedback_timeout
        ]
        if (
            pairs
            and hasattr(self, "joint_pub")
            and not self.conflict
            and self.ready
            and now - self.ready_at < self.ready_timeout
        ):
            self._publish_joints(
                self.joint_pub, [n for n, p in pairs], [p for n, p in pairs]
            )
        if getattr(self, "base_pub", None):
            message = self._Base()
            message.data = [0.0, 0.0, 0.0]
            self.base_pub.publish(message)

    def status(self):
        return {
            "backend": "x1_ros",
            "contract": self.config.get("hardware", {}).get(
                "contract", "legacy-model-topic"
            ),
            "ready": self.ready
            and time.monotonic() - self.ready_at < self.ready_timeout
            and bool(self._thread and self._thread.is_alive()),
            "allowed_groups": sorted(self.allowed_groups),
            "physical_stop_ack": False,
            "software_fence_ack": True,
            "reject_reason": self.gate.reason,
            "error": self.last_error,
            "publisher_conflict": self.conflict,
            "gripper_feedback_source": "finger_joint_encoders",
            "gripper_ready": {
                side: all(
                    time.monotonic() - self.hand_feedback_at.get(name, -math.inf) < 0.5
                    for name in calibration.get("finger_joint_names", [])
                )
                and bool(calibration.get("finger_joint_names"))
                for side, calibration in self.grippers.items()
                if isinstance(calibration, dict)
            },
        }

    def close(self):
        self._quit.set()
        if self._thread:
            self._thread.join(timeout=3.0)
            if self._thread.is_alive():
                raise RuntimeError("ROS adapter thread failed to stop")

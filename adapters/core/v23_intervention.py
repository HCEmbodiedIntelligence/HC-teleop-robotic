"""Actual v23 reset acknowledgements and generation-tagged IK output.

ControllerNode uses a single-threaded executor: reset, target acceptance and IK
cannot interleave. A late computed result retains its old authority token and
is rejected by the middleware's final gate after a revoke.
"""
import json
import math
import time
import uuid

from middleware.core.control_ownership import ControlToken


class V23Intervention:
    def __init__(self, node, grippers=None):
        from geometry_msgs.msg import PoseArray
        from sensor_msgs.msg import JointState
        from std_msgs.msg import Header, String
        self.node = node
        self.instance = uuid.uuid4().hex
        self.active = False
        self.token = None
        self.phase = ''
        self.ready = False
        self.error = ''
        self.permit_stamp = 0
        self.cutoff = 0
        self.permit_time = self.feedback_time = self.target_time = -math.inf
        self.target_stamp = 0
        self.homing = False
        self.home_command = None
        self.gripper_limits = {cfg['sim_joint']: float(cfg['sim_closed_position'])
                               for side, cfg in (grippers or {}).items() if side in ('right', 'left')}
        self.auxiliary = None
        self.ack_pub = node.create_publisher(String, '/status/control/ack', 10)
        self.record_ee_target = node.create_publisher(PoseArray, '/internal/recording/control/solver/ee_targets', 10)
        self.record_joint_target = node.create_publisher(JointState, '/internal/recording/control/solver/joint_targets', 10)
        self.output = node.create_publisher(JointState, '/internal/control/joint_targets', 10)
        node.create_subscription(Header, '/control/motion/permit', self.permit, 1)
        node.create_subscription(PoseArray, '/internal/vr/owned_poses', self.pose, 1)
        node.create_subscription(JointState, '/internal/vr/owned_home', self.home, 1)
        node.create_subscription(JointState, '/internal/vr/aux_joint_targets', self.aux, 1)
        node.create_timer(.1, self.heartbeat)

    def fresh(self, stamp):
        return stamp > 0 and -.05 <= (self.node.get_clock().now().nanoseconds-stamp)*1e-9 <= .25

    def permit(self, message):
        if not message.frame_id.startswith('hc1:'):
            return
        try:
            key, phase, instance, _ = message.frame_id.split('#')
            token = ControlToken.from_key(key)
            stamp = message.stamp.sec*1_000_000_000 + message.stamp.nanosec
            if instance != self.instance or phase not in ('revoke','grant','off') or stamp <= self.permit_stamp or not self.fresh(stamp):
                return
            if phase == 'grant' and (token != self.token or self.phase not in ('revoke','grant') or not self.ready):
                return
            changed = (token, phase) != (self.token, self.phase)
            self.permit_stamp, self.permit_time = stamp, time.monotonic()
            if changed:
                self.token, self.phase, self.active = token, phase, phase != 'off'
                if phase == 'grant':
                    self.node.control_source = token.source
                    self.node.source_target_pending = False
                self.cutoff, self.ready, self.error = stamp, False, ''
                self.target_stamp, self.target_time = 0, -math.inf
                self.homing, self.home_command = False, None
                self.auxiliary = None
                if phase == 'off':
                    from std_msgs.msg import String
                    self.node.control_source = ''
                    self.node._control_source_callback(String(data='vr'))
                # Reset is acknowledged only after new measured feedback has
                # actually reset the integrator, never on a timer assumption.
                self.node.reset_pending = True
            self.heartbeat()
        except (ValueError, TypeError, AttributeError):
            return

    def feedback(self, message):
        stamp = message.header.stamp.sec*1_000_000_000 + message.header.stamp.nanosec
        if (not self.fresh(stamp) or len(message.name) != len(message.position)
                or len(set(message.name)) != len(message.name)
                or not set(self.node.controller.free_joint_names) <= set(message.name)
                or not all(math.isfinite(v) for v in message.position)):
            return
        self.feedback_time = time.monotonic()
        if self.token is not None and not self.ready and stamp > self.cutoff:
            try:
                self.node.controller.reset_to_feedback()
                self.node.reset_pending = False
                self.ready = True
            except (ValueError, RuntimeError) as exc:
                self.error = str(exc)
            self.heartbeat()

    def authorized(self):
        now = time.monotonic()
        return (self.active and self.ready and not self.error and self.phase == 'grant'
                and self.token.source == 'vr' and now-self.permit_time <= .3
                and now-self.feedback_time <= .25 and now-self.target_time <= .2)

    def home(self, message):
        try:
            key, frame = message.header.frame_id.split('#', 1)
            stamp = message.header.stamp.sec*1_000_000_000 + message.header.stamp.nanosec
            if (not self.active or not self.ready or self.phase != 'grant' or self.token.source != 'vr'
                    or key != self.token.key() or frame != 'home'
                    or stamp <= max(self.cutoff, self.target_stamp) or not self.fresh(stamp)
                    or time.monotonic()-self.permit_time > .3
                    or len(message.name) != len(message.position) or len(set(message.name)) != len(message.name)
                    or not set(self.node.controller.free_joint_names) <= set(message.name)
                    or not all(math.isfinite(v) for v in message.position)):
                return
            if not self.homing:
                self.node.controller.reset_to_feedback()
            self.homing, self.home_command = True, message
            self.auxiliary = None
            self.target_stamp, self.target_time = stamp, time.monotonic()
        except (ValueError, AttributeError, TypeError):
            return

    def aux(self, message):
        """Accept standard virtual-gripper targets from this manual generation."""
        try:
            key, frame = message.header.frame_id.split('#', 1)
            stamp = message.header.stamp.sec*1_000_000_000 + message.header.stamp.nanosec
            previous = (self.auxiliary.header.stamp.sec*1_000_000_000 + self.auxiliary.header.stamp.nanosec
                        if self.auxiliary is not None else self.cutoff)
            if (not self.active or not self.ready or self.phase != 'grant' or self.token.source != 'vr'
                    or key != self.token.key() or frame or self.homing
                    or stamp <= max(self.cutoff, previous) or not self.fresh(stamp)
                    or time.monotonic()-self.permit_time > .3
                    or not message.name or len(message.name) != len(message.position)
                    or len(set(message.name)) != len(message.name)
                    or any(n not in self.gripper_limits or not math.isfinite(v) or not 0 <= v <= self.gripper_limits[n]
                           for n, v in zip(message.name, message.position))):
                return
            self.auxiliary = message
        except (ValueError, AttributeError, TypeError):
            return

    def publish(self, message):
        from sensor_msgs.msg import JointState
        from std_msgs.msg import Header
        target = JointState(header=Header(stamp=message.header.stamp, frame_id=self.token.key()+'#'),
                            name=list(message.name), position=list(message.position))
        aux = self.auxiliary
        if aux is not None:
            stamp = aux.header.stamp.sec*1_000_000_000 + aux.header.stamp.nanosec
            if self.fresh(stamp):
                target.name.extend(aux.name)
                target.position.extend(aux.position)
        self.output.publish(target)

    def pose(self, message):
        try:
            key, frame = message.header.frame_id.split('#', 1)
            stamp = message.header.stamp.sec*1_000_000_000 + message.header.stamp.nanosec
            if (not self.active or not self.ready or self.phase != 'grant' or self.token.source != 'vr'
                    or key != self.token.key() or frame != 'generic_task_bases'
                    or stamp <= max(self.cutoff, self.target_stamp) or not self.fresh(stamp)
                    or time.monotonic()-self.permit_time > .3):
                return
            if self.node._ee_callback(message, owned=True):
                if self.homing:
                    self.node.controller.reset_to_feedback()
                self.homing, self.home_command = False, None
                self.target_stamp, self.target_time = stamp, time.monotonic()
                if self.record_ee_target.get_subscription_count():
                    # The MCAP header contains the coordinate frame without the ownership token.
                    from geometry_msgs.msg import PoseArray
                    from std_msgs.msg import Header
                    self.record_ee_target.publish(PoseArray(
                        header=Header(stamp=message.header.stamp, frame_id=frame), poses=message.poses))
        except (ValueError, AttributeError, TypeError):
            return

    def heartbeat(self):
        from std_msgs.msg import String
        if self.active and self.phase == 'grant' and time.monotonic()-self.permit_time > .3:
            self.error = 'v23 control permit expired'
        self.ack_pub.publish(String(data=json.dumps(dict(participant='solver', instance=self.instance,
            stamp_ns=self.node.get_clock().now().nanoseconds,
            token=self.token.key() if self.token and self.ready else '', phase=self.phase,
            success=not self.error, reason=self.error))))

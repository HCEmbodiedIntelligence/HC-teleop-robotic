"""Public ServoJ endpoints on the v23 controller's existing ROS executor."""
import json
import math
import time

from humanoid_motion_server.servo_j import ServoJServer
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState
from std_msgs.msg import String

from .robot_api import RobotApiProfile, SERVO_J_COMMANDED_STATE


class ProfileServoJ:
    def __init__(self, node, boundary, directory, feedback_node):
        self.boundary = boundary
        self.api = RobotApiProfile(directory)
        self.groups = self.api.servo_j_endpoints
        self.members = self.api.servo_j_members
        self.allowed = {g: frozenset(self.api.servo_j_joints[n]['public_name'] for n in names)
                        for g, names in self.members.items() if g not in self.api.binary_grips}
        self.server = ServoJServer(node, self.groups, self.command,
                                   callback_group=boundary.bridge._command_callback_group)
        self.commanded_pub = node.create_publisher(JointState, SERVO_J_COMMANDED_STATE, 10)
        self.commanded_joints = {n: self.api.servo_j_joints[n] for group, names in self.members.items()
                                 if group not in self.api.binary_grips for n in names}
        self.body_feedback_joints = {n: joint for n, joint in self.api.servo_j_joints.items()
                                     if n not in self.api.native_hand_names}
        self.grips_in_body_feedback = self.api.teleop['grippers'].get('feedback_in_joint_state', False)
        self.commanded_layouts = {}
        self.hand_state = None
        if self.api.config.get('hand_servo_j') and self.api.hand_state_endpoint != self.server.state_topic:
            self.hand_state = node.create_publisher(JointState, self.api.hand_state_endpoint, 1)
        self.layouts = {}
        self.command_layouts = {}
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.info = node.create_publisher(String, '/status/robot/info', qos)
        self.info.publish(String(data=json.dumps(self.describe(), allow_nan=False)))
        self.timer = feedback_node.create_timer(.1, self.health,
            callback_group=boundary.bridge._ownership_callback_group)

    def describe(self):
        return self.api.describe_servo_j(self.boundary.bridge.config)

    def health(self):
        bridge = self.boundary.bridge
        with bridge._lock:
            ready = bridge._ready_locked(time.monotonic())
            fresh = self.boundary.feedback_fresh() if ready else False
        self.server.update_status(ready, fresh)

    def command(self, group, message):
        boundary = self.boundary
        bridge = boundary.bridge
        with bridge._lock:
            token = None
            if bridge.ownership.active:
                token = bridge.ownership.token
                if token.source != 'motion' or not bridge.ownership.allows(token):
                    return
        names = message.name
        grip = self.api.binary_grips.get(group)
        if grip is not None:
            if (len(names) != 1 or names[0] != grip.name or len(message.position) != 1
                    or message.velocity or message.effort
                    or message.position[0] not in (0., 1.)):
                boundary.reject(f'{group} requires a named binary position')
                return
            message.position = grip.encode(message.position[0])
            message.name = grip.command_names
            bridge.forward_live_command('motion', message, boundary.publish_selected, token, transfer=True)
            return
        layout = self.command_layouts.get(group)
        if layout is None or layout[0] != names:
            if not self.allowed[group].issuperset(names):
                boundary.reject('ServoJ joint outside group '+group)
                return
            joints = [self.api.servo_j_public[n] for n in names]
            transforms = [(i, j['scale'], j['offset']) for i, j in enumerate(joints)
                          if j['scale'] != 1 or j['offset'] != 0]
            layout = (list(names), [j['model_name'] for j in joints], transforms)
            self.command_layouts[group] = layout
        if len(message.position) != len(names) or any(
                len(v) not in (0, len(names)) for v in (message.velocity, message.effort)):
            boundary.reject('invalid ServoJ array lengths')
            return
        # rclpy gives this subscription its own message. Move it into the actor
        # queue, retaining stamp, order and all three numeric buffers.
        message.name = layout[1]
        for i, scale, offset in layout[2]:
            message.position[i] = message.position[i]*scale + offset
            if message.velocity:
                message.velocity[i] *= scale
            if message.effort:
                message.effort[i] /= scale
        bridge.forward_live_command('motion', message, boundary.publish_selected, token,
                                    transfer=True)

    def commanded(self, message):
        """Report the native command after publication, without changing its original buffers."""
        if self.commanded_pub.get_subscription_count() == 0:
            return
        names = message.name
        # Keep one layout per leading joint so alternating arms/hands reuse their maps.
        layout = self.commanded_layouts.get(names[0])
        if layout is None or names != layout[0]:
            mapped = [(i, self.commanded_joints[n]) for i, n in enumerate(names) if n in self.commanded_joints]
            public = [j['public_name'] for _, j in mapped]
            direct = len(mapped) == len(names) and all(j['scale'] == 1 and j['offset'] == 0 for _, j in mapped)
            # X1 also has continuous simulation markers on its body command.
            # An independent hand command carries physical hand axes, never just that marker.
            grips = (() if mapped else tuple(grip for grip in self.api.binary_grips.values()
                                              if any(n in names for n in grip.native_names)))
            layout = (list(names), public, mapped, direct, grips)
            self.commanded_layouts[names[0]] = layout
        _, public, mapped, direct, grips = layout
        positions = message.position
        if grips:
            self._publish_grips(message, self.commanded_pub, grips, use_marker=True)
            return
        target = JointState(header=message.header)
        if direct and all(map(math.isfinite, positions)):
            target.name = public
            target.position = positions
        else:
            for i, joint in mapped:
                q = positions[i]
                if math.isfinite(q):
                    target.name.append(joint['public_name'])
                    target.position.append((q-joint['offset'])/joint['scale'])
            if not target.name:
                return
        self.commanded_pub.publish(target)

    @staticmethod
    def _publish_grips(message, publisher, grips, *, use_marker=False):
        target = None
        for grip in grips:
            bit = grip.decode(message.name, message.position, use_marker=use_marker)
            if bit is not None:
                if target is None:
                    target = JointState(header=message.header)
                target.name.append(grip.name)
                target.position.append(bit)
        if target is not None:
            publisher.publish(target)

    def feedback(self, message, stream):
        if stream == "hands" or self.grips_in_body_feedback:
            if self.hand_state is not None and self.hand_state.get_subscription_count():
                self._publish_grips(message, self.hand_state, self.api.binary_grips.values())
            if stream == "hands":
                return
        publisher = self.server.state
        if publisher.get_subscription_count() == 0:
            return
        names = message.name
        layout = self.layouts.get(stream)
        if layout is None or layout[0] != names:
            mapped = [(i, self.body_feedback_joints[n]) for i, n in enumerate(names) if n in self.body_feedback_joints]
            public = [j['public_name'] for _, j in mapped]
            identity = len(mapped) == len(names) and all(j['scale'] == 1 and j['offset'] == 0
                                                        for _, j in mapped)
            layout = (list(names), public, mapped, identity)
            self.layouts[stream] = layout
        _, public, mapped, identity = layout
        if not public:
            return
        result = JointState(header=message.header, name=public)
        if identity:
            result.position = message.position
            result.velocity = message.velocity
            result.effort = message.effort
        else:
            result.position = [(message.position[i]-j['offset'])/j['scale'] for i, j in mapped]
            if len(message.velocity) == len(names):
                result.velocity = [message.velocity[i]/j['scale'] for i, j in mapped]
            if len(message.effort) == len(names):
                result.effort = [message.effort[i]*j['scale'] for i, j in mapped]
        publisher.publish(result)

"""Public ServoJ endpoints on the v23 controller's existing ROS executor."""
import json
import time

from humanoid_motion_server.servo_j import ServoJServer
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState
from std_msgs.msg import String

from .robot_api import RobotApiProfile


class ProfileServoJ:
    def __init__(self, node, boundary, directory, feedback_node):
        self.boundary = boundary
        self.api = RobotApiProfile(directory)
        self.groups = self.api.servo_j_endpoints
        self.members = self.api.servo_j_members
        self.allowed = {g: frozenset(self.api.servo_j_joints[n]['public_name'] for n in names)
                        for g, names in self.members.items()}
        self.server = ServoJServer(node, self.groups, self.command,
                                   callback_group=boundary.bridge._command_callback_group)
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
        return self.api.describe_servo_j()

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

    def feedback(self, message, stream):
        publisher = self.hand_state if stream == "hands" else self.server.state
        if publisher is None or publisher.get_subscription_count() == 0:
            return
        names = message.name
        layout = self.layouts.get(stream)
        if layout is None or layout[0] != names:
            mapped = [(i, self.api.servo_j_joints[n]) for i, n in enumerate(names) if n in self.api.servo_j_joints]
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

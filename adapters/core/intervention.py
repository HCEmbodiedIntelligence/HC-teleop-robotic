"""Existing VR adapter's semantic clutch and relative intervention reference.

All wire types are existing ROS standard types. No device button is interpreted
by the ownership manager, and no untagged pose can become an intervention target.
"""
import json
import math
import time
import uuid

from middleware.core.control_ownership import ControlToken


class ClutchInput:
    def __init__(self, control):
        self.side = control.get('takeover_clutch_side', 'right')
        self.axis = ('trigger', 'grip')[int(control.get('clutch_axis', 1))]
        self.threshold = float(control.get('clutch_threshold', .55))
        self.release_threshold = float(control.get('clutch_release_threshold', self.threshold-.05))
        self.pressed = False
        self.sequence = -1
        self.source_time = -math.inf

    def sample(self, value):
        sequence = int(value['sequence'])
        source_time = float(value['vr_timestamp'])
        if sequence == self.sequence or source_time <= self.source_time:
            return None
        self.sequence, self.source_time = sequence, source_time
        try:
            axis = value['inputs'][self.side][self.axis]
            valid = (value.get('protocol_version', 0) >= 2 and
                     type(axis) in (float, int) and math.isfinite(axis) and 0 <= axis <= 1)
            for side in ('left', 'right'):
                pose = value['poses'][side]
                xyz, quat = pose['position'], pose['quaternion']
                valid &= (value['tracking'][side] is True and len(xyz) == 3 and len(quat) == 4 and
                          all(math.isfinite(float(v)) for v in xyz+quat) and sum(v*v for v in quat) > .1)
            if valid:
                self.pressed = axis >= (self.release_threshold if self.pressed else self.threshold)
            return bool(valid), self.pressed
        except (KeyError, TypeError, ValueError):
            return False, self.pressed


class InterventionTeleop:
    def __init__(self, node):
        from std_msgs.msg import Header, String
        from sensor_msgs.msg import JointState
        from geometry_msgs.msg import PoseArray
        self.node = node
        self.instance = uuid.uuid4().hex
        self.input_instance = uuid.uuid4().hex
        self.input_sequence = 0
        self.input = ClutchInput(node.control)
        self.active, self.granted, self.valid = False, False, False
        self.token = None
        self.phase = ''
        self.ack_error = ''
        self.permit_stamp = 0
        self.permit_time = -math.inf
        self.last_input = -math.inf
        self.grippers = {}
        self.triggers = {}
        self.ack_pub = node.create_publisher(String, '/status/control/ack', 100)
        self.input_pub = node.create_publisher(String, '/events/teleop/clutch', 100)
        self.pose_pub = node.create_publisher(PoseArray, '/internal/vr/owned_poses', 1)
        self.aux_pub = node.create_publisher(JointState, '/internal/vr/aux_joint_targets', 1)
        node.create_subscription(Header, '/control/motion/permit', self.permit, 1)
        node.create_timer(.1, self.heartbeat)

    def observe(self, payload):
        from std_msgs.msg import String
        try:
            value = json.loads(payload)
            if time.monotonic()-self.last_input > .3 and float(value['vr_timestamp']) < self.input.source_time:
                self.input = ClutchInput(self.node.control)
                self.input_instance = uuid.uuid4().hex
                self.granted = False
            sample = self.input.sample(value)
            if sample is None:
                return
            stamp = int(value['hc_received_ns'])
            age = self.node.get_clock().now().nanoseconds-stamp
            self.valid = sample[0] and -50_000_000 <= age <= 300_000_000
            self.last_input = time.monotonic()
            self.input_sequence += 1
            self.triggers = {side: fields['trigger'] for side, fields in value['inputs'].items()
                if type(fields.get('trigger')) in (int, float) and math.isfinite(fields['trigger']) and 0 <= fields['trigger'] <= 1}
            self.input_pub.publish(String(data=json.dumps(dict(
                device_instance=self.input_instance, sequence=self.input_sequence, source_sequence=value['sequence'], stamp_ns=stamp,
                valid=self.valid, gripped=sample[1]))))
        except (KeyError, ValueError, TypeError):
            self.valid = False

    def heartbeat(self, success=True, reason=''):
        from std_msgs.msg import String
        if not success:
            self.ack_error = reason
        if self.active and time.monotonic()-self.permit_time > .3:
            self.granted = False
            self.ack_error = 'control permit timeout'
        success, reason = not self.ack_error, self.ack_error
        self.ack_pub.publish(String(data=json.dumps(dict(participant='teleop', instance=self.instance,
            stamp_ns=self.node.get_clock().now().nanoseconds,
            token=self.token.key() if self.token else '', phase=self.phase, success=success, reason=reason))))

    def permit(self, message):
        if not message.frame_id.startswith('hc1:'):
            return
        try:
            key, phase, _, peer = message.frame_id.split('#')
            token = ControlToken.from_key(key)
            stamp = message.stamp.sec*1_000_000_000+message.stamp.nanosec
            if peer != self.instance or phase not in {'revoke', 'grant', 'off'} or stamp <= self.permit_stamp:
                return
            if not -50_000_000 <= self.node.get_clock().now().nanoseconds-stamp <= 300_000_000:
                return
            if phase == 'grant' and (self.token != token or self.phase == 'off'):
                return
            changed = (token, phase) != (self.token, self.phase)
            self.permit_stamp, self.permit_time = stamp, time.monotonic()
            self.active = phase != 'off'
            if not changed:
                self.heartbeat()
                return
            self.granted = False
            self.node._release_all(send_base_zero=True)
            self.node.generic_aux_command.clear()
            self.node.homing = False
            self.token, self.phase = token, phase
            self.ack_error = ''
            self.grippers = {}
            if phase == 'grant' and token.source == 'vr':
                self.anchor()
                self.granted = True
            self.heartbeat()
        except (ValueError, KeyError, TypeError) as exc:
            self.heartbeat(False, str(exc))

    def anchor(self):
        node, now = self.node, time.monotonic()
        if (not self.valid or not self.input.pressed or now-self.last_input > .3 or
                now-node.joint_state_stamp > min(.2, node.control['joint_state_timeout'])):
            raise ValueError('fresh tracked controllers and measured feedback required')
        node._sync_model()
        for arm in node.arms.values():
            if arm.pose is None or now-arm.pose_stamp > min(.2, node.control['pose_timeout']):
                raise ValueError('arm tracking is stale')
            node._engage_arm(arm)
            if not arm.active:
                raise ValueError('relative arm reference unavailable')
        # These actuators have measured feedback and a checked final routing
        # path. A hand without that capability stays held; no guessed position.
        if node.robot_api.driver['kind'] == 'openarmx_hc_robot_adapter':
            for side, cfg in node.config['grippers'].items():
                if side not in ('left', 'right'):
                    continue
                for index, name in enumerate(cfg['finger_joint_names']):
                    if name in node.joint_state and side in self.triggers:
                        q = node.joint_state[name]
                        self.grippers[name] = [side, self.triggers[side], q, q,
                            cfg['finger_open'][index], cfg['finger_closed'][index]]

    def tick(self, now, feedback_fresh):
        """Dedicated intervention branch reuses arm mapping, holds body/base."""
        from sensor_msgs.msg import JointState
        node = self.node
        if (not self.granted or not self.valid or not self.input.pressed or not feedback_fresh or
                now-self.last_input > .3 or now-self.permit_time > .3):
            node._publish_status(now, feedback_fresh)
            return
        if any(not arm.active or arm.reference_vr is None or arm.reference_local_ee is None
               for arm in node.arms.values()):
            self.granted = False
            self.heartbeat(False, 'manual reference was revoked')
            return
        for arm in node.arms.values():
            node._update_arm_target(arm)
        node._publish_target_poses()
        values = {}
        for name, reference in self.grippers.items():
            side, trigger, initial, previous, opened, closed = reference
            if side not in self.triggers:
                continue
            desired = initial+(self.triggers[side]-trigger)*(closed-opened)
            desired = max(min(opened, closed), min(max(opened, closed), desired))
            step = float(node.config['grippers']['max_velocity'])/float(node.control['rate_hz'])
            reference[3] = max(previous-step, min(previous+step, desired))
            values[name] = reference[3]
        if values:
            msg = JointState(name=list(values), position=list(values.values()))
            msg.header.stamp = node.get_clock().now().to_msg()
            msg.header.frame_id = self.token.key()+'#'
            self.aux_pub.publish(msg)
        node._publish_status(now, feedback_fresh)

    def poses(self, message):
        if self.granted:
            message.header.frame_id = self.token.key()+'#'+message.header.frame_id
            self.pose_pub.publish(message)

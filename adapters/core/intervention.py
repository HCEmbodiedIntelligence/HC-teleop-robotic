"""Existing VR adapter's semantic clutch and relative intervention reference.

All wire types are existing ROS standard types. No device button is interpreted
by the ownership manager, and no untagged pose can become an intervention target.
"""
import json
import math
import time
import uuid

from middleware.core.control_ownership import ControlToken
from middleware.core.protocol import is_sequence_newer
from .arm_teleop_math import binary_trigger_state


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
        sequence, source_time = value['sequence'], value['vr_timestamp']
        if (type(sequence) is not int or not 0 <= sequence <= 0xffffffff
                or type(source_time) not in (int, float) or not math.isfinite(source_time)):
            raise ValueError('invalid VR input sequence or timestamp')
        if ((self.sequence >= 0 and not is_sequence_newer(sequence, self.sequence))
                or source_time <= self.source_time):
            return None
        self.sequence, self.source_time = sequence, source_time
        try:
            axis = value['inputs'][self.side][self.axis]
            # The gateway reports head loss only once; every later manual
            # sample must remain invalid until head tracking recovers.
            valid = (value.get('protocol_version', 0) >= 2 and
                     value['tracking']['head'] is True and
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
        self.cutoff = 0
        self.permit_time = -math.inf
        self.last_input = -math.inf
        self._pause_held = self._resume_held = self._resume_armed = False
        self.grippers = {}
        self.triggers = {}
        self.ack_pub = node.create_publisher(String, '/status/control/ack', 100)
        self.input_pub = node.create_publisher(String, '/events/teleop/clutch', 100)
        if node.backend == 'v23':
            self.record_action_pose = node.create_publisher(PoseArray, '/internal/recording/action/ee_poses', 10)
        self.pose_pub = node.create_publisher(PoseArray, '/internal/vr/owned_poses', 1)
        self.aux_pub = node.create_publisher(JointState, '/internal/vr/aux_joint_targets', 1)
        self.owned_pub = node.create_publisher(JointState, '/internal/control/joint_targets', 10)
        node.create_subscription(Header, '/control/motion/permit', self.permit, 1)
        self.home_pub = node.create_publisher(JointState, '/internal/vr/owned_home', 1)
        node.create_subscription(Header, '/internal/vr/home_request', self.home_request, 1)
        node.create_timer(.1, self.heartbeat)
        self.hand_feedback = {}
        self.hand_commands = {}
        self.last_hand_step = 0.0
        if node.backend == 'v23':
            from rclpy.qos import qos_profile_sensor_data
            node.create_subscription(JointState, '/observation/hands/joint_states', self.hands, qos_profile_sensor_data)
            for side in ('right', 'left'):
                topic = node.config['grippers'][side]['finger_topic']
                node.create_subscription(JointState, topic,
                    lambda msg, side=side: self.hand_command(msg, side), qos_profile_sensor_data)

    def hand_command(self, message, side):
        """Remember the last final setpoint for a bounded X1 hand handover."""
        stamp = message.header.stamp.sec*1_000_000_000 + message.header.stamp.nanosec
        if (len(message.name) != len(message.position) or len(set(message.name)) != len(message.name)
                or not all(math.isfinite(v) for v in message.position)
                or not -.05 <= (self.node.get_clock().now().nanoseconds-stamp)*1e-9 <= .25):
            return
        for name, value in zip(message.name, message.position):
            if (name in self.node.config['grippers'][side]['finger_joint_names'] and
                    stamp > self.hand_commands.get(name, (0, 0))[1]):
                self.hand_commands[name] = (value, stamp)

    def hands(self, message):
        stamp = message.header.stamp.sec*1_000_000_000 + message.header.stamp.nanosec
        if (len(message.name) != len(message.position) or len(set(message.name)) != len(message.name)
                or not all(math.isfinite(v) for v in message.position)
                or not -.05 <= (self.node.get_clock().now().nanoseconds-stamp)*1e-9 <= .25):
            return
        self.hand_feedback.update({name: (q, time.monotonic()) for name, q in zip(message.name, message.position)})

    def observe(self, payload):
        from std_msgs.msg import String
        try:
            value = json.loads(payload)
            stamp = value['hc_received_ns']
            if type(stamp) is not int or stamp <= 0:
                raise ValueError('invalid HC reception timestamp')
            if time.monotonic()-self.last_input > .3 and float(value['vr_timestamp']) < self.input.source_time:
                self.input = ClutchInput(self.node.control)
                self.input_instance = uuid.uuid4().hex
                self.granted = False
                self._pause_held = self._resume_held = self._resume_armed = False
            sample = self.input.sample(value)
            if sample is None:
                return
            age = self.node.get_clock().now().nanoseconds-stamp
            self.valid = sample[0] and -50_000_000 <= age <= 300_000_000
            buttons = value['inputs']['right']
            held, pressed = buttons.get('held_mask', 0), buttons.get('pressed_mask', 0)
            valid_buttons = (type(held) is int and type(pressed) is int
                             and 0 <= held <= 0x7ff and 0 <= pressed <= 0x7ff)
            pause = resume = False
            if self.valid and valid_buttons:
                # A must be released before arming; B may stop on first input.
                # Treat repeated pressed bits as held until a release sample.
                a, b = bool((held | pressed) & 1), bool((held | pressed) & 2)
                pause = b and not self._pause_held
                resume = a and not self._resume_held and self._resume_armed and not b
                if not a:
                    self._resume_armed = True
                self._pause_held, self._resume_held = b, a
            else:
                self._resume_armed = False
            self.last_input = time.monotonic()
            self.input_sequence += 1
            self.triggers = {side: fields['trigger'] for side, fields in value['inputs'].items()
                if type(fields.get('trigger')) in (int, float) and math.isfinite(fields['trigger']) and 0 <= fields['trigger'] <= 1}
            self.input_pub.publish(String(data=json.dumps(dict(
                device_instance=self.input_instance, sequence=self.input_sequence, source_sequence=value['sequence'], stamp_ns=stamp,
                valid=self.valid, gripped=sample[1],
                pause_requested=pause, resume_requested=resume))))
        except (KeyError, ValueError, TypeError):
            self.valid = False
            self._resume_armed = False

    def heartbeat(self, success=True, reason=''):
        from std_msgs.msg import String
        if not success:
            self.ack_error = reason
        # Only an open grant is a lease. During revoke/HOLD the output is
        # already closed; starting capture workers must not expire a permission
        # that was never granted. A later grant still needs a fresh token/stamp.
        if self.active and self.phase == 'grant' and time.monotonic()-self.permit_time > .3:
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
            self.cutoff = stamp
            self.granted = False
            self.node._release_all(send_base_zero=True)
            self.node.generic_aux_command.clear()
            self.node.homing = False
            self.node.body.target_base = None
            self.token, self.phase = token, phase
            if phase == 'grant':
                self.node.control_source = token.source
                self.node.source_feedback_pending = False
            elif phase == 'off':
                from std_msgs.msg import String
                self.node.control_source = ''
                self.node._control_source_callback(String(data='vr'))
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
        self.last_hand_step = now
        if (not self.valid or not self.input.pressed or now-self.last_input > .3 or
                now-node.joint_state_stamp > min(.2, node.control['joint_state_timeout'])):
            raise ValueError('fresh tracked controllers and measured feedback required')
        node._sync_model()
        if node.backend == 'v23':
            for name, (q, stamp) in self.hand_feedback.items():
                if now-stamp <= .25:
                    node.joint_state[name] = q
        for arm in node.arms.values():
            if arm.pose is None or now-arm.pose_stamp > min(.2, node.control['pose_timeout']):
                raise ValueError('arm tracking is stale')
            node._engage_arm(arm)
            if not arm.active:
                raise ValueError('relative arm reference unavailable')
        # These actuators have measured feedback and a checked final routing
        # path. A hand without that capability stays held; no guessed position.
        if node.backend == 'v23' or node.robot_api.driver['kind'] == 'openarmx_hc_robot_adapter':
            for side, cfg in node.config['grippers'].items():
                if side not in ('left', 'right'):
                    continue
                for index, name in enumerate(cfg['finger_joint_names']):
                    if name not in node.joint_state or side not in self.triggers:
                        continue
                    q = node.joint_state[name]
                    opened, closed = cfg['finger_open'][index], cfg['finger_closed'][index]
                    previous = q
                    if node.backend == 'v23':
                        measured = self.hand_feedback.get(name)
                        if not measured or now-measured[1] > .25:
                            continue
                        q = previous = measured[0]
                        # A measured finger can lie outside the calibrated command
                        # interval. Prefer a fresh, legal final command as slew seed
                        # in that case. All generated commands still pass HC checks.
                        lo, hi = sorted((opened, closed))
                        sent = self.hand_commands.get(name)
                        if not lo <= q <= hi and sent and lo <= sent[0] <= hi:
                            age = (node.get_clock().now().nanoseconds-sent[1])*1e-9
                            if -.05 <= age <= .25:
                                previous = sent[0]
                    self.grippers[name] = [side, self.triggers[side], q, previous, opened, closed]

    def can_home(self):
        now = time.monotonic()
        return (self.node.backend == 'v23' and self.active and self.granted
                and self.phase == 'grant' and self.token.source == 'vr'
                and self.valid and self.input.pressed and now-self.last_input <= .3
                and now-self.permit_time <= .3 and self.node.hardware_ready)

    def home_request(self, message):
        stamp = message.stamp.sec*1_000_000_000 + message.stamp.nanosec
        if (not self.can_home() or message.frame_id != self.token.key()
                or stamp <= self.cutoff
                or not -50_000_000 <= self.node.get_clock().now().nanoseconds-stamp <= 250_000_000):
            return
        if self.node._home_allowed():
            self.node._set_generic_home_targets()

    def tick(self, now, feedback_fresh):
        """Dedicated intervention branch reuses arm mapping, holds body/base."""
        from sensor_msgs.msg import JointState
        node = self.node
        if (not self.granted or not self.valid or not self.input.pressed or not feedback_fresh or
                now-self.last_input > .3 or now-self.permit_time > .3):
            node._publish_status(now, feedback_fresh)
            return
        if node.homing and node.backend == 'v23':
            command = dict(node.joint_state)
            node._update_homing(command)
            if not node._collision_command_allowed(command, now):
                node._publish_status(now, feedback_fresh)
                return
            names = node._homing_joint_names()
            msg = JointState(name=names, position=[float(command[n]) for n in names])
            msg.header.stamp = node.get_clock().now().to_msg()
            msg.header.frame_id = self.token.key()+'#home'
            self.home_pub.publish(msg)
            node.last_command.update(command)
            if not node.homing:
                from .solver_rearm import SolverRearmGate
                node.solver_rearm = SolverRearmGate()
                self.anchor()
            node._publish_status(now, feedback_fresh)
            return
        if any(not arm.active or arm.reference_vr is None or arm.reference_local_ee is None
               for arm in node.arms.values()):
            self.granted = False
            self.heartbeat(False, 'manual reference was revoked')
            return
        for arm in node.arms.values():
            node._update_arm_target(arm)
        if node.backend != 'v23':
            node._publish_target_poses()
        values = {}
        if node.backend == 'v23':
            # Use elapsed time: dropped control ticks must not slow a fresh
            # trigger indefinitely or allow a large step after a stall.
            dt = max(0., min(.05, now-self.last_hand_step))
            self.last_hand_step = now
        else:
            dt = 1./float(node.control['rate_hz'])
        step = float(node.config['grippers']['max_velocity'])*dt
        for name, reference in self.grippers.items():
            side, trigger, initial, previous, opened, closed = reference
            if side not in self.triggers:
                continue
            # X1 uses the same absolute trigger mapping as ordinary VR teleop.
            # Preserve the relative mapping on other backends.
            if node.backend == 'v23':
                desired = opened+self.triggers[side]*(closed-opened)
            else:
                desired = initial+(self.triggers[side]-trigger)*(closed-opened)
            desired = max(min(opened, closed), min(max(opened, closed), desired))
            reference[3] = max(previous-step, min(previous+step, desired))
            values[name] = reference[3]
        if values:
            if node.backend == 'v23':
                auxiliary = {}
                for side in ('right', 'left'):
                    cfg = node.config['grippers'][side]
                    if side not in self.triggers or not set(values).intersection(cfg['finger_joint_names']):
                        continue
                    trigger = self.triggers[side]
                    if cfg.get('binary_state_joint'):
                        values[cfg['binary_state_joint']] = binary_trigger_state(
                            trigger, node.config['grippers'].get('binary_trigger_threshold', .5))
                    if node.config['grippers'].get('include_sim_joints', True):
                        auxiliary[cfg['sim_joint']] = node._gripper_position(side, trigger)
                if auxiliary:
                    aux = JointState(name=list(auxiliary), position=list(auxiliary.values()))
                    aux.header.stamp = node.get_clock().now().to_msg()
                    aux.header.frame_id = self.token.key()+'#'
                    self.aux_pub.publish(aux)
                    node.last_command.update(auxiliary)
            msg = JointState(name=list(values), position=list(values.values()))
            msg.header.stamp = node.get_clock().now().to_msg()
            msg.header.frame_id = self.token.key()+'#'
            if node.backend == 'v23':
                self.owned_pub.publish(msg)
            else:
                self.aux_pub.publish(msg)
        if node.backend == 'v23':
            node._publish_target_poses()
        node._publish_status(now, feedback_fresh)

    def poses(self, message, *, action_pose=None):
        if self.granted:
            message.header.frame_id = self.token.key()+'#'+message.header.frame_id
            self.pose_pub.publish(message)
            if action_pose is not None and self.record_action_pose.get_subscription_count():
                self.record_action_pose.publish(action_pose)

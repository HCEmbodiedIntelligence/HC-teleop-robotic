"""X1 model-name topics selected by the existing HC ownership gate.

The own gateway remains the only publisher on /hc_teleop command inputs.
Selected targets use the existing /action hardware adapter; no Motion API is
called here. Feedback and safety checks also apply when recording is disabled.
"""
import math
from collections import Counter
from pathlib import Path
import time
from types import SimpleNamespace
from xml.etree import ElementTree

import yaml


class LegacyTopicControl:
    def __init__(self, node, bridge, output, *, feedback_node=None, hand_node=None):
        from sensor_msgs.msg import JointState
        from rclpy.qos import qos_profile_sensor_data
        from adapters.core.collision_safety import ModelCollisionGuard

        self.node, self.bridge, self.output = node, bridge, output
        profile = Path(bridge.config['legacy_control_profile'])
        self.teleop = yaml.safe_load((profile/'arm_teleop.yaml').read_text())
        self.settings = bridge.config.get('intervention', {})
        self.max_age = .25
        self.arm_velocity = float(self.settings.get('max_arm_velocity_rad_s', .25))
        self.max_error = float(self.settings.get('max_target_error_rad', .30))
        if not all(math.isfinite(x) and x > 0 for x in (self.arm_velocity, self.max_error)):
            raise ValueError('legacy velocity and target error must be finite and positive')
        self.arms = tuple(n for side in ('right', 'left') for n in self.teleop['arms'][side]['joint_names'])
        self.waist = tuple(self.teleop['body']['waist_joint_names'])
        self.feedback_order = self.arms + self.waist
        self.home_joints = self.feedback_order + tuple(self.teleop['body'].get('neck_joint_names', []))
        self.hands = {side: tuple(self.teleop['grippers'][side]['finger_joint_names']) for side in ('right','left')}
        self.hand_names = set().union(*map(set, self.hands.values()))
        self.sim_joints = {self.teleop['grippers'][side]['sim_joint'] for side in self.hands
                           if self.teleop['grippers'][side].get('sim_joint')}
        # The dataset layout includes configured model joints even when their
        # controller is disabled. Missing commands are NaN, never feedback.
        self.arm_fields = (self.arms + tuple(n for n in self.teleop['robot']['initial_joints']
                                            if n not in self.arms)
                           + tuple(self.teleop['grippers'][s]['sim_joint'] for s in ('right', 'left')))
        self.model_only_joints = set(self.arm_fields) - set(self.home_joints) - self.sim_joints
        self.positions, self.sample_ns, self.received, self.speeds = {}, {}, {}, {}
        self.last_command_stamp, self.last_command, self.command_time = {}, {}, {}
        self.generation = None
        self.last_rejection = ''
        self.recent_rejection = ''
        self.diagnostics = Counter()
        self.last_rejection_ns = 0
        self.last_clip = {}
        self.last_arm_publish_time = None
        self.arm_publish_gap_max_ms = 0.0
        urdf = (profile/self.teleop['robot']['urdf_path']).resolve()
        self.limits = {}
        for joint in ElementTree.parse(urdf).getroot().findall('joint'):
            if joint.attrib['type'] == 'fixed':
                continue
            limit = joint.find('limit')
            self.limits[joint.attrib['name']] = ((-math.inf, math.inf) if joint.attrib['type'] == 'continuous'
                else (float(limit.attrib['lower']), float(limit.attrib['upper'])))
        for side, names in self.hands.items():
            cfg = self.teleop['grippers'][side]
            for name, opened, closed in zip(names, cfg['finger_open'], cfg['finger_closed']):
                self.limits[name] = (min(opened, closed), max(opened, closed))
        self.collision = None
        node.get_logger().info('[control-limits] '+str(dict(
            arm_velocity_rad_s=self.arm_velocity, target_error_rad=self.max_error,
            feedback_command_max_age_s=self.max_age,
            arm_position_limits={n:self.limits[n] for n in self.arms},
            hand_velocity_rad_s=self.teleop['grippers']['max_velocity'])))
        if self.teleop['control'].get('collision_avoidance_enabled', True):
            self.collision = ModelCollisionGuard(SimpleNamespace(urdf_path=urdf, teleop=self.teleop,
                groups={'arms': self.arms, 'waist': self.waist}))
        hand_receiver = hand_node if hand_node is not None else node
        hand_group = (bridge._hand_command_callback_group if hand_node is not None
                      else bridge._command_callback_group)
        self.hand_pubs = {s: hand_receiver.create_publisher(JointState, self.teleop['grippers'][s]['finger_topic'], 10)
                          for s in self.hands}
        self.subscriptions = []
        def subscribe(kind, topic, callback, qos=qos_profile_sensor_data, *, hand=False):
            receiver = hand_receiver if hand else node
            self.subscriptions.append(receiver.create_subscription(kind, topic, callback, qos,
                callback_group=hand_group if hand else bridge._command_callback_group))
        # Feedback and authority must keep draining while command preparation
        # runs. Their existing bridge lock still serializes state and final send.
        receiver = feedback_node or node
        feedback_group = (bridge._ownership_callback_group if feedback_node is not None
                          else bridge._command_callback_group)
        for topic, callback in [('/observation/joint_states', self.state),
                                ('/observation/hands/joint_states', self.hand_state)]:
            self.subscriptions.append(receiver.create_subscription(
                JointState, topic, callback, qos_profile_sensor_data,
                callback_group=feedback_group))
        subscribe(JointState, '/hc_teleop/joint_cmd', lambda msg: self.program(msg, 'arms'))
        for side in self.hands:
            subscribe(JointState, '/hc_teleop/joint_cmd_finger_'+side,
                      lambda msg, side=side: self.program(msg, side), hand=True)
            subscribe(JointState, f'/control/vr/{side}_hand/joint_states',
                      lambda msg, side=side: self.bridge.forward_live_command(
                          'vr', self.select(msg, self.hand_fields(side)), self.publish_selected), hand=True)

    def hand_fields(self, side):
        marker = self.teleop['grippers'][side].get('binary_state_joint')
        return self.hands[side] + ((marker,) if marker else ())

    @staticmethod
    def stamp(message):
        return message.header.stamp.sec*1_000_000_000 + message.header.stamp.nanosec

    @staticmethod
    def values(message, *, allow_missing=False):
        if (not message.name or len(message.name) != len(message.position)
                or len(set(message.name)) != len(message.name)
                or not all(isinstance(n, str) and n for n in message.name)
                or not all(math.isfinite(v) or (allow_missing and math.isnan(v)) for v in message.position)):
            raise ValueError('invalid joint names or positions')
        return dict(zip(message.name, message.position))

    def fresh(self, names, *, now=None, wall_ns=None):
        # Empty hand-only feedback requirements need no ROS clock lookup.
        if not names:
            return True
        now = time.monotonic() if now is None else now
        wall = self.node.get_clock().now().nanoseconds if wall_ns is None else wall_ns
        return all(n in self.positions and now-self.received[n] <= self.max_age
                   and -.05 <= (wall-self.sample_ns[n])*1e-9 <= self.max_age for n in names)

    def feedback_fresh(self):
        return self.fresh(self.feedback_order)

    def handover_ready(self):
        return self.feedback_fresh() and all(n in self.speeds and abs(self.speeds[n]) <= .02 for n in self.arms)

    def update_state(self, message, allowed):
        values = self.values(message)
        stamp, now = self.stamp(message), time.monotonic()
        if not stamp or not -.05 <= (self.node.get_clock().now().nanoseconds-stamp)*1e-9 <= self.max_age:
            raise ValueError('stale measured feedback')
        measured_velocity = dict(zip(message.name, message.velocity)) if len(message.velocity) == len(message.name) else {}
        for name, value in values.items():
            if name not in allowed or stamp <= self.sample_ns.get(name, 0):
                continue
            dt = (stamp-self.sample_ns.get(name, 0))*1e-9
            speed = measured_velocity.get(name)
            if speed is None and name in self.positions and .001 <= dt <= self.max_age:
                speed = (value-self.positions[name])/dt
            if speed is not None and math.isfinite(speed):
                self.speeds[name] = speed
            else:
                self.speeds.pop(name, None)
            self.positions[name], self.sample_ns[name], self.received[name] = value, stamp, now
        return values

    def state(self, message):
        try:
            with self.bridge._lock:
                self.update_state(message, self.limits.keys()-self.hand_names)
        except (ValueError, TypeError):
            return

    def hand_state(self, message):
        try:
            with self.bridge._lock:
                self.update_state(message, self.hand_names)
        except (ValueError, TypeError):
            return

    @staticmethod
    def target_message(message, names, positions):
        # Copy only JointState fields, without recursively reconstructing the
        # entire generated ROS object on every projection and preparation.
        target = type(message)(name=list(names), position=list(positions))
        target.header.stamp.sec = message.header.stamp.sec
        target.header.stamp.nanosec = message.header.stamp.nanosec
        target.header.frame_id = message.header.frame_id
        return target

    @staticmethod
    def select(message, names):
        indices = {name:index for index,name in enumerate(message.name)}
        selected = [n for n in names if n in indices]
        target = LegacyTopicControl.target_message(
            message, selected, [message.position[indices[n]] for n in selected])
        for field in ('velocity', 'effort'):
            source = getattr(message, field)
            if len(source) == len(message.name):
                setattr(target, field, [source[indices[n]] for n in selected])
        return target

    def program(self, message, channel):
        try:
            values = self.values(message, allow_missing=set(message.name) == set(self.arm_fields))
            names = set(values)
            fields = self.arm_fields if channel == 'arms' else self.hand_fields(channel)
            allowed = set(fields)
            if not names <= allowed:
                raise ValueError('unsupported own command joints')
            selected = self.select(message, [n for n in fields if n in values and not math.isnan(values[n])])
            with self.bridge._lock:
                self.diagnostics['program_received_'+channel] += 1
                token = self.bridge.ownership.token if self.bridge.ownership.active else None
                if not self.bridge.ownership.active or self.bridge.ownership.state != 'AUTO':
                    self.diagnostics['program_blocked_authority_'+channel] += 1
                    return
            self.bridge.forward_live_command('motion', selected, self.publish_selected, token)
        except (ValueError, TypeError) as exc:
            self.reject(str(exc))

    def reject(self, reason):
        """Retain the latest rejection even if the next target succeeds."""
        with self.bridge._lock:
            self.last_rejection = reason
            self.recent_rejection = reason
            self.last_rejection_ns = self.node.get_clock().now().nanoseconds
            self.diagnostics['rejected:'+reason.split(':', 1)[0]] += 1

    def receive_owned(self, message, token, *, frame_id=None):
        try:
            values = self.values(message, allow_missing=set(message.name) == set(self.arm_fields))
            names = set(values)
            home = frame_id == 'home'
            allowed = set(self.arm_fields) if home else set(self.arm_fields) | self.hand_names
            if token.source != 'vr' or not names <= allowed:
                return
            groups = (self.arm_fields,) if home else (self.arm_fields, *self.hands.values())
            for group in groups:
                if not names.intersection(group):
                    continue
                markers = (set() if home else self.sim_joints if group == self.arm_fields else
                           {self.teleop['grippers'][side]['binary_state_joint']
                            for side in self.hands if group == self.hands[side]})
                target = self.select(message, [n for n in message.name
                    if n in set(group) | markers and not math.isnan(values[n])])
                if frame_id is not None:
                    target.header.frame_id = frame_id
                self.bridge.forward_live_command('vr', target, self.publish_selected, token)
        except (ValueError, TypeError):
            return

    def prepare(self, message):
        try:
            original = self.values(message, allow_missing=set(message.name) == set(self.arm_fields))
            if not set(original) <= self.limits.keys() | self.sim_joints | self.model_only_joints:
                raise ValueError('unknown command joints')
            values = {n:v for n,v in original.items() if not math.isnan(v)}
            model = {n:values.pop(n) for n in list(values) if n in self.model_only_joints}
            # Standard JointState includes virtual/binary gripper fields. Keep
            # them on the action topic; only physical joints enter control checks.
            labels = {n: values.pop(n) for n in list(values) if n in self.sim_joints}
            if not values:
                raise ValueError('command has no physical joints')
            hand = next((s for s in self.hands if set(values) <= set(self.hands[s])), None)
            for side, cfg in ((s, self.teleop['grippers'][s]) for s in self.hands):
                marker = cfg['sim_joint']
                if marker not in labels:
                    continue
                if hand:
                    if side != hand or labels[marker] not in (0., 1.):
                        raise ValueError('invalid binary gripper field')
                elif not set(values).intersection(self.arms) or not 0 <= labels[marker] <= float(cfg['sim_closed_position']):
                    raise ValueError('invalid virtual gripper field')
            stamp = self.stamp(message)
            now = time.monotonic()
            with self.bridge._lock:
                wall_ns = self.node.get_clock().now().nanoseconds
                # Hands retain the existing open-loop command path. Physical
                # feedback is mandatory for arms/body, not for finger targets.
                if not set(values) <= self.limits.keys() or not self.fresh(
                        set(values)-self.hand_names, now=now, wall_ns=wall_ns):
                    raise ValueError('required feedback missing/stale or unknown joints')
                if not -.05 <= (wall_ns-stamp)*1e-9 <= self.max_age:
                    raise ValueError('stale command')
                if any(stamp <= self.last_command_stamp.get(n, 0) for n in values):
                    raise ValueError('reordered command')
                for name, value in values.items():
                    lo, hi = self.limits[name]
                    if not lo-1e-6 <= value <= hi+1e-6:
                        raise ValueError(f'joint limit: {name} target={value:.6f} range=[{lo:.6f},{hi:.6f}]')
                generation = self.bridge.ownership.token
                active = self.bridge.ownership.active
                if active:
                    allowed = set(self.home_joints) | self.hand_names
                    if not set(values) <= allowed:
                        raise ValueError('body joints held during intervention')
                # Program targets have an extra measured-error/slew envelope.
                # A VR takeover keeps ordinary teleoperation targets unchanged;
                # ownership fencing must not turn it into a second trajectory
                # limiter after the normal teleoperation controller.
                if active and generation.source == 'motion':
                    for name, value in values.items():
                        if name in self.waist:
                            continue  # Preserve supplied waist commands; do not apply hand slew.
                        if name in self.arms and abs(value-self.positions[name]) > self.max_error:
                            raise ValueError(f'target too far from measured position: {name} target={value:.6f} measured={self.positions[name]:.6f} error={abs(value-self.positions[name]):.6f} limit={self.max_error:.6f}')
                        reference = self.positions[name] if name in self.arms else value
                        previous = self.last_command.get(name, reference) if generation == self.generation else reference
                        dt = min(.05, max(.001, now-self.command_time.get(name, now-.01))) if generation == self.generation else .01
                        speed = self.arm_velocity if name in self.arms else float(self.teleop['grippers']['max_velocity'])
                        values[name] = min(previous+speed*dt, max(previous-speed*dt, value))
                        if abs(values[name]-value) > 1e-9:
                            self.diagnostics['arm_velocity_clipped' if name in self.arms else 'hand_velocity_clipped'] += 1
                            self.last_clip = dict(joint=name, requested=value, sent=values[name],
                                delta_rad=value-values[name], limit_rad_s=speed, dt_s=dt,
                                stamp_ns=wall_ns)
                measured = dict(self.positions)
            if self.collision and set(values) & set(self.arms):
                if not self.feedback_fresh():
                    raise ValueError('collision check requires fresh body feedback')
                allowed, collision, _ = self.collision.check(measured, values)
                if not allowed:
                    raise ValueError('collision check refused: '+str(collision))
            values.update(labels)
            values.update(model)
            names = self.arm_fields if active and set(values).intersection(self.arms) else message.name
            target = self.target_message(message, names, [values.get(n, math.nan) for n in names])
            # A rate-limited position target must not carry the original speed.
            target.velocity, target.effort = [], []
            return target, stamp, generation
        except (ValueError, TypeError) as exc:
            self.reject(str(exc))
            return None

    def commit(self, prepared, *, intervention=False):
        target, stamp, generation = prepared
        physical = {n for n,v in zip(target.name, target.position) if math.isfinite(v)} - self.sim_joints - self.model_only_joints
        # Final validation still samples the real ROS clock afresh; never
        # reuse the preparation timestamp across collision/authority work.
        now, wall_ns = time.monotonic(), self.node.get_clock().now().nanoseconds
        if (not self.fresh(physical-self.hand_names, now=now, wall_ns=wall_ns)
                or generation != self.bridge.ownership.token
                or not -.05 <= (wall_ns-stamp)*1e-9 <= self.max_age
                or any(stamp <= self.last_command_stamp.get(n, 0) for n in physical)):
            self.reject('commit freshness or authority changed')
            return False
        if generation != self.generation:
            self.last_command.clear()
            self.command_time.clear()
        self.generation = generation
        self.last_command_stamp.update(dict.fromkeys(physical, stamp))
        self.last_command.update((n, v) for n, v in zip(target.name, target.position) if n in physical)
        self.command_time.update(dict.fromkeys(physical, time.monotonic()))
        channel = 'arms' if set(target.name) & set(self.arms) else 'hands'
        self.last_rejection = ''
        self.diagnostics['accepted_'+channel] += 1
        return True

    def publish_selected(self, message):
        names = {n for n,v in zip(message.name, message.position) if math.isfinite(v)} - self.sim_joints - self.model_only_joints
        for side, group in self.hands.items():
            if names <= set(group):
                self.hand_pubs[side].publish(message)
                return
        self.output.publish(message)
        now = time.monotonic()
        if self.last_arm_publish_time is not None:
            self.arm_publish_gap_max_ms = max(self.arm_publish_gap_max_ms,
                (now-self.last_arm_publish_time)*1000)
        self.last_arm_publish_time = now
        self.diagnostics['published_arms'] += 1

    def status(self, *, wall_ns=None):
        now = time.monotonic()
        wall = self.node.get_clock().now().nanoseconds if wall_ns is None else wall_ns
        timing = {}
        for name in self.feedback_order:
            if name not in self.positions:
                timing[name] = dict(missing=True)
                continue
            timing[name] = dict(receive_age_ms=(now-self.received[name])*1000,
                                source_age_ms=(wall-self.sample_ns[name])/1e6)
        return dict(mode='legacy_topics', last_rejection=self.last_rejection,
                    recent_rejection=self.recent_rejection,
                    last_rejection_ns=self.last_rejection_ns,
                    diagnostics=dict(self.diagnostics), last_clip=dict(self.last_clip),
                    limits=dict(arm_velocity_rad_s=self.arm_velocity, target_error_rad=self.max_error,
                        feedback_command_max_age_s=self.max_age, hand_velocity_rad_s=self.teleop['grippers']['max_velocity']),
                    feedback_ready=self.feedback_fresh(),
                    feedback_timing_ms=timing,
                    hands_ready={side:self.fresh(names) for side,names in self.hands.items()})

    def close(self):
        if self.collision:
            self.collision.close()

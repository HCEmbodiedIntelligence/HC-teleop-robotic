"""Real node callbacks with fake joints on a localhost-only, isolated ROS domain."""
import importlib.util
import json
import os
from pathlib import Path
from unittest.mock import Mock

import numpy as np
import pytest


@pytest.fixture
def rig(monkeypatch):
    import rclpy
    from rclpy.node import Node
    from adapters.core.arm_teleop_node import RobotArmTeleopNode
    assert 200 <= int(os.environ['ROS_DOMAIN_ID']) <= 232
    assert os.environ['ROS_LOCALHOST_ONLY'] == '1'
    root = Path(__file__).resolve().parents[1]
    original = Node.__init__
    def init(node, *args, **kwargs):
        kwargs['cli_args'] = ['--ros-args', '--disable-external-lib-logs']
        original(node, *args, **kwargs)
    monkeypatch.setattr(Node, '__init__', init)
    spec = importlib.util.spec_from_file_location('switch_solver', root/'adapters/v23/script/control_v2_3_ros2.py')
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    rclpy.init()
    teleop = solver = None
    try:
        teleop = RobotArmTeleopNode(str(root/'adapters/robots/x1/arm_teleop.yaml'), backend='v23')
        solver = module.ControllerNode(str(root/'adapters/robots/x1/controller_v23.yml'))
        # No executor is spun: callbacks are driven explicitly, with no hardware.
        teleop.initial_homing_done = True
        teleop.hardware_ready = True
        teleop.command_pub = Mock()
        teleop.solver_reset_pub = Mock()
        solver.command_pub = Mock()
        solver.reset_ack_pub = Mock()
        yield teleop, solver
    finally:
        if solver: solver.destroy_node()
        if teleop: teleop.destroy_node()
        rclpy.shutdown()


def feedback(teleop, solver, offset=0.0):
    from sensor_msgs.msg import JointState
    values = {n: float(v) for n, v in teleop.initial_joints.items()}
    values['Joint1_R'] += offset
    msg = JointState(name=list(values), position=list(values.values()))
    msg.header.stamp = solver.get_clock().now().to_msg()
    return msg


def change(rig, source):
    from std_msgs.msg import String
    for node in rig: node._control_source_callback(String(data=source))


def right_input(teleop, *, a=False, grip=True, outward=False):
    from std_msgs.msg import String
    pose = {'position': [.1, .2, .3], 'quaternion': [0, 0, 0, 1]}
    teleop._vr_data_callback(String(data=json.dumps({
        'tracking': {'head': True, 'right': True}, 'poses': {'head': pose, 'right': pose},
        'inputs': {'right': {'held_mask': int(a), 'grip': float(grip), 'primary_axis': [int(outward), 0]},
                   'left': {'primary_axis': [-int(outward), 0]}}})))


@pytest.mark.parametrize('resume', ['vr', 'enable_topic'])
def test_resume_with_grip_after_replay_starts_at_measured_pose(rig, resume):
    from std_msgs.msg import Bool
    teleop, solver = rig
    home = feedback(*rig)
    teleop._joint_state_callback(home)
    solver._joint_callback(home)
    teleop._enabled_callback(Bool(data=False))  # Replay pauses VR without switching source.
    moved = feedback(*rig, offset=.6)
    teleop._joint_state_callback(moved)
    solver._joint_callback(moved)
    if resume == 'enable_topic':
        teleop._enabled_callback(Bool(data=True))
    right_input(teleop, a=True, outward=True)
    teleop._generic_command_callback(home)
    teleop.command_pub.publish.assert_not_called()
    teleop._joint_state_callback(moved)  # Delayed pre-resume feedback.
    assert teleop.source_feedback_pending
    fresh = feedback(*rig, offset=.6)
    teleop._joint_state_callback(fresh)
    assert not teleop.source_feedback_pending and not teleop.homing
    assert teleop.last_command['Joint1_R'] == teleop.initial_joints['Joint1_R'] + .6
    teleop.command_pub.publish.assert_called_once()
    hold = teleop.command_pub.publish.call_args.args[0]
    expected = dict(zip(fresh.name, fresh.position))
    assert len(hold.name) == 14 and list(hold.position) == [expected[n] for n in hold.name]
    teleop.solver_reset_pub.publish.assert_called_once()
    solver._reset_callback(Bool(data=True))
    solver._joint_callback(feedback(*rig, offset=.6))
    teleop._solver_reset_ack_callback(solver.reset_ack_pub.publish.call_args.args[0])
    right_input(teleop, outward=True)  # Held across resume, including the home-stick position.
    teleop.controller_target_pub = Mock()
    teleop._control_tick()
    assert not teleop.homing and not teleop.arms['right'].active
    targets = teleop.controller_target_pub.publish.call_args.args[0]
    targets.header.stamp = home.header.stamp
    assert solver._ee_callback(targets) is False
    solver._tick()
    solver.command_pub.publish.assert_not_called()
    targets.header.stamp = solver.get_clock().now().to_msg()
    assert solver._ee_callback(targets) is True
    solver._tick()
    teleop._generic_command_callback(solver.command_pub.publish.call_args.args[0])
    sent = teleop.command_pub.publish.call_args.args[0]
    assert sent.position[sent.name.index('Joint1_R')] == teleop.initial_joints['Joint1_R'] + .6
    teleop._control_tick()
    assert not teleop.arms['right'].active and teleop.solver_rearm.blocked
    right_input(teleop, grip=False)
    teleop._control_tick()
    right_input(teleop)
    teleop._control_tick()
    assert teleop.arms['right'].active and not teleop.solver_rearm.blocked
    right_input(teleop, grip=False, outward=True)
    teleop._control_tick()
    assert teleop.homing  # A new intentional home gesture still works.


def test_feedback_loss_waits_and_reanchors_without_solver_output(rig):
    from std_msgs.msg import Bool
    teleop, solver = rig
    teleop._enabled_callback(Bool(data=False))
    right_input(teleop, a=True)
    teleop._hardware_ready_callback(Bool(data=False))
    teleop._joint_state_callback(feedback(*rig, offset=.2))
    teleop._hardware_ready_callback(Bool(data=True))
    partial = feedback(*rig, offset=.2)
    missing = partial.name.index('Joint1_R')
    partial.name.pop(missing)
    partial.position.pop(missing)
    teleop._joint_state_callback(partial)
    teleop._generic_command_callback(feedback(*rig))
    teleop.command_pub.publish.assert_not_called()
    teleop.solver_reset_pub.publish.assert_not_called()
    teleop._joint_state_callback(feedback(*rig, offset=.2))
    teleop.command_pub.publish.assert_called_once()
    teleop.command_pub.publish.reset_mock()
    teleop.joint_state_source_ns -= 1_000_000_000
    teleop._generic_command_callback(feedback(*rig))
    assert teleop.source_feedback_pending
    teleop.command_pub.publish.assert_not_called()
    teleop._joint_state_callback(feedback(*rig, offset=.4))
    hold = teleop.command_pub.publish.call_args.args[0]
    assert hold.position[hold.name.index('Joint1_R')] == teleop.initial_joints['Joint1_R'] + .4


def test_program_mode_stops_shadow_ik_and_homing(rig):
    from std_msgs.msg import Bool
    teleop, solver = rig
    msg = feedback(*rig)
    teleop._joint_state_callback(msg)
    solver._joint_callback(msg)
    teleop._set_generic_home_targets()
    assert teleop.homing
    change(rig, 'motion')
    assert not teleop.homing
    teleop._home_topic_callback(Bool(data=True))
    assert not teleop.homing
    before = solver.controller.command_q.copy()
    for _ in range(20):
        solver._tick()
        teleop._control_tick()
    np.testing.assert_array_equal(solver.controller.command_q, before)
    solver.command_pub.publish.assert_not_called()
    teleop.command_pub.publish.assert_not_called()


def test_switch_requires_new_complete_feedback_and_new_targets(rig):
    from geometry_msgs.msg import Pose, PoseArray
    teleop, solver = rig
    old = feedback(*rig)
    teleop._joint_state_callback(old)
    solver._joint_callback(old)
    change(rig, 'motion')
    change(rig, 'vr')
    teleop._joint_state_callback(old)
    solver._joint_callback(old)
    assert teleop.source_feedback_pending and solver.reset_pending
    partial = feedback(*rig, offset=.4)
    partial.name.pop()
    partial.position.pop()
    solver._joint_callback(partial)
    assert solver.reset_pending
    msg = feedback(*rig, offset=.4)
    teleop._joint_state_callback(msg)
    solver._joint_callback(msg)
    assert not teleop.source_feedback_pending and not solver.reset_pending
    np.testing.assert_array_equal(solver.controller.command_q, solver.controller.q)
    assert teleop.last_command['Joint1_R'] == teleop.initial_joints['Joint1_R']+.4
    solver.reset_ack_pub.publish.assert_called_once()
    before = solver.controller.command_q.copy()
    solver._tick()
    np.testing.assert_array_equal(solver.controller.command_q, before)
    assert solver.source_target_pending
    poses = PoseArray()
    poses.header.stamp = old.header.stamp
    for task in solver.controller.pose_tasks:
        value = solver.controller.interface.relative_pose(task.root, task.frame, solver.controller.q)
        # Use the existing teleop conversion to supply measured Cartesian targets.
        from scipy.spatial.transform import Rotation
        xyz, quat = value.translation, Rotation.from_matrix(value.rotation).as_quat()
        pose = Pose()
        pose.position.x, pose.position.y, pose.position.z = map(float, xyz)
        pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w = map(float, quat)
        poses.poses.append(pose)
    assert solver._ee_callback(poses) is False
    poses.header.stamp = solver.get_clock().now().to_msg()
    assert solver._ee_callback(poses) is True
    assert not solver.source_target_pending
    solver._tick()
    output = solver.command_pub.publish.call_args.args[0]
    measured = dict(zip(msg.name, msg.position))
    delta = np.abs(np.array(output.position)-[measured[n] for n in output.name])
    assert np.max(delta) <= max(solver.controller.velocity_limits)*solver.controller.dt + 1e-8


def test_page_home_after_switch_starts_at_measured_position(rig):
    from std_msgs.msg import Bool
    teleop, solver = rig
    msg = feedback(*rig)
    teleop._joint_state_callback(msg)
    solver._joint_callback(msg)
    change(rig, 'motion')
    # Simulate stale commanded home with the robot moved by the program.
    teleop.last_command = dict(teleop.initial_joints)
    teleop.generic_aux_command['Joint1_R'] = teleop.initial_joints['Joint1_R']
    change(rig, 'vr')
    msg = feedback(*rig, offset=.6)
    teleop._joint_state_callback(msg)
    solver._joint_callback(msg)
    teleop._home_topic_callback(Bool(data=True))
    assert teleop.homing
    command = dict(teleop.joint_state)
    started = teleop.homing_last_time
    teleop._monotonic = lambda: started + .01
    teleop._update_homing(command)
    acceleration = teleop.control['home_joint_acceleration']
    assert abs(command['Joint1_R'] - (
        teleop.joint_state['Joint1_R'] - .5 * acceleration * .01 ** 2
    )) < 1e-10
    assert 'Joint1_R' not in teleop.generic_aux_command
    assert teleop.homing


def test_home_near_goal_does_not_snap_across_tolerance(rig):
    from std_msgs.msg import Bool
    teleop, solver = rig
    msg=feedback(*rig,offset=.04)
    teleop._joint_state_callback(msg)
    teleop._home_topic_callback(Bool(data=True))
    command=dict(teleop.joint_state)
    started = teleop.homing_last_time
    teleop._monotonic = lambda: started + .01
    teleop._update_homing(command)
    step=.5*teleop.control['home_joint_acceleration']*.01**2
    assert abs(command['Joint1_R']-teleop.joint_state['Joint1_R']) <= step+1e-10
    assert teleop.homing


def test_home_uses_acceleration_limit_and_only_enabled_joints(rig):
    from std_msgs.msg import Bool
    teleop, _solver = rig
    msg = feedback(*rig, offset=.6)
    # Disabled left arm and waist feedback must not hold Home open.
    msg.position[msg.name.index('Joint1_L')] += .8
    msg.position[msg.name.index('leg_1')] += .4
    teleop._joint_state_callback(msg)
    teleop._home_topic_callback(Bool(data=True))
    names = teleop._homing_joint_names()
    assert names == [f'Joint{i}_R' for i in range(1, 8)]
    now = teleop.homing_last_time
    previous_velocity = 0.0
    periods = (.006, .017, .029, .011)
    for index in range(500):
        dt = periods[index % len(periods)]
        now += dt
        teleop._monotonic = lambda now=now: now
        command = dict(teleop.joint_state)
        teleop._update_homing(command)
        velocity = teleop.homing_velocity.get('Joint1_R', 0.0)
        assert abs(velocity) <= teleop.control['home_joint_velocity'] + 1e-10
        assert abs(velocity - previous_velocity) <= (
            teleop.control['home_joint_acceleration'] * dt + 1e-10
        )
        previous_velocity = velocity
        teleop.last_command.update(command)
        for name in names:
            teleop.joint_state[name] = command[name]
        if not teleop.homing:
            break
    assert not teleop.homing


def test_duplicate_source_does_not_restart_reset_and_home_requires_manual_grant(rig):
    from std_msgs.msg import Bool
    teleop, solver = rig
    change(rig, 'motion')
    cutoff = solver.reset_cutoff_ns
    change(rig, 'motion')
    assert solver.reset_cutoff_ns == cutoff
    change(rig, 'vr')
    msg = feedback(*rig)
    teleop._joint_state_callback(msg)
    solver._joint_callback(msg)
    teleop.intervention.active = True
    teleop._home_topic_callback(Bool(data=True))
    assert not teleop.homing


def test_source_subscription_is_transient_local(rig):
    from rclpy.qos import DurabilityPolicy
    for node in rig:
        subscriptions = [s for s in node.subscriptions if s.topic_name == '/control/source']
        assert len(subscriptions) == 1
        assert subscriptions[0].qos_profile.durability == DurabilityPolicy.TRANSIENT_LOCAL


def test_ros_source_delivery_reset_handshake_and_page_home(rig):
    import time
    from rclpy.node import Node
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
    from sensor_msgs.msg import JointState
    from std_msgs.msg import Bool, String
    teleop, solver = rig
    # Restore the real reset publisher so the two actual nodes handshake over DDS.
    teleop.solver_reset_pub = teleop.create_publisher(Bool, '/control/solver/reset', 1)
    from std_msgs.msg import Header
    publications = solver.controller.cfg['ros_interface']['pub_topic']
    solver.reset_ack_pub = Mock(wraps=solver.create_publisher(Header,
        publications.get('reset_ack', '/status/solver/reset_ack'), 1))
    solver.command_pub = Mock(wraps=solver.create_publisher(JointState,
        publications['joint_target'], 1))
    probe = Node('source_switch_fake_hardware')
    executor = SingleThreadedExecutor()
    for node in (*rig, probe): executor.add_node(node)
    source = probe.create_publisher(String, '/control/source', QoSProfile(
        depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL))
    state = probe.create_publisher(JointState, '/observation/joint_states', 1)
    home = probe.create_publisher(Bool, '/teleop/arm/home', 1)
    timer = probe.create_timer(.01, lambda: state.publish(feedback(*rig, offset=.6)))
    def pump(seconds):
        end = time.monotonic()+seconds
        while time.monotonic() < end: executor.spin_once(timeout_sec=.002)
    def until(predicate):
        end = time.monotonic()+3
        while not predicate() and time.monotonic() < end: executor.spin_once(timeout_sec=.002)
        assert predicate()
    try:
        until(lambda: source.get_subscription_count() == 2 and home.get_subscription_count() == 1)
        source.publish(String(data='motion'))
        until(lambda: all(node.control_source == 'motion' for node in rig))
        pump(.08)
        before = solver.controller.command_q.copy()
        teleop.command_pub.reset_mock()
        solver.command_pub.reset_mock()
        pump(.1)
        np.testing.assert_array_equal(solver.controller.command_q, before)
        teleop.command_pub.publish.assert_not_called()
        solver.command_pub.publish.assert_not_called()
        source.publish(String(data='vr'))
        until(lambda: all(node.control_source == 'vr' for node in rig)
              and not teleop.source_feedback_pending and not solver.reset_pending
              and not solver.source_target_pending)
        until(lambda: teleop.solver_rearm.state in {'wait_release', 'wait_press'})
        teleop.command_pub.reset_mock()
        home.publish(Bool(data=True))
        until(lambda: teleop.homing and any(
            call.args[0].name == teleop._homing_joint_names()
            for call in teleop.command_pub.publish.call_args_list
        ))
        output = next(
            call.args[0] for call in teleop.command_pub.publish.call_args_list
            if call.args[0].name == teleop._homing_joint_names()
        )
        value = output.position[output.name.index('Joint1_R')]
        max_first_step = .5 * teleop.control['home_joint_acceleration'] * .05 ** 2
        assert abs(value - (teleop.initial_joints['Joint1_R']+.6)) <= max_first_step+1e-9
        assert output.name == teleop._homing_joint_names()
    finally:
        executor.shutdown()
        probe.destroy_node()


def test_dagger_targets_in_recording_and_replay(rig, tmp_path):
    import time
    import rclpy
    from mcap.reader import make_reader
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message
    from sensor_msgs.msg import JointState
    from middleware.core.control.control_ownership import ControlToken
    from middleware.core.recording.dataset_topics import DatasetTopics
    from middleware.core.recording.ros_recording_executor import RosRecordingExecutor
    from middleware.core.recording.topic_player import TopicPlayer
    from middleware.core.recording.topic_recorder import TopicRecorder

    teleop, solver = rig
    teleop._joint_state_callback(feedback(*rig))
    owner = teleop.intervention
    ik = solver.intervention
    owner.token = ik.token = ControlToken('instance', 'session', 1, 'vr', 'manual')
    owner.active = ik.active = ik.ready = True
    ik.phase = 'grant'
    owner.pose_pub = Mock()
    ik.output = Mock()
    teleop.target_pub = Mock()
    teleop.controller_target_pub = Mock()
    arm_names = [n for s in ('right', 'left') for n in teleop.config['arms'][s]['joint_names']]
    solver.controller.step = Mock(return_value=(arm_names, [teleop.initial_joints[n] for n in arm_names]))
    solver.reset_pending = False

    node = rclpy.create_node('recording_projection_test')
    projections = DatasetTopics(node, {'joint_groups': {}, 'x1_recording_fk': True})
    pose = '/action/ee_poses'
    right = '/action/right_arm/ee_pose'
    left = '/action/left_arm/ee_pose'
    ee = '/control/solver/ee_targets'
    joints = '/control/solver/joint_targets'
    action = '/action/joint_states'
    hands = [f'/action/{s}_hand/joint_states' for s in ('right', 'left')]
    waist = '/action/waist/joint_states'
    fk = '/observation/ee_poses'
    fk_right = '/observation/right_arm/ee_pose'
    fk_left = '/observation/left_arm/ee_pose'
    triggers = '/observation/hands/triggers'
    types = {pose: 'geometry_msgs/msg/PoseArray', right: 'geometry_msgs/msg/PoseStamped',
             left: 'geometry_msgs/msg/PoseStamped', ee: 'geometry_msgs/msg/PoseArray',
             joints: 'sensor_msgs/msg/JointState', action: 'sensor_msgs/msg/JointState',
             waist: 'sensor_msgs/msg/JointState', fk: 'geometry_msgs/msg/PoseArray',
             fk_left: 'geometry_msgs/msg/PoseStamped', fk_right: 'geometry_msgs/msg/PoseStamped',
             triggers: 'geometry_msgs/msg/PoseArray'}
    types.update({topic:'sensor_msgs/msg/JointState' for topic in hands})
    action_pub = node.create_publisher(JointState, action, 10)
    hand_pubs = [node.create_publisher(JointState, topic, 10) for topic in hands]
    from geometry_msgs.msg import PoseArray
    trigger_pub = node.create_publisher(PoseArray, triggers, 10)
    subscriptions = [dict(topic=k, type=t, outputs=['record']) for k,t in types.items()]
    recorder = TopicRecorder({'directory': str(tmp_path)}, tmp_path, topics=subscriptions)
    receiver = RosRecordingExecutor(
        {'legacy_control_profile': 'x1', 'domain_id': int(os.environ['ROS_DOMAIN_ID']),
         'subscriptions': subscriptions},
        recorder)

    def until(condition):
        deadline = time.monotonic() + 5
        while not condition():
            assert time.monotonic() < deadline, json.dumps(receiver.status(), default=str)
            rclpy.spin_once(node, timeout_sec=.02)

    try:
        receiver.prepare_recording()
        until(lambda: all(p.get_subscription_count() for p in
                          (owner.record_action_pose, ik.record_ee_target, ik.record_joint_target, action_pub, trigger_pub, *hand_pubs)))
        # The projection node and the raw recorder both consume the arm target.
        until(lambda: owner.record_action_pose.get_subscription_count() >= 2)
        until(lambda: teleop.record_actual_pub.get_subscription_count() >= 2)
        until(lambda: all(projections.publishers[k].get_subscription_count() for k in
                          (right, left, '/internal/recording' + fk_right, '/internal/recording' + fk_left)))
        # DDS graph discovery can precede cross-process sample delivery.
        def receiving():
            if receiver.status()['received']:
                return True
            action_pub.publish(JointState())
            return False
        until(receiving)
        path = Path(recorder.start('dagger_labels', {'profile_id': 'x1'}))
        conflicting_pose = PoseArray()
        conflicting_pose.header.frame_id = 'marvin_arm_base'
        node.create_publisher(PoseArray, fk, 10).publish(conflicting_pose)
        teleop._sync_model()
        teleop._publish_actual_poses()
        recorder.record(dict(topic='/events/control/intervention', payload={'test': True}))
        owner.granted = False
        teleop._publish_target_poses()
        owner.pose_pub.publish.assert_not_called()
        owner.granted = True
        teleop._publish_target_poses()
        owner.pose_pub.publish.assert_called_once()
        owned_target = owner.pose_pub.publish.call_args.args[0]
        ik.permit_time = ik.feedback_time = time.monotonic()
        aux = JointState(name=['R_ban', 'L_ban'], position=[.035, .035])
        aux.header.stamp = solver.get_clock().now().to_msg()
        aux.header.frame_id = ik.token.key()+'#'
        ik.aux(aux)
        ik.pose(owned_target)
        ik.pose(owned_target)  # Duplicate stamp is rejected.
        solver._tick()
        ik.output.publish.assert_called_once()
        solver.command_pub.publish.assert_not_called()
        teleop.target_pub.publish.assert_not_called()
        teleop.controller_target_pub.publish.assert_not_called()
        owned_joint = ik.output.publish.call_args.args[0]
        actual = JointState(name=owned_joint.name, position=owned_joint.position)
        actual.header.stamp = owned_joint.header.stamp
        assert actual.name == arm_names + ['R_ban', 'L_ban']
        action_pub.publish(actual)
        actual_hands = []
        for side, pub in zip(('right', 'left'), hand_pubs):
            cfg = teleop.config['grippers'][side]
            hand = JointState(name=cfg['finger_joint_names'] + [cfg['binary_state_joint']],
                              position=cfg['finger_open'] + [0.])
            hand.header.stamp = actual.header.stamp
            actual_hands.append(hand)
            pub.publish(hand)
        trigger_message = PoseArray()
        trigger_message.header.frame_id = 'real_trigger_sample'
        trigger_pub.publish(trigger_message)
        until(lambda: receiver.status()['recorded'] >= 12)
        receiver.stop()
        recorder.stop()
    finally:
        receiver.stop()
        recorder.shutdown()
        node.destroy_node()

    with path.open('rb') as stream:
        reader = make_reader(stream)
        summary = reader.get_summary()
        assert {c.topic for c in summary.channels.values()} == set(types)
        waist_channel = next(c.id for c in summary.channels.values() if c.topic == waist)
        assert summary.statistics.channel_message_counts.get(waist_channel, 0) == 0
        assert [m.name for m in reader.iter_metadata()] == [
            'hc_teleop.session', 'hc_teleop.capture_window', 'hc_teleop.capture_audit']
        samples = [(c.topic, deserialize_message(m.data, get_message(s.name)))
                   for s,c,m in reader.iter_messages()]
    assert len(samples) == 12
    data = dict(samples)
    assert data.keys() == {pose, right, left, ee, joints, action, fk, fk_left, fk_right, triggers, *hands}
    assert [data[t] for t in hands] == actual_hands
    assert data[triggers] == trigger_message
    assert data[fk].header.frame_id == 'zhi_Link'
    assert data[fk_left].header == data[fk_right].header == data[fk].header
    assert data[fk_right].pose == data[fk].poses[0] and data[fk_left].pose == data[fk].poses[1]
    assert data[pose].header.frame_id == 'zhi_Link'
    assert data[right].header == data[left].header == data[pose].header
    assert data[right].pose == data[pose].poses[0] and data[left].pose == data[pose].poses[1]
    assert data[ee].poses == owned_target.poses
    assert data[ee].header.frame_id == 'generic_task_bases'
    assert data[ee].header.stamp == data[pose].header.stamp == owned_target.header.stamp
    owned_joint = ik.output.publish.call_args.args[0]
    assert data[joints].name == arm_names and data[joints].position == owned_joint.position[:14]
    assert data[joints].header.stamp == owned_joint.header.stamp and not data[joints].header.frame_id
    assert data[action] == actual

    bridge = Mock()
    player = TopicPlayer(tmp_path, lambda: bridge)
    player.play(path.name, speed=10, mode='drive')
    player._thread.join(5)
    assert player.status()['state'] == 'completed'
    assert [call.args[0] for call in bridge.publish_raw.call_args_list] == [action, *hands]


def test_manual_aux_is_generation_scoped_and_expires(rig):
    import copy
    import time
    from sensor_msgs.msg import JointState
    from middleware.core.control.control_ownership import ControlToken
    teleop, solver = rig
    ik = solver.intervention
    ik.active = ik.ready = True
    ik.phase = 'grant'
    ik.token = ControlToken('instance', 'session', 1, 'vr', 'manual')
    ik.permit_time = time.monotonic()
    ik.output = Mock()
    aux = JointState(name=['R_ban','L_ban'], position=[.3,.7])
    aux.header.stamp = solver.get_clock().now().to_msg()
    aux.header.frame_id = ik.token.key()+'#'
    ik.aux(aux)
    assert ik.auxiliary is aux
    duplicate = copy.deepcopy(aux); duplicate.position[0] = .9
    ik.aux(duplicate)
    assert ik.auxiliary.position[0] == .3
    wrong = copy.deepcopy(aux)
    wrong.header.stamp = solver.get_clock().now().to_msg()
    wrong.header.frame_id = ControlToken('instance', 'session', 2, 'vr', 'manual').key()+'#'
    ik.aux(wrong)
    assert ik.auxiliary is aux
    target = JointState(name=['Joint1_R'], position=[.1])
    ik.publish(target)
    assert ik.output.publish.call_args.args[0].name == ['Joint1_R','R_ban','L_ban']
    aux.header.stamp.sec -= 1
    ik.publish(target)
    assert ik.output.publish.call_args.args[0].name == ['Joint1_R']
    assert target.name == ['Joint1_R']


@pytest.mark.parametrize("resetting", [False, True])
@pytest.mark.parametrize("invalid", ["stale", "future", "incomplete", "duplicate", "nonfinite", "length"])
def test_invalid_feedback_never_mutates_solver_state(rig, resetting, invalid):
    teleop, solver = rig
    solver._joint_callback(feedback(*rig))
    before = solver.controller.q.copy()
    if resetting:
        solver._reset_references()
    msg = feedback(*rig, offset=.6)
    if invalid == "stale":
        msg.header.stamp.sec -= 1
    elif invalid == "future":
        msg.header.stamp.sec += 1
    elif invalid == "incomplete":
        index = msg.name.index(solver.controller.free_joint_names[0])
        del msg.name[index]
        del msg.position[index]
    elif invalid == "duplicate":
        msg.name.append(msg.name[0])
        msg.position.append(msg.position[0])
    elif invalid == "nonfinite":
        msg.position[0] = float("nan")
    else:
        msg.position.pop()
    solver._joint_callback(msg)
    np.testing.assert_array_equal(solver.controller.q, before)
    solver.reset_ack_pub.publish.assert_not_called()
    assert sum(solver.feedback_rejections.values()) == 1
    assert solver.reset_pending == resetting


def test_reset_retry_stays_blocked_until_ack_and_new_grip(rig):
    teleop, solver = rig
    teleop.initial_homing_done = True
    teleop.solver_rearm.start_homing()
    teleop.solver_rearm.homing_complete()
    teleop._joint_state_callback(feedback(*rig))
    assert teleop.solver_rearm.state == teleop.solver_rearm.WAIT_ACK
    cutoff = teleop.solver_rearm.reset_cutoff_ns
    teleop.command_pub.publish.reset_mock()
    teleop.solver_reset_pub.publish.reset_mock()
    teleop._last_solver_reset_retry_ns -= 500_000_000
    teleop._joint_state_callback(feedback(*rig))
    teleop.solver_reset_pub.publish.assert_called_once()
    assert teleop.solver_rearm.reset_cutoff_ns == cutoff
    assert teleop.solver_rearm.state == teleop.solver_rearm.WAIT_ACK
    before = dict(teleop.last_command)
    teleop._generic_command_callback(feedback(*rig, offset=.6))
    output = teleop.command_pub.publish.call_args.args[0]
    actual = dict(zip(output.name, output.position))
    for arm in teleop.arms.values():
        for name in arm.joint_names:
            assert actual[name] == before[name]
    assert teleop.solver_rearm.state == teleop.solver_rearm.WAIT_ACK


def test_reset_wait_logs_missing_feedback_instead_of_silent_zero_hz(rig):
    _, solver = rig
    solver._reset_references()
    logger = Mock()
    solver.get_logger = Mock(return_value=logger)
    solver._tick()
    warning = logger.warning.call_args.args[0]
    assert "feedback_received=0" in warning
    assert "output remains blocked" in warning
    solver.command_pub.publish.assert_not_called()


def test_clutch_schmitt_hysteresis_and_stale_release(rig):
    teleop, _ = rig
    arm = teleop.arms['right']
    for value, expected in [(.54, False), (.55, True), (.50, True), (.32549, True),
                            (.31, True), (.30, False), (.5, False), (.56, True)]:
        right_input(teleop, grip=value)
        assert teleop._clutch_pressed(arm) is expected
    assert not teleop._input_fresh(arm, arm.joy_stamp + 1.)
    right_input(teleop, grip=.5)
    assert not teleop._clutch_pressed(arm)


def test_real_clutch_release_freezes_integrator_and_resume_needs_fresh_handshake(rig):
    from std_msgs.msg import Header
    teleop, solver = rig
    teleop.enabled = True
    teleop.controller_target_pub = Mock()
    home = feedback(*rig)
    teleop._joint_state_callback(home)
    solver._joint_callback(home)
    right_input(teleop)
    teleop._control_tick()
    assert teleop.arms['right'].active
    solver._ee_callback(teleop.controller_target_pub.publish.call_args.args[0])
    solver._tick()
    # Simulate the pre-release integrator having an ordinary tracking lead.
    solver.controller.command_q[solver.clutch_q_indices] += .1
    right_input(teleop, grip=False)
    teleop.solver_clutch_pub = Mock()
    teleop._control_tick()
    hold = teleop.solver_clutch_pub.publish.call_args.args[0]
    assert hold.frame_id == 'hold'
    solver._clutch_callback(hold)
    frozen = solver.controller.command_q[solver.clutch_q_indices].copy()
    for _ in range(5):
        solver._tick()
        assert np.allclose(solver.controller.command_q[solver.clutch_q_indices], frozen)
    assert solver.command_pub.publish.call_args.args[0].header.frame_id == 'solver_clutch_hold'
    # A real re-press requests an arm-scoped reset, with outputs still blocked.
    right_input(teleop)
    teleop._control_tick()
    resume = teleop.solver_clutch_pub.publish.call_args.args[0]
    assert resume.frame_id == 'resume' and teleop.solver_rearm.blocked
    solver._clutch_callback(resume)
    solver._joint_callback(home)  # Feedback predating re-press is rejected.
    assert solver.clutch_reset_pending
    fresh = feedback(*rig, offset=.02)
    other_columns = [i for i in range(len(solver.controller.free_joint_names)) if i not in solver.clutch_columns]
    other_indices = solver.controller.interface.q_indices[other_columns]
    body_before = solver.controller.command_q[other_indices].copy()
    teleop._joint_state_callback(fresh)
    solver._joint_callback(fresh)
    assert not solver.clutch_reset_pending
    assert np.allclose(solver.controller.command_q[other_indices], body_before)
    assert np.allclose(solver.controller.command_q[solver.clutch_q_indices], solver.controller.q[solver.clutch_q_indices])
    ack = solver.reset_ack_pub.publish.call_args.args[0]
    assert ack.frame_id == 'clutch'
    teleop._solver_reset_ack_callback(ack)
    assert teleop.solver_rearm.state == teleop.solver_rearm.WAIT_SOLVER
    held = solver.command_pub.publish.call_args.args[0]
    held.header.stamp = solver.get_clock().now().to_msg()
    teleop._generic_command_callback(held)
    assert teleop.solver_rearm.state == teleop.solver_rearm.WAIT_SOLVER
    teleop._control_tick()
    targets = teleop.controller_target_pub.publish.call_args.args[0]
    assert solver._ee_callback(targets)
    solver._tick()
    first_output = solver.command_pub.publish.call_args.args[0]
    measured = dict(zip(fresh.name, fresh.position))
    first = dict(zip(first_output.name, first_output.position))
    assert max(abs(first[n] - measured[n]) for n in teleop.arms['right'].joint_names) <= .00121
    teleop._generic_command_callback(first_output)
    teleop._control_tick()
    assert not teleop.solver_rearm.blocked and teleop.arms['right'].active
    # An old hold event must not undo the successful resume.
    solver._clutch_callback(hold)
    assert not solver.clutch_held


def test_clutch_pending_reset_cannot_be_opened_by_nonclutch_ack(rig):
    from std_msgs.msg import Header
    teleop, solver = rig
    teleop._clutch_resume_pending = True
    teleop.solver_rearm.start_homing()
    teleop.solver_rearm.homing_complete()
    teleop.solver_rearm.observe_feedback()
    teleop.solver_rearm.reset_published(1)
    teleop._solver_reset_ack_callback(Header(stamp=solver.get_clock().now().to_msg()))
    assert teleop.solver_rearm.state == teleop.solver_rearm.WAIT_ACK


def test_queued_vr_frame_does_not_refresh_input_age(rig):
    from geometry_msgs.msg import PoseStamped
    from sensor_msgs.msg import Joy
    from builtin_interfaces.msg import Time
    teleop, _ = rig
    now = teleop.get_clock().now().nanoseconds
    stamp = Time(sec=int((now-600_000_000)//1_000_000_000), nanosec=int((now-600_000_000)%1_000_000_000))
    arm = teleop.arms['right']
    arm.pose_stamp = arm.joy_stamp = 0.
    from std_msgs.msg import Header
    pose = PoseStamped(header=Header(stamp=stamp))
    pose.pose.orientation.w = 1.
    teleop._arm_pose_callback(arm, pose, sample_time=1.)
    teleop._joy_callback(arm, Joy(header=pose.header, axes=[0., 1.]))
    assert arm.pose_stamp == arm.joy_stamp == 0.


def test_held_grip_after_input_timeout_is_not_an_automatic_resume(rig):
    teleop, solver = rig
    fresh = feedback(*rig)
    teleop._joint_state_callback(fresh)
    solver._joint_callback(fresh)
    teleop.arms['right'].active = True
    teleop.solver_clutch_pub = Mock()
    teleop._update_solver_clutch(False, input_fresh=False)
    assert teleop._clutch_solver_held
    assert not teleop.solver_rearm.release_observed
    teleop._update_solver_clutch(True, input_fresh=True)
    assert teleop.solver_clutch_pub.publish.call_count == 1
    assert teleop.solver_rearm.blocked and not teleop._clutch_resume_pending
    teleop.solver_rearm.observe_clutch(False)  # A fresh real release.
    teleop._update_solver_clutch(True, input_fresh=True)
    assert teleop._clutch_resume_pending


def test_hardware_loss_cancels_clutch_handshake_without_reopening_output(rig):
    from std_msgs.msg import Bool, Header
    teleop, solver = rig
    teleop._clutch_solver_held = True
    teleop._clutch_resume_pending = True
    teleop.solver_clutch_pub = Mock()
    teleop._hardware_ready_callback(Bool(data=False))
    assert not teleop._clutch_solver_held and not teleop._clutch_resume_pending
    assert teleop.solver_rearm.blocked and not teleop.hardware_ready
    clear = teleop.solver_clutch_pub.publish.call_args.args[0]
    assert clear.frame_id == 'clear'
    # Delayed clutch ACK cannot satisfy a later ordinary/full reset.
    teleop.solver_rearm.homing_complete()
    teleop.solver_rearm.observe_feedback()
    teleop.solver_rearm.reset_published(1)
    teleop._solver_reset_ack_callback(Header(stamp=solver.get_clock().now().to_msg(), frame_id='clutch'))
    assert teleop.solver_rearm.state == teleop.solver_rearm.WAIT_ACK

"""Real node callbacks with fake joints on a localhost-only, isolated ROS domain."""
import importlib.util
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
    teleop._update_homing(command)
    step = teleop.control['home_joint_velocity']/teleop.control['rate_hz']
    assert abs(command['Joint1_R'] - (teleop.joint_state['Joint1_R']-step)) < 1e-10
    assert 'Joint1_R' not in teleop.generic_aux_command
    assert teleop.homing


def test_home_near_goal_does_not_snap_across_tolerance(rig):
    from std_msgs.msg import Bool
    teleop, solver = rig
    msg=feedback(*rig,offset=.04)
    teleop._joint_state_callback(msg)
    teleop._home_topic_callback(Bool(data=True))
    command=dict(teleop.joint_state)
    teleop._update_homing(command)
    step=teleop.control['home_joint_velocity']/teleop.control['rate_hz']
    assert abs(command['Joint1_R']-teleop.joint_state['Joint1_R']) <= step+1e-10
    assert teleop.homing


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
        until(lambda: teleop.homing and teleop.command_pub.publish.called)
        output = teleop.command_pub.publish.call_args_list[0].args[0]
        value = output.position[output.name.index('Joint1_R')]
        step = teleop.control['home_joint_velocity']/teleop.control['rate_hz']
        assert abs(value - (teleop.initial_joints['Joint1_R']+.6)) <= step+1e-9
    finally:
        executor.shutdown()
        probe.destroy_node()

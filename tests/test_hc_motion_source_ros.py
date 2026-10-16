"""Real HC routes/mux/Motion/OpenArmX adapter with local fake controllers; no CAN.

Requires the consolidated ROS installation and OpenArmX packages. Domain 216 is
localhost-only; abort if discovery finds an existing node before starting peers.
"""
import asyncio
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import time
from types import SimpleNamespace
from unittest.mock import AsyncMock, Mock, patch
import uuid

import pytest

rclpy = pytest.importorskip('rclpy')
pytest.importorskip('openarmx_hc_robot_adapter.node')
from controller_manager_msgs.msg import ControllerState
from controller_manager_msgs.srv import ListControllers
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, Float64MultiArray
from openarmx_hc_robot_adapter.node import OpenArmXHcRobotAdapter

from middleware.core.app import MiddlewareRuntime, create_app
from middleware.core.config import ConfigStore, validate_config
from middleware.core.ros_bridge import RosBridge


ROOT = Path(__file__).resolve().parents[1]


def test_motion_source_movej_stop_recovery_and_subscription(tmp_path, monkeypatch):
    if os.environ.get('HC_UNIFIED_REPORT_DIR'):
        tmp_path=Path(os.environ['HC_UNIFIED_REPORT_DIR'])/'http-regression'
        tmp_path.mkdir(parents=True,exist_ok=True)
    monkeypatch.setenv('ROS_LOCALHOST_ONLY', '1')
    monkeypatch.setenv('ROS_DOMAIN_ID', '216')
    monkeypatch.setenv('ROS_LOG_DIR', str(tmp_path / 'ros-log'))
    rclpy.init(args=['--ros-args', '-p', 'hardware_mode:=fake', '-p',
                    'resource_id:=api-test-' + uuid.uuid4().hex,
                    '-p', 'hc_feedback_topic:=/internal/robot/joint_states',
                    '-p', 'hc_command_topic:=/internal/robot/joint_commands'], domain_id=216)
    robot = Node('hc_api_fake_controllers', use_global_arguments=False)
    executor = SingleThreadedExecutor()
    executor.add_node(robot)
    adapter = bridge = motion = client = None
    motion_log = (tmp_path / 'motion.log').open('w+')
    client_log = (tmp_path / 'client.log').open('w+')

    def pump(seconds):
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline:
            executor.spin_once(timeout_sec=.005)

    def until(condition, timeout=5.):
        deadline = time.monotonic() + timeout
        while not condition():
            assert time.monotonic() < deadline, 'ROS integration condition timed out'
            executor.spin_once(timeout_sec=.005)

    def stop_process(process):
        if process is not None and process.poll() is None:
            os.killpg(process.pid, signal.SIGINT)
            try:
                process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGKILL)
                process.wait(timeout=5)

    try:
        pump(.3)
        assert set(robot.get_node_names()) <= {robot.get_name()}, 'isolated ROS domain is occupied'
        adapter = OpenArmXHcRobotAdapter()
        executor.add_node(adapter)
        names = [name for side in ('right', 'left') for name in
                 adapter.gate.joint_names[side] + (adapter.gate.gripper_joint_names[side],)]
        positions = dict.fromkeys(names, 0.)
        send_feedback = [True]
        commands, vr_enabled, hardware_enabled = [], [], []

        def controllers(_request, response):
            for side in ('right', 'left'):
                state = ControllerState(name=side + '_forward_position_controller', state='active')
                joints = adapter.gate.joint_names[side] + (adapter.gate.gripper_joint_names[side],)
                state.claimed_interfaces = [joint + '/position' for joint in joints]
                response.controller.append(state)
            return response

        robot.create_service(ListControllers, '/controller_manager/list_controllers', controllers)
        feedback = robot.create_publisher(JointState, '/joint_states', 10)

        def sample():
            if send_feedback[0]:
                msg = JointState(name=names, position=[positions[n] for n in names],
                                 velocity=[0.] * len(names))
                msg.header.stamp = robot.get_clock().now().to_msg()
                feedback.publish(msg)

        robot.create_timer(.01, sample)
        for side in ('right', 'left'):
            def apply_command(message, side=side):
                joints = adapter.gate.joint_names[side] + (adapter.gate.gripper_joint_names[side],)
                positions.update(zip(joints, message.data))
            robot.create_subscription(Float64MultiArray, f'/{side}_forward_position_controller/commands',
                                      apply_command, 10)
        robot.create_subscription(JointState, '/action/joint_states', commands.append, 10)
        robot.create_subscription(Bool, '/control/teleop/enabled',
                                  lambda msg: vr_enabled.append(msg.data), 10)
        robot.create_subscription(Bool, '/control/hardware/enabled',
                                  lambda msg: hardware_enabled.append(msg.data), 10)
        vr_target = robot.create_publisher(JointState, '/control/vr/joint_states', 10)
        vr_switch = robot.create_publisher(Bool, '/control/teleop/enabled', 10)

        config = validate_config({'ros': {'domain_id': 216, 'robot_api_profile':str(ROOT/'adapters/robots/openarmx')},
                                  'robot_profiles': {'root': str(tmp_path / 'profiles')}})
        store = ConfigStore(tmp_path / 'config.yaml')
        store.save(config)
        runtime = MiddlewareRuntime(config, tmp_path)
        runtime.emit = Mock()
        bridge = runtime.ros = RosBridge(config['ros'], runtime.emit)
        # Exercise the actual HTTP handlers without starting unrelated camera/VR services.
        with patch('middleware.core.app.MiddlewareRuntime', return_value=runtime):
            app = create_app(store)
        handlers = {route.resource.canonical: route.handler for route in app.router.routes()
                    if route.method == 'POST'}

        def post(path, payload=None):
            request = SimpleNamespace(json=AsyncMock(return_value=payload or {}), can_read_body=True)
            response = asyncio.run(handlers[path](request))
            assert response.status == 202

        bridge.start()
        until(lambda: bridge.status()['state'] == 'running')
        post('/api/safety/stop', {'reason': 'test startup'})
        until(lambda: adapter.gate.emergency_stopped)
        until(lambda: adapter._eligible(time.monotonic()))
        post('/api/teleop/source', {'source': 'motion'})
        until(lambda: vr_enabled and vr_enabled[-1] is False)
        pump(.1)
        post('/api/safety/resume')
        until(lambda: bridge.status()['command_mux']['hardware_ready'])
        assert not adapter.gate.emergency_stopped
        assert hardware_enabled[-1] is True and vr_enabled[-1] is False

        motion = subprocess.Popen([str(ROOT / 'tools/run_motion.sh'), '--profile',
                                   str(ROOT / 'adapters/robots/openarmx')], cwd=ROOT,
                                  stdout=motion_log, stderr=subprocess.STDOUT, start_new_session=True)

        def start_demo(command):
            return subprocess.Popen([sys.executable, str(ROOT / 'examples/robot_api_demo.py'), *command],
                                    cwd=ROOT, stdout=client_log, stderr=subprocess.STDOUT,
                                    start_new_session=True)

        move_args = ['move-j', '--group', 'left_arm', '--joint', 'left_joint_4',
                     '--delta', '.02', '--wait', '15']
        client = start_demo(move_args)
        until(lambda: client.poll() is not None, 35.)
        client_log.flush()
        assert client.returncode == 0, (tmp_path / 'client.log').read_text()
        assert motion.poll() is None, (tmp_path / 'motion.log').read_text()
        assert positions['openarmx_left_joint4'] > .005
        assert commands and commands[-1].name
        assert '"code": 0' in (tmp_path / 'client.log').read_text()

        # Switching to VR must revoke motion and require explicit hardware resume.
        post('/api/teleop/source', {'source': 'vr'})
        until(lambda: adapter.gate.emergency_stopped and
              not bridge.status()['command_mux']['hardware_ready'])
        assert not bridge.status()['command_mux']['output_enabled']
        post('/api/safety/resume')
        until(lambda: bridge.status()['command_mux']['hardware_ready'])
        assert vr_enabled[-1] is True
        stale = JointState(name=[f'left_joint_{i}' for i in range(1,8)],
                           position=[positions[n] for n in adapter.gate.joint_names['left']])
        stale.position[3] = .04
        stale.header.stamp = robot.get_clock().now().to_msg()
        vr_target.publish(stale)
        until(lambda: positions['openarmx_left_joint4'] > .03)
        vr_switch.publish(Bool(data=False))
        pump(.1)
        assert not adapter.gate.emergency_stopped  # Disabling VR alone cannot stop hardware.

        post('/api/safety/stop')
        until(lambda: adapter.gate.emergency_stopped and
              not bridge.status()['command_mux']['hardware_ready'])
        held = positions['openarmx_left_joint4']
        post('/api/safety/resume')
        until(lambda: bridge.status()['command_mux']['hardware_ready'])
        vr_target.publish(stale)  # Pre-stop command must not be replayed after resume.
        pump(.2)
        assert positions['openarmx_left_joint4'] == pytest.approx(held)

        # Feedback restoration alone must not restore output permission.
        send_feedback[0] = False
        until(lambda: not bridge.status()['command_mux']['output_enabled'], 2.)
        until(lambda: adapter.gate.emergency_stopped)
        send_feedback[0] = True
        until(lambda: adapter._eligible(time.monotonic()))
        pump(.1)
        assert adapter.gate.emergency_stopped
        post('/api/safety/resume')
        until(lambda: bridge.status()['command_mux']['hardware_ready'])

        # Cancel a real running MoveJ through the unchanged emergency-stop API.
        post('/api/teleop/source', {'source': 'motion'})
        until(lambda: adapter.gate.emergency_stopped and
              not bridge.status()['command_mux']['hardware_ready'])
        post('/api/safety/resume')
        until(lambda: bridge.status()['command_mux']['hardware_ready'])
        before = positions['openarmx_left_joint4']
        move_args[move_args.index('--delta') + 1] = '.3'
        client = start_demo(move_args)
        until(lambda: positions['openarmx_left_joint4'] > before + .005, 15.)
        post('/api/safety/stop')
        until(lambda: client.poll() is not None, 10.)
        assert client.returncode != 0
        until(lambda: adapter.gate.emergency_stopped)
        pump(.1)
        held = positions['openarmx_left_joint4']
        pump(.2)
        assert positions['openarmx_left_joint4'] == pytest.approx(held)

        client = start_demo(['subscribe', '--duration', '1.5', '--period', '.3'])
        until(lambda: client.poll() is not None, 5.)
        assert client.returncode == 0
        text = (tmp_path / 'client.log').read_text()
        assert 'position_by_name' in text and '/observation/joint_states' in text
        (tmp_path / 'result.json').write_text(json.dumps({
            'real_motion_service': True, 'hardware_mode': 'fake', 'domain': 216,
            'move_j_succeeded': True, 'vr_disabled_during_motion': True,
            'stop_and_feedback_recovery_checked': True, 'subscribe_demo_received': True,
        }, indent=2))
    finally:
        stop_process(client)
        stop_process(motion)
        executor.shutdown()
        if adapter is not None:
            adapter.destroy_node()
        robot.destroy_node()
        if bridge is not None:
            bridge.stop()
        if rclpy.ok():
            rclpy.shutdown()
        motion_log.close()
        client_log.close()

"""Feedback stays independent from optional GUI debug rendering."""
import sys
import threading
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import Mock

import pytest

pytest.importorskip('rclpy')
pytest.importorskip('pybullet')
sys.path.insert(0, str(Path(__file__).parents[1] / 'simulation'))
from simulation import general_sim_robot_control_node_ros2 as simulation


def test_feedback_uses_one_batch_without_metadata_or_drawing(monkeypatch):
    from builtin_interfaces.msg import Time
    node = SimpleNamespace(
        _update_base_velocity=Mock(), publish_base_pose=Mock(),
        feedback_joint_names=['a', 'b'], feedback_joint_indices=[1, 4],
        robot_id=9, physics_clent_id=3, bullet_lock=threading.RLock(),
        get_clock=lambda: SimpleNamespace(now=lambda: SimpleNamespace(to_msg=lambda: Time())),
        joint_state_pub=Mock(), hardware_ready_pub=Mock(), hardware_ready=True,
        _draw_visualizations=Mock(),
    )
    batch = Mock(return_value=[(.1,), (.4,)])
    monkeypatch.setattr(simulation.p, 'getJointStates', batch)
    monkeypatch.setattr(simulation.p, 'getJointInfo', Mock(side_effect=AssertionError('metadata in feedback')))
    monkeypatch.setattr(simulation.p, 'getJointState', Mock(side_effect=AssertionError('individual joint RPC')))
    simulation.SimRobotController.update_joint_state(node)
    batch.assert_called_once_with(9, [1, 4], physicsClientId=3)
    message = node.joint_state_pub.publish.call_args.args[0]
    assert message.name == ['a', 'b']
    assert list(message.position) == [.1, .4]
    node._draw_visualizations.assert_not_called()


def test_target_callback_only_caches_optional_visualization(monkeypatch):
    monkeypatch.setattr(simulation, 'debug_draw_pose', Mock(side_effect=AssertionError('drawing in ROS callback')))
    poses = [object(), object()]
    node = SimpleNamespace(visualization_enabled=False, arms=[1, 2], target_ee_poses=None)
    simulation.SimRobotController.target_ee_callback(node, SimpleNamespace(poses=poses))
    assert node.target_ee_poses is None
    node.visualization_enabled = True
    simulation.SimRobotController.target_ee_callback(node, SimpleNamespace(poses=poses))
    assert node.target_ee_poses == poses


def test_remote_rendering_is_bounded_in_worker(monkeypatch):
    stop = threading.Event()
    calls = []
    node = SimpleNamespace(
        physics_stop=stop, bullet_lock=threading.RLock(), physics_clent_id=3,
        visualization_enabled=True, manual_rendering=True,
        _draw_visualizations=lambda: calls.append('draw'),
    )
    worker = []
    monkeypatch.setattr(simulation.threading, 'Thread', lambda target, **kw: SimpleNamespace(start=lambda: worker.append(target)))
    monkeypatch.setattr(simulation.rclpy, 'ok', lambda: True)
    monkeypatch.setattr(simulation.p, 'isConnected', lambda *a: True)
    def step(**kw):
        calls.append('step')
        if calls.count('step') == 3:
            stop.set()
    monkeypatch.setattr(simulation.p, 'stepSimulation', step)
    monkeypatch.setattr(simulation.p, 'configureDebugVisualizer', lambda *a, **kw: calls.append(('render', a, kw)))
    monkeypatch.setattr(simulation.time, 'monotonic', lambda: 10.0)
    simulation.SimRobotController._start_physics(node)
    worker[0]()
    assert calls.count('step') == 3
    assert calls.count('draw') == 1
    renders = [call for call in calls if isinstance(call, tuple)]
    assert len(renders) == 1
    assert renders[0] == ('render', (simulation.p.COV_ENABLE_SINGLE_STEP_RENDERING, 1), {'physicsClientId': 3})


def test_slow_x11_frame_leaves_time_for_control(monkeypatch):
    stop = threading.Event()
    clock = [10.0]
    steps, frames = [], []
    node = SimpleNamespace(
        physics_stop=stop, bullet_lock=threading.RLock(), physics_clent_id=3,
        visualization_enabled=False, manual_rendering=True,
    )
    worker = []
    monkeypatch.setattr(simulation.threading, 'Thread', lambda target, **kw: SimpleNamespace(start=lambda: worker.append(target)))
    monkeypatch.setattr(simulation.rclpy, 'ok', lambda: True)
    monkeypatch.setattr(simulation.p, 'isConnected', lambda *a: True)
    monkeypatch.setattr(simulation.time, 'monotonic', lambda: clock[0])

    def step(**kwargs):
        steps.append(clock[0])
        clock[0] += 0.01
        if len(steps) == 3:
            stop.set()

    def render(*args, **kwargs):
        frames.append(clock[0])
        clock[0] += 1.0  # A frame slower than the nominal display interval.

    monkeypatch.setattr(simulation.p, 'stepSimulation', step)
    monkeypatch.setattr(simulation.p, 'configureDebugVisualizer', render)
    simulation.SimRobotController._start_physics(node)
    worker[0]()
    assert len(steps) == 3
    assert len(frames) == 1

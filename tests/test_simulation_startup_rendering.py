"""Batch GUI initialization before restoring rendering; never start on failure."""
import sys
from pathlib import Path
from unittest.mock import Mock

import pytest

pytest.importorskip('rclpy')
pytest.importorskip('pybullet')
sys.path.insert(0, str(Path(__file__).parents[1] / 'simulation'))
from simulation import general_sim_robot_control_node_ros2 as simulation


@pytest.fixture
def startup(tmp_path, monkeypatch):
    config = tmp_path / 'robot.yaml'
    config.write_text('simulation: {}\n')
    events = []
    logger = Mock()
    monkeypatch.setattr(simulation.Node, '__init__', lambda *a, **k: None)
    monkeypatch.setattr(simulation.SimRobotController, 'get_logger', lambda self: logger)
    monkeypatch.setattr(simulation.SimRobotController, 'destroy_node', lambda self: events.append('destroy'))
    monkeypatch.setattr(simulation.p, 'connect', lambda mode, **k: events.append(('connect', mode)) or 7)
    monkeypatch.setattr(simulation.p, 'configureDebugVisualizer',
                        lambda option, value, **k: events.append(('render', value, k['physicsClientId'])))
    monkeypatch.setattr(simulation.p, 'isConnected', lambda **k: True)
    monkeypatch.setattr(simulation.p, 'disconnect', lambda **k: events.append(('disconnect', k['physicsClientId'])))
    monkeypatch.setattr(simulation.SimRobotController, '_initialize_robot',
                        lambda self, *a: events.append('initialize'))
    monkeypatch.setattr(simulation.SimRobotController, '_start_physics', lambda self: events.append('physics'))
    monkeypatch.setenv('DISPLAY', 'localhost:12.0')
    return config, events, logger


def test_initialization_finishes_before_rendering_and_physics_start(startup):
    config, events, logger = startup
    node = simulation.SimRobotController(str(config), headless=False)
    assert events == [('connect', simulation.p.GUI), ('render', 0, 7),
                      'initialize', ('render', 1, 7), 'physics']
    messages = [call.args[0] for call in logger.info.call_args_list]
    assert 'DISPLAY=localhost:12.0' in messages[0]
    assert any('connect finished in' in message for message in messages)
    assert any('ready in' in message for message in messages)
    assert node.physics_thread is None  # Mocked worker was never actually created.


def test_direct_initialization_keeps_same_order_without_gui(startup):
    config, events, logger = startup
    simulation.SimRobotController(str(config), headless=True)
    assert events[0] == ('connect', simulation.p.DIRECT)
    assert events[1:] == [('render', 0, 7), 'initialize', ('render', 1, 7), 'physics']


@pytest.mark.parametrize('stage', ['initialize', 'enable_rendering'])
def test_failed_initialization_disconnects_without_starting_thread(startup, monkeypatch, stage):
    config, events, logger = startup

    def fail_initialize(self, *args):
        events.append('initialize')
        raise RuntimeError('assembly failed')

    def fail_render(option, value, **kwargs):
        events.append(('render', value, kwargs['physicsClientId']))
        if value:
            raise RuntimeError('first frame failed')

    if stage == 'initialize':
        monkeypatch.setattr(simulation.SimRobotController, '_initialize_robot', fail_initialize)
    else:
        monkeypatch.setattr(simulation.p, 'configureDebugVisualizer', fail_render)
    node = object.__new__(simulation.SimRobotController)
    with pytest.raises(RuntimeError):
        node.__init__(str(config))
    assert events[-2:] == [('disconnect', 7), 'destroy']
    assert 'physics' not in events
    assert node.physics_stop.is_set()
    assert node.physics_thread is None
    assert not any('ready in' in call.args[0] for call in logger.info.call_args_list)


def test_partial_worker_start_failure_signals_joins_and_disconnects(startup, monkeypatch):
    config, events, logger = startup
    worker = Mock()
    worker.is_alive.return_value = True

    def failed_start(self):
        self.physics_thread = worker
        raise RuntimeError('worker startup failed')

    monkeypatch.setattr(simulation.SimRobotController, '_start_physics', failed_start)
    node = object.__new__(simulation.SimRobotController)
    with pytest.raises(RuntimeError, match='worker startup failed'):
        node.__init__(str(config))
    assert node.physics_stop.is_set()
    worker.join.assert_called_once_with(timeout=2)
    assert events[-2:] == [('disconnect', 7), 'destroy']

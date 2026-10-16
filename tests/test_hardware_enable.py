"""Hardware execution permission must be independent of VR input selection."""
import copy
from types import SimpleNamespace
from unittest.mock import Mock, patch

import pytest

pytest.importorskip('std_msgs.msg')
from rosidl_runtime_py.set_message import set_message_fields
from rosidl_runtime_py.utilities import get_message

from middleware.core.app import MiddlewareRuntime
from middleware.core.config import ConfigError, validate_config
from middleware.core.ros_bridge import RosBridge


class Wire:
    def __init__(self):
        self.messages = []
        self.publishers = {}
        self.node = Mock()
        self.node.create_publisher.side_effect = lambda _type, topic, _qos: SimpleNamespace(
            publish=lambda msg: self.messages.append((topic, copy.deepcopy(msg))))

    def drain(self, bridge):
        bridge._drain_commands(self.node, self.publishers, get_message, set_message_fields)

    def values(self, topic='/control/hardware/enabled'):
        return [msg.data for name, msg in self.messages if name == topic]


def bridge_for(source='vr'):
    return RosBridge({'enabled': True, 'command_mux': {'source': source}}, lambda *a: None)


@pytest.mark.parametrize('source', ['vr', 'motion', 'exoskeleton'])
def test_resume_enables_hardware_for_every_source_but_only_selected_vr(source):
    bridge, wire = bridge_for(source), Wire()
    runtime = SimpleNamespace(
        config={'safety': {}, 'ros': {'command_mux': {'source': source}}},
        ros=bridge, player=None, emit=Mock(), _safety_reason='stopped')
    MiddlewareRuntime._on_safety_resume(runtime, 'test resume')
    wire.drain(bridge)
    assert wire.values() == [True]
    assert wire.values('/control/teleop/enabled') == [source == 'vr']
    assert wire.values('/control/emergency_stop') == [False]
    wire.drain(bridge)
    assert wire.values() == [True]  # No periodic re-enable of a subsequently stopped adapter.


@pytest.mark.parametrize('revoke', ['source', 'stop', 'ready_loss'])
def test_queued_enable_cannot_survive_a_later_revocation(revoke):
    bridge, wire = bridge_for(), Wire()
    bridge.receive_hardware_ready(True)
    bridge.set_command_output_enabled(True)
    if revoke == 'source':
        bridge.set_control_source('motion')
    elif revoke == 'stop':
        bridge.emergency_stop('/control/emergency_stop', 'test')
    else:
        bridge.receive_hardware_ready(False)
    wire.drain(bridge)
    assert wire.values() == [False]
    assert bridge.status()['command_mux']['output_enabled'] is False


def test_switch_ready_timeout_and_replay_end_propagate_disable():
    bridge, wire = bridge_for(), Wire()
    bridge.set_control_source('motion')
    wire.drain(bridge)
    assert wire.values() == [False]
    bridge.receive_hardware_ready(True)
    bridge.set_command_output_enabled(True)
    wire.drain(bridge)
    assert wire.values()[-1] is True
    assert bridge.begin_replay()
    bridge.publish('/control/teleop/enabled', 'std_msgs/msg/Bool', {'data': False})
    wire.drain(bridge)
    assert wire.values() == [False, True]  # Pausing VR must not stop replay hardware.
    bridge.end_replay()
    wire.drain(bridge)
    assert wire.values()[-1] is False
    bridge.set_command_output_enabled(True)
    wire.drain(bridge)
    with patch('middleware.core.ros_bridge.time.monotonic',
               return_value=bridge._hardware_ready_last + 1):
        wire.drain(bridge)
    assert wire.values()[-1] is False


def test_older_queued_stop_does_not_overwrite_later_explicit_resume():
    bridge, wire = bridge_for('motion'), Wire()
    bridge.emergency_stop('/control/emergency_stop', 'test stop')
    bridge.set_command_output_enabled(True)
    wire.drain(bridge)
    assert wire.values('/control/emergency_stop') == [True]
    assert wire.values() == [True]
    assert bridge.status()['command_mux']['output_enabled'] is True


@pytest.mark.parametrize('topic', ['/control/teleop/enabled', '/control/emergency_stop'])
def test_config_cannot_alias_vr_or_stop_to_hardware_enable(topic):
    with pytest.raises(ConfigError, match='hardware_enable_topic'):
        validate_config({'ros': {'command_mux': {'hardware_enable_topic': topic}}})

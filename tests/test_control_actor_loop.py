"""Single sender/latest slots retain the original safety and ownership gates."""
import copy
import threading
import time
from types import SimpleNamespace

import pytest

pytest.importorskip('sensor_msgs.msg')
from rclpy.serialization import serialize_message
from rosidl_runtime_py.set_message import set_message_fields
from rosidl_runtime_py.utilities import get_message

from test_legacy_control import Rig, message
from test_prepared_publication import live_rig


def actor(rig=None):
    rig = live_rig() if rig is None else rig
    bridge = rig.bridge
    bridge._control_actor_mode = True
    bridge._actor_thread_ident = threading.get_ident()
    return rig


def arm_target(rig, offset=0., stamp=None):
    return message({name: rig.boundary.positions[name]+offset for name in rig.boundary.arms}, stamp)


def test_latest_slots_copy_original_and_keep_arms_hands_independent():
    rig = actor()
    bridge, boundary = rig.bridge, rig.boundary
    now = time.time_ns()
    originals = []
    for index in range(10):
        target = arm_target(rig, offset=index*.0001, stamp=now+index*1_000_000)
        originals.append(copy.deepcopy(target))
        assert bridge.forward_live_command('vr', target, boundary.publish_selected)
        target.position[0] += 2.  # producer mutation cannot change the slot
    for side, names in boundary.hands.items():
        target = message({n:boundary.positions[n] for n in names}, now+20_000_000)
        assert bridge.forward_live_command('vr', target, boundary.publish_selected)
    assert len(bridge._actor_targets) == 3
    assert not [m for topic,m in rig.wire if topic.startswith('/action/')]
    bridge._drain_actor_targets()
    sent = [m for topic,m in rig.wire if topic == '/action/joint_states']
    assert len(sent) == 1
    assert sent[0].header == originals[-1].header
    assert sent[0].position == originals[-1].position
    assert len([m for topic,m in rig.wire if topic.startswith('/action/')]) == 3
    assert bridge._actor_counts['coalesced'] == 9
    assert bridge._mux_received['vr'] == 12
    assert bridge._mux_channel_received == dict(arms=10, right_hand=1, left_hand=1, other=0)
    assert not bridge.forward_live_command('vr', originals[-1], boundary.publish_selected)


def test_unselected_sources_and_arbitrary_layouts_do_not_grow_mailbox():
    rig = actor()
    bridge = rig.bridge
    assert bridge.forward_live_command('vr', arm_target(rig), rig.boundary.publish_selected)
    for index in range(100):
        assert not bridge.forward_live_command('motion', message({'unknown_'+str(index):0.}), lambda _: None)
        assert not bridge.forward_live_command('arbitrary'+str(index), arm_target(rig), lambda _: None)
    assert len(bridge._actor_targets) == 1


@pytest.mark.parametrize('operation', ['stop', 'source', 'ready', 'replay', 'epoch'])
def test_revoke_clears_all_pending_and_does_not_catch_up_after_resume(operation):
    rig = actor()
    bridge = rig.bridge
    assert bridge.forward_live_command('vr', arm_target(rig), rig.boundary.publish_selected)
    if operation == 'stop':
        bridge.emergency_stop('/control/emergency_stop', 'test stop')
    elif operation == 'source':
        bridge.set_control_source('motion')
    elif operation == 'ready':
        bridge.receive_hardware_ready(False)
    elif operation == 'replay':
        assert bridge.begin_replay()
        bridge.end_replay()
    else:
        with bridge._lock:
            bridge._revoke_locked()
    rig.feed()
    bridge._command_source = 'vr'
    bridge.set_command_output_enabled(True)
    rig.wire.clear()
    bridge._drain_actor_targets()
    assert not [m for topic,m in rig.wire if topic.startswith('/action/')]
    assert bridge.forward_live_command('vr', arm_target(rig), rig.boundary.publish_selected)
    bridge._drain_actor_targets()
    assert len([m for topic,m in rig.wire if topic == '/action/joint_states']) == 1


@pytest.mark.parametrize('failure', ['stop', 'feedback', 'parent_exit'])
def test_checkpoint_after_prepare_revokes_before_actual_send(failure):
    rig = actor()
    bridge, boundary = rig.bridge, rig.boundary
    completed = []
    original = boundary.prepare_publication
    def prepare(*args):
        result = original(*args)
        completed.append(True)
        return result
    boundary.prepare_publication = prepare
    def pump():
        if not completed:
            return
        if failure == 'stop':
            bridge.emergency_stop('/control/emergency_stop', 'stop during prepare')
        elif failure == 'feedback':
            for name in boundary.feedback_order:
                boundary.received[name] = time.monotonic()-.251
        else:
            bridge._stop.set()
    bridge._actor_pump = pump
    assert bridge.forward_live_command('vr', arm_target(rig), boundary.publish_selected)
    bridge._drain_actor_targets()
    assert completed
    assert not [m for topic,m in rig.wire if topic == '/action/joint_states']
    assert not boundary.last_command_stamp


def test_manual_input_watchdog_runs_again_after_prepare_but_auto_does_not_need_headset():
    rig = actor(Rig())
    bridge, boundary, control = rig.bridge, rig.boundary, rig.control
    # AUTO retains original policy: invalid/disconnected headset alone is not a
    # command prerequisite, while required peer heartbeats and feedback remain.
    control.input_valid = False
    assert control.state == 'AUTO'
    target = arm_target(rig)
    assert bridge.forward_live_command('motion', target, boundary.publish_selected, control.token)
    bridge._drain_actor_targets()
    assert any(topic == '/action/joint_states' for topic,_ in rig.wire)
    control.input(device_instance='vr', sequence=2, valid=True, gripped=True, stamp_ns=time.time_ns())
    rig.advance(); rig.advance()
    assert control.state == 'MANUAL'
    rig.wire.clear()
    original = boundary.prepare_publication
    def prepare(*args):
        result = original(*args)
        control.input_time_ns = control.clock()-control.input_timeout_ns-1
        return result
    boundary.prepare_publication = prepare
    assert bridge.forward_live_command('vr', arm_target(rig), boundary.publish_selected, control.token)
    bridge._drain_actor_targets()
    assert control.state == 'FAULT'
    assert not [m for topic,m in rig.wire if topic == '/action/joint_states']


def test_replay_latest_slot_preserves_epoch_and_stop_wins_over_positive_queue():
    rig = actor()
    bridge = rig.bridge
    node = rig.boundary.node
    from rclpy.time import Time
    node.get_clock.return_value.now.side_effect = lambda: Time(nanoseconds=time.time_ns())
    assert bridge.begin_replay()
    for _ in range(20):
        assert bridge.publish_raw('/action/joint_states', 'sensor_msgs/msg/JointState', serialize_message(arm_target(rig)))
    assert len(bridge._actor_replay) == 1
    bridge.end_replay()
    bridge.set_command_output_enabled(True)
    assert bridge.begin_replay()
    assert not bridge._actor_replay
    bridge.publish('/control/emergency_stop', 'std_msgs/msg/Bool', {'data':False})
    bridge.emergency_stop('/control/emergency_stop', 'must win')
    rig.wire.clear()
    bridge._drain_commands(node, {}, get_message, set_message_fields)
    assert [(topic,m.data) for topic,m in rig.wire if topic == '/control/emergency_stop'] == [('/control/emergency_stop', True)]
    assert not [m for topic,m in rig.wire if topic == '/action/joint_states']


def test_host_fence_is_checked_after_generic_serialization():
    rig = actor()
    bridge, node = rig.bridge, rig.boundary.node
    generation = [0]
    bridge._actor_command_fence = lambda: (generation[0], bridge._output_epoch)
    bridge._actor_command_allowed = lambda fence,*_: fence == (generation[0], bridge._output_epoch)
    assert bridge.publish('/control/emergency_stop', 'std_msgs/msg/Bool', {'data':False})
    def slow_fields(msg, data):
        set_message_fields(msg, data)
        generation[0] += 1
    bridge._drain_commands(node, {}, get_message, slow_fields)
    assert not [m for topic,m in rig.wire if topic == '/control/emergency_stop']


def test_future_stamped_target_cannot_survive_revoke_resume_inside_pre_dispatch_pump():
    rig = actor()
    bridge = rig.bridge
    # A header 30 ms in the future is tolerated in the current epoch, but must
    # never authorize an old slot after an immediate stop/resume transaction.
    target = arm_target(rig, stamp=time.time_ns()+30_000_000)
    assert bridge.forward_live_command('vr', target, rig.boundary.publish_selected)
    calls = []
    def pump():
        calls.append(True)
        if len(calls) == 2:  # after slot removal, before the initial send gate
            bridge.set_command_output_enabled(False)
            bridge.set_command_output_enabled(True)
    bridge._actor_pump = pump
    bridge._drain_actor_targets()
    assert len(calls) >= 2
    assert not [m for topic,m in rig.wire if topic == '/action/joint_states']
    assert bridge._mux_rejection_reasons['vr:receipt_epoch_changed'] == 1


def test_a_second_thread_cannot_drain_actual_dds_commands():
    rig = actor()
    bridge = rig.bridge
    assert bridge.publish('/test/actor/noncontrol', 'std_msgs/msg/Bool', {'data':True})
    errors = []
    def other_thread():
        try:
            bridge._drain_commands(rig.boundary.node, {}, get_message, set_message_fields)
        except RuntimeError as error:
            errors.append(str(error))
    thread = threading.Thread(target=other_thread)
    thread.start(); thread.join(timeout=1.)
    assert errors == ['control DDS commands must be drained by the actor thread']
    assert not [m for topic,m in rig.wire if topic == '/test/actor/noncontrol']


def test_stop_set_rejects_old_positive_queue_even_when_its_epoch_still_matches():
    rig = actor()
    bridge = rig.bridge
    bridge.publish('/control/emergency_stop', 'std_msgs/msg/Bool', {'data':False})
    bridge._stop.set()
    bridge._drain_commands(rig.boundary.node, {}, get_message, set_message_fields)
    assert not [m for topic,m in rig.wire if topic == '/control/emergency_stop']


def test_manual_action_evidence_enters_fifo_outside_gate_before_release_close():
    rig = actor(Rig())
    bridge, control, boundary = rig.bridge, rig.control, rig.boundary
    control.input(device_instance='vr', sequence=2, valid=True, gripped=True, stamp_ns=time.time_ns())
    rig.advance(); rig.advance()
    assert control.state == 'MANUAL'
    order = []
    def sink(evidence):
        assert not bridge._lock._is_owned()
        order.append(('action', evidence['command_seq']))
    bridge._action_sink = sink
    control.segment_open = True
    control.close_segment = lambda *_: order.append(('close', None))
    assert bridge.forward_live_command('vr', arm_target(rig), boundary.publish_selected, control.token)
    bridge._drain_actor_targets()
    control.input(device_instance='vr', sequence=3, valid=True, gripped=False, stamp_ns=time.time_ns())
    assert order == [('action', 1), ('close', None)]


@pytest.mark.parametrize('actor_mode', [False, True])
def test_only_actor_dedicated_target_topics_use_depth_one(actor_mode):
    from unittest.mock import Mock, patch
    from middleware.core.legacy_control import LegacyTopicControl
    from rclpy.qos import HistoryPolicy, ReliabilityPolicy, qos_profile_sensor_data
    rig = Rig()
    bridge = rig.bridge
    bridge._control_actor_mode = actor_mode
    bridge._ownership_callback_group = None
    node = rig.boundary.node
    node.create_subscription.reset_mock()
    with patch('adapters.core.collision_safety.ModelCollisionGuard'):
        boundary = LegacyTopicControl(node, bridge, Mock())
    subscriptions = {call.args[1]: call.args[3] for call in node.create_subscription.call_args_list}
    target_topics = {'/hc_teleop/joint_cmd', '/hc_teleop/joint_cmd_finger_right',
        '/hc_teleop/joint_cmd_finger_left', '/control/vr/right_hand/joint_states',
        '/control/vr/left_hand/joint_states'}
    for topic in target_topics:
        qos = subscriptions[topic]
        assert qos.depth == (1 if actor_mode else qos_profile_sensor_data.depth)
        assert qos.reliability == ReliabilityPolicy.BEST_EFFORT
        assert qos.history == HistoryPolicy.KEEP_LAST
    for topic in ('/observation/joint_states', '/observation/hands/joint_states'):
        assert subscriptions[topic] is qos_profile_sensor_data
    node.create_subscription.reset_mock()
    bridge._setup_ownership_ros(node, {}, Mock(), Mock(), [])
    subscriptions = {call.args[1]: call.args[3] for call in node.create_subscription.call_args_list}
    assert subscriptions['/internal/control/joint_targets'] == 10
    boundary.close()

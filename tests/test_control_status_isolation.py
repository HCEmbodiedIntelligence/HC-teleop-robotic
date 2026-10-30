"""Control receipt must survive slow observation/status publication."""
import json
import threading
import time
from types import SimpleNamespace
from unittest.mock import Mock
from std_msgs.msg import Header

from test_legacy_control import Rig


def setup_callbacks(rig):
    bridge, node = rig.bridge, rig.boundary.node
    callbacks, timers, messages, publishers = {}, {}, [], {}
    telemetry = Mock()
    node.create_subscription.side_effect = lambda cls, topic, callback, *a, **kw: callbacks.setdefault(topic, callback)
    def timer(period, callback, **kwargs):
        timers[callback._hc_timing_name] = callback
        return Mock()
    node.create_timer.side_effect = telemetry.create_timer.side_effect = timer
    node.get_clock.return_value.now.side_effect = lambda: SimpleNamespace(
        nanoseconds=time.time_ns(), to_msg=lambda: Header().stamp)
    def publisher(cls, topic, qos):
        publishers[topic] = SimpleNamespace(publish=lambda msg: messages.append((topic, msg.data)))
        return publishers[topic]
    node.create_publisher.side_effect = publisher
    bridge._ownership_callback_group = None
    bridge._setup_ownership_ros(node, {}, Mock(), Mock(), [], status_node=telemetry)
    return callbacks, timers, messages, publishers


def input_frame(sequence, **overrides):
    data = dict(device_instance='vr', sequence=sequence, valid=True, gripped=False,
                stamp_ns=time.time_ns(), pause_requested=False, resume_requested=False)
    data.update(overrides)
    return SimpleNamespace(data=json.dumps(data))


def test_slow_status_publication_does_not_block_input_ack_or_immediate_pause():
    rig = Rig()
    callbacks, timers, messages, publishers = setup_callbacks(rig)
    entered, release = threading.Event(), threading.Event()
    # Block the transport itself, not a control callback or an actuator send.
    def slow_status(msg):
        entered.set()
        assert release.wait(1.5), 'test status release timed out'
        messages.append(('/status/control/ownership', msg.data))
    publishers['/status/control/ownership'].publish = slow_status
    worker = threading.Thread(target=timers['ownership_status'], daemon=True)
    worker.start()
    try:
        assert entered.wait(.5)
        start = time.monotonic()
        callbacks['/events/teleop/clutch'](input_frame(2))
        for peer in rig.control.required_peers:
            callbacks['/status/control/ack'](SimpleNamespace(data=json.dumps(dict(
                participant=peer, instance=peer, stamp_ns=time.time_ns(),
                token=rig.control.token.key(), phase='grant', success=True))))
        callbacks['/events/teleop/clutch'](input_frame(3, pause_requested=True))
        assert time.monotonic()-start < .2
        assert rig.control.input_diagnostics['last_sequence'] == 3
        assert rig.control.state == 'HOLD'
        assert not rig.bridge._command_output_enabled
        assert worker.is_alive(), 'status transport must still be blocked'
    finally:
        release.set()
        worker.join(2)
    timers['ownership_status']()
    states = [json.loads(data) for topic, data in messages if topic == '/status/control/ownership']
    events = [json.loads(data) for topic, data in messages if topic == '/events/control/intervention']
    assert states[-1]['state'] == 'HOLD'
    sequence = [event['sequence'] for event in events]
    assert sequence == sorted(set(sequence))
    assert sequence[-1] == rig.control.event_seq


def test_stable_tick_retains_feedback_guard():
    rig = Rig()
    _, timers, _, _ = setup_callbacks(rig)
    timers['ownership_tick']()
    assert rig.control.state == 'AUTO'
    rig.boundary.sample_ns[rig.boundary.feedback_order[0]] -= 300_000_000
    timers['ownership_tick']()
    assert rig.control.state == 'FAULT'
    assert rig.control.reason == 'necessary robot feedback lost'


def test_stable_tick_retains_hardware_ready_timeout():
    rig = Rig()
    _, timers, _, _ = setup_callbacks(rig)
    rig.bridge._hardware_ready_last -= rig.bridge._ready_timeout + .01
    timers['ownership_tick']()
    assert rig.control.state == 'FAULT'
    assert rig.control.reason == 'hardware ready timeout'
    assert not rig.bridge._command_output_enabled


def test_old_input_is_still_rejected_and_real_heartbeat_loss_still_faults():
    rig = Rig()
    callbacks, timers, _, _ = setup_callbacks(rig)
    callbacks['/events/teleop/clutch'](input_frame(2, stamp_ns=time.time_ns()-310_000_000))
    assert rig.control.input_diagnostics['last_rejection'] == 'sample_age_out_of_range'
    rig.control.peer_times['solver'] -= 310_000_000
    timers['ownership_tick']()
    assert rig.control.state == 'FAULT'
    assert rig.control.reason == 'solver acknowledgement heartbeat lost'

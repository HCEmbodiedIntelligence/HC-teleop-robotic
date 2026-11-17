"""Recording labels cannot grant control or hide actual publication gaps."""
import copy
import json
import time
from pathlib import Path

import pytest
from mcap.reader import make_reader
from rclpy.serialization import serialize_message, deserialize_message
from sensor_msgs.msg import JointState
from std_msgs.msg import String

from middleware.core.recording_action_labels import (
    RecordingActionLabels, ActionLabelBuffer, action_label_policy,
    ACTION_TOPIC, PUBLISHED_TOPIC, PROVENANCE_TOPIC, FEEDBACK_TOPIC, STATUS_TOPIC,
    OWNERSHIP_TOPIC, JOINT_TYPE,
)

BASE = 1_790_000_000_000_000_000
POLICY = dict(feedback_timeout_s=.25, status_timeout_s=.25)


def joint(topic, stamp, *, names=('right',), values=(.1,), age=0):
    msg = JointState(name=list(names), position=list(values))
    msg.header.stamp.sec, msg.header.stamp.nanosec = divmod(stamp-age, 10**9)
    msg.header.frame_id = 'original'
    return dict(topic=topic, msg_type=JOINT_TYPE, _raw=serialize_message(msg), stamp_ns=stamp)


def status(stamp, **changes):
    payload = dict(stamp_ns=stamp, mode='ik_rearming', enabled=True, feedback_fresh=True,
        stop_reason='', control_source='vr', source_feedback_pending=False,
        generic_controller=dict(hardware_ready=True, rearm_state='wait_press'),
        collision_avoidance=dict(blocked=False))
    payload.update(changes)
    return dict(topic=STATUS_TOPIC, msg_type='std_msgs/msg/String',
                stamp_ns=stamp, _raw=serialize_message(String(data=json.dumps(payload))))


def owner(stamp, *, allowed=True, state='OFF', epoch=5):
    payload = dict(state=state, recording_output=dict(allowed=allowed, source='vr',
                   output_epoch=epoch, stamp_ns=stamp))
    return dict(topic=OWNERSHIP_TOPIC, msg_type='std_msgs/msg/String', stamp_ns=stamp,
                _raw=serialize_message(String(data=json.dumps(payload))))


def policy_ready():
    labels = RecordingActionLabels(**POLICY)
    labels.process(joint(ACTION_TOPIC, BASE))
    labels.process(status(BASE))
    labels.process(owner(BASE))
    return labels


def test_feedback_label_is_measured_position_only_and_does_not_mutate_source():
    labels = policy_ready()
    event = joint(FEEDBACK_TOPIC, BASE+10_000_000,
                  names=('disabled_left', 'right'), values=(2., .3), age=2_000_000)
    original = copy.deepcopy(event)
    result = labels.process(event, output_allowed=labels.output_allowed(event['stamp_ns']))
    assert event == original
    action = next(row for row in result if row['topic'] == ACTION_TOPIC)
    decoded = deserialize_message(action['_raw'], JointState)
    assert decoded.name == ['right'] and list(decoded.position) == [.3]
    assert not decoded.velocity and not decoded.effort
    assert decoded.header == deserialize_message(event['_raw'], JointState).header
    assert action['stamp_ns'] == BASE+8_000_000
    provenance = result[-1]['payload']
    assert provenance['source'] == 'feedback_hold'
    assert provenance['sent_to_robot'] is False and provenance['publication_ns'] is None
    assert provenance['feedback_receive_ns'] == event['stamp_ns']
    assert provenance['output_epoch'] == 5


def test_unmeasured_simulation_fields_and_nan_placeholders_are_not_filled():
    labels = RecordingActionLabels(**POLICY, eligible_joint_names=('right', 'disabled_left'))
    labels.process(joint(ACTION_TOPIC, BASE, names=('right', 'R_ban', 'disabled_left'),
                         values=(.1, 0., float('nan'))))
    labels.process(status(BASE)); labels.process(owner(BASE))
    event = joint(FEEDBACK_TOPIC, BASE+10_000_000, names=('right',), values=(.4,))
    output = labels.process(event, output_allowed=True)
    action = next(row for row in output if row['topic'] == ACTION_TOPIC)
    decoded = deserialize_message(action['_raw'], JointState)
    assert decoded.name == ['right'] and list(decoded.position) == [.4]


@pytest.mark.parametrize('changes', [
    dict(enabled=False), dict(feedback_fresh=False), dict(stop_reason='FAULT'),
    dict(mode='homing'), dict(mode='arms_grippers'), dict(mode='hold'),
    dict(control_source='motion'), dict(source_feedback_pending=True),
    dict(stamp_ns=BASE-300_000_000),
    dict(collision_avoidance=dict(blocked=True)),
    dict(generic_controller=dict(hardware_ready=False, rearm_state='wait_press')),
    dict(generic_controller=dict(hardware_ready=True, rearm_state='wait_feedback')),
])
def test_only_normal_grip_rearm_can_generate_feedback_labels(changes):
    labels = policy_ready()
    labels.process(status(BASE+1, **changes))
    event = joint(FEEDBACK_TOPIC, BASE+10_000_000)
    assert labels.process(event, output_allowed=True) == [event]


@pytest.mark.parametrize('case', ['default_closed', 'fault', 'estop', 'closed', 'stale_guard',
                                'stale_status', 'stale_feedback', 'future_feedback',
                                'missing_joint', 'nan', 'duplicate_name', 'repeated_feedback', 'unknown_layout'])
def test_fail_closed_or_bad_feedback_never_creates_action(case):
    labels = policy_ready()
    event = joint(FEEDBACK_TOPIC, BASE+10_000_000)
    allowed = True
    if case in {'fault', 'estop', 'closed'}:
        labels.process(owner(BASE+1, allowed=case != 'closed',
                             state={'fault': 'FAULT', 'estop': 'ESTOP'}.get(case, 'OFF')))
    elif case == 'stale_guard':
        labels.process(status(BASE+300_000_000)); event = joint(FEEDBACK_TOPIC, BASE+310_000_000)
    elif case == 'stale_status':
        labels.process(owner(BASE+300_000_000)); event = joint(FEEDBACK_TOPIC, BASE+310_000_000)
    elif case == 'stale_feedback': event = joint(FEEDBACK_TOPIC, BASE+10_000_000, age=250_000_001)
    elif case == 'future_feedback': event = joint(FEEDBACK_TOPIC, BASE+10_000_000, age=-1)
    elif case == 'missing_joint': event = joint(FEEDBACK_TOPIC, BASE+10_000_000, names=('left',))
    elif case == 'nan': event = joint(FEEDBACK_TOPIC, BASE+10_000_000, values=(float('nan'),))
    elif case == 'duplicate_name':
        event = joint(FEEDBACK_TOPIC, BASE+10_000_000, names=('right', 'right'), values=(.1, .2))
    elif case == 'repeated_feedback': labels.last_feedback_ns = event['stamp_ns']
    elif case == 'unknown_layout': labels.names = ()
    if case == 'default_closed':
        assert labels.process(event) == [event]
    else:
        allowed = labels.output_allowed(event['stamp_ns'])
        assert labels.process(event, output_allowed=allowed) == [event]


def test_exact_publication_evidence_survives_rearm_label_replacement():
    labels = policy_ready()
    event = joint(ACTION_TOPIC, BASE+5_000_000)
    result = labels.process(event, output_allowed=True)
    assert all(row['topic'] != ACTION_TOPIC for row in result)
    raw = next(row for row in result if row['topic'] == PUBLISHED_TOPIC)
    assert raw['_raw'] == event['_raw'] and raw['stamp_ns'] == event['stamp_ns']
    assert result[-1]['payload']['included_in_action_labels'] is False
    labels.process(status(BASE+6_000_000, mode='arms_grippers'))
    assert labels.process(event, output_allowed=True)[0] == event


def test_delayed_phase_messages_are_reordered_but_future_state_never_backfills():
    buffer = ActionLabelBuffer(POLICY)
    for event in [joint(FEEDBACK_TOPIC, BASE+20_000_000), status(BASE+5_000_000),
                  owner(BASE+5_000_000), joint(ACTION_TOPIC, BASE)]:
        buffer.offer(event)
    assert list(buffer.ready(BASE+49_000_000)) == []
    result = list(buffer.ready(BASE+100_000_000))
    assert len([row for row in result if row.get('payload', {}).get('source') == 'feedback_hold']) == 1
    late = joint(FEEDBACK_TOPIC, BASE+1_000_000)
    buffer.offer(late)
    assert list(buffer.ready(BASE+100_000_000)) == [late]


def test_buffer_pressure_flushes_evidence_without_dropping():
    buffer = ActionLabelBuffer(POLICY, max_events=2)
    events = [joint(ACTION_TOPIC, BASE+i) for i in range(4)]
    for event in events: buffer.offer(event)
    first = list(buffer.ready(BASE))
    last = list(buffer.ready(BASE, final=True))
    evidence = [row for row in first+last if row['topic'] == PUBLISHED_TOPIC]
    assert [row['_raw'] for row in evidence] == [row['_raw'] for row in events]
    assert buffer.pressure_flushes == 2


def test_actual_mcap_separates_label_clock_and_exact_publication_evidence(tmp_path):
    from middleware.core.topic_recorder import TopicRecorder
    topics = [dict(topic=topic, type=kind) for topic, kind in [
        (ACTION_TOPIC, JOINT_TYPE), (FEEDBACK_TOPIC, JOINT_TYPE),
        (STATUS_TOPIC, 'std_msgs/msg/String'), (OWNERSHIP_TOPIC, 'std_msgs/msg/String')]]
    recorder = TopicRecorder(dict(directory=str(tmp_path)), tmp_path, topics=topics)
    recorder.action_label_policy = POLICY
    now = time.time_ns()
    publications = [joint(ACTION_TOPIC, now), joint(ACTION_TOPIC, now+30_000_000)]
    try:
        path = Path(recorder.start('labels', {'profile_id': 'x1'}))
        for event in [publications[0], owner(now+1), status(now+2),
                      joint(FEEDBACK_TOPIC, now+10_000_000), publications[1],
                      joint(FEEDBACK_TOPIC, now+40_000_000)]:
            assert recorder.record(event)
        result = recorder.stop()
        assert result['error'] is None
        with path.open('rb') as stream:
            reader = make_reader(stream, validate_crcs=True)
            metadata = {m.name: m.metadata for m in reader.iter_metadata()}
            rows = [(c.topic, m.data, m.log_time) for _, c, m in reader.iter_messages()]
        real = [(raw, stamp) for topic, raw, stamp in rows if topic == PUBLISHED_TOPIC]
        assert real == [(row['_raw'], row['stamp_ns']) for row in publications]
        labels = [stamp for topic, _, stamp in rows if topic == ACTION_TOPIC]
        assert labels == [now, now+10_000_000, now+40_000_000]
        provenance = [json.loads(raw) for topic, raw, _ in rows if topic == PROVENANCE_TOPIC]
        assert len(provenance) == 4
        assert metadata['hc_teleop.capture_window']['action_at_publication'] == 'false'
        assert metadata['hc_teleop.capture_window']['real_publication_topic'] == PUBLISHED_TOPIC
        assert json.loads(metadata['hc_teleop.action_labels']['json'])['feedback_hold_frames'] == 2
    finally:
        recorder.shutdown()


def test_page_selection_and_existing_feedback_threshold_are_respected():
    from middleware.core.ros_recording_executor import recording_subscriptions
    config = dict(legacy_control_profile=str(Path(__file__).resolve().parents[1]/'adapters/robots/x1'), subscriptions=[
        dict(topic=topic, type=kind, outputs=['record'], max_hz=10 if topic == STATUS_TOPIC else 0)
        for topic, kind in [(ACTION_TOPIC, JOINT_TYPE), (FEEDBACK_TOPIC, JOINT_TYPE),
                            (STATUS_TOPIC, 'std_msgs/msg/String')]])
    policy = action_label_policy(config)
    assert policy['feedback_timeout_s'] == policy['status_timeout_s'] == .25
    assert 'Joint1_R' in policy['eligible_joint_names']
    assert 'R_ban' not in policy['eligible_joint_names']
    assert OWNERSHIP_TOPIC in [row['topic'] for row in recording_subscriptions(config)]
    config['subscriptions'][1]['enabled'] = False
    assert action_label_policy(config) is None
    assert OWNERSHIP_TOPIC not in [row['topic'] for row in recording_subscriptions(config)]


def test_drive_replay_uses_real_commands_and_excludes_feedback_labels(tmp_path):
    from mcap.writer import Writer
    from middleware.core.topic_recorder import _get_msg_def
    from middleware.core.topic_player import TopicPlayer
    from unittest.mock import Mock
    path = tmp_path/'drive.mcap'
    with path.open('wb') as stream:
        writer = Writer(stream); writer.start(profile='ros2')
        writer.add_metadata('hc_teleop.session', {'profile_id': 'x1'})
        writer.add_metadata('hc_teleop.record_only', {'topic': ACTION_TOPIC})
        sid = writer.register_schema(JOINT_TYPE, 'ros2msg', _get_msg_def(JOINT_TYPE))
        for topic, value in [(ACTION_TOPIC, .8), (PUBLISHED_TOPIC, .2)]:
            channel = writer.register_channel(topic, 'cdr', sid)
            raw = joint(topic, BASE, values=(value,))['_raw']
            writer.add_message(channel, BASE, raw, publish_time=BASE)
        writer.finish()
    ros = Mock()
    player = TopicPlayer(tmp_path, lambda: ros, active_profile_provider=lambda: 'x1')
    try:
        player.play(path.name, mode='drive')
        deadline = time.monotonic()+2
        while player.status()['state'] == 'playing' and time.monotonic() < deadline:
            time.sleep(.01)
        calls = ros.publish_raw.call_args_list
        assert len(calls) == 1
        assert calls[0].args[0] == ACTION_TOPIC
        assert list(deserialize_message(calls[0].args[2], JointState).position) == [.2]
    finally:
        player.stop()


def test_monotonic_action_output_rejects_backdated_feedback_hold_and_late_action():
    labels = policy_ready()
    # Emit a published action onto ACTION_TOPIC before delayed rearm status arrives.
    labels.process(status(BASE + 1_000_000, mode='arms_grippers'))
    pub_res = labels.process(joint(ACTION_TOPIC, BASE + 20_000_000), output_allowed=True)
    assert any(row['topic'] == ACTION_TOPIC for row in pub_res)

    # Rearm status arrives covering earlier timestamps; feedback earlier than +20ms
    # must not jump ACTION_TOPIC backward in time.
    labels.process(status(BASE + 25_000_000))
    early_fb = joint(FEEDBACK_TOPIC, BASE + 26_000_000, age=10_000_000)  # source=BASE+16ms < 20ms
    assert labels.process(early_fb, output_allowed=True) == [early_fb]
    assert labels.rejected_feedback == 1

    # Feedback strictly newer than last_action_output_ns emits normally.
    fresh_fb = joint(FEEDBACK_TOPIC, BASE + 30_000_000)  # source=BASE+30ms > 20ms
    fresh_res = labels.process(fresh_fb, output_allowed=True)
    assert any(row['topic'] == ACTION_TOPIC for row in fresh_res)

    # If a delayed ACTION_TOPIC publication later arrives with <= 30ms during normal mode,
    # PUBLISHED_TOPIC still keeps it while ACTION_TOPIC stays monotonic.
    labels.process(status(BASE + 31_000_000, mode='arms_grippers'))
    late_pub = labels.process(joint(ACTION_TOPIC, BASE + 28_000_000), output_allowed=True)
    assert all(row['topic'] != ACTION_TOPIC for row in late_pub)
    assert any(row['topic'] == PUBLISHED_TOPIC for row in late_pub)


def test_action_label_buffer_absorbs_150ms_producer_skew_without_gap():
    buffer = ActionLabelBuffer(POLICY, delay_ns=150_000_000, max_events=2048)
    # Simulate dds_small arriving ahead of PublicationCapture's preroll burst by ~120ms.
    for event in [
        joint(ACTION_TOPIC, BASE),
        status(BASE + 10_000_000),
        owner(BASE + 10_000_000),
        joint(FEEDBACK_TOPIC, BASE + 120_000_000),
    ]:
        buffer.offer(event)
    # At +200ms wall time, +120ms is held (< 150ms), so late preroll actions still order before it.
    early_flushed = list(buffer.ready(BASE + 200_000_000))
    assert all(row.get('stamp_ns', 0) <= BASE + 50_000_000 for row in early_flushed)
    buffer.offer(joint(FEEDBACK_TOPIC, BASE + 30_000_000))
    buffer.offer(joint(ACTION_TOPIC, BASE + 40_000_000))
    final_flushed = early_flushed + list(buffer.ready(BASE + 400_000_000, final=True))
    action_stamps = [row['stamp_ns'] for row in final_flushed if row['topic'] == ACTION_TOPIC]
    assert action_stamps == sorted(action_stamps)
    assert BASE + 30_000_000 in action_stamps
    assert BASE + 120_000_000 in action_stamps


def test_action_label_buffer_holds_ahead_stream_within_skew_window():
    buffer = ActionLabelBuffer(POLICY, delay_ns=150_000_000, skew_wait_ns=500_000_000, max_events=4096)
    # Direction 1: dds_small delivers +230ms while published_action is only at +10ms.
    for event in [
        joint(ACTION_TOPIC, BASE + 10_000_000),
        status(BASE + 20_000_000),
        owner(BASE + 20_000_000),
        joint(FEEDBACK_TOPIC, BASE + 230_000_000),
    ]:
        buffer.offer(event)
    # Only +10ms is at or below min(10ms, 230ms); +20ms/+230ms wait up to 500ms for lagging published_action.
    flushed_init = list(buffer.ready(BASE + 400_000_000))
    assert not any(
        row.get('topic') == ACTION_TOPIC and row.get('stamp_ns', 0) > BASE + 10_000_000
        for row in flushed_init
    )
    assert not any(
        row.get('topic') == PROVENANCE_TOPIC
        and row.get('payload', {}).get('source') == 'feedback_hold'
        for row in flushed_init
    )

    # Delayed published_action & intermediate feedback arrive before 500ms skew limit.
    buffer.offer(joint(ACTION_TOPIC, BASE + 30_000_000))
    buffer.offer(joint(FEEDBACK_TOPIC, BASE + 40_000_000))

    # Direction 2: published_action jumps to +930ms while dds_small is lagging at +650ms.
    buffer.offer(joint(FEEDBACK_TOPIC, BASE + 650_000_000))
    buffer.offer(joint(ACTION_TOPIC, BASE + 800_000_000))
    buffer.offer(joint(ACTION_TOPIC, BASE + 930_000_000))
    # At +1000ms (200ms after +800ms > 150ms, < 500ms), +800ms waits for lagging dds_small.
    early = list(buffer.ready(BASE + 1_000_000_000))
    early_actions = [row['stamp_ns'] for row in early if row['topic'] == ACTION_TOPIC]
    assert BASE + 40_000_000 in early_actions
    assert BASE + 800_000_000 not in early_actions

    buffer.offer(status(BASE + 780_000_000))
    buffer.offer(owner(BASE + 780_000_000))
    buffer.offer(joint(FEEDBACK_TOPIC, BASE + 790_000_000))
    buffer.offer(joint(FEEDBACK_TOPIC, BASE + 940_000_000))
    flushed = early + list(buffer.ready(BASE + 1_200_000_000, final=True))
    action_stamps = [row['stamp_ns'] for row in flushed if row['topic'] == ACTION_TOPIC]
    assert action_stamps == sorted(action_stamps)
    assert BASE + 790_000_000 in action_stamps
    assert BASE + 940_000_000 in action_stamps

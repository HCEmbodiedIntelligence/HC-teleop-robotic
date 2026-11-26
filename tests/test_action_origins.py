import json

import pytest

from middleware.core.action_origins import ActionOriginIndex, append_origin_range
from middleware.core.recording_cdr import write_string


def add(index, log_time, source, header=None, *, encoding='json', **extra):
    published = source == 'published_action'
    marker = dict(schema_version='hc-action-provenance/v1', source=source, action_log_time_ns=log_time,
                  source_stamp_ns=header, sent_to_robot=published,
                  publication_ns=log_time if published else None, included_in_action_labels=True)
    marker.update(extra)
    payload = json.dumps(marker)
    if encoding == 'json':
        index.add_event(log_time, payload.encode(), encoding, 'teleop_event')
    else:
        index.add_event(log_time, write_string(payload), encoding, 'std_msgs/msg/String')


@pytest.mark.parametrize('encoding', ['json', 'cdr'])
@pytest.mark.parametrize('clock', ['log_time', 'header'])
def test_mixed_label_sources_match_selected_clock(encoding, clock):
    idx = ActionOriginIndex()
    add(idx, 110, 'published_action', 100, encoding=encoding)
    add(idx, 220, 'feedback_hold', 210, encoding=encoding)
    add(idx, 220, 'published_action', 200, encoding=encoding, included_in_action_labels=False)
    add(idx, 330, 'published_action', 320, encoding=encoding, included_in_action_labels=False)
    # Numerical identity is deliberately uninformative for all three labels.
    actions = [(t, [1.0]) for t in (110, 220, 330)]
    stamps = [100, 210, 320] if clock == 'header' else [110, 220, 330]
    origins = idx.align(actions, [100, 210, 320], clock)
    ranges = []
    for i, stamp in enumerate([None, stamps[0], stamps[0], stamps[1], stamps[2]]):
        append_origin_range(ranges, i, None if stamp is None else (stamp, [1.0]), origins)
    assert [(r['start_frame'], r['end_frame_exclusive'], r['source']) for r in ranges] == [
        (0, 1, 'missing'), (1, 3, 'published_action'), (3, 4, 'feedback_hold'), (4, 5, 'unverified')]


def test_conflicting_or_unmatchable_evidence_stays_unverified():
    idx = ActionOriginIndex()
    add(idx, 10, 'published_action', 1)
    add(idx, 10, 'feedback_hold', 1)
    add(idx, 20, 'feedback_hold', 2)
    add(idx, 30, 'published_action', 300)
    add(idx, 40, 'feedback_hold', 4, sent_to_robot=True)
    idx.add_event(50, b'{bad', 'json', 'teleop_event')
    actions = [(t, [0.0]) for t in (10, 20, 20, 30, 40, 50)]
    assert idx.align(actions, [1, 2, 3, 4, 5, 6], 'header') == {}



def test_ordinary_x1_export_tracks_selected_action_through_retiming(tmp_path):
    from mcap.writer import Writer
    from rclpy.serialization import serialize_message
    from sensor_msgs.msg import JointState
    from middleware.core.lerobot_export import prepare_mcap
    from middleware.core.recording_action_labels import ACTION_TOPIC, FEEDBACK_TOPIC, PROVENANCE_TOPIC
    from middleware.core.x1_recording import X1_BODY_JOINTS

    mcap = tmp_path/'source.mcap'
    start = 1_000_000_000
    with mcap.open('wb') as output:
        writer = Writer(output)
        writer.start(profile='ros2')
        writer.add_metadata('hc_teleop.session', {'profile_id': 'x1'})
        joint = writer.register_schema('sensor_msgs/msg/JointState', 'ros2msg', b'')
        state = writer.register_channel(FEEDBACK_TOPIC, 'cdr', joint)
        action = writer.register_channel(ACTION_TOPIC, 'cdr', joint)
        event = writer.register_schema('teleop_event', 'jsonschema', b'{}')
        marker = writer.register_channel(PROVENANCE_TOPIC, 'json', event)

        def joints(channel, received_ms, header_ms):
            msg = JointState(name=list(X1_BODY_JOINTS), position=[0.3]*len(X1_BODY_JOINTS))
            msg.header.stamp.sec = 1
            msg.header.stamp.nanosec = header_ms*1_000_000
            stamp = start + received_ms*1_000_000
            writer.add_message(channel, stamp, serialize_message(msg), stamp)

        for i in range(4):
            joints(state, 100*i, 100*i)
        for received, header, origin in ((110, 80, 'published_action'), (200, 200, 'feedback_hold'),
                                         (300, 300, 'unverified')):
            joints(action, received, header)
            if origin != 'unverified':
                stamp = start+received*1_000_000
                public = origin == 'published_action'
                evidence = dict(schema_version='hc-action-provenance/v1', source=origin,
                                publication_ns=stamp if public else None, sent_to_robot=public,
                                source_stamp_ns=start+header*1_000_000, action_log_time_ns=stamp,
                                included_in_action_labels=True)
                writer.add_message(marker, stamp, json.dumps(evidence).encode(), stamp)
        writer.finish()
    stage = tmp_path/'staging'
    stage.mkdir()
    manifest = json.loads(prepare_mcap(mcap, stage, 10, 'origins', 'x1').read_text())
    ranges = manifest['sync_audit']['primary_action_origin_ranges']
    assert ranges == [[dict(start_frame=i, end_frame_exclusive=i+1, source=source)
                       for i, source in enumerate(['missing', 'published_action', 'feedback_hold', 'unverified'])]]

"""Regression checks for the operator's real recording acceptance criteria."""
import struct
import json

from mcap.writer import Writer

from tools.diagnostics.audit_mcap_timing import ACTION, COLOR, DEPTH, TOPICS, VR_TARGET, audit

BASE = 1_790_000_000_000_000_000


def recording(path, *, delayed=False, missing_depth=False, action_gap=False, boundary=True, vr_target=False):
    with path.open('wb') as stream:
        writer = Writer(stream)
        writer.start(profile='ros2')
        schemas = {kind: writer.register_schema(kind, 'ros2msg', b'') for kind in (
            'sensor_msgs/msg/JointState', 'sensor_msgs/msg/CompressedImage')}
        if boundary:
            writer.add_metadata('hc_teleop.capture_window', {
                'schema_version': 'hc-capture-window/v1', 'action_at_publication': 'true',
                'window_start_ns': str(BASE), 'admission_start_ns': str(BASE),
                'stop_requested_ns': str(BASE+100_000_000),
                'raw_ingress_closed_ns': str(BASE+350_000_000)})
        for topic in TOPICS+((VR_TARGET,) if vr_target else ()):
            kind = 'sensor_msgs/msg/CompressedImage' if topic in (COLOR, DEPTH) else 'sensor_msgs/msg/JointState'
            channel = writer.register_channel(topic, 'cdr', schemas[kind])
            for offset in (0, 33_000_000, 66_000_000, 99_000_000):
                if missing_depth and topic == DEPTH and offset == 33_000_000:
                    continue
                stamp = BASE+offset
                sec, ns = divmod(stamp, 10**9)
                raw = b'\x00\x01\x00\x00'+struct.pack('<iI', sec, ns)
                reception = stamp
                if delayed and topic in (COLOR, DEPTH) and offset == 33_000_000:
                    reception += 70_000_000
                if action_gap and topic == ACTION and offset == 99_000_000:
                    reception += 50_000_000
                writer.add_message(channel, reception, raw, publish_time=reception)
        writer.finish()


def test_continuous_source_with_late_arrival_uses_header_for_rgbd(tmp_path):
    path = tmp_path/'late.mcap'
    recording(path, delayed=True)
    result = audit(path)
    assert result['result'] == 'PASS'
    assert result['topics'][COLOR]['log']['gaps_gt_threshold'] == 1
    assert result['topics'][COLOR]['header']['gaps_gt_threshold'] == 0
    assert result['rgbd']['paired_exact'] == 4


def test_missing_depth_is_not_hidden_by_ordinal_pairing(tmp_path):
    path = tmp_path/'missing.mcap'
    recording(path, missing_depth=True)
    result = audit(path)
    assert result['result'] == 'FAIL'
    assert result['rgbd']['paired_exact'] == 3
    assert result['rgbd']['color_without_depth'] == 1
    assert result['rgbd']['color_unmatched_ns'] == [BASE+33_000_000]
    assert result['topics'][DEPTH]['header']['max_gap_ms'] == 66


def test_action_publication_gap_fails_even_with_continuous_header(tmp_path):
    path = tmp_path/'action.mcap'
    recording(path, action_gap=True)
    result = audit(path)
    assert result['result'] == 'FAIL'
    assert result['topics'][ACTION]['log']['max_gap_ms'] == 83
    assert result['topics'][ACTION]['header']['max_gap_ms'] == 33
    assert any('publication/reception gaps' in f for f in result['failures'])


def test_missing_precise_boundary_never_claims_pass(tmp_path):
    path = tmp_path/'legacy.mcap'
    recording(path, boundary=False)
    result = audit(path)
    assert result['result'] == 'UNVERIFIED'
    assert not result['failures']
    assert any('admission' in issue for issue in result['unverified'])


def test_optional_vr_stage_preserves_original_header_correspondence(tmp_path):
    path = tmp_path/'stages.mcap'
    recording(path, vr_target=True)
    result = audit(path)
    assert result['result'] == 'PASS'
    assert result['control_pipeline']['vr_target_available'] is True
    assert result['control_pipeline']['exact_vr_action_header_matches'] == 4
    assert result['control_pipeline']['vr_headers_without_action'] == 0
    assert result['control_pipeline']['action_headers_without_vr'] == 0


def test_older_recording_without_optional_vr_stage_is_explicitly_unavailable(tmp_path):
    path = tmp_path/'older.mcap'
    recording(path)
    result = audit(path)
    assert result['result'] == 'PASS'
    assert result['control_pipeline']['vr_target_available'] is False


def test_filled_labels_cannot_hide_actual_publication_gap(tmp_path):
    from tools.diagnostics.audit_mcap_timing import PUBLISHED_ACTION, ACTION_PROVENANCE
    path = tmp_path/'labels.mcap'
    recording(path, vr_target=True)
    # Build a complete fixture with continuous labels and sparse real sends.
    with path.open('wb') as stream:
        writer = Writer(stream); writer.start(profile='ros2')
        writer.add_metadata('hc_teleop.action_labels', {'json': '{}'})
        writer.add_metadata('hc_teleop.capture_window', {
            'schema_version': 'hc-capture-window/v1', 'action_at_publication': 'false',
            'real_publication_topic': PUBLISHED_ACTION, 'real_action_at_publication': 'true',
            'window_start_ns': str(BASE), 'admission_start_ns': str(BASE),
            'stop_requested_ns': str(BASE+100_000_000), 'raw_ingress_closed_ns': str(BASE+350_000_000)})
        schema = writer.register_schema('sensor_msgs/msg/JointState', 'ros2msg', b'')
        prov = writer.register_channel(ACTION_PROVENANCE, 'json', 0)
        for topic in TOPICS+(PUBLISHED_ACTION,):
            kind = 'sensor_msgs/msg/CompressedImage' if topic in (COLOR, DEPTH) else 'sensor_msgs/msg/JointState'
            sid = writer.register_schema(kind, 'ros2msg', b'') if kind != 'sensor_msgs/msg/JointState' else schema
            channel = writer.register_channel(topic, 'cdr', sid)
            offsets = (0, 99_000_000) if topic == PUBLISHED_ACTION else (0, 33_000_000, 66_000_000, 99_000_000)
            for offset in offsets:
                stamp = BASE+offset
                data = b'\x00\x01\x00\x00'+struct.pack('<iI', *divmod(stamp, 10**9))
                writer.add_message(channel, stamp, data, publish_time=stamp)
                if topic == ACTION:
                    real = offset in (0, 99_000_000)
                    payload = dict(schema_version='hc-action-provenance/v1',
                        source='published_action' if real else 'feedback_hold',
                        action_log_time_ns=stamp, publication_ns=stamp if real else None,
                        sent_to_robot=real, included_in_action_labels=True)
                    writer.add_message(prov, stamp, json.dumps(payload).encode(), publish_time=stamp)
        writer.finish()
    result = audit(path)
    assert result['topics'][ACTION]['log']['max_gap_ms'] == 33
    assert result['real_publication']['log']['max_gap_ms'] == 99
    assert result['label_provenance']['sources'] == dict(published_action=2, feedback_hold=2)
    assert result['label_provenance']['unmarked_action_frames'] == 0
    assert any('real publication gaps' in failure for failure in result['failures'])
    assert result['result'] == 'FAIL'

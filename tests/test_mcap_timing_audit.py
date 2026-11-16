"""Regression checks for the operator's real recording acceptance criteria."""
import struct

from mcap.writer import Writer

from tools.diagnostics.audit_mcap_timing import ACTION, COLOR, DEPTH, TOPICS, audit

BASE = 1_790_000_000_000_000_000


def recording(path, *, delayed=False, missing_depth=False, action_gap=False, boundary=True):
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
        for topic in TOPICS:
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

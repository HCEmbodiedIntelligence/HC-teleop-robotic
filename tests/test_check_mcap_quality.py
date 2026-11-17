"""Tests for tools/diagnostics/check_mcap_quality.py."""
import json
import struct
import time

from mcap.writer import Writer

from tools.diagnostics.check_mcap_quality import find_latest_mcap, format_report, inspect_mcap, main

BASE = 1_794_000_000_000_000_000


def _cdr_header_msg(stamp_ns: int) -> bytes:
    sec, ns = divmod(stamp_ns, 10**9)
    return b'\x00\x01\x00\x00' + struct.pack('<iI', sec, ns)


def _cdr_string_msg(payload: dict) -> bytes:
    raw = json.dumps(payload).encode('utf-8') + b'\x00'
    return b'\x00\x01\x00\x00' + struct.pack('<I', len(raw)) + raw


def _write_sample_mcap(
    path,
    *,
    head_offset_ms: float = 0.0,
    tail_cut_ms: float = 0.0,
    mid_gap_ms: float = 0.0,
    include_vr_gap: bool = False,
    include_low_rate: bool = True,
    empty_topic: str | None = None,
    unpaired_depth: bool = False,
    with_window: bool = True,
):
    admission = BASE + 250_000_000
    stop = admission + 200_000_000
    with path.open('wb') as stream:
        writer = Writer(stream)
        writer.start(profile='ros2')
        js = writer.register_schema('sensor_msgs/msg/JointState', 'ros2msg', b'')
        img = writer.register_schema('sensor_msgs/msg/CompressedImage', 'ros2msg', b'')
        st = writer.register_schema('std_msgs/msg/String', 'ros2msg', b'')

        if with_window:
            writer.add_metadata('hc_teleop.capture_window', {
                'schema_version': 'hc-capture-window/v1',
                'action_at_publication': 'true',
                'window_start_ns': str(BASE),
                'admission_start_ns': str(admission),
                'stop_requested_ns': str(stop),
                'raw_ingress_closed_ns': str(stop + 250_000_000),
                'buffered_topics': json.dumps(['/observation/joint_states']),
            })

        obs = writer.register_channel('/observation/joint_states', 'cdr', js)
        act = writer.register_channel('/action/joint_states', 'cdr', js)
        split = writer.register_channel('/observation/ee_poses', 'cdr', js)
        color = writer.register_channel('/observation/cameras/head/color/compressed', 'cdr', img)
        depth = writer.register_channel('/observation/cameras/head/depth/compressed', 'cdr', img)
        vr = writer.register_channel('/vrdata', 'cdr', st)
        teleop = writer.register_channel('/status/teleop', 'cdr', st) if include_low_rate else None
        if empty_topic:
            writer.register_channel(empty_topic, 'cdr', js)

        # Prerolled /observation/joint_states and /action/joint_states at 100 Hz
        t = BASE + round(head_offset_ms * 1e6)
        end_t = stop - round(tail_cut_ms * 1e6)
        i = 0
        while t <= end_t:
            if mid_gap_ms > 0 and i == 10:
                t += round(mid_gap_ms * 1e6)
                if t > end_t:
                    break
            writer.add_message(obs, t, _cdr_header_msg(t), publish_time=t)
            writer.add_message(act, t, _cdr_header_msg(t), publish_time=t)
            t += 10_000_000
            i += 1

        # Non-prerolled split topic starts at admission_start_ns (or BASE when no capture_window)
        split_start = admission if with_window else BASE
        t = split_start
        while t <= stop:
            writer.add_message(split, t, _cdr_header_msg(t), publish_time=t)
            t += 10_000_000

        # Paired RGBD at ~30 Hz from window_start
        t = BASE
        idx = 0
        while t <= stop:
            writer.add_message(color, t, _cdr_header_msg(t), publish_time=t)
            if not (unpaired_depth and idx == 1):
                writer.add_message(depth, t, _cdr_header_msg(t), publish_time=t)
            t += 33_000_000
            idx += 1

        # /vrdata at ~60 Hz from admission_start (or BASE when no capture_window)
        t = admission if with_window else BASE
        seq = 1
        while t <= stop:
            rx = t
            if include_vr_gap and seq == 4:
                rx += 45_000_000
                t = rx
            payload = {'sequence': seq, 'vr_timestamp': seq * 0.016, 'hc_received_ns': rx}
            writer.add_message(vr, rx + 1_000_000, _cdr_string_msg(payload), publish_time=rx + 1_000_000)
            t += 16_000_000
            seq += 1

        if teleop is not None:
            writer.add_message(teleop, admission + 10_000_000, _cdr_string_msg({'ok': True}), publish_time=admission + 10_000_000)
            writer.add_message(teleop, admission + 210_000_000, _cdr_string_msg({'ok': True}), publish_time=admission + 210_000_000)

        writer.finish()


def test_clean_recording_passes_and_separates_low_rate_and_empty(tmp_path):
    clean = tmp_path / 'clean.mcap'
    _write_sample_mcap(clean, include_low_rate=False)
    report = inspect_mcap(clean)
    assert report['result'] == 'PASS'
    assert report['failures'] == []
    assert report['topics']['/action/joint_states']['status'] == 'PASS'
    assert report['topics']['/observation/ee_poses']['status'] == 'PASS'
    assert report['rgbd'][0]['paired_exact'] > 0
    text = format_report(report)
    assert 'PASS' in text and '/action/joint_states*' in text

    with_warn = tmp_path / 'warn.mcap'
    _write_sample_mcap(with_warn, include_low_rate=True, empty_topic='/control/joint_cmd')
    warn_report = inspect_mcap(with_warn)
    assert warn_report['result'] == 'WARN'
    assert warn_report['topics']['/status/teleop']['status'] == 'LOW_RATE'
    assert warn_report['empty_topics'] == ['/control/joint_cmd']
    strict_report = inspect_mcap(with_warn, strict=True)
    assert strict_report['result'] == 'FAIL'


def test_detects_head_tail_and_interframe_gaps_and_vr_attribution(tmp_path):
    broken = tmp_path / 'broken.mcap'
    _write_sample_mcap(broken, head_offset_ms=65.0, tail_cut_ms=55.0, mid_gap_ms=55.0, include_vr_gap=True, unpaired_depth=True)
    report = inspect_mcap(broken)
    assert report['result'] == 'FAIL'
    obs_issues = ' '.join(report['topics']['/observation/joint_states']['issues'])
    assert 'head_gap_window_start=' in obs_issues
    assert 'tail_gap=' in obs_issues
    assert 'gaps>40ms=1' in obs_issues
    assert report['topics']['/vrdata']['vr_gap_attribution'][0]['seq_delta'] == 1
    assert any('RGBD unpaired' in item for item in report['failures'])


def test_fallback_without_capture_window_and_cli_latest(tmp_path, capsys):
    older = tmp_path / 'older.mcap'
    newer = tmp_path / 'newer.mcap'
    _write_sample_mcap(older, with_window=False)
    time.sleep(0.01)
    _write_sample_mcap(newer, include_low_rate=False)
    assert find_latest_mcap(tmp_path) == newer

    rc = main([str(older), '--json'])
    assert rc == 0
    parsed = json.loads(capsys.readouterr().out)
    assert parsed['window_source'] == 'mcap_summary_fallback'
    assert parsed['result'] == 'WARN'


def test_inherent_low_rate_and_on_demand_topics_warn_unless_strict(tmp_path):
    path = tmp_path / 'low_rate_topics.mcap'
    _write_sample_mcap(path, include_low_rate=False)
    admission = BASE + 250_000_000
    low_rate_topics = [
        ('/observation/hands/joint_states', 'sensor_msgs/msg/JointState', _cdr_header_msg),
        ('/observation/hands/vibration_feedback', 'std_msgs/msg/String', lambda _: _cdr_string_msg({'ok': 1})),
        ('/status/base', 'std_msgs/msg/String', lambda _: _cdr_string_msg({'ok': 1})),
        ('/status/control/ownership', 'std_msgs/msg/String', lambda _: _cdr_string_msg({'state': 'OFF'})),
        ('/action/base/command', 'std_msgs/msg/Float64MultiArray', lambda _: _cdr_string_msg({'v': 0})),
        ('/action/base/velocity', 'geometry_msgs/msg/TwistStamped', _cdr_header_msg),
    ]
    # Append sparse/on-demand messages directly to the existing MCAP file via fresh write.
    extra = tmp_path / 'with_low_rate.mcap'
    from mcap.reader import make_reader
    with path.open('rb') as src, extra.open('wb') as dst:
        writer = Writer(dst)
        writer.start(profile='ros2')
        reader = make_reader(src)
        for meta in reader.iter_metadata():
            writer.add_metadata(meta.name, meta.metadata)
        sids = {}
        chans = {}
        for schema, channel, message in reader.iter_messages():
            if schema.id not in sids:
                sids[schema.id] = writer.register_schema(schema.name, schema.encoding, schema.data)
            if channel.id not in chans:
                chans[channel.id] = writer.register_channel(channel.topic, channel.message_encoding, sids[schema.id])
            writer.add_message(chans[channel.id], message.log_time, message.data, publish_time=message.publish_time)
        for topic, schema_name, encode in low_rate_topics:
            sid = writer.register_schema(schema_name, 'ros2msg', b'')
            ch = writer.register_channel(topic, 'cdr', sid)
            for t in (admission + 100_000_000, admission + 190_000_000):
                writer.add_message(ch, t, encode(t), publish_time=t)
        writer.finish()

    report = inspect_mcap(extra)
    assert report['result'] == 'WARN'
    for topic, _, _ in low_rate_topics:
        assert report['topics'][topic]['status'] == 'LOW_RATE'
    strict_report = inspect_mcap(extra, strict=True)
    assert strict_report['result'] == 'FAIL'

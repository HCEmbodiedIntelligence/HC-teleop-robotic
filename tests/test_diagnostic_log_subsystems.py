"""Per-subsystem log sources and the error summary behind the dashboard cards."""
import time

from middleware.core.diagnostic_logs import DiagnosticLogs


def wait_for(service, predicate, timeout=5.):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        result = service.snapshot(limit=500, priority=False)
        if predicate(result):
            return result
        time.sleep(.05)
    raise AssertionError(result)


def make_service(tmp_path, hardware):
    project = tmp_path / 'project'
    session = project / 'runtime/teleop_logs/session_20261116_105459'
    (session / 'control/reload_1').mkdir(parents=True)
    (session / 'middleware').mkdir()
    now = time.time()
    (session / 'control/reload_1/v23_solver.log').write_text(
        f'[INFO] [{now:.6f}] [solver]: ready\n[ERROR] [{now:.6f}] [solver]: IK rejected right arm\n')
    (session / 'middleware/middleware.log').write_text(f'[WARN] [{now:.6f}] [mw]: Emergency stop: startup\n')
    service = DiagnosticLogs(project, session / 'middleware', poll_interval=.02,
                             external_roots=[('hardware', hardware)])
    return service.start(), now


def test_logs_are_grouped_by_subsystem_and_hardware_keeps_only_warnings(tmp_path):
    hardware = tmp_path / 'HC_X1/runtime'
    (hardware / 'session_20261116_085929/drivers').mkdir(parents=True)
    (hardware / 'session_20261116_085929/drivers/camera.log').write_text('[ERROR] old hardware session\n')
    latest = hardware / 'session_20261116_105534'
    (latest / 'drivers').mkdir(parents=True)
    service, now = make_service(tmp_path, hardware)
    try:
        (latest / 'joint_smoother.log').write_text(
            f'[INFO] [{now:.6f}] [smoother]: stream resumed\n'
            f'[WARN] [{now:.6f}] [smoother]: feedback tracking lag detected\n')
        (latest / 'drivers/chassis.log').write_text(f'[ERROR] [{now:.6f}] [chassis]: SLAM Init Localization Failed\n')
        result = wait_for(service, lambda r: {e['source'] for e in r['entries']} >= {
            'control/v23_solver', 'middleware/middleware', 'hardware/joint_smoother', 'hardware/chassis'})
        entries = result['entries']
        hardware_entries = [e for e in entries if e['subsystem'] == 'hardware']
        assert {e['level'] for e in hardware_entries} == {'warning', 'error'}
        assert not any('old hardware session' in e['message'] for e in entries)
        assert {e['file'] for e in hardware_entries} == {'hardware/joint_smoother.log', 'hardware/drivers/chassis.log'}

        summary = {group['subsystem']: group for group in result['summary']}
        assert summary['control']['errors'] == 1 and summary['control']['state'] == 'error'
        assert 'IK rejected' in summary['control']['latest_error']['message']
        assert summary['middleware']['state'] == 'warning' and summary['middleware']['errors'] == 0
        assert summary['hardware']['recent_errors'] == 1
        assert [group['state'] for group in result['summary']][:2] == ['error', 'error']

        only_control = service.snapshot(subsystem='control', limit=500)
        assert {e['subsystem'] for e in only_control['entries']} == {'control'}
        # The summary describes the whole session, not the filtered view.
        assert {g['subsystem'] for g in only_control['summary']} == set(summary)
    finally:
        service.close()


def test_old_errors_are_counted_but_do_not_raise_the_alarm():
    old = time.time() - 3600
    from datetime import datetime, timezone
    stamp = datetime.fromtimestamp(old, timezone.utc).isoformat()
    groups = DiagnosticLogs._summary([
        {'id': 1, 'time': stamp, 'level': 'error', 'source': 'control/v23_solver', 'subsystem': 'control',
         'message': 'old', 'kind': 'saved_log'}])
    assert groups[0]['errors'] == 1
    assert groups[0]['recent_errors'] == 0
    assert groups[0]['state'] == 'warning'


def test_missing_hardware_root_is_ignored(tmp_path):
    service, _ = make_service(tmp_path, tmp_path / 'absent')
    try:
        result = wait_for(service, lambda r: len(r['entries']) >= 3)
        assert {e['subsystem'] for e in result['entries']} == {'control', 'middleware'}
    finally:
        service.close()


def test_pre_session_hardware_logs_are_not_backfilled_into_new_session(tmp_path):
    import os
    hardware = tmp_path / 'HC_X1/runtime'
    latest = hardware / 'session_20261123_092058'
    (latest / 'drivers').mkdir(parents=True)
    old_ts = time.time() - 3600
    upper = latest / 'drivers/upper_body.log'
    upper.write_text(
        f'[hc_tj_marvin_bridge_node-1] [WARN] [{old_ts:.6f}] [bridge]: old warning from 1 hour ago\n')
    dormant = latest / 'drivers/chassis.log'
    dormant.write_text('[ERROR] dormant pre-session error without timestamp\n')
    os.utime(dormant, (old_ts, old_ts))

    service, now = make_service(tmp_path, hardware)
    try:
        # Append a fresh post-startup warning to the already-running upper_body log.
        with upper.open('a') as stream:
            stream.write(f'[hc_tj_marvin_bridge_node-1] [WARN] [{now:.6f}] [bridge]: fresh current warning\n')
        result = wait_for(service, lambda r: any('fresh current warning' in e['message'] for e in r['entries']))
        messages = [e['message'] for e in result['entries']]
        assert any('fresh current warning' in m for m in messages)
        assert not any('old warning from 1 hour ago' in m for m in messages)
        assert not any('dormant pre-session error' in m for m in messages)
        summary = {group['subsystem']: group for group in result['summary']}
        assert summary['hardware']['warnings'] == 1
        assert summary['hardware']['latest_warning'] is not None
        assert 'fresh current warning' in summary['hardware']['latest_warning']['message']
    finally:
        service.close()


def test_ansi_codes_are_stripped_from_saved_logs(tmp_path):
    project = tmp_path / 'project'
    session = project / 'runtime/teleop_logs/session_ansi'
    (session / 'control').mkdir(parents=True)
    (session / 'middleware').mkdir()
    (session / 'control/launcher.log').write_text('\x1b[1;34m[HC-Control]\x1b[0m ready\n')
    service = DiagnosticLogs(project, session / 'middleware', poll_interval=.02).start()
    try:
        result = wait_for(service, lambda r: any('[HC-Control] ready' in e['message'] for e in r['entries']))
        assert all('\x1b[' not in e['message'] for e in result['entries'])
    finally:
        service.close()


def test_rosout_filter_captures_inference_fault_clipping_and_skips_local_nodes(tmp_path):
    import json
    from middleware.core.diagnostic_logs import RosoutLogFilter

    now_box = [100.0]
    filt = RosoutLogFilter(clock=lambda: now_box[0])
    # Local nodes already tailed from disk must be skipped to prevent duplication.
    assert filt.process('hc_robot_vr_teleop', 40, 'Teleop disabled by emergency stop') == []
    assert filt.process('hc_tj_marvin_bridge', 30, 'Marvin SDK warning') == []
    assert filt.process('_ros2cli_daemon_13_abc', 40, 'cli error') == []

    # Exact [motion-diag] INFO heartbeat carrying a latched inference fault.
    fault_text = (
        'inference/protocol failure: first action jumps 0.021554 rad from feedback; '
        'limit is 0.020000 rad; joint=Joint7_R target_rad=-0.107931 feedback_rad=-0.086377'
    )
    diag_msg = '[motion-diag] ' + json.dumps({
        'inference_wait': 'fault', 'control_wait': 'disabled_or_fault',
        'fault': fault_text, 'feedback_pause': None, 'max_clip_rad': 0.0,
        'ownership_timing_rejections': 0, 'allowed': True, 'gate_reason': '',
    })
    first = filt.process('holobrain_hc_tj_async_client_32b8618a', 20, diag_msg, stamp_sec=1795417381, stamp_nanosec=0)
    assert len(first) == 1
    assert first[0]['kind'] == 'inference_fault'
    assert first[0]['subsystem'] == 'inference'
    assert first[0]['source'] == 'inference/holobrain_hc_tj_async_client'
    assert first[0]['payload']['level'] == 'error'
    assert 'Joint7_R' in first[0]['payload']['message']

    # Repeated 1 Hz heartbeats with the same latched fault must not spam.
    now_box[0] += 1.0
    assert filt.process('holobrain_hc_tj_async_client_32b8618a', 20, diag_msg, stamp_sec=1795417382) == []

    # Action clipping reported inside [motion-diag] surfaces as warning (rate-limited).
    clip_msg = '[motion-diag] ' + json.dumps({
        'fault': None, 'feedback_pause': None, 'max_clip_rad': 0.125,
        'ownership_timing_rejections': 0, 'allowed': True, 'gate_reason': '',
    })
    recovered_and_clip = filt.process('holobrain_hc_tj_async_client_32b8618a', 20, clip_msg, stamp_sec=1795417385)
    kinds = {e['kind']: e for e in recovered_and_clip}
    assert 'inference_recovered' in kinds
    assert 'inference_clip' in kinds
    assert kinds['inference_clip']['payload']['level'] == 'warning'

    # Direct remote ROS2 ERROR/WARN logs are also captured and deduplicated.
    now_box[0] += 10.0
    direct = filt.process('holobrain_hc_tj_async_client_32b8618a', 40, 'inference HTTP request timed out')
    assert len(direct) == 1 and direct[0]['kind'] == 'rosout' and direct[0]['payload']['level'] == 'error'
    assert filt.process('holobrain_hc_tj_async_client_32b8618a', 40, 'inference HTTP request timed out') == []

    # End-to-end ingestion into DiagnosticLogs creates an explicit inference incident and card.
    service = DiagnosticLogs(tmp_path, poll_interval=.02).start()
    try:
        for ev in first + recovered_and_clip:
            service.emit(ev)
        result = wait_for(service, lambda r: any(e['kind'] == 'inference_clip' for e in r['entries']))
        assert any(i['subsystem'] == 'inference' and i['relation'] == 'explicit' and 'Joint7_R' in i['root_message']
                   for i in result['incidents'])
        summary = {g['subsystem']: g for g in result['summary']}
        assert summary['inference']['errors'] == 1
        assert summary['inference']['warnings'] == 1
    finally:
        service.close()

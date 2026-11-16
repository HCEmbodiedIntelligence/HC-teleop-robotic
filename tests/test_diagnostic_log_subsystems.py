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

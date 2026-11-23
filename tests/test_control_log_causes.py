"""Real ownership detectors must retain their evidence across epoch revocation."""
import time
from types import SimpleNamespace

from middleware.core.app import MiddlewareRuntime
from middleware.core.diagnostic_logs import DiagnosticLogs
from test_control_ownership import Rig


def logged_incident(tmp_path, events):
    logs = DiagnosticLogs(tmp_path, poll_interval=.02).start()
    try:
        for event in events:
            assert logs.emit({'kind': 'control_event', 'source': 'ros_bridge', 'payload': event})
        deadline = time.monotonic() + 3
        while time.monotonic() < deadline:
            snapshot = logs.snapshot(limit=500, priority=False)
            if len(snapshot['entries']) >= len(events):
                return snapshot
            time.sleep(.01)
        raise AssertionError(snapshot)
    finally:
        logs.close()


def test_heartbeat_detector_links_fault_across_epochs_without_relaxing_watchdog(tmp_path):
    rig = Rig()
    rig.auto()
    control = rig.control
    previous_token = control.token
    rig.ns += 301_000_000
    control.watchdog()
    assert control.state == 'FAULT'
    assert not control.allows(previous_token)
    assert control.input_timeout_ns == 300_000_000
    detector = next(e for e in control.events if e['kind'] == 'heartbeat_timeout')
    revoked = control.events[-1]
    assert detector['epoch'] + 1 == revoked['epoch']
    assert detector['reason'] != revoked['reason']
    assert revoked['cause_event_id'] == detector['event_id']
    snapshot = logged_incident(tmp_path, list(control.events))
    incident = next(i for i in snapshot['incidents'] if 'heartbeat lost' in i['title'])
    root = next(e for e in snapshot['entries'] if e['id'] == incident['root_cause_id'])
    assert root['kind'] == 'heartbeat_timeout'
    assert root['details']['gap_ms'] > root['details']['limit_ms']
    assert 'heartbeat' in incident['root_message']
    assert str(root['details']['limit_ms']) in incident['root_message']
    assert incident['relation'] == 'explicit'


def test_manual_timeout_old_records_still_show_direct_diagnostic(tmp_path):
    rig = Rig()
    rig.auto()
    rig.input(True)
    rig.ack()
    rig.ack()
    assert rig.control.state == 'MANUAL'
    rig.ns += 301_000_000
    rig.control.input_lost('teleop input timeout')
    snapshot = logged_incident(tmp_path, list(rig.control.events))
    incident = snapshot['incidents'][-1]
    root = next(e for e in snapshot['entries'] if e['id'] == incident['root_cause_id'])
    assert root['kind'] == 'manual_input_timeout'
    assert root['details']['input_gap_ms'] > root['details']['limit_ms']


def test_unrelated_session_timeout_is_not_inferred_as_cause(tmp_path):
    rig = Rig()
    rig.auto()
    rig.control.fault('solver acknowledgement heartbeat lost')
    events = list(rig.control.events)
    events.insert(-1, dict(events[-1], kind='heartbeat_timeout', session_id='another-session',
                           sequence=999, event_id='unrelated', gap_ms=450, limit_ms=300))
    snapshot = logged_incident(tmp_path, events)
    incident = snapshot['incidents'][-1]
    assert incident['root_cause_id'] is None
    assert incident['relation'] == 'unknown'


def test_gateway_fault_uses_original_detector_instead_of_only_stop_message(tmp_path):
    runtime = MiddlewareRuntime.__new__(MiddlewareRuntime)
    logs = runtime.diagnostics = DiagnosticLogs(tmp_path, poll_interval=.02).start()
    runtime.events = []
    runtime.recorder = runtime.vr = runtime.loop = None
    runtime.config = {'safety': {'enabled': True, 'stop_topic': '/control/emergency_stop'}}
    stops = []
    runtime.ros = SimpleNamespace(ownership=SimpleNamespace(active=False),
        status=lambda: {'command_mux': {'source': 'vr'}},
        emergency_stop=lambda *args: stops.append(args))
    try:
        runtime._on_vr_timeout('head tracking invalid')
        deadline = time.monotonic() + 3
        while time.monotonic() < deadline:
            snapshot = logs.snapshot(limit=500, priority=False)
            if len(snapshot['entries']) == 3:
                break
            time.sleep(.01)
        assert len(stops) == 1
        assert snapshot['incidents'][-1]['relation'] == 'explicit'
        root_id = snapshot['incidents'][-1]['root_cause_id']
        assert next(e for e in snapshot['entries'] if e['id'] == root_id)['kind'] == 'vr_input_fault'
        assert len(snapshot['incidents'][-1]['chain']) == 3
        assert snapshot['incidents'][-1]['source'] == 'vr_gateway'
        assert snapshot['incidents'][-1]['time']
    finally:
        logs.close()


def test_routine_lifecycle_safety_stops_do_not_create_error_incidents(tmp_path):
    logs = DiagnosticLogs(tmp_path, poll_interval=.02).start()
    try:
        for reason in (
            'middleware startup',
            'loading control configuration; explicit resume required',
            'middleware stopping; explicit resume required',
            'control configuration reload; explicit resume required',
            'robot profile applied: hc_tj; restart teleop',
        ):
            assert logs.emit({'kind': 'safety_stop', 'source': 'middleware', 'payload': {'reason': reason}})
        # Real fault safety_stop and startup failure MUST still stay error/incident.
        assert logs.emit({'kind': 'safety_stop', 'source': 'middleware', 'payload': {'reason': 'head tracking invalid'}})
        assert logs.log('error', 'middleware startup failed: bad config')
        deadline = time.monotonic() + 3
        while time.monotonic() < deadline:
            snapshot = logs.snapshot(limit=500, priority=False)
            if len(snapshot['entries']) >= 7:
                break
            time.sleep(.01)
        assert len(snapshot['incidents']) == 1
        assert snapshot['incidents'][0]['title'] == 'head tracking invalid'
        errors = [e for e in snapshot['entries'] if e['level'] == 'error']
        assert {e['message'] for e in errors} == {'head tracking invalid', 'middleware startup failed: bad config'}
    finally:
        logs.close()


def test_fault_ack_events_do_not_duplicate_errors(tmp_path):
    rig = Rig()
    rig.auto()
    rig.input(True)
    rig.ack()
    rig.ack()
    rig.ns += 301_000_000
    rig.control.input_lost('teleop input timeout')
    snapshot = logged_incident(tmp_path, list(rig.control.events))
    ack_errors = [e for e in snapshot['entries'] if e['kind'] == 'ack' and e['level'] == 'error']
    assert ack_errors == []
    assert snapshot['incidents'][-1]['root_cause_id'] is not None


def test_control_rejections_and_arm_velocity_clipping_surface_with_details(tmp_path):
    logs = DiagnosticLogs(tmp_path, poll_interval=.02).start()
    try:
        assert logs.emit({
            'kind': 'control_diagnostic',
            'source': 'ros_bridge',
            'payload': {
                'state': 'AUTO',
                'epoch': 25,
                'counts': {'rejected:target too far from measured position': 4},
                'recent_rejection': 'target too far from measured position: Joint3_R target=1.200000 measured=0.700000 error=0.500000 limit=0.300000',
            },
        })
        assert logs.emit({
            'kind': 'control_clip',
            'source': 'ros_bridge',
            'payload': {
                'state': 'AUTO',
                'epoch': 25,
                'count': 3,
                'total': 3,
                'last_clip': {
                    'joint': 'Joint7_R', 'requested': -0.066, 'sent': -0.0676,
                    'delta_rad': 0.0016, 'limit_rad_s': 1.0, 'dt_s': 0.01,
                },
            },
        })
        deadline = time.monotonic() + 3
        while time.monotonic() < deadline:
            snapshot = logs.snapshot(limit=500, level='warning', priority=False)
            if len(snapshot['entries']) >= 2:
                break
            time.sleep(.01)
        by_kind = {e['kind']: e for e in snapshot['entries']}
        assert by_kind['control_diagnostic']['level'] == 'error'
        assert 'Joint3_R' in by_kind['control_diagnostic']['message']
        assert by_kind['control_clip']['level'] == 'warning'
        assert 'Joint7_R' in by_kind['control_clip']['message']
        assert '1 rad/s' in by_kind['control_clip']['message']
    finally:
        logs.close()

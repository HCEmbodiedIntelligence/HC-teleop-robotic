"""Process-boundary faults use the real ownership FSM, without robot ROS."""
import os
import queue
import subprocess
import sys
import threading
import time
from collections import deque
from types import SimpleNamespace

import pytest

from middleware.core.control_ownership import ControlOwnership
from middleware.core.control_process import CoreProcessClient, RemoteOwnership, RemotePublicationCapture


class FakeBridge:
    """Small transport harness; authority decisions remain ControlOwnership."""
    def __init__(self, config, emit):
        self.config, self.emit = config, emit
        self._stop = threading.Event()
        self._command_output_enabled = True
        self._output_epoch = 0
        self._command_source = 'vr'
        self._replay_active = False
        self.wire = []
        self.queue = []
        self.pumps = 0
        self.ownership = ControlOwnership(revoke=self._ownership_revoke,
            required_peers=(), recording_enabled=False)

    def _ownership_revoke(self):
        self._command_output_enabled = False
        self._revoke_locked()

    def _revoke_locked(self):
        self._output_epoch += 1

    def status(self):
        return dict(state='running', ownership=self.ownership.snapshot(),
            command_mux=dict(output_enabled=self._command_output_enabled,
                source=self._command_source, replay_active=self._replay_active),
            pumps=self.pumps, wire=list(self.wire[-40:]), child_flag=self.config.get('record_action_at_publication'))

    def control_ready(self):
        return True

    def set_command_output_enabled(self, enabled):
        if enabled and self.ownership.active:
            raise ValueError('intervention owns output')
        self._revoke_locked()
        self._command_output_enabled = bool(enabled)
        return True

    def emergency_stop(self, topic, reason):
        self.ownership.fault(reason, estop=True)
        self._command_output_enabled = False
        self._revoke_locked()
        self.wire.append((topic, True))
        if self.config.get('stop_log'):
            with open(self.config['stop_log'], 'w') as stream:
                stream.write(reason)
        return True

    def publish(self, topic, msg_type, data):
        if topic == '/control/test_crash':
            os._exit(17)
        if topic == '/control/test_stall':
            self.emit({'kind': 'stall_entered'}, [])
            time.sleep(data.get('seconds', 1.5))
        if topic == '/control/test_internal_fault':
            self._ownership_revoke()
        if topic == '/control/test_open_segment':
            self.ownership.open_segment(self.ownership.token, time.time_ns())
        if topic == '/control/test_close_segment':
            self.ownership.close_segment(self.ownership.token, time.time_ns(), 0, 'test')
        if topic == '/control/test_manual':
            control = self.ownership
            control.recording_enabled = True
            control.enable()
            control.input(device_instance='test', sequence=0, valid=True, gripped=False, stamp_ns=time.time_ns())
            control.input(device_instance='test', sequence=1, valid=True, gripped=True, stamp_ns=time.time_ns())
            control.advance(control_ready=True, manual_ready=True)
            control.advance(control_ready=True, manual_ready=True)
        self.queue.append((topic, data, self._actor_command_fence()))
        return True

    def publish_raw(self, topic, msg_type, raw):
        if not self._command_output_enabled or not self._replay_active:
            return False
        return self.publish(topic, msg_type, raw)

    def begin_replay(self):
        if not self._command_output_enabled:
            return False
        self._replay_active = True
        return True

    def recover_intervention(self):
        return self.ownership.recover(control_ready=True)

    def set_control_source(self, source):
        if self.ownership.active:
            raise ValueError('intervention session owns source selection')
        if self._command_source != source:
            self._ownership_revoke()
        self._command_source = source
        return True

    def end_replay(self, require_reset=True):
        self._revoke_locked()
        self._replay_active = False
        if require_reset:
            self._command_output_enabled = False

    def _flush(self):
        while self.queue:
            topic, data, fence = self.queue.pop(0)
            if self._actor_command_allowed(fence, 'publish', data):
                self.wire.append((topic, data))

    def _run(self):
        self._actor_flush_commands = self._flush
        self._actor_ready()
        while not self._stop.is_set():
            self._actor_pump()
            if self._stop.is_set():
                break
            self._flush()
            self.pumps += 1
            if self.config.get('observer_flood'):
                self.emit({'kind': 'test'}, [])
            time.sleep(.001)


def wait_until(predicate, timeout=3.):
    deadline = time.monotonic()+timeout
    while time.monotonic() < deadline:
        if predicate():
            return
        time.sleep(.01)
    assert predicate()


@pytest.fixture
def client():
    value = CoreProcessClient({}, bridge_factory=FakeBridge)
    value.start()
    yield value
    value.stop()


def test_start_closed_and_preserve_config_changes_before_spawn():
    config = {}
    client = CoreProcessClient(config, bridge_factory=FakeBridge)
    config['record_action_at_publication'] = True
    try:
        client.start()
        assert client.snapshot()['child_flag'] is True
        assert client.snapshot()['command_mux']['output_enabled'] is False
        result = client.call('resume_output', '/control/emergency_stop')
        assert tuple(result['authorization_fence']) == client.authorization_fence()
    finally:
        client.stop()


def test_stale_authorization_replay_and_internal_fault_epoch(client):
    fence = client.authorization_fence()
    client.revoke_intent()
    with pytest.raises(ValueError, match='revoked'):
        client.call('resume_output', '/control/emergency_stop', expected_fence=fence)
    client.call('resume_output', '/control/emergency_stop')
    assert client.call('begin_replay')
    replay_fence = client.authorization_fence()
    client.call('end_replay', urgent=True)
    client.call('resume_output', '/control/emergency_stop')
    assert client.call('begin_replay')
    with pytest.raises(ValueError, match='revoked'):
        client.call('publish_raw', '/action/joint_states', 'sensor_msgs/msg/JointState', b'old',
                    expected_fence=replay_fence)
    fence = client.authorization_fence()
    client.call('publish', '/control/test_internal_fault', 'test', {})
    assert client.authorization_fence() != fence
    with pytest.raises(ValueError, match='revoked'):
        client.call('resume_output', '/control/emergency_stop', expected_fence=fence)


def test_structured_request_dedup_and_stale_disable_use_original_fsm(client):
    first = dict(request_id='enable-1', session_id='', expected_epoch=0)
    fence = client.authorization_fence()
    result = client.call('apply_intervention', 'enable', first, recording_ready=True, preparing_capture=False)
    assert result['applied'] is True
    duplicate = client.call('apply_intervention', 'enable', first, expected_fence=fence,
                            recording_ready=True, preparing_capture=False)
    assert duplicate['applied'] is False
    before = client.authorization_fence()
    with pytest.raises(ValueError, match='stale session'):
        client.call('apply_intervention', 'disable',
                    dict(request_id='stale-disable', session_id='', expected_epoch=0),
                    recording_ready=False, preparing_capture=False, urgent=True)
    assert client.authorization_fence() == before
    assert RemoteOwnership(client).snapshot()['state'] == 'HOLD'
    with pytest.raises(ValueError, match='owns source'):
        client.call('set_control_source', 'motion', urgent=True)
    assert client.authorization_fence() == before
    assert RemoteOwnership(client).snapshot()['state'] == 'HOLD'


def test_auto_input_loss_remains_nonblocking_and_preserves_model_authority(client):
    client.call('owner.start_model', recording_ready=False)
    owner = RemoteOwnership(client)
    wait_until(lambda: owner.snapshot()['state'] == 'RETURNING')
    fence = client.authorization_fence()
    started = time.monotonic()
    assert owner.input_lost('headset disconnected')
    assert time.monotonic()-started < .05
    wait_until(lambda: owner.snapshot()['input_diagnostics'].get('last_loss_reason') == 'headset disconnected')
    assert owner.snapshot()['state'] == 'RETURNING'
    assert client.authorization_fence() == fence


def test_slow_observer_cannot_stall_actor_or_replies():
    blocked = threading.Event()
    release = threading.Event()
    def observer(*args):
        blocked.set()
        release.wait(2.)
    client = CoreProcessClient({'observer_flood': True}, bridge_factory=FakeBridge, on_event=observer)
    try:
        client.start()
        assert blocked.wait(1.)
        previous = client.snapshot()['pumps']
        time.sleep(.4)
        assert client.call('clock') > 0
        wait_until(lambda: client.snapshot()['pumps'] > previous+100)
        assert client.snapshot()['control_process']['observer_dropped'] > 0
    finally:
        release.set()
        client.stop()


def test_recording_hook_roundtrip_has_child_clock_and_no_parent_generation_deadlock(client):
    clocks = []
    owner = RemoteOwnership(client)
    owner.open_segment = lambda *_args: clocks.append(owner.ros_clock())
    client.call('publish', '/control/test_open_segment', 'test', {})
    assert clocks and clocks[0] > 0
    client.flush_recording_events()


@pytest.mark.parametrize('failure', ['crash', 'stall'])
def test_core_failure_stops_and_never_automatically_reauthorizes(failure):
    stops = []
    client = CoreProcessClient({}, bridge_factory=FakeBridge, fallback_stop=stops.append)
    try:
        client.start()
        with pytest.raises(RuntimeError):
            client.call('publish', '/control/test_'+failure, 'test', {}, timeout=.1)
        wait_until(lambda: bool(stops))
        assert client.snapshot()['command_mux']['output_enabled'] is False
        with pytest.raises(RuntimeError):
            client.call('resume_output', '/control/emergency_stop')
    finally:
        client.stop()


def test_dead_core_capture_cleanup_is_idempotent_and_explicitly_incomplete(client):
    capture = RemotePublicationCapture(client, lambda: True)
    client.process.terminate()
    client.process.join(1.)
    fence = capture.close_admission()
    capture.end_generation(timeout=.01)
    capture.close()
    assert capture.close_admission() == fence
    capture.end_generation(timeout=.01)
    capture.close()
    assert 'incomplete' in capture.status()['error']


def test_parent_death_closes_child_output_and_exits(tmp_path):
    stop_log = str(tmp_path/'child-stop.txt')
    code = '''
import os
from test_control_process import FakeBridge
from middleware.core.control_process import CoreProcessClient
if __name__ == '__main__':
    client = CoreProcessClient({'stop_log': %r}, bridge_factory=FakeBridge)
    client.start()
    client.call('resume_output', '/control/emergency_stop')
    os._exit(0)
''' % stop_log
    environment = dict(os.environ, PYTHONPATH=os.pathsep.join([os.getcwd(), os.path.join(os.getcwd(), 'tests')]))
    process = subprocess.Popen([sys.executable, '-c', code], env=environment,
                               stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    assert process.wait(timeout=5.) == 0
    wait_until(lambda: os.path.exists(stop_log))
    assert 'host or IPC failed' in open(stop_log).read()


def test_priority_stop_invalidates_a_resume_queued_before_stop(client):
    entered = threading.Event()
    client.on_event = lambda event, _outputs: entered.set() if event['kind'] == 'stall_entered' else None
    errors = []
    def request(op, *args, **kwargs):
        try:
            client.call(op, *args, **kwargs)
        except (ValueError, RuntimeError) as exc:
            errors.append(str(exc))
    slow = threading.Thread(target=request, args=('publish', '/control/test_stall', 'test', {'seconds': .2}))
    slow.start()
    assert entered.wait(1.)
    old_fence = client.authorization_fence()
    resume = threading.Thread(target=request, args=('resume_output', '/control/emergency_stop'),
                              kwargs={'expected_fence': old_fence})
    resume.start()
    wait_until(lambda: len(client._pending) == 2)
    started = time.monotonic()
    client.revoke_intent()
    assert client.notify('emergency_stop', '/control/emergency_stop', 'test urgent stop')
    assert time.monotonic()-started < .05
    slow.join(2.)
    resume.join(2.)
    assert len(errors) == 2 and all('revoked' in error for error in errors)
    wait_until(lambda: client.snapshot()['command_mux']['output_enabled'] is False)


@pytest.mark.parametrize('full_session', [False, True])
def test_original_manual_close_is_delivered_after_later_revoke(client, full_session):
    from middleware.core.correction_recording import CorrectionSession
    # Exercise the actual open/close implementation without file writer setup.
    capture = CorrectionSession.__new__(CorrectionSession)
    capture.lock = threading.RLock()
    capture.full_session, capture.closing = full_session, False
    capture.active, capture.manual_token = ((0, None) if full_session else None), None
    capture.prepared = deque([0, 1])
    capture.boundaries, capture.bounds = queue.Queue(), [0, 0, 0, 0]
    capture.slots = [{'recorder': None, 'job': {'recording_id': 'test', 'metadata': {}}}]
    capture.service = SimpleNamespace(takeovers=None)
    capture.registry_events = deque()
    owner = RemoteOwnership(client)
    owner.open_segment = capture.open
    owner.close_segment = capture.close
    blocked, release = threading.Event(), threading.Event()
    def manual_changed(token, *_args):
        if token is None:
            blocked.set()
            release.wait(2.)
    owner.manual_changed = manual_changed
    client.call('publish', '/control/test_manual', 'test', {})
    assert owner.snapshot()['state'] == 'MANUAL'
    error = []
    def hold():
        try:
            client.call('owner.hold', urgent=True)
        except Exception as exc:
            error.append(str(exc))
    worker = threading.Thread(target=hold)
    worker.start()
    assert blocked.wait(1.)
    client.revoke_intent()
    client.notify('emergency_stop', '/control/emergency_stop', 'later stop')
    worker.join(1.)
    release.set()
    client.flush_recording_events()
    if full_session:
        assert capture.manual_token is None
    else:
        assert capture.active is None
    boundaries = []
    while not capture.boundaries.empty():
        boundaries.append(capture.boundaries.get_nowait()['op'])
    assert boundaries == ['open', 'close']
    assert not error


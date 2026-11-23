"""Process-boundary faults use the real ownership FSM, without robot ROS."""
import os
import json
import queue
import subprocess
import sys
import threading
import time
from collections import deque
from types import SimpleNamespace

import pytest

from middleware.core.control.control_ownership import ControlOwnership
from middleware.core.control.control_process import CoreProcessClient, RemoteOwnership, RemotePublicationCapture
from middleware.core.control.control_process import IsolatedRosBridge, publish_readonly_observation
from middleware.core.control.control_process import _ParentLiveness


def test_parent_sentinel_reuses_one_registration_and_polls_every_checkpoint(monkeypatch):
    import middleware.core.control.control_process as module
    parent_calls, created, registrations, polls = [], [], [], []
    class Poller:
        def register(self, descriptor, mask):
            registrations.append((descriptor, mask))
        def poll(self, timeout):
            polls.append(timeout)
            return []
    parent = SimpleNamespace(sentinel=123, is_alive=lambda:pytest.fail('per-check selector fallback used'))
    monkeypatch.setattr(module.mp, 'parent_process', lambda:parent_calls.append(True) or parent)
    monkeypatch.setattr(module.os, 'getppid', lambda:321)
    monkeypatch.setattr(module.select, 'poll', lambda:created.append(True) or Poller())
    liveness = _ParentLiveness(321)
    for _ in range(20):
        assert liveness.is_alive()
    assert len(parent_calls) == len(created) == len(registrations) == 1
    assert registrations[0][0] == 123
    assert polls == [0]*20
    monkeypatch.setattr(module.os, 'getppid', lambda:322)
    assert not liveness.is_alive()


def test_parent_sentinel_hangup_detected_even_before_ppid_changes(monkeypatch):
    import middleware.core.control.control_process as module
    read_fd, write_fd = os.pipe()
    monkeypatch.setattr(module.mp, 'parent_process', lambda:SimpleNamespace(sentinel=read_fd))
    monkeypatch.setattr(module.os, 'getppid', lambda:123)
    try:
        liveness = _ParentLiveness(123)
        assert liveness.is_alive()
        os.close(write_fd)
        write_fd = None
        assert not liveness.is_alive()
        assert not liveness.is_alive()
        os.fstat(read_fd)  # The checker never closes the parent-owned handle.
        os.close(read_fd)
        read_fd = None
        assert not liveness.is_alive()  # POLLNVAL is also fail-closed.
    finally:
        for descriptor in (read_fd, write_fd):
            if descriptor is not None:
                os.close(descriptor)


def test_parent_sentinel_error_fails_closed_and_nonpoll_platform_keeps_old_check(monkeypatch):
    import middleware.core.control.control_process as module
    monkeypatch.setattr(module.os, 'getppid', lambda:123)
    parent = SimpleNamespace(sentinel=5, is_alive=lambda:False)
    monkeypatch.setattr(module.mp, 'parent_process', lambda:parent)
    class Poller:
        def register(self, *_args): pass
        def poll(self, timeout): raise OSError('sentinel unavailable')
    monkeypatch.setattr(module.select, 'poll', Poller)
    assert not _ParentLiveness(123).is_alive()
    monkeypatch.delattr(module.select, 'poll')
    assert not _ParentLiveness(123).is_alive()
    parent.is_alive = lambda:True
    assert _ParentLiveness(123).is_alive()
    monkeypatch.setattr(module.mp, 'parent_process', lambda:None)
    assert _ParentLiveness(123).is_alive()


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

    def _revoke_locked(self, *, keep_replay=False):
        self._output_epoch += 1
        if not keep_replay:
            self._replay_active = False

    def status(self):
        return dict(state='running', ownership=self.ownership.snapshot(),
            command_mux=dict(output_enabled=self._command_output_enabled,
                source=self._command_source, replay_active=self._replay_active),
            pumps=self.pumps, wire=list(self.wire[-40:]), child_flag=self.config.get('record_action_at_publication'),
            tick_sends=getattr(self, 'tick_sends', 0),
            authority_blocked=getattr(self, '_actor_authority_blocked', lambda:False)())

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
            if self.config.get('model_tick'):
                self.ownership.advance(control_ready=True, manual_ready=True)
                if self.ownership.state == 'AUTO':
                    # Model the original ROS tick's attempt to re-open output.
                    # Host latch and final gate must remain negative even
                    # before multiprocessing delivers the negative RPC bytes.
                    self._command_output_enabled = True
                    if not self._actor_authority_blocked():
                        self.tick_sends = getattr(self, 'tick_sends', 0)+1
                self._actor_urgent_check()
            self._flush()
            self.pumps += 1
            if self.config.get('observer_flood'):
                self.emit({'kind': 'test'}, [])
            if self.config.get('observer_ros_flood'):
                hook = getattr(self, '_observer_ros_publish', None)
                if hook:
                    for topic in ('/status/control/ownership', '/events/control/intervention', '/internal/control/action_evidence'):
                        hook(topic, {'sequence':self.pumps})
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
        assert client.snapshot()['control_process']['python_switch_interval_ms'] > 0
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
    client.call('emergency_stop', '/control/emergency_stop', 'deliver delayed negative intent', urgent=True)
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
        wait_until(lambda:client.snapshot()['control_process']['observer_dropped'] > 0, timeout=1.)
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
from middleware.core.control.control_process import CoreProcessClient
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


def test_pending_negative_generation_blocks_ros_auto_tick_and_only_exact_negative_ack_clears_it():
    client = CoreProcessClient({'model_tick':True}, bridge_factory=FakeBridge)
    try:
        client.start()
        client.call('owner.start_model', recording_ready=False)
        wait_until(lambda: client.snapshot()['ownership']['state'] == 'AUTO')
        generation = client.revoke_intent()
        wait_until(lambda: client.snapshot()['control_process']['pending_negative_generation'] == generation)
        before = client.snapshot()['tick_sends']
        time.sleep(.15)  # Deliberately withhold the negative RPC feeder.
        assert client.snapshot()['tick_sends'] == before
        assert client.snapshot()['command_mux']['output_enabled'] is False
        assert client.snapshot()['ownership']['state'] == 'AUTO'  # No copied FSM/forced HOLD.
        client.call('set_command_output_enabled', False, urgent=True, negative_generation=generation-1)
        client.call('clock')
        client.call('owner.input_lost', 'input packet missing')  # AUTO input loss cannot acknowledge a global stop.
        with pytest.raises(ValueError, match='negative safety request is pending'):
            client.call('resume_output', '/control/emergency_stop')
        client.call('owner.disable', urgent=True, negative_generation=generation)
        client.call('resume_output', '/control/emergency_stop')  # Fresh explicit operator recovery.
        wait_until(lambda: client.snapshot()['command_mux']['output_enabled'] is True)
        assert client.snapshot()['control_process']['pending_negative_generation'] is None
    finally:
        client.stop()


def test_readonly_parent_handler_preserves_original_json_and_rejects_all_actuator_topics():
    calls = []
    telemetry = SimpleNamespace(publish=lambda *args: calls.append(args) or True)
    payload = {'state':'MANUAL', 'names':['右臂'], 'active':True, 'epoch':4}
    assert publish_readonly_observation(telemetry, '/status/control/ownership', payload)
    topic, kind, fields = calls[0]
    assert topic == '/status/control/ownership' and kind == 'std_msgs/msg/String'
    assert json.loads(fields['data']) == payload
    for forbidden in ('/action/joint_states', '/control/emergency_stop', '/control/hardware/enabled', '/control/motion/permit'):
        with pytest.raises(ValueError, match='read-only'):
            publish_readonly_observation(telemetry, forbidden, payload)
    with pytest.raises(ValueError):
        publish_readonly_observation(telemetry, '/status/control/ownership', {'invalid':float('nan')})
    assert len(calls) == 1


def test_readonly_telemetry_queue_rejection_is_visible_without_revoking_authority():
    client = CoreProcessClient({}, on_observer_ros=lambda *_args:False)
    worker = threading.Thread(target=client._observer_ros, daemon=True)
    worker.start()
    try:
        client.channels['observer_ros'].put(('/status/control/ownership', {'state':'AUTO'}))
        wait_until(lambda:client.snapshot()['control_process']['observer_ros_errors'] == 1)
        assert 'telemetry queue' in client.snapshot()['control_process']['observer_ros_last_error']
        assert not client.channels['failure_stop'].is_set()
    finally:
        client._stopped.set()
        worker.join(1.)


def hold_shared_lock(channels, kind, ready):
    # Test-only fault injection: terminate the process without releasing the
    # same primitive used by normal ownership/fence/Event access.
    lock = channels['failure_stop']._cond if kind == 'failure_stop' else channels[kind].get_lock()
    lock.acquire()
    ready.set()
    time.sleep(60.)


@pytest.mark.parametrize('kind,entry', [
    ('parent_generation', 'authorization_fence'),
    ('parent_generation', 'revoke_intent'),
    ('core_epoch', 'authorization_fence'),
    ('ownership_active', 'active'),
    ('failure_stop', 'failed'),
])
def test_killed_owner_holding_shared_lock_cannot_block_fallback_or_parent_cleanup(kind, entry):
    stops = []
    client = CoreProcessClient({}, fallback_stop=stops.append)
    ready = client.context.Event()
    process = client.context.Process(target=hold_shared_lock, args=(client.channels, kind, ready), daemon=True)
    client.process = process
    process.start()
    assert ready.wait(3.)
    entered, done = threading.Event(), threading.Event()
    results = []
    def invoke():
        entered.set()
        if entry == 'active':
            results.append(RemoteOwnership(client).active)
        elif entry == 'failed':
            client._failed('held Event fault injection')
        else:
            results.append(getattr(client, entry)())
        done.set()
    worker = threading.Thread(target=invoke, daemon=True)
    worker.start()
    assert entered.wait(1.)
    time.sleep(.001)
    process.kill()
    process.join(1.)
    try:
        assert done.wait(.5), 'parent caller remained on the killed owner lock'
        assert client._fallback_completed.wait(.5)
        assert len(stops) == 1
        started = time.monotonic()
        assert client.authorization_fence() == (client.boot, -1, -1)
        assert RemoteOwnership(client).active is False
        assert client.revoke_intent() == -1
        capture = RemotePublicationCapture(client, lambda:True)
        capture.close()
        assert 'incomplete' in capture.status()['error']
        client.stop()
        assert time.monotonic()-started < .5
        assert client.snapshot()['command_mux']['output_enabled'] is False
    finally:
        if process.is_alive():
            process.kill()
            process.join(1.)
        client.stop()
        worker.join(.5)


def test_slow_readonly_dds_and_ui_observers_cannot_hold_actor_admin_or_record_fifo(monkeypatch):
    from middleware.core.control import ros_bridge
    entered_ros, entered_ui, release = threading.Event(), threading.Event(), threading.Event()
    published, manual = [], []
    class Telemetry:
        def __init__(self, *_args, **_kwargs): pass
        def start(self): pass
        def stop(self): pass
        def publish(self, topic, kind, payload):
            published.append((topic, kind, payload))
            entered_ros.set()
            release.wait(2.)
            return True
    monkeypatch.setattr(ros_bridge, 'RosBridge', Telemetry)
    def ui(*_args):
        entered_ui.set()
        release.wait(2.)
    facade = IsolatedRosBridge({'observer_flood':True, 'observer_ros_flood':True}, ui, bridge_factory=FakeBridge)
    facade.ownership.manual_changed = lambda token,*_args: manual.append(token)
    try:
        facade.start()
        assert entered_ros.wait(1.) and entered_ui.wait(1.)
        before = facade._client.snapshot()['pumps']
        time.sleep(.4)
        started = time.monotonic()
        assert facade._client.call('clock') > 0
        assert time.monotonic()-started < .2
        wait_until(lambda: facade._client.snapshot()['pumps'] > before+100)
        assert facade._client.snapshot()['control_process']['observer_ros_dropped'] > 0
        facade._client.call('publish', '/control/test_manual', 'test', {})
        facade._client.call('owner.hold', urgent=True)
        facade.flush_recording_events()
        assert len(manual) == 2 and manual[0] is not None and manual[1] is None
        # Defense in depth in the parent consumer, even if a malformed event
        # somehow reached the observer transport.
        release.set()
        facade._client.channels['observer_ros'].put(('/control/hardware/enabled', {'data':True}))
        wait_until(lambda: facade._client.snapshot()['control_process']['observer_ros_errors'] > 0)
        assert {topic for topic,_kind,_payload in published} <= {
            '/status/control/ownership', '/events/control/intervention', '/internal/control/action_evidence'}
    finally:
        release.set()
        facade.stop()


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
    from middleware.core.recording.correction_recording import CorrectionSession
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


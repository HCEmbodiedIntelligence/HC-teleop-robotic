"""A single control owner in a spawned process; observers never own its gate.

The existing RosBridge/ControlOwnership implement the decisions. This module
only transports administrative requests, immutable recording bytes and bounded
observation snapshots. No actuator target is forwarded through an IPC FIFO.
"""
from __future__ import annotations

import copy
import json
import multiprocessing as mp
import os
import queue
import sys
import threading
import time
import uuid
from types import SimpleNamespace


_HOOKS = {'manual_changed', 'dagger_changed', 'open_segment', 'close_segment', 'capture_ready'}
_BRIDGE_METHODS = {'publish', 'publish_raw', 'request_manual_home', 'begin_replay',
                   'end_replay', 'emergency_stop', 'set_control_source',
                   'set_command_output_enabled', 'recover_intervention', 'control_ready'}
_OWNER_METHODS = {'enable', 'disable', 'start_model', 'set_dagger', 'start_auto',
                  'takeover', 'release', 'hold', 'fault', 'recover', 'input_lost',
                  'finish_capture_preparation'}
_POSITIVE_OWNER = {'enable', 'start_model', 'start_auto', 'takeover', 'recover', 'set_dagger'}
_CONTROL_PREFIXES = ('/control/', '/internal/', '/teleop/', '/action/', '/hc_teleop/')
_READONLY_ROS_TOPICS = frozenset({'/status/control/ownership',
    '/events/control/intervention', '/internal/control/action_evidence'})


def publish_readonly_observation(telemetry, topic, payload):
    """Parent-only serialization; this path can never publish an actuator."""
    if topic not in _READONLY_ROS_TOPICS or not isinstance(payload, dict):
        raise ValueError('unsupported read-only control observation')
    return telemetry.publish(topic, 'std_msgs/msg/String',
                             {'data': json.dumps(payload, ensure_ascii=False, allow_nan=False)})


def _negative_operation(op, args):
    return (op in {'emergency_stop', 'end_replay', 'shutdown', 'owner.disable', 'owner.hold', 'owner.fault'}
            or op == 'set_command_output_enabled' and args and not args[0])


def _offer_latest(target, value):
    try:
        target.put_nowait(value)
        return
    except queue.Full:
        pass
    try:
        target.get_nowait()
    except queue.Empty:
        pass
    try:
        target.put_nowait(value)
    except queue.Full:
        pass


def _generation(shared):
    with shared.get_lock():
        return int(shared.value)


class _RecordingIngress:
    def __init__(self, resources, finish):
        self.resources, self.finish = resources, finish

    def record(self, event):
        return self.record_publication(event)

    def record_publication(self, event):
        from .topic_recorder import enqueue_record
        return enqueue_record(self.resources, event, allow_closed=True, finished=self.finish)


def _control_worker(config, channels, capture_spec, bridge_factory=None):
    """Only this process invokes the authoritative bridge and its ROS sender."""
    from .ros_bridge import RosBridge
    bridge_factory = bridge_factory or RosBridge
    core_config = copy.deepcopy(config)
    core_config.update(_control_actor_mode=True, subscriptions=[])
    core_config['node_name'] = config.get('node_name', 'hc_teleop_middleware')+'_control_owner'
    core_config['dataset_topics'] = {'enabled': False}
    counters = {'observer_dropped': 0, 'observer_ros_dropped': 0,
                'observer_ros_rejected': 0, 'recording_events_dropped': 0,
                'rpc_replies_dropped': 0, 'stale_queued_commands': 0}
    hook_sequence = 0
    pumping, hard_stopped = False, False
    seen_parent = _generation(channels['parent_generation'])
    applied_parent = seen_parent
    last_snapshot = 0.
    child_selected = [True]
    bridge = None

    def emit(event, outputs):
        try:
            channels['observers'].put_nowait((event, outputs))
        except queue.Full:
            counters['observer_dropped'] += 1

    bridge = bridge_factory(core_config, emit)
    def observer_ros_publish(topic, payload):
        if topic not in _READONLY_ROS_TOPICS or not isinstance(payload, dict):
            counters['observer_ros_rejected'] += 1
            return False
        try:
            channels['observer_ros'].put_nowait((topic, payload))
            return True
        except queue.Full:
            counters['observer_ros_dropped'] += 1
            return False
    if channels['observer_ros_enabled']:
        bridge._observer_ros_publish = observer_ros_publish
    original_revoke = bridge._revoke_locked

    def revoke():
        original_revoke()
        with channels['core_epoch'].get_lock():
            channels['core_epoch'].value = bridge._output_epoch

    bridge._revoke_locked = revoke
    bridge._command_output_enabled = False
    bridge._revoke_locked()

    def fence():
        return (channels['boot'], _generation(channels['parent_generation']), bridge._output_epoch)

    def verify(expected):
        if expected is None or tuple(expected) != fence():
            raise ValueError('authorization revoked while request was waiting; explicit fresh authorization required')
        if channels['failure_stop'].is_set():
            raise RuntimeError('control host failed; restart and explicit authorization required')
        if authority_blocked():
            raise ValueError('negative safety request is pending; fresh authorization required after stop completes')

    def authority_blocked():
        return (channels['failure_stop'].is_set()
                or _generation(channels['parent_generation']) > applied_parent)

    def record_event(kind, name='', args=(), **fields):
        nonlocal hook_sequence
        hook_sequence += 1
        event = dict(sequence=hook_sequence, kind=kind, name=name, args=args,
                     clock_ns=bridge.ownership.ros_clock(), fence=fence(), **fields)
        try:
            channels['hooks'].put_nowait(event)
            return event
        except queue.Full:
            counters['recording_events_dropped'] += 1
            bridge.ownership.recording_error = 'control recording observer queue full; capture incomplete'
            raise RuntimeError(bridge.ownership.recording_error)

    def async_hook(name, *args):
        try:
            record_event('hook', name, args)
        except RuntimeError:
            pass  # Explicit incomplete evidence; never block an active sender.

    def blocking_hook(name, *args):
        if bridge._command_output_enabled:
            raise RuntimeError('recording preparation requires closed control output')
        request = record_event('hook_request', name, args)
        deadline = time.monotonic() + .5
        try:
            while time.monotonic() < deadline:
                if (channels['failure_stop'].is_set() or
                        _generation(channels['parent_generation']) != request['fence'][1]):
                    raise RuntimeError('recording preparation revoked')
                try:
                    reply = channels['hook_replies'].get(timeout=.005)
                except queue.Empty:
                    continue
                if reply.get('sequence') != request['sequence']:
                    continue
                if reply.get('error'):
                    raise RuntimeError(reply['error'])
                record_event('hook_ack', request_sequence=request['sequence'])
                return reply.get('result')
            raise RuntimeError('recording preparation acknowledgement timed out')
        except Exception:
            try:
                record_event('hook_cancel', request_sequence=request['sequence'])
            except RuntimeError:
                pass
            raise

    owner = bridge.ownership
    owner.manual_changed = lambda *args: async_hook('manual_changed', *args)
    owner.dagger_changed = lambda *args: async_hook('dagger_changed', *args)
    owner.open_segment = lambda *args: blocking_hook('open_segment', *args)
    owner.close_segment = lambda *args: blocking_hook('close_segment', *args)
    owner.capture_ready = lambda: bool(channels['capture_ready'].value and
        0 <= time.monotonic()-channels['capture_ready_at'].value <= .25)
    bridge._action_sink = lambda evidence: async_hook('action', evidence)
    if capture_spec:
        from .publication_capture import PublicationCapture
        bridge._publication_capture = PublicationCapture(
            _RecordingIngress(capture_spec['resources'], capture_spec['finish']),
            selected=lambda: child_selected[0])

    def snapshot():
        result = bridge.status()
        result['control_process'] = dict(pid=os.getpid(), boot_id=channels['boot'],
            actor=True, parent_pid=channels['parent_pid'],
            python_switch_interval_ms=sys.getswitchinterval()*1000,
            applied_negative_generation=applied_parent,
            pending_negative_generation=(_generation(channels['parent_generation']) if authority_blocked() else None),
            **counters)
        result['control_ready'] = bridge.control_ready()
        capture = getattr(bridge, '_publication_capture', None)
        result['publication_capture'] = capture.status() if capture else None
        result['authorization_fence'] = fence()
        result['snapshot_monotonic_ns'] = time.monotonic_ns()
        return result

    def update_snapshot(force=False):
        nonlocal last_snapshot
        channels['ownership_active'].value = bool(owner.active)
        if force or time.monotonic()-last_snapshot >= .1:
            _offer_latest(channels['status'], snapshot())
            last_snapshot = time.monotonic()

    def intervention(name, data, recording_ready, preparing_capture, expected):
        operations = {
            'enable': lambda: owner.enable(preparing_capture=preparing_capture),
            'start_model': lambda: owner.start_model(control_ready=bridge.control_ready(), recording_ready=recording_ready),
            'enable_dagger': lambda: owner.set_dagger(True, preparing_capture=preparing_capture),
            'disable_dagger': lambda: owner.set_dagger(False),
            'takeover': owner.takeover, 'hold': owner.hold, 'disable': owner.disable,
            'recover': bridge.recover_intervention,
            'return': lambda: owner.start_auto(recording_ready=recording_ready, control_ready=bridge.control_ready()),
        }
        if name not in operations:
            raise ValueError('unknown intervention operation')
        applied = [False]
        def apply():
            if name in {'enable', 'enable_dagger'} and not owner.required_peers <= owner.peers.keys():
                raise ValueError('DAgger requires control participants: '+', '.join(sorted(owner.required_peers)))
            if name in {'enable', 'start_model', 'enable_dagger', 'takeover', 'recover', 'return'}:
                guarded(operations[name], expected)
            else:
                operations[name]()
            applied[0] = True
        result = owner.request(data.get('request_id'), data.get('session_id', ''),
                               data.get('expected_epoch'), apply)
        return dict(ownership=result, applied=applied[0], request_id=data.get('request_id'))

    def guarded(operation, expected):
        # Recording preparation may wait for a parent hook. Never hold the
        # parent's urgent-intent lock across that wait. The actor is the sole
        # sender, and checks the shared intent again before actual publication.
        verify(expected)
        try:
            return operation()
        finally:
            with channels['parent_generation'].get_lock():
                changed = expected[1] != channels['parent_generation'].value
            if changed or channels['failure_stop'].is_set():
                bridge._command_output_enabled = False
                bridge._revoke_locked()
                raise ValueError('authorization revoked during request; explicit fresh authorization required')

    def dispatch(request):
        op, args, kwargs = request['op'], request.get('args', ()), request.get('kwargs', {})
        expected = request.get('fence')
        if op == 'clock':
            return owner.ros_clock()
        if op == 'record_barrier':
            return record_event('barrier')['sequence']
        if op == 'capture_begin':
            child_selected[0] = bool(kwargs.pop('selected', True))
            return bridge._publication_capture.begin_generation(**kwargs)
        if op == 'capture_fence':
            return bridge._publication_capture.close_admission()
        if op == 'capture_status':
            return bridge._publication_capture.status()
        if op == 'capture_close':
            if bridge._command_output_enabled:
                raise RuntimeError('publication capture shutdown requires closed control output')
            state = bridge._publication_capture.status()
            if state['inflight'] or state['pending_ingress']:
                raise RuntimeError('publication capture must be drained before close')
            # The proxy already drained the generation. Closing this empty
            # worker is still lifecycle work, not an active control operation.
            bridge._publication_capture.close()
            return None
        if op == 'shutdown':
            bridge._command_output_enabled = False
            bridge._revoke_locked()
            update_snapshot(True)
            bridge._stop.set()
            return None
        if op == 'resume_output':
            def resume():
                bridge.set_command_output_enabled(True)
                bridge.publish(args[0], 'std_msgs/msg/Bool', {'data': False})
                bridge.publish('/control/teleop/enabled', 'std_msgs/msg/Bool',
                               {'data': bridge._command_source == 'vr'})
                flush = getattr(bridge, '_actor_flush_commands', None)
                if flush is not None:
                    flush()
                return dict(authorization_fence=fence(), ownership=owner.snapshot())
            return guarded(resume, expected)
        if op == 'apply_intervention':
            name, data = args
            # Original request validation must precede a stale disable/hold.
            # request() deduplicates before invoking our fenced action. A retry
            # of an already-applied request remains idempotent after its epoch
            # changed; stale new actions still fail the original transaction.
            return intervention(name, data, expected=expected, **kwargs)
        if op.startswith('owner.'):
            method = op[6:]
            if method not in _OWNER_METHODS:
                raise ValueError('unsupported ownership operation')
            if method in {'start_model', 'start_auto', 'recover'}:
                kwargs['control_ready'] = bridge.control_ready()
            if method in _POSITIVE_OWNER and not (method == 'set_dagger' and not args[0]):
                return guarded(lambda: getattr(owner, method)(*args, **kwargs), expected)
            return getattr(owner, method)(*args, **kwargs)
        if op not in _BRIDGE_METHODS:
            raise ValueError('unsupported control operation')
        positive = (op in {'begin_replay', 'recover_intervention', 'set_control_source'} or
                    op == 'set_command_output_enabled' and args[0])
        data_request = op in {'publish', 'publish_raw'}
        if positive or data_request:
            return guarded(lambda: getattr(bridge, op)(*args, **kwargs), expected)
        return getattr(bridge, op)(*args, **kwargs)

    def urgent_check():
        nonlocal hard_stopped, seen_parent
        parent = mp.parent_process()
        if os.getppid() != channels['parent_pid'] or parent is not None and not parent.is_alive():
            channels['failure_stop'].set()
        if channels['failure_stop'].is_set():
            if not hard_stopped:
                hard_stopped = True  # A negative flush can checkpoint recursively.
                bridge.emergency_stop(config.get('stop_topic', '/control/emergency_stop'),
                                      'control process host or IPC failed; manual restart required')
                flush = getattr(bridge, '_actor_flush_commands', None)
                if flush is not None:
                    flush()
                bridge._stop.set()
                update_snapshot(True)
            return False
        parent_gen = _generation(channels['parent_generation'])
        if parent_gen != seen_parent or parent_gen > applied_parent and bridge._command_output_enabled:
            bridge._command_output_enabled = False
            bridge._revoke_locked()
            seen_parent = parent_gen
        return not bridge._stop.is_set()

    def pump():
        nonlocal pumping, applied_parent
        if pumping:
            return
        pumping = True
        try:
            if not urgent_check():
                return
            # Bound administrative work per turn. Status stays at 10 Hz rather
            # than re-running all feedback/UI diagnostics for each RPC.
            deadline = time.monotonic()+.001
            for source in ('urgent', 'requests'):
                for _ in range(4 if source == 'urgent' else 2):
                    if time.monotonic() >= deadline:
                        break
                    try:
                        request = channels[source].get_nowait()
                    except queue.Empty:
                        break
                    try:
                        value = dispatch(request)
                        if _negative_operation(request['op'], request.get('args', ())):
                            generation = request.get('negative_generation')
                            if (type(generation) is int and 0 <= generation <= request['fence'][1]
                                    and request['fence'][0] == channels['boot']):
                                applied_parent = max(applied_parent, generation)
                        response = dict(id=request['id'], result=value)
                    except Exception as exc:
                        response = dict(id=request['id'], error=str(exc), error_type=type(exc).__name__)
                    if not request['id']:
                        continue
                    response['ownership'] = owner.snapshot()
                    response['authorization_fence'] = fence()
                    response['snapshot_monotonic_ns'] = time.monotonic_ns()
                    try:
                        channels['replies'].put_nowait(response)
                    except queue.Full:
                        counters['rpc_replies_dropped'] += 1
                        channels['failure_stop'].set()
            update_snapshot()
        finally:
            pumping = False

    bridge._actor_pump = pump
    bridge._actor_urgent_check = urgent_check
    bridge._actor_authority_blocked = authority_blocked
    bridge._actor_ready = lambda: update_snapshot(True)
    bridge._actor_command_fence = lambda: (_generation(channels['parent_generation']), bridge._output_epoch)
    def allowed(expected, command, data):
        result = tuple(expected) == bridge._actor_command_fence()
        if authority_blocked():
            negative = getattr(bridge, '_actor_negative_command', lambda c,d:c == 'stop')
            if not negative(command, data):
                result = False
        if not result:
            counters['stale_queued_commands'] += 1
        return result
    bridge._actor_command_allowed = allowed
    try:
        bridge._run()
    except BaseException as exc:
        _offer_latest(channels['status'], dict(state='error', error=type(exc).__name__+': '+str(exc)))
    finally:
        # Normal shutdown is drained by the parent before this point. Crashes
        # remain explicit; never let a writer stall prevent process shutdown.
        capture = getattr(bridge, '_publication_capture', None)
        if capture is not None:
            state = capture.status()
            if not state.get('pending_ingress') and not state.get('inflight'):
                try:
                    capture.close()
                except Exception:
                    pass
        # Log/UI snapshots are explicitly disposable. A paused parent observer
        # must not hold process exit in multiprocessing's feeder finalizer.
        for name in ('observers', 'observer_ros', 'status'):
            channels[name].cancel_join_thread()


class CoreProcessClient:
    """Cached observations and bounded synchronous administrative requests."""
    def __init__(self, config, *, on_event=lambda *a: None, hooks=None,
                 fallback_stop=lambda *a: None, bridge_factory=None, on_observer_ros=None):
        # Runtime finishes configuring publication capture after constructing
        # the facade. Spawn must see that completed configuration.
        self.config = config
        self.context = mp.get_context('spawn')
        self.boot = uuid.uuid4().hex
        self.channels = {name: self.context.Queue(maxsize=size) for name, size in
                         [('requests', 64), ('urgent', 16), ('replies', 64),
                          ('status', 2), ('observers', 256), ('observer_ros', 256), ('hooks', 2048), ('hook_replies', 8)]}
        self.channels.update(boot=self.boot, parent_pid=os.getpid(),
            parent_generation=self.context.Value('Q', 0), core_epoch=self.context.Value('q', -1),
            ownership_active=self.context.Value('b', False), capture_ready=self.context.Value('b', False),
            capture_ready_at=self.context.Value('d', 0.),
            failure_stop=self.context.Event(), observer_ros_enabled=on_observer_ros is not None)
        self.on_event, self.hooks, self.fallback_stop = on_event, hooks or {}, fallback_stop
        self.bridge_factory = bridge_factory
        self.on_observer_ros = on_observer_ros
        self._observer_ros_errors = 0
        self._observer_ros_last_error = ''
        self.capture_spec = None
        self.process = None
        self._cache = dict(state='starting', ownership=dict(state='OFF'), command_mux=dict(output_enabled=False))
        self._lock, self._pending_lock = threading.RLock(), threading.Lock()
        self._pending, self._request_seq = {}, 0
        self._failure = ''
        self._fallback_completed = threading.Event()
        self._fallback_error = ''
        self._failure_signal_thread = None
        self._closing = False
        self._stopped = threading.Event()
        self._threads = []
        self._hook_condition = threading.Condition()
        self._last_hook_done = 0
        self._hook_context = threading.local()
        self._last_snapshot_received = time.monotonic()

    def snapshot(self):
        with self._lock:
            result = copy.deepcopy(self._cache)
            failure = self._failure
            result.setdefault('control_process', {}).update(
                observer_ros_errors=self._observer_ros_errors,
                observer_ros_last_error=self._observer_ros_last_error,
                fallback_stop_completed=self._fallback_completed.is_set(),
                fallback_stop_error=self._fallback_error)
        if failure:
            result.update(state='error', error=failure)
            result.setdefault('command_mux', {})['output_enabled'] = False
        result['isolated_control_process'] = True
        return result

    def authorization_fence(self):
        if self._failure:
            return self.boot, -1, -1
        try:
            result = self.boot, int(self._shared_value('parent_generation')), int(self._shared_value('core_epoch'))
        except RuntimeError:
            return self.boot, -1, -1
        return (self.boot, -1, -1) if self._failure else result

    def _shared_value(self, name, value=None, *, write=False):
        """A killed owner cannot strand a parent caller on an orphan lock."""
        if self._failure:
            raise RuntimeError(self._failure)
        shared = self.channels[name]
        lock = shared.get_lock()
        if not lock.acquire(timeout=.02):
            self._failed('control shared '+name+' lock timed out; explicit restart required')
            raise RuntimeError(self._failure)
        try:
            if self._failure:
                raise RuntimeError(self._failure)
            if write:
                shared.get_obj().value = value
            return shared.get_obj().value
        finally:
            lock.release()

    def revoke_intent(self):
        if self._failure:
            return -1
        shared = self.channels['parent_generation']
        lock = shared.get_lock()
        if not lock.acquire(timeout=.02):
            self._failed('control shared parent_generation lock timed out; explicit restart required')
            return -1
        try:
            if self._failure:
                return -1
            shared.get_obj().value += 1
            generation = int(shared.get_obj().value)
        finally:
            lock.release()
        with self._lock:
            self._cache.setdefault('command_mux', {})['output_enabled'] = False
        return generation

    def _failed(self, reason):
        with self._lock:
            first = not self._failure
            self._failure = self._failure or reason
        if first:
            # This fallback uses only the independent parent telemetry bridge.
            # A terminated process may have died inside *any* shared lock.
            try:
                self.fallback_stop(reason)
            except Exception as exc:
                with self._lock:
                    self._fallback_error = str(exc)
            finally:
                self._fallback_completed.set()
            def signal_owner():
                self.channels['failure_stop'].set()
                shared = self.channels['parent_generation']
                with shared.get_lock():
                    shared.get_obj().value += 1
            self._failure_signal_thread = threading.Thread(target=signal_owner,
                name='control-failure-signal', daemon=True)
            self._failure_signal_thread.start()

    def _cache_snapshot(self, value):
        if value:
            with self._lock:
                self._last_snapshot_received = time.monotonic()
                if value.get('snapshot_monotonic_ns', 0) >= self._cache.get('snapshot_monotonic_ns', 0):
                    self._cache = value

    def start(self, timeout=15.):
        if self.process is not None:
            return
        self.process = self.context.Process(target=_control_worker,
            args=(copy.deepcopy(self.config), self.channels, self.capture_spec, self.bridge_factory),
            name='teleop-control-owner', daemon=True)
        self.process.start()
        for name, target in [('control-ipc-reader', self._receive), ('control-record-hooks', self._record_hooks),
                             ('control-observers', self._observers), ('control-observer-ros', self._observer_ros),
                             ('control-capture-ready', self._capture_readiness), ('control-watchdog', self._watchdog)]:
            thread = threading.Thread(target=target, name=name, daemon=True)
            self._threads.append(thread)
            thread.start()
        deadline = time.monotonic()+timeout
        while time.monotonic() < deadline:
            status = self.snapshot()
            if status['state'] == 'running':
                return
            if self._failure or status['state'] == 'error':
                raise RuntimeError(status.get('error') or self._failure)
            time.sleep(.01)
        self._failed('control process startup timed out')
        raise RuntimeError(self._failure)

    def call(self, op, *args, expected_fence=None, urgent=False, allow_failed=False, timeout=1.,
             negative_generation=None, **kwargs):
        if self.process is None or not self.process.is_alive():
            self._failed('control process unavailable; explicit restart required')
            raise RuntimeError(self._failure)
        if self._failure:
            raise RuntimeError(self._failure)
        with self._pending_lock:
            self._request_seq += 1
            identifier = self._request_seq
            event = threading.Event()
            self._pending[identifier] = [event, None]
        request = dict(id=identifier, op=op, args=args, kwargs=kwargs,
                       fence=tuple(expected_fence) if expected_fence is not None else self.authorization_fence())
        if _negative_operation(op, args):
            request['negative_generation'] = (request['fence'][1] if negative_generation is None else negative_generation)
        try:
            try:
                self.channels['urgent' if urgent else 'requests'].put_nowait(request)
            except queue.Full:
                self._failed('control administrative queue full')
                raise RuntimeError(self._failure)
            if not event.wait(timeout):
                self._failed('control request timed out; authorization fenced')
                raise RuntimeError(self._failure)
            with self._pending_lock:
                reply = self._pending[identifier][1]
            if reply.get('error'):
                error = ValueError if reply.get('error_type') in {'ValueError', 'OwnershipConflict'} else RuntimeError
                raise error(reply['error'])
            return reply.get('result')
        finally:
            with self._pending_lock:
                self._pending.pop(identifier, None)

    def notify(self, op, *args, urgent=True, negative_generation=None, **kwargs):
        """Negative input edges never wait on a VR receive/timeout thread."""
        if self.process is None or not self.process.is_alive() or self._failure:
            self._failed('control process unavailable; explicit restart required')
            return False
        request = dict(id=0, op=op, args=args, kwargs=kwargs, fence=self.authorization_fence())
        if _negative_operation(op, args):
            request['negative_generation'] = (request['fence'][1] if negative_generation is None else negative_generation)
        try:
            self.channels['urgent' if urgent else 'requests'].put_nowait(request)
            return True
        except queue.Full:
            self._failed('control administrative queue full')
            return False

    def _receive(self):
        while not self._stopped.is_set():
            try:
                reply = self.channels['replies'].get(timeout=.01)
                self._cache_snapshot(reply.get('snapshot'))
                with self._lock:
                    if 'ownership' in reply:
                        self._cache['ownership'] = reply['ownership']
                    if 'authorization_fence' in reply:
                        self._cache['authorization_fence'] = reply['authorization_fence']
                    self._cache['snapshot_monotonic_ns'] = reply.get('snapshot_monotonic_ns',
                        self._cache.get('snapshot_monotonic_ns', 0))
                with self._pending_lock:
                    pending = self._pending.get(reply['id'])
                    if pending is not None:
                        pending[1] = reply
                        pending[0].set()
            except queue.Empty:
                pass
            while True:
                try:
                    self._cache_snapshot(self.channels['status'].get_nowait())
                except queue.Empty:
                    break
            if not self._closing and self.process is not None and not self.process.is_alive():
                self._failed('control process exited; explicit restart required')
            if (not self._closing and self._cache.get('state') == 'running' and
                    time.monotonic()-self._last_snapshot_received > 1.):
                self._failed('control process heartbeat timed out; explicit restart required')
        
    def _capture_readiness(self):
        while not self._stopped.is_set():
            callback = self.hooks.get('capture_ready')
            if callback is not None:
                try:
                    ready = bool(callback())
                except Exception:
                    ready = False
                try:
                    self._shared_value('capture_ready', ready, write=True)
                    self._shared_value('capture_ready_at', time.monotonic(), write=True)
                except RuntimeError:
                    return
            self._stopped.wait(.05)

    def _watchdog(self):
        # Queue.get() can itself wait on a partial message from a killed feeder.
        # Liveness must have an independent, entirely parent-local clock path.
        while not self._stopped.wait(.05):
            if self._closing:
                continue
            if self.process is not None and not self.process.is_alive():
                self._failed('control process exited; explicit restart required')
            elif self._cache.get('state') == 'running' and time.monotonic()-self._last_snapshot_received > 1.:
                self._failed('control process heartbeat timed out; explicit restart required')

    def _observers(self):
        while not self._stopped.is_set():
            try:
                event, outputs = self.channels['observers'].get(timeout=.05)
                self.on_event(event, outputs)
            except queue.Empty:
                pass
            except Exception:
                pass  # The observer is deliberately outside the real-time process.

    def _observer_ros(self):
        # A blocked UI observer cannot hold ownership DDS snapshots or vice
        # versa. Neither of these consumers shares the recording hook FIFO.
        while not self._stopped.is_set():
            try:
                topic, payload = self.channels['observer_ros'].get(timeout=.05)
                if topic not in _READONLY_ROS_TOPICS or not isinstance(payload, dict):
                    raise ValueError('unsupported read-only control observation')
                if self.on_observer_ros is not None:
                    if self.on_observer_ros(topic, payload) is False:
                        raise RuntimeError('read-only ROS observer rejected by telemetry queue')
            except queue.Empty:
                continue
            except Exception as exc:
                with self._lock:
                    self._observer_ros_errors += 1
                    self._observer_ros_last_error = str(exc)

    def _record_hooks(self):
        opened = {}
        while not self._stopped.is_set():
            try:
                item = self.channels['hooks'].get(timeout=.05)
            except queue.Empty:
                continue
            sequence, kind, name = item['sequence'], item['kind'], item['name']
            self._hook_context.clock_ns = item['clock_ns']
            reply = dict(sequence=sequence)
            try:
                if kind in {'hook', 'hook_request'}:
                    callback = self.hooks.get(name)
                    if callback is None and kind == 'hook_request':
                        raise RuntimeError('recording hook unavailable: '+name)
                    if (kind == 'hook_request' and name == 'open_segment'
                            and tuple(item['fence']) != self.authorization_fence()):
                        raise RuntimeError('recording hook belongs to revoked authorization')
                    result = callback(*item['args']) if callback else None
                    reply['result'] = result
                    if name == 'open_segment':
                        opened[sequence] = item
                elif kind == 'hook_ack':
                    opened.pop(item['request_sequence'], None)
                elif kind == 'hook_cancel':
                    original = opened.pop(item['request_sequence'], None)
                    if original is not None:
                        callback = self.hooks.get('close_segment')
                        if callback:
                            callback(original['args'][0], item['clock_ns'], 0,
                                     'control revoked or recording preparation timed out')
            except Exception as exc:
                reply['error'] = str(exc)
            finally:
                self._hook_context.clock_ns = None
                if kind == 'hook_request':
                    try:
                        self.channels['hook_replies'].put_nowait(reply)
                    except queue.Full:
                        self._failed('recording hook acknowledgements could not be delivered')
                with self._hook_condition:
                    self._last_hook_done = max(self._last_hook_done, sequence)
                    self._hook_condition.notify_all()

    def flush_recording_events(self, timeout=2.):
        sequence = self.call('record_barrier', timeout=timeout, allow_failed=True)
        deadline = time.monotonic()+timeout
        with self._hook_condition:
            while self._last_hook_done < sequence:
                remaining = deadline-time.monotonic()
                if remaining <= 0:
                    raise RuntimeError('recording event drain timed out; capture incomplete')
                self._hook_condition.wait(remaining)

    def stop(self):
        if self.process is None:
            return
        self._closing = True
        try:
            if self.process.is_alive():
                if not self._failure:
                    try:
                        self.call('shutdown', urgent=True)
                    except RuntimeError as exc:
                        self._failed(str(exc))
                self.process.join(3.)
            if self.process.is_alive():
                self.process.terminate()
                self.process.join(2.)
        finally:
            self._stopped.set()
            for value in self.channels.values():
                if hasattr(value, 'cancel_join_thread'):
                    value.cancel_join_thread()
            for thread in self._threads:
                thread.join(1.)


class RemoteOwnership:
    def __init__(self, client):
        object.__setattr__(self, '_client', client)

    def __setattr__(self, name, value):
        if name in _HOOKS:
            self._client.hooks[name] = value
        else:
            raise AttributeError('remote ownership attributes are read-only')

    def snapshot(self, **_kwargs):
        return self._client.snapshot().get('ownership', {'state': 'OFF'})

    def ros_clock(self):
        stamp = getattr(self._client._hook_context, 'clock_ns', None)
        if stamp is not None:
            return stamp
        if not self._client._closing and not self._client._failure:
            return self._client.call('clock')
        stamp = self.snapshot().get('published_stamp_ns')
        if not stamp:
            raise RuntimeError('control clock unavailable')
        return stamp

    @property
    def active(self):
        if self._client._failure:
            return False
        try:
            return bool(self._client._shared_value('ownership_active'))
        except RuntimeError:
            return False

    def __getattr__(self, name):
        if name in _OWNER_METHODS:
            def invoke(*args, expected_fence=None, **kwargs):
                urgent = name in {'disable', 'hold', 'fault', 'input_lost'}
                negative_generation = None
                if name in {'disable', 'hold', 'fault'}:
                    negative_generation = self._client.revoke_intent()
                if name == 'input_lost':
                    return self._client.notify('owner.'+name, *args, **kwargs)
                return self._client.call('owner.'+name, *args,
                    expected_fence=expected_fence, urgent=urgent, negative_generation=negative_generation, **kwargs)
            return invoke
        aliases = {'instance': 'controller_instance_id', 'session': 'session_id', 'epoch': 'epoch'}
        value = self.snapshot()
        if name == 'token':
            from .control_ownership import ControlToken
            return ControlToken(*(value.get(k, 0 if k == 'epoch' else '') for k in
                ('controller_instance_id', 'session_id', 'epoch', 'source', 'intervention_id')))
        if name == 'required_peers':
            return frozenset(value.get(name, ('solver', 'teleop')))
        if name == 'recording_enabled':
            return value.get(name, self._client.config.get('intervention', {}).get('automatic_recording', True))
        if name == 'peers':
            return value.get(name, {})
        if name in value or name in aliases:
            return value.get(aliases.get(name, name), '' if name != 'epoch' else 0)
        raise AttributeError(name)


class RemotePublicationCapture:
    def __init__(self, client, selected):
        self.client, self.selected = client, selected
        self._closed = False
        self._error = ''
        self._admission_fence = None
        self._last_status_at = 0
        self._last = dict(generation=0, active=False, captured=0, enqueued=0, dropped=0,
                          failed=0, outside_preroll=0, queued=0, inflight=0, pending_ingress=0, error='')

    def status(self):
        snapshot = self.client.snapshot()
        current = snapshot.get('publication_capture')
        observed_at = snapshot.get('snapshot_monotonic_ns', 0)
        if current and not self._closed and observed_at >= self._last_status_at:
            self._last = current
        result = dict(self._last)
        if self._error or self.client._failure:
            result['error'] = self._error or self.client._failure
        return result

    def begin_generation(self, **kwargs):
        if self._closed:
            raise RuntimeError('publication capture is closed')
        generation = self.client.call('capture_begin', selected=bool(self.selected()), **kwargs)
        self._admission_fence = None
        self._error = ''
        return generation

    def close_admission(self):
        if self._admission_fence is not None:
            return self._admission_fence
        if self._closed:
            self._admission_fence = time.time_ns(), time.monotonic_ns()
            return self._admission_fence
        try:
            self._admission_fence = tuple(self.client.call('capture_fence', urgent=True, allow_failed=True))
        except RuntimeError as exc:
            self._error = str(exc)+'; publication fence unavailable, capture incomplete'
            # Cleanup must complete even if the owner died. This is explicitly
            # an incomplete fallback boundary, never a claim of a core fence.
            self._admission_fence = time.time_ns(), time.monotonic_ns()
        return self._admission_fence

    def end_generation(self, timeout=10.):
        self.close_admission()
        deadline = time.monotonic()+timeout
        while not self._closed:
            try:
                self._last = self.client.call('capture_status', allow_failed=True)
                self._last_status_at = self.client.snapshot().get('snapshot_monotonic_ns', 0)
            except RuntimeError as exc:
                self._error = str(exc)+'; published action drain incomplete'
                return
            if not self._last['inflight'] and not self._last['pending_ingress']:
                return
            if time.monotonic() >= deadline:
                self._error = 'published action drain timed out; capture incomplete'
                return
            time.sleep(.005)

    def close(self):
        if self._closed:
            return
        self.end_generation()
        if not self._error and not self._last['inflight'] and not self._last['pending_ingress']:
            try:
                self.client.call('capture_close', allow_failed=True)
            except RuntimeError as exc:
                self._error = str(exc)+'; publication capture incomplete'
        self._closed = True


class IsolatedRosBridge:
    """Parent facade: telemetry is local; all authority resides in the child."""
    isolated_control_process = True

    def __init__(self, config, on_event, on_frame=None, camera_topic='', camera_topics=None,
                 *, bridge_factory=None):
        from .ros_bridge import RosBridge
        self.config, self.on_event = config, on_event
        self._legacy_enabled = bool(config.get('legacy_control_profile'))
        telemetry_config = copy.deepcopy(config)
        telemetry_config.pop('legacy_control_profile', None)
        telemetry_config.pop('robot_api_profile', None)
        telemetry_config['command_mux'] = {**telemetry_config.get('command_mux', {}), 'enabled': False}
        self._telemetry = RosBridge(telemetry_config, on_event, on_frame, camera_topic, camera_topics)
        self._action_sink = lambda evidence: None
        hooks = {'action': lambda evidence: self._action_sink(evidence)}
        self._client = CoreProcessClient(config, on_event=on_event, hooks=hooks,
            fallback_stop=self._fallback_stop, bridge_factory=bridge_factory,
            on_observer_ros=lambda topic, payload: publish_readonly_observation(self._telemetry, topic, payload))
        self.ownership = RemoteOwnership(self._client)
        self._boundary = SimpleNamespace(status=lambda: self._client.snapshot().get('legacy_control', {}))
        self._publication_capture = None

    def _fallback_stop(self, reason):
        # Negative hardware stop only; no fallback publisher can reopen output.
        self._telemetry.publish(self.config.get('stop_topic', '/control/emergency_stop'),
                                'std_msgs/msg/Bool', {'data': True})
        def log_failure():
            self.on_event({'kind': 'log', 'source': 'control_process',
                'payload': {'level': 'error', 'message': reason}}, ['websocket'])
        threading.Thread(target=log_failure, name='control-failure-log', daemon=True).start()

    def create_publication_capture(self, recorder, selected):
        if self._client.process is not None:
            raise RuntimeError('publication ingress must be configured before control process starts')
        self._client.capture_spec = dict(resources=recorder.ipc_resources(), finish=recorder._finish_event)
        self._publication_capture = RemotePublicationCapture(self._client, selected)
        return self._publication_capture

    def start(self):
        self._telemetry.start()
        try:
            self._client.start()
        except Exception:
            self._client.stop()
            self._telemetry.stop()
            raise

    def stop(self):
        try:
            if self._client.process is not None and self._client.process.is_alive():
                try:
                    self.set_command_output_enabled(False)
                    self.flush_recording_events()
                except (ValueError, RuntimeError):
                    pass
            if self._publication_capture is not None:
                self._publication_capture.close()
        finally:
            self._client.stop()
            self._telemetry.stop()

    def status(self):
        result = self._telemetry.status()
        core = self._client.snapshot()
        for key in ('command_mux', 'ownership', 'legacy_control', 'control_actor', 'callback_timings', 'slow_callbacks',
                    'command_stage_timings', 'slow_command_stages', 'control_contention_samples',
                    'control_process', 'authorization_fence', 'snapshot_monotonic_ns'):
            if key in core:
                result[key] = core[key]
        result['isolated_control_process'] = True
        if core.get('state') != 'running':
            result.update(state=core.get('state', 'starting'), error=core.get('error'))
        return result

    def get_topic_health(self):
        return self._telemetry.get_topic_health()

    def authorization_fence(self):
        return self._client.authorization_fence()

    def publish(self, topic, msg_type, data, *, expected_fence=None):
        if str(topic).startswith(_CONTROL_PREFIXES) or str(topic) == '/io_teleop/joint_cmd':
            if str(topic).endswith('/emergency_stop') and data.get('data') is True:
                return self.emergency_stop(topic, 'explicit emergency stop publication')
            return self._client.call('publish', topic, msg_type, data, expected_fence=expected_fence)
        return self._telemetry.publish(topic, msg_type, data)

    def publish_raw(self, topic, msg_type, raw_data, *, expected_fence=None):
        if str(topic).startswith(_CONTROL_PREFIXES) or str(topic) == '/io_teleop/joint_cmd':
            return self._client.call('publish_raw', topic, msg_type, raw_data, expected_fence=expected_fence)
        return self._telemetry.publish_raw(topic, msg_type, raw_data)

    def emergency_stop(self, topic, reason):
        generation = self._client.revoke_intent()
        return self._client.notify('emergency_stop', topic, reason, negative_generation=generation)

    def set_command_output_enabled(self, enabled, *, expected_fence=None):
        generation = None
        if not enabled:
            generation = self._client.revoke_intent()
        return self._client.call('set_command_output_enabled', bool(enabled),
                                 expected_fence=expected_fence, urgent=not enabled, negative_generation=generation)

    def resume_output(self, stop_topic, *, expected_fence):
        return self._client.call('resume_output', stop_topic, expected_fence=expected_fence)

    def recover_intervention(self, *, expected_fence=None):
        return self._client.call('recover_intervention', expected_fence=expected_fence)

    def set_control_source(self, source):
        source = str(source).strip().lower()
        if source not in {'vr', 'exoskeleton', 'motion'}:
            raise ValueError('control source must be vr, exoskeleton or motion')
        return self._client.call('set_control_source', source, urgent=True)

    def begin_replay(self):
        return self._client.call('begin_replay')

    def end_replay(self, require_reset=True):
        generation = self._client.revoke_intent()
        return self._client.call('end_replay', require_reset, urgent=True, negative_generation=generation)

    def request_manual_home(self):
        return self._client.call('request_manual_home')

    def control_ready(self):
        return bool(self._client.snapshot().get('control_ready'))

    def apply_intervention(self, name, data, *, recording_ready=False, preparing_capture=False,
                           expected_fence=None):
        return self._client.call('apply_intervention', name, data,
            recording_ready=recording_ready, preparing_capture=preparing_capture,
            expected_fence=expected_fence, urgent=name in {'disable', 'hold', 'disable_dagger'})

    def flush_recording_events(self, timeout=2.):
        return self._client.flush_recording_events(timeout)

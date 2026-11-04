"""Robot-side intervention transactions, independent of ROS, devices and models.

The caller shares this lock with the final actuator publication. Hooks must be
bounded, memory-only operations; disk work belongs to the recording workers.
An acknowledgement is an execution fact, never a timer-based assumption.
"""
from collections import OrderedDict, deque
from dataclasses import asdict, dataclass
import math
import threading
import time
import uuid


@dataclass(frozen=True)
class ControlToken:
    controller_instance_id: str
    session_id: str
    epoch: int
    source: str
    intervention_id: str = ''

    def key(self):
        return ':'.join(('hc1', self.controller_instance_id, self.session_id,
                         str(self.epoch), self.source, self.intervention_id))

    @classmethod
    def from_key(cls, value):
        fields = value.split(':')
        if len(fields) != 6 or fields[0] != 'hc1' or not fields[3].isdigit():
            raise ValueError('invalid internal authority identity')
        return cls(fields[1], fields[2], int(fields[3]), fields[4], fields[5])


class OwnershipConflict(ValueError):
    pass


class ControlOwnership:
    def __init__(self, *, lock=None, clock=time.monotonic_ns, ros_clock=time.time_ns,
                 revoke=lambda: None, permit=lambda *args: None,
                 open_segment=lambda *args: None, close_segment=lambda *args: None,
                 emit=lambda event: None, transition_timeout_sec=2., input_timeout_sec=.3,
                 required_peers=('motion', 'teleop'), recording_enabled=True):
        self.lock = lock or threading.RLock()
        self.required_peers = frozenset(required_peers)
        self.recording_enabled = bool(recording_enabled)
        self.clock, self.ros_clock = clock, ros_clock
        self.revoke, self.permit = revoke, permit
        self.open_segment, self.close_segment, self.emit = open_segment, close_segment, emit
        self.manual_changed = lambda token, stamp, reason: None
        self.dagger_changed = lambda enabled: None
        self.instance = uuid.uuid4().hex
        self.session = ''
        self.epoch = 0
        self.state, self.phase, self.reason = 'OFF', '', ''
        self.token = ControlToken(self.instance, '', 0, '')
        self.cutoff_ns = 0
        self.deadline_ns = 0
        self.transition_timeout_ns = int(transition_timeout_sec * 1e9)
        self.input_timeout_ns = int(input_timeout_sec * 1e9)
        self.peers = {}
        self.peer_times = {}
        self.acks = set()
        self.events = deque(maxlen=512)
        self.requests = OrderedDict()
        self.event_seq = 0
        self.command_seq = 0
        self.segment_open = False
        self.resume_auto = False
        self.gripped = False
        self.input_armed = False
        self.input_valid = False
        self.input_time_ns = 0
        self.input_device = ''
        self.input_sequence = -1
        self.preparing_capture = False
        self.dagger_enabled = False
        self.input_ready_stamp_ns = 0
        self.input_diagnostics = dict(received=0, accepted=0, rejected=0,
                                      last_rejection='', last_loss_reason='')
        self.capture_ready = lambda: False
        self.recording_error = ''
        self.release_ns = None
        self.metrics = deque(maxlen=10000)
        self.pending_auto_metric = None

    @property
    def active(self):
        return self.state != 'OFF'

    def _event(self, kind, **fields):
        self.event_seq += 1
        event = dict(schema='hc-control-event/v1', sequence=self.event_seq,
                     monotonic_ns=self.clock(), stamp_ns=self.ros_clock(), kind=kind,
                     state=self.state, phase=self.phase, reason=self.reason,
                     **asdict(self.token), **fields)
        self.events.append(event)
        self.emit(event)

    def _close(self, reason):
        if self.segment_open:
            self.segment_open = False
            try:
                self.close_segment(self.token, self.ros_clock(), self.command_seq, reason)
            except Exception as exc:
                self.recording_error = str(exc)
            self._event('manual_close', last_command_seq=self.command_seq, end_reason=reason)

    def _invalidate(self, state, source='', reason=''):
        # This hook closes the final gate before any capture or peer work.
        self.revoke()
        if self.state == 'MANUAL':
            self.manual_changed(None, self.ros_clock(), reason)
        self._close(reason)
        self.epoch += 1
        self.cutoff_ns = self.ros_clock()
        self.state, self.phase, self.reason = state, 'revoke', reason
        self.token = ControlToken(self.instance, self.session, self.epoch, source,
                                  uuid.uuid4().hex if source == 'vr' else '')
        self.command_seq = 0
        self.acks.clear()
        self.deadline_ns = self.clock() + self.transition_timeout_ns
        self._event('revoked')
        self.permit(self.token, 'revoke', self.cutoff_ns, self.active)

    def _set_dagger_enabled(self, enabled):
        if self.dagger_enabled != bool(enabled):
            self.dagger_enabled = bool(enabled)
            self.dagger_changed(self.dagger_enabled)

    def enable(self, *, preparing_capture=False, dagger_enabled=True):
        with self.lock:
            if self.active:
                raise OwnershipConflict('intervention session already active')
            self.session = uuid.uuid4().hex
            self._set_dagger_enabled(dagger_enabled)
            self.input_armed = self.input_valid = self.gripped = self.resume_auto = False
            self.input_sequence = -1
            self.preparing_capture = preparing_capture
            self.input_ready_stamp_ns = self.ros_clock()
            self.recording_error = ''
            self._invalidate('HOLD', reason='session enabled; explicit start required')
            return self.snapshot()

    def finish_capture_preparation(self, session_id):
        with self.lock:
            if not self.preparing_capture or self.session != session_id or self.state not in {'HOLD', 'AUTO', 'RETURNING'}:
                return
            self.preparing_capture = False
            self.input_ready_stamp_ns = self.ros_clock()
            # Samples observed while writers/subscriptions were starting must
            # not arm a later held Grip. Require a fresh release after readiness.
            self.input_armed = self.input_valid = self.gripped = False
            self.reason = 'DAgger capture prepared; fresh Grip release required'
            self._event('capture_ready')

    def disable(self):
        with self.lock:
            self._set_dagger_enabled(False)
            self.preparing_capture = False
            self.resume_auto = False
            self._invalidate('OFF', reason='session disabled; output remains closed')

    def start_model(self, *, control_ready, recording_ready=False):
        """Start/resume model authority; a new session never enables takeover."""
        with self.lock:
            if not control_ready or not self.required_peers <= self.peers.keys():
                raise OwnershipConflict('feedback or control participants are not ready')
            if not self.active:
                self.enable(dagger_enabled=False)
            self.start_auto(control_ready=control_ready, recording_ready=recording_ready)

    def set_dagger(self, enabled, *, preparing_capture=False):
        """Toggle manual takeover within a model session, preserving pause/fault."""
        with self.lock:
            if not self.active:
                raise OwnershipConflict('start model control before enabling DAgger')
            if enabled and self.state not in {'AUTO', 'HOLD'}:
                raise OwnershipConflict('DAgger enable requires stable model control or pause')
            if self.dagger_enabled == enabled:
                return
            self._set_dagger_enabled(enabled)
            self.preparing_capture = bool(enabled and preparing_capture)
            self.input_ready_stamp_ns = self.ros_clock()
            self.input_armed = self.input_valid = self.gripped = False
            if not enabled and self.state in {'TAKEOVER', 'MANUAL'}:
                self.release()
            self._event('dagger_enabled' if enabled else 'dagger_disabled')

    def start_auto(self, *, recording_ready, control_ready):
        """Explicitly authorize program actions; headset presence is optional."""
        with self.lock:
            if self.preparing_capture or self.state != 'HOLD':
                raise OwnershipConflict('start requires HOLD and completed preparation')
            if self.dagger_enabled and self._input_fresh() and self.gripped:
                raise OwnershipConflict('release the active manual Grip before starting')
            if (self.dagger_enabled and self.recording_enabled and not recording_ready) or not control_ready or not self.required_peers <= self.peers.keys():
                raise OwnershipConflict('capture, feedback or control participants are not ready')
            self.resume_auto = True
            self._invalidate('RETURNING', 'motion', 'explicit start')

    def takeover(self):
        with self.lock:
            if (not self.dagger_enabled or self.preparing_capture or self.state not in {'AUTO', 'HOLD', 'RETURNING'}
                    or not self._input_fresh() or not self.input_armed or not self.gripped):
                return False
            self.resume_auto = self.state == 'AUTO' or (self.state == 'RETURNING' and self.resume_auto)
            self.release_ns = None
            self._invalidate('TAKEOVER', 'vr', 'manual request')
            return True

    def release(self):
        with self.lock:
            if self.state not in {'TAKEOVER', 'MANUAL'}:
                return False
            self.release_ns = self.clock()
            if self.resume_auto:
                self._invalidate('RETURNING', 'motion', 'normal release')
            else:
                self._invalidate('HOLD', reason='released into explicit hold')
            return True

    def hold(self, reason='explicit pause'):
        with self.lock:
            if not self.active:
                return
            # Neither pause nor an ordinary resume clears a fault/estop latch.
            if self.state in {'FAULT', 'ESTOP'}:
                return
            # A Grip sample queued before this pause cannot immediately take
            # over again. Observe a fresh release before accepting another edge.
            self.input_armed = False
            self.input_ready_stamp_ns = self.ros_clock()
            if self.state == 'HOLD' and not self.resume_auto:
                return
            self.resume_auto = False
            self._invalidate('HOLD', reason=reason)

    def fault(self, reason, *, estop=False):
        with self.lock:
            if not self.active or self.state == 'ESTOP' or (self.state == 'FAULT' and not estop):
                return
            self.resume_auto = self.input_armed = False
            self._invalidate('ESTOP' if estop else 'FAULT', reason=reason)

    def recover(self, *, control_ready):
        with self.lock:
            if self.state not in {'FAULT', 'ESTOP', 'HOLD'}:
                self.hold('explicit resume returns to HOLD')
                return
            if not control_ready:
                raise OwnershipConflict('fault conditions have not cleared')
            self.input_armed = self.resume_auto = False
            self.input_ready_stamp_ns = self.ros_clock()
            self._invalidate('HOLD', reason='explicit recovery; explicit program start required')

    def _input_fresh(self):
        return self.input_valid and self.clock()-self.input_time_ns <= self.input_timeout_ns

    def input_lost(self, reason):
        """Disarm manual input; only an active manual handover/output must stop."""
        with self.lock:
            self.input_valid = self.input_armed = self.gripped = False
            self.input_ready_stamp_ns = self.ros_clock()
            self.input_diagnostics.update(last_loss_reason=reason,
                                          last_loss_stamp_ns=self.input_ready_stamp_ns)
            if self.state in {'TAKEOVER', 'MANUAL'}:
                self.fault(reason)
                if reason == 'teleop input timeout':
                    self._event('manual_input_timeout',
                        input_gap_ms=(self.clock()-self.input_time_ns)/1e6,
                        limit_ms=self.input_timeout_ns/1e6,
                        last_accepted_stamp_ns=self.input_diagnostics.get('last_accepted_source_ns'),
                        last_sequence=self.input_sequence)

    def _reject_input(self, reason):
        """Keep bounded diagnostic facts without changing the input gate."""
        self.input_diagnostics['rejected'] += 1
        self.input_diagnostics.update(
            last_rejection=reason,
            last_rejected_source_ns=self.input_diagnostics['last_source_ns'],
            last_rejected_age_ms=self.input_diagnostics['last_age_ms'],
            last_rejected_callback_ns=self.input_diagnostics['last_callback_ns'])
        return False

    def input(self, *, device_instance, sequence, valid, gripped, stamp_ns,
              pause_requested=False, resume_requested=False):
        """Accept a semantic sample, with pause taking priority over Grip.

        Return True only for a fresh valid sample. The caller may dispatch an
        explicit resume after this method, under the same control lock. Resume
        suppresses Grip handling in its frame; it never clears a fault here.
        """
        with self.lock:
            now = self.clock()
            # A new packet must not reuse an armed release from before a gap,
            # even when it arrives before the periodic watchdog runs.
            if self.input_valid and not self._input_fresh():
                self.input_lost('teleop input timeout')
            age = self.ros_clock() - stamp_ns
            self.input_diagnostics['received'] += 1
            self.input_diagnostics.update(last_source_ns=stamp_ns,
                last_callback_ns=stamp_ns+age, last_age_ms=age/1e6,
                last_sequence=sequence)
            if sequence <= self.input_sequence and device_instance == self.input_device:
                return self._reject_input('out_of_order_sequence')
            if not -50_000_000 <= age <= self.input_timeout_ns:
                return self._reject_input('sample_age_out_of_range')
            if self.active and stamp_ns < self.input_ready_stamp_ns:
                return self._reject_input('sample_before_rearm_cutoff')
            device_changed = bool(self.input_device and device_instance != self.input_device and self.active)
            if device_instance != self.input_device:
                if self.input_device and self.active:
                    self.input_lost('teleop input instance changed')
                self.input_device, self.input_sequence, self.input_armed = device_instance, -1, False
            self.input_sequence, self.input_time_ns = sequence, now
            self.input_valid = bool(valid)
            if not valid:
                self.input_lost('teleop input invalid/tracking lost')
                return self._reject_input('invalid_semantic_input')
            if self.preparing_capture:
                self.input_valid = self.input_armed = self.gripped = False
                return self._reject_input('capture_preparing')
            if device_changed:
                self.gripped = bool(gripped)
                self.input_armed = False
                return self._reject_input('device_changed_release_required')
            if self.active and not self.dagger_enabled:
                # Model-only mode ignores controller takeover/pause/resume.
                # Physical emergency stop remains on its independent route.
                self.input_armed = self.gripped = False
                return self._reject_input('dagger_disabled')
            previous = self.gripped
            self.gripped = bool(gripped)
            if pause_requested:
                self.hold('VR controller B button pressed')
                self.input_diagnostics['accepted'] += 1
                self.input_diagnostics['last_accepted_source_ns'] = stamp_ns
                return True
            resume_requested = resume_requested and self.state in {'HOLD', 'FAULT', 'ESTOP'}
            if not gripped:
                self.input_armed = True
                if previous and self.active and not resume_requested:
                    self.release()
            elif not previous and self.input_armed and self.active and not resume_requested:
                self.takeover()
            self.input_diagnostics['accepted'] += 1
            self.input_diagnostics['last_accepted_source_ns'] = stamp_ns
            return True

    def peer(self, participant, instance):
        with self.lock:
            if participant not in self.required_peers or not instance:
                return
            previous = self.peers.get(participant)
            self.peers[participant] = instance
            self.peer_times[participant] = self.clock()
            if previous and previous != instance and self.active:
                self.fault(participant + ' restarted')

    def acknowledge(self, token, phase, participant, instance, success=True, reason=''):
        with self.lock:
            if (token != self.token or phase != self.phase or
                    self.peers.get(participant) != instance or not self.active):
                return False
            if not success:
                self.fault(participant + ': ' + reason)
                return False
            if participant in self.acks:
                return True
            self.acks.add(participant)
            self._event('ack', participant=participant)
            return True

    def advance(self, *, control_ready, manual_ready):
        """Called after peer ACKs and fresh physical feedback, without sleeps."""
        with self.lock:
            if self.state not in {'TAKEOVER', 'RETURNING'}:
                return
            if self.clock() >= self.deadline_ns:
                self.fault('handover timeout: ' + self.phase + '; ack=' + ','.join(sorted(self.acks)))
                return
            if self.state == 'TAKEOVER' and not self._input_fresh():
                self.input_lost('teleop input timeout during handover')
                return
            if not control_ready or not self.required_peers <= self.acks:
                return
            if self.phase == 'revoke':
                if self.state == 'TAKEOVER' and (not self._input_fresh() or not self.gripped or not manual_ready):
                    return
                if self.state == 'TAKEOVER' and self.recording_enabled:
                    # The capture range is established before either receiver
                    # may accept the first manual command. Failure is visible
                    # and never prevents a human stopping the program.
                    try:
                        self.open_segment(self.token, self.ros_clock())
                        self.segment_open = True
                        self._event('manual_open')
                    except Exception as exc:
                        self.recording_error = str(exc)
                        self._event('recording_failed', error=str(exc))
                self.phase = 'grant'
                self.acks.clear()
                self.cutoff_ns = self.ros_clock()
                self.permit(self.token, 'grant', self.cutoff_ns, True)
            elif self.phase == 'grant':
                self.state = 'MANUAL' if self.state == 'TAKEOVER' else 'AUTO'
                if self.state == 'MANUAL':
                    self.manual_changed(self.token, self.ros_clock(), '')
                self._event('permission_open')
                if self.state == 'AUTO' and self.release_ns is not None:
                    metric = dict(release_to_open_ns=self.clock()-self.release_ns,
                                  epoch=self.epoch, stamp_ns=self.ros_clock())
                    self.metrics.append(metric)
                    self.pending_auto_metric = metric
                    self.release_ns = None

    def watchdog(self):
        with self.lock:
            if self.preparing_capture and self.state in {'HOLD', 'AUTO', 'RETURNING'} and self.capture_ready():
                self.finish_capture_preparation(self.session)
            if not self.preparing_capture and self.active and self.input_valid and self.clock()-self.input_time_ns > self.input_timeout_ns:
                self.input_lost('teleop input timeout')
            if self.state in {'AUTO', 'MANUAL'}:
                for peer in self.required_peers:
                    if self.clock()-self.peer_times.get(peer, 0) > self.input_timeout_ns:
                        self._event('heartbeat_timeout', participant=peer,
                                    gap_ms=(self.clock()-self.peer_times.get(peer, 0))/1e6,
                                    limit_ms=self.input_timeout_ns/1e6)
                        self.fault(peer+' acknowledgement heartbeat lost')
                        break

    def allows(self, token):
        return (token == self.token and self.phase == 'grant' and
                ((self.state == 'AUTO' and token.source == 'motion') or
                 (self.dagger_enabled and self.state == 'MANUAL' and token.source == 'vr' and self.gripped and self._input_fresh())))

    def sent(self, token, target, stamp_ns):
        """Invoked only after hardware publication, under the final gate lock."""
        if not self.allows(token):
            raise OwnershipConflict('command does not own the final output')
        if token.source != 'vr':
            if self.pending_auto_metric is not None and self.pending_auto_metric['epoch'] == token.epoch:
                self.pending_auto_metric['open_to_first_final_command_ns'] = max(0, stamp_ns-self.pending_auto_metric['stamp_ns'])
                self.pending_auto_metric = None
            return None
        self.command_seq += 1
        commanded = [(n,v) for n,v in zip(target.name, target.position) if math.isfinite(v)]
        return dict(schema='hc-human-action/v1', **asdict(token), command_seq=self.command_seq,
                    stamp_ns=stamp_ns, action_stage='sent', action_boundary='hc_final',
                    intervention=True, names=[n for n,_ in commanded], positions=[v for _,v in commanded])

    def request(self, request_id, session_id, expected_epoch, action):
        with self.lock:
            if not isinstance(request_id, str) or not request_id or len(request_id) > 256:
                raise OwnershipConflict('request_id is required')
            if not isinstance(session_id, str) or type(expected_epoch) is not int:
                raise OwnershipConflict('session_id and integer expected_epoch are required')
            if request_id in self.requests:
                return self.snapshot()
            if session_id != self.session or expected_epoch != self.epoch:
                raise OwnershipConflict('stale session or epoch')
            action()
            self.requests[request_id] = self.epoch
            if len(self.requests) > 512:
                self.requests.popitem(last=False)
            return self.snapshot()

    def snapshot(self):
        with self.lock:
            return dict(state=self.state, phase=self.phase, reason=self.reason,
                        dagger_enabled=self.dagger_enabled,
                        recent_events=list(self.events)[-16:],
                        **asdict(self.token), input_valid=self.input_valid,
                        input_armed=self.input_armed, gripped=self.gripped,
                        input_diagnostics=dict(self.input_diagnostics),
                        input_rearm_cutoff_ns=self.input_ready_stamp_ns,
                        preparing_capture=self.preparing_capture,
                        required_peers=sorted(self.required_peers), recording_enabled=self.recording_enabled,
                        acknowledgements=sorted(self.acks), peers=dict(self.peers),
                        recording_error=self.recording_error, command_seq=self.command_seq,
                        cutoff_ns=self.cutoff_ns, sequence=self.event_seq,
                        published_stamp_ns=self.ros_clock(),
                        metrics=list(self.metrics)[-64:], metric_count=len(self.metrics))

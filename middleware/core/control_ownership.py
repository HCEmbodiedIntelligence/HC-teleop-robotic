"""Robot-side intervention transactions, independent of ROS, devices and models.

The caller shares this lock with the final actuator publication. Hooks must be
bounded, memory-only operations; disk work belongs to the recording workers.
An acknowledgement is an execution fact, never a timer-based assumption.
"""
from collections import OrderedDict, deque
from dataclasses import asdict, dataclass
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
                 emit=lambda event: None, transition_timeout_sec=2., input_timeout_sec=.3):
        self.lock = lock or threading.RLock()
        self.clock, self.ros_clock = clock, ros_clock
        self.revoke, self.permit = revoke, permit
        self.open_segment, self.close_segment, self.emit = open_segment, close_segment, emit
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
        self.input_ready_stamp_ns = 0
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

    def enable(self, *, preparing_capture=False):
        with self.lock:
            if self.active:
                raise OwnershipConflict('intervention session already active')
            self.session = uuid.uuid4().hex
            self.input_armed = self.input_valid = self.gripped = self.resume_auto = False
            self.input_sequence = -1
            self.preparing_capture = preparing_capture
            self.input_ready_stamp_ns = 0
            self.recording_error = ''
            self._invalidate('HOLD', reason='session enabled; explicit start required')
            return self.snapshot()

    def finish_capture_preparation(self, session_id):
        with self.lock:
            if not self.preparing_capture or self.session != session_id or self.state != 'HOLD':
                return
            self.preparing_capture = False
            self.input_ready_stamp_ns = self.ros_clock()
            # Samples observed while writers/subscriptions were starting must
            # not arm a later held Grip. Require a fresh release after readiness.
            self.input_armed = self.input_valid = self.gripped = False
            self.reason = 'capture prepared; observe release before starting'
            self._event('capture_ready')

    def disable(self):
        with self.lock:
            self.preparing_capture = False
            self.resume_auto = False
            self._invalidate('OFF', reason='session disabled; output remains closed')

    def start_auto(self, *, recording_ready, control_ready):
        with self.lock:
            if self.preparing_capture or self.state != 'HOLD' or self.gripped or not self.input_armed or not self.input_valid:
                raise OwnershipConflict('start requires HOLD and a fresh observed release')
            if not recording_ready or not control_ready or not {'motion', 'teleop'} <= self.peers.keys():
                raise OwnershipConflict('capture, feedback or control participants are not ready')
            self.resume_auto = True
            self._invalidate('RETURNING', 'motion', 'explicit start')

    def takeover(self):
        with self.lock:
            if self.preparing_capture or self.state not in {'AUTO', 'HOLD', 'RETURNING'}:
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
            self._invalidate('HOLD', reason='explicit recovery; observe release before starting')

    def input(self, *, device_instance, sequence, valid, gripped, stamp_ns):
        """Semantic device sample. Invalid/default input never means release."""
        with self.lock:
            now = self.clock()
            age = self.ros_clock() - stamp_ns
            if sequence <= self.input_sequence and device_instance == self.input_device:
                return
            if not -50_000_000 <= age <= self.input_timeout_ns:
                return
            if self.active and stamp_ns < self.input_ready_stamp_ns:
                return
            if device_instance != self.input_device:
                if self.input_device and self.active:
                    self.fault('teleop input instance changed')
                self.input_device, self.input_sequence, self.input_armed = device_instance, -1, False
            self.input_sequence, self.input_time_ns = sequence, now
            self.input_valid = bool(valid)
            if not valid:
                self.input_armed = False
                if self.active:
                    self.fault('teleop input invalid/tracking lost')
                return
            if self.preparing_capture:
                self.input_valid = self.input_armed = self.gripped = False
                return
            previous = self.gripped
            self.gripped = bool(gripped)
            if not gripped:
                self.input_armed = True
                if previous and self.active:
                    self.release()
            elif not previous and self.input_armed and self.active:
                self.takeover()

    def peer(self, participant, instance):
        with self.lock:
            if participant not in {'motion', 'teleop'} or not instance:
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
            if not control_ready or not {'motion', 'teleop'} <= self.acks:
                return
            if self.phase == 'revoke':
                if self.state == 'TAKEOVER' and (not self.gripped or not manual_ready):
                    return
                if self.state == 'TAKEOVER':
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
                self._event('permission_open')
                if self.state == 'AUTO' and self.release_ns is not None:
                    metric = dict(release_to_open_ns=self.clock()-self.release_ns,
                                  epoch=self.epoch, stamp_ns=self.ros_clock())
                    self.metrics.append(metric)
                    self.pending_auto_metric = metric
                    self.release_ns = None

    def watchdog(self):
        with self.lock:
            if self.preparing_capture and self.state == 'HOLD' and self.capture_ready():
                self.finish_capture_preparation(self.session)
            if not self.preparing_capture and self.active and self.input_valid and self.clock()-self.input_time_ns > self.input_timeout_ns:
                self.input_valid = self.input_armed = False
                self.fault('teleop input timeout')
            if self.state in {'AUTO', 'MANUAL'}:
                for peer in ('motion', 'teleop'):
                    if self.clock()-self.peer_times.get(peer, 0) > self.input_timeout_ns:
                        self.fault(peer+' acknowledgement heartbeat lost')
                        break

    def allows(self, token):
        return (token == self.token and self.phase == 'grant' and
                ((self.state == 'AUTO' and token.source == 'motion') or
                 (self.state == 'MANUAL' and token.source == 'vr' and self.gripped and self.input_valid)))

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
        return dict(schema='hc-human-action/v1', **asdict(token), command_seq=self.command_seq,
                    stamp_ns=stamp_ns, action_stage='sent', action_boundary='hc_final',
                    intervention=True, names=list(target.name), positions=list(target.position))

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
                        **asdict(self.token), input_valid=self.input_valid,
                        input_armed=self.input_armed, gripped=self.gripped,
                        preparing_capture=self.preparing_capture,
                        acknowledgements=sorted(self.acks), peers=dict(self.peers),
                        recording_error=self.recording_error, command_seq=self.command_seq,
                        cutoff_ns=self.cutoff_ns, sequence=self.event_seq,
                        metrics=list(self.metrics)[-64:], metric_count=len(self.metrics))

"""Recorder-only action labels; no ROS publishers or control authorization.

A successful action remains byte-for-byte available on a separate evidence
channel. Feedback labels are admitted only with explicit, current authorization
from the caller *and* a fresh, healthy Grip-rearm status. This module never
replays missing samples or uses a later status to label an earlier observation.
"""
from copy import deepcopy
from pathlib import Path
import heapq
import json
import math

from .capture_window import header_stamp_ns
from .recording_cdr import read_joint_state, read_string, position_label


ACTION_TOPIC = '/action/joint_states'
PUBLISHED_TOPIC = '/internal/recording/published_action'
PROVENANCE_TOPIC = '/events/recording/action_provenance'
FEEDBACK_TOPIC = '/observation/joint_states'
STATUS_TOPIC = '/status/teleop'
OWNERSHIP_TOPIC = '/status/control/ownership'
JOINT_TYPE = 'sensor_msgs/msg/JointState'


def action_label_policy(config):
    """Require selected phase evidence and unthrottled action/feedback inputs."""
    from .x1_recording import is_x1
    if not is_x1(config) or config.get('recording_action_feedback_hold', True) is not True:
        return None
    selected = {item.get('topic'): item for item in config.get('subscriptions', [])
                if item.get('enabled', True) and 'record' in item.get('outputs', [])}
    required = {ACTION_TOPIC: JOINT_TYPE, FEEDBACK_TOPIC: JOINT_TYPE,
                STATUS_TOPIC: 'std_msgs/msg/String'}
    if any(topic not in selected or selected[topic].get('type') != kind
           or topic != STATUS_TOPIC and float(selected[topic].get('max_hz', 0)) > 0
           for topic, kind in required.items()):
        return None
    from ..control.legacy_control import LegacyTopicControl
    import yaml
    try:
        profile = yaml.safe_load((Path(config['legacy_control_profile'])/'arm_teleop.yaml').read_text())
        physical = [name for side in ('right', 'left') for name in profile['arms'][side]['joint_names']]
        physical += profile['body'].get('waist_joint_names', []) + profile['body'].get('neck_joint_names', [])
    except (OSError, KeyError, TypeError, yaml.YAMLError):
        return None  # An unknown layout cannot justify a feedback-derived action.
    return dict(feedback_timeout_s=LegacyTopicControl.FEEDBACK_MAX_AGE_S,
                status_timeout_s=LegacyTopicControl.FEEDBACK_MAX_AGE_S,
                eligible_joint_names=tuple(dict.fromkeys(physical)))


class RecordingActionLabels:
    """Forward-only transformation run exclusively by the recording writer.

    ``output_allowed`` must be an explicit, fresh control-owner observation.
    A stale dashboard status alone must never turn a FAULT/closed output into a
    healthy training label. The default is therefore False. Thresholds are
    supplied by the caller from the existing feedback policy, not widened here.

    The caller may reorder input by event time before calling ``process``;
    otherwise late status/feedback simply cannot produce a derived label.
    """
    def __init__(self, *, feedback_timeout_s, status_timeout_s, eligible_joint_names=None):
        if (not math.isfinite(feedback_timeout_s) or feedback_timeout_s <= 0
                or not math.isfinite(status_timeout_s) or status_timeout_s <= 0):
            raise ValueError('recording label freshness thresholds must be positive and finite')
        self.feedback_timeout_ns = int(feedback_timeout_s * 1e9)
        self.status_timeout_ns = int(status_timeout_s * 1e9)
        self.eligible_joint_names = (None if eligible_joint_names is None else frozenset(eligible_joint_names))
        self.names = ()
        self.status = None
        self.status_ns = 0
        self.owner = None
        self.owner_ns = 0
        self.last_feedback_ns = 0
        self.last_action_ns = 0
        self.last_action_output_ns = 0
        self.published_frames = self.feedback_frames = self.rejected_feedback = 0

    @staticmethod
    def _joint_message(event, *, allow_placeholders=False):
        message = read_joint_state(event['_raw'])
        if (not message.name or len(set(message.name)) != len(message.name)
                or any(not name for name in message.name)
                or len(message.position) != len(message.name)
                or any(math.isinf(float(value)) or not allow_placeholders and math.isnan(float(value))
                       for value in message.position)):
            raise ValueError('invalid joint positions or names')
        return message

    @staticmethod
    def _provenance(event, source, *, source_topic, source_stamp_ns, status_ns=None):
        stamp = int(event['stamp_ns'])
        payload = dict(schema_version='hc-action-provenance/v1', source=source,
                       action_topic=ACTION_TOPIC, source_topic=source_topic,
                       source_stamp_ns=source_stamp_ns, sample_time_ns=source_stamp_ns,
                       action_log_time_ns=stamp,
                       publication_ns=stamp if source == 'published_action' else None,
                       sent_to_robot=source == 'published_action')
        if status_ns is not None:
            payload['rearm_status_time_ns'] = status_ns
        return dict(kind='recording_action_provenance', topic=PROVENANCE_TOPIC,
                    stamp_ns=stamp, payload=payload, _recording_derived=True)

    def _observe_status(self, event):
        stamp = int(event['stamp_ns'])
        if stamp < self.status_ns:
            return
        self.status, self.status_ns = None, stamp
        try:
            payload = event.get('payload')
            if payload is None:
                payload = json.loads(read_string(event['_raw']))
            if isinstance(payload, dict):
                self.status = deepcopy(payload)
        except Exception:
            pass  # Malformed newest status revokes labels; never reuse older status.

    def _healthy_rearm(self, now_ns):
        status = self.status
        if status is None or not 0 <= now_ns-self.status_ns <= self.status_timeout_ns:
            return False
        source_stamp = status.get('stamp_ns', 0)
        if (not isinstance(source_stamp, int) or source_stamp <= 0
                or not 0 <= now_ns-source_stamp <= self.status_timeout_ns):
            return False
        generic = status.get('generic_controller')
        collision = status.get('collision_avoidance')
        return bool(status.get('mode') == 'ik_rearming'
                    and status.get('enabled') is True
                    and status.get('feedback_fresh') is True
                    and status.get('stop_reason') == ''
                    and status.get('control_source') == 'vr'
                    and status.get('source_feedback_pending') is False
                    and isinstance(generic, dict)
                    and generic.get('hardware_ready') is True
                    and generic.get('rearm_state') in {'wait_release', 'wait_press'}
                    and isinstance(collision, dict) and collision.get('blocked') is False)

    def output_allowed(self, now_ns):
        owner = self.owner
        if not owner or not 0 <= now_ns-self.owner_ns <= self.status_timeout_ns:
            return False
        guard = owner.get('recording_output', {})
        if not isinstance(guard, dict):
            return False
        stamp = guard.get('stamp_ns', 0)
        return bool(guard.get('allowed') is True and guard.get('source') == 'vr'
                    and owner.get('state') in {'OFF', 'HOLD', 'MANUAL'}
                    and isinstance(stamp, int) and stamp > 0
                    and 0 <= now_ns-stamp <= self.status_timeout_ns)

    def process(self, event, *, output_allowed=False):
        """Return the original event plus any explicitly marked recording events."""
        result = [event]
        if event.get('_recording_derived'):
            return result
        topic = event.get('topic')
        if topic == OWNERSHIP_TOPIC:
            stamp = int(event['stamp_ns'])
            if stamp >= self.owner_ns:
                self.owner, self.owner_ns = None, stamp
                try:
                    value = json.loads(read_string(event['_raw']))
                    if isinstance(value, dict):
                        self.owner = value
                except Exception:
                    pass
            return result
        if topic == STATUS_TOPIC:
            self._observe_status(event)
            return result
        if event.get('msg_type') != JOINT_TYPE or not isinstance(event.get('_raw'), bytes):
            return result
        if topic == ACTION_TOPIC:
            try:
                message = self._joint_message(event, allow_placeholders=True)
                names = tuple(name for name, value in zip(message.name, message.position)
                              if math.isfinite(float(value)) and
                              (self.eligible_joint_names is None or name in self.eligible_joint_names))
                if int(event['stamp_ns']) >= self.last_action_ns:
                    self.names = names
                    self.last_action_ns = int(event['stamp_ns'])
            except Exception:
                self.names = ()  # Preserve the wire record, but do not derive from it.
            result.append({**event, 'topic': PUBLISHED_TOPIC, '_recording_derived': True})
            provenance = self._provenance(event, 'published_action', source_topic=ACTION_TOPIC,
                source_stamp_ns=header_stamp_ns(event['_raw'], JOINT_TYPE))
            # In the rearm phase the training action comes from feedback;
            # the exact real command remains on PUBLISHED_TOPIC in all cases.
            in_labels = not (output_allowed is True and self._healthy_rearm(int(event['stamp_ns'])))
            stamp = int(event['stamp_ns'])
            if in_labels and stamp <= self.last_action_output_ns:
                in_labels = False
            provenance['payload']['included_in_action_labels'] = in_labels
            if not in_labels:
                result = result[1:]
            else:
                self.last_action_output_ns = stamp
            result.append(provenance)
            self.published_frames += 1
            return result
        if topic != FEEDBACK_TOPIC:
            return result
        stamp = int(event['stamp_ns'])
        source_stamp = header_stamp_ns(event['_raw'], JOINT_TYPE)
        if (output_allowed is not True or not self.names or not self._healthy_rearm(stamp)
                or source_stamp is None
                or not 0 <= stamp-source_stamp <= self.feedback_timeout_ns
                or source_stamp <= self.last_feedback_ns
                or source_stamp <= self.last_action_output_ns):
            self.rejected_feedback += 1
            return result
        try:
            feedback = self._joint_message(event, allow_placeholders=True)
            positions = dict(zip(feedback.name, feedback.position))
            if any(name not in positions or not math.isfinite(float(positions[name])) for name in self.names):
                raise ValueError('feedback does not cover the command joint set')
            # Feedback velocities/efforts describe measured motion, not a hold
            # command. Keep only measured positions in this derived target.
            derived = dict(kind='ros_message', topic=ACTION_TOPIC, msg_type=JOINT_TYPE,
                           record_only=True, _raw=position_label(feedback.header_prefix, self.names,
                               [positions[name] for name in self.names]),
                           stamp_ns=source_stamp, _recording_derived=True)
        except Exception:
            self.rejected_feedback += 1
            return result
        self.last_feedback_ns = source_stamp
        self.last_action_output_ns = source_stamp
        self.feedback_frames += 1
        result.append(derived)
        result.append(self._provenance(derived, 'feedback_hold', source_topic=FEEDBACK_TOPIC,
                                      source_stamp_ns=source_stamp, status_ns=self.status_ns))
        result[-1]['payload']['included_in_action_labels'] = True
        result[-1]['payload']['feedback_receive_ns'] = stamp
        result[-1]['payload']['rearm_status_source_stamp_ns'] = self.status['stamp_ns']
        if self.owner:
            result[-1]['payload']['output_epoch'] = self.owner.get('recording_output', {}).get('output_epoch')
        return result

    def summary(self):
        return dict(schema_version='hc-action-labels/v1', published_frames=self.published_frames,
                    feedback_hold_frames=self.feedback_frames,
                    rejected_feedback=self.rejected_feedback,
                    feedback_timeout_ns=self.feedback_timeout_ns,
                    status_timeout_ns=self.status_timeout_ns,
                    eligible_joint_names=sorted(self.eligible_joint_names) if self.eligible_joint_names is not None else None,
                    published_topic=PUBLISHED_TOPIC, provenance_topic=PROVENANCE_TOPIC,
                    label_policy='fresh feedback only during healthy wait_release/wait_press; caller-authorized',
                    action_clock='published: publication wall time; feedback_hold: original feedback header time')


class ActionLabelBuffer:
    """Bounded, recorder-only 50 ms receive-time reorder for independent IPC producers.

    Late events are still preserved. The forward-only policy refuses to derive
    a sample using a status whose receive time is in that sample's future.
    Pressure flushes evidence early rather than dropping packets or blocking
    the control process. Images never enter this buffer.
    """
    TOPICS = {ACTION_TOPIC, FEEDBACK_TOPIC, STATUS_TOPIC, OWNERSHIP_TOPIC}
    _DDS_TOPICS = {FEEDBACK_TOPIC, STATUS_TOPIC, OWNERSHIP_TOPIC}

    def __init__(self, policy, *, delay_ns=50_000_000, max_events=512, skew_wait_ns=0):
        self.labels = RecordingActionLabels(**policy)
        self.delay_ns, self.max_events = delay_ns, max_events
        self.skew_wait_ns = max(0, int(skew_wait_ns))
        self.heap = []
        self.sequence = 0
        self.pressure_flushes = 0
        self.last_action_stamp_ns = None
        self.last_dds_stamp_ns = None

    def offer(self, event):
        stamp = int(event['stamp_ns'])
        topic = event.get('topic')
        if topic == ACTION_TOPIC:
            if self.last_action_stamp_ns is None or stamp > self.last_action_stamp_ns:
                self.last_action_stamp_ns = stamp
        elif topic in self._DDS_TOPICS:
            if self.last_dds_stamp_ns is None or stamp > self.last_dds_stamp_ns:
                self.last_dds_stamp_ns = stamp
        self.sequence += 1
        heapq.heappush(self.heap, (stamp, self.sequence, event))

    def ready(self, now_ns, *, final=False):
        while self.heap:
            stamp = self.heap[0][0]
            if not final and len(self.heap) <= self.max_events:
                delay = self.delay_ns
                if self.skew_wait_ns > delay and (
                    self.last_action_stamp_ns is None
                    or self.last_dds_stamp_ns is None
                    or stamp > min(self.last_action_stamp_ns, self.last_dds_stamp_ns)
                ):
                    delay = self.skew_wait_ns
                if stamp > now_ns - delay:
                    break
            if len(self.heap) > self.max_events:
                self.pressure_flushes += 1
            stamp, _, event = heapq.heappop(self.heap)
            yield from self.labels.process(event, output_allowed=self.labels.output_allowed(stamp))

    def summary(self):
        return dict(self.labels.summary(), reorder_delay_ns=self.delay_ns,
                    pressure_flushes=self.pressure_flushes)

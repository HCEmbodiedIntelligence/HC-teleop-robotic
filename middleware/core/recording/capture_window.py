"""Recorder-only sensor windows. Never change source data or actuator timing."""
from array import array
from collections import deque
import json
import struct
import time


WINDOW_METADATA = 'hc_teleop.capture_window'
AUDIT_METADATA = 'hc_teleop.capture_audit'
PREROLL_NS = 250_000_000
LATE_ARRIVAL_NS = 250_000_000
_HEADER_TYPES = {
    'sensor_msgs/msg/JointState', 'sensor_msgs/msg/Image',
    'sensor_msgs/msg/CompressedImage', 'geometry_msgs/msg/PoseStamped',
    'geometry_msgs/msg/PoseArray', 'geometry_msgs/msg/TwistStamped',
    'geometry_msgs/msg/WrenchStamped', 'nav_msgs/msg/Odometry',
    'std_msgs/msg/Header',
}


def header_stamp_ns(raw, msg_type):
    """Read only the unchanged leading Header of standard CDR1 ROS messages."""
    if msg_type not in _HEADER_TYPES or not isinstance(raw, bytes) or len(raw) < 12:
        return None
    representation = int.from_bytes(raw[:2], 'big')
    if representation not in (0, 1):
        return None
    sec, nano = struct.unpack_from('<iI' if representation == 1 else '>iI', raw, 4)
    return sec * 1_000_000_000 + nano if sec >= 0 and nano < 1_000_000_000 and (sec or nano) else None


def sample_time(event):
    """Return a source clock plus an explicit fallback, without rewriting CDR."""
    received = int(event['stamp_ns'])
    header = header_stamp_ns(event.get('_raw'), event.get('msg_type'))
    if header is not None and -20_000_000 <= received - header <= 2_000_000_000:
        return header, 'header'
    return received, 'invalid_header' if event.get('msg_type') in _HEADER_TYPES else 'receive'


def buffered_sensor(item):
    return (item.get('topic') in {'/status/teleop', '/status/control/ownership'}
            and item.get('type') == 'std_msgs/msg/String'
            or item.get('type') in {'sensor_msgs/msg/Image', 'sensor_msgs/msg/CompressedImage'}
            or item.get('topic') in {'/observation/joint_states', '/observation/hands/joint_states',
                '/control/joint_cmd', '/control/solver/joint_targets', '/control/vr/joint_states'}
            and item.get('type') in _HEADER_TYPES)


class CaptureWindow:
    """Small shared lifecycle record; raw callback reads never touch send locks."""
    def __init__(self, context):
        self._values = context.Array('q', 9, lock=True)
        self.buffered_topics = ()

    def reset(self, enabled, *, action_at_publication=False):
        with self._values.get_lock():
            self._values[:] = [int(enabled), 0, 0, 0, PREROLL_NS, LATE_ARRIVAL_NS,
                               0, 0, int(action_at_publication)]

    def open(self, now_ns=None):
        with self._values.get_lock():
            self._values[1] = time.time_ns() if now_ns is None else int(now_ns)
            self._values[7] = time.monotonic_ns()

    def request_stop(self, now_ns=None, *, now_monotonic_ns=None):
        with self._values.get_lock():
            if self._values[1] and not self._values[2]:
                self._values[2] = time.time_ns() if now_ns is None else int(now_ns)
                self._values[6] = time.monotonic_ns() if now_monotonic_ns is None else int(now_monotonic_ns)
            return self._values[2]

    def close_raw(self, now_ns=None):
        with self._values.get_lock():
            self._values[3] = time.time_ns() if now_ns is None else int(now_ns)

    def enabled(self):
        with self._values.get_lock():
            return bool(self._values[0])

    def late_remaining_seconds(self):
        with self._values.get_lock():
            deadline = self._values[6] + self._values[5]
        return max(0., (deadline-time.monotonic_ns())/1e9)

    def allows(self, event):
        stamp, clock = sample_time(event)
        with self._values.get_lock():
            enabled, start, end, closed, pre_roll = self._values[:5]
        return (not enabled or bool(start and stamp >= start - pre_roll
                and (not end or stamp <= end))), clock

    def snapshot(self):
        with self._values.get_lock():
            enabled, start, end, closed, pre_roll, late, stop_mono, start_mono, action = self._values[:]
        if not enabled or not start:
            return {}
        return dict(schema_version='hc-capture-window/v1', admission_start_ns=start,
            window_start_ns=start-pre_roll, stop_requested_ns=end,
            raw_ingress_closed_ns=closed, pre_roll_ns=pre_roll,
            late_arrival_budget_ns=late,
            admission_start_monotonic_ns=start_mono, stop_requested_monotonic_ns=stop_mono,
            action_at_publication='true' if action else 'false',
            buffered_topics=list(self.buffered_topics),
            warm_sensor_qos='BEST_EFFORT; small stamped evidence depth=64 minimum; images use configured recording depth; persistent across ordinary idle/active transitions',
            sample_clock=('action: publication_ns (MCAP log_time)' if action else 'action: DDS wall receive time (no publication proof)')
                + '; header-bearing sensors: unchanged ROS header.stamp; other/invalid-header: wall receive time (fallback counted)',
            action_reserved_before_stop='drained without new admission; may complete after stop')


class SensorPreroll:
    """Keep original packets across transport delay, then filter by source time.

    Receive-time retention includes the 250 ms window plus compression delay.
    A hard byte limit bounds an unexpectedly large or misconfigured camera.
    """
    def __init__(self, topics, seconds=.75, byte_budget=32*1024*1024):
        self.topics = set(topics)
        self.seconds, self.byte_budget = seconds, byte_budget
        self.samples = deque()
        self.buffered_bytes = self.evicted_bytes = 0

    def _trim(self, now):
        while self.samples and (now-self.samples[0][0] > self.seconds
                or self.buffered_bytes > self.byte_budget or len(self.samples) > 4096):
            over_budget = self.buffered_bytes > self.byte_budget
            _, event = self.samples.popleft()
            self.buffered_bytes -= len(event['_raw'])
            if over_budget:
                self.evicted_bytes += len(event['_raw'])

    def remember(self, now, event):
        if event['topic'] in self.topics:
            self.samples.append((now, event))
            self.buffered_bytes += len(event['_raw'])
            self._trim(now)

    def snapshot(self, now):
        self._trim(now)
        return list(self.samples)


class CaptureAudit:
    """Bounded evidence about recorded frames; never gate or remove samples."""
    def __init__(self, max_samples=2_000_000, *, capture_window=None):
        self.max_samples, self.samples, self.truncated = max_samples, 0, False
        self.stamps, self.receive, self.fallbacks = {}, {}, {}
        self.capture_window = capture_window
        self.action = dict(frames=0, after_stop=0, max_stop_overhang_ms=0.)

    def observe(self, event):
        topic = str(event.get('topic', ''))
        if topic == '/action/joint_states' and self.capture_window is not None:
            window = self.capture_window.snapshot()
            publication = window.get('action_at_publication') == 'true'
            self.action['clock'] = 'publication_ns' if publication else 'DDS wall receive time'
            self.action['frames'] += 1
            stamp = int(event['stamp_ns'])
            self.action['first_ns'] = min(stamp, self.action.get('first_ns', stamp))
            self.action['last_ns'] = max(stamp, self.action.get('last_ns', stamp))
            if publication and window.get('stop_requested_ns', 0) and stamp > window['stop_requested_ns']:
                self.action['after_stop'] += 1
                self.action['max_stop_overhang_ms'] = max(self.action['max_stop_overhang_ms'],
                    (stamp-window['stop_requested_ns'])/1e6)
        if event.get('msg_type') not in {'sensor_msgs/msg/Image', 'sensor_msgs/msg/CompressedImage'}:
            return
        _, clock = sample_time(event)
        if clock != 'header':
            self.fallbacks[topic] = self.fallbacks.get(topic, 0) + 1
        stamp = header_stamp_ns(event.get('_raw'), event.get('msg_type'))
        if stamp is None:
            return
        if self.samples >= self.max_samples:
            self.truncated = True
            return
        self.stamps.setdefault(topic, array('q')).append(stamp)
        self.receive.setdefault(topic, array('q')).append(int(event['stamp_ns']))
        self.samples += 1

    @staticmethod
    def _stream(values):
        gaps = [b-a for a, b in zip(values, values[1:])]
        ordered = values if all(g >= 0 for g in gaps) else sorted(values)
        sorted_gaps = [b-a for a, b in zip(ordered, ordered[1:])]
        return dict(frames=len(values), first_header_ns=min(values), last_header_ns=max(values),
            gaps_gt_40ms=sum(g > 40_000_000 for g in sorted_gaps),
            max_gap_ms=max(sorted_gaps, default=0)/1e6,
            duplicates=sum(g == 0 for g in sorted_gaps), out_of_order=sum(g < 0 for g in gaps))

    def snapshot(self):
        streams, rgbd = {}, {}
        window = self.capture_window.snapshot() if self.capture_window is not None else {}
        for topic, values in self.stamps.items():
            streams[topic] = self._stream(values)
            streams[topic].update(first_receive_ns=min(self.receive[topic]),
                last_receive_ns=max(self.receive[topic]), invalid_header_fallbacks=self.fallbacks.get(topic, 0))
            if window:
                streams[topic].update(
                    header_before_window=sum(stamp < window['window_start_ns'] for stamp in values),
                    header_after_stop=sum(stamp > window['stop_requested_ns'] for stamp in values)
                        if window['stop_requested_ns'] else None)
        for topic, colors in self.stamps.items():
            if '/color' not in topic:
                continue
            depth_topic = topic.replace('/color', '/depth', 1)
            if depth_topic not in self.stamps:
                continue
            colors, depths = sorted(colors), sorted(self.stamps[depth_topic])
            i = j = paired = 0
            while i < len(colors) and j < len(depths):
                if colors[i] == depths[j]:
                    paired += 1; i += 1; j += 1
                elif colors[i] < depths[j]:
                    i += 1
                else:
                    j += 1
            rgbd[topic.rsplit('/color', 1)[0]] = dict(color_topic=topic, depth_topic=depth_topic,
                paired_exact=paired, color_unmatched=len(colors)-paired,
                depth_unmatched=len(depths)-paired, pairing='exact unchanged ROS header.stamp, not array index')
        return dict(schema_version='hc-capture-audit/v1', truncated=self.truncated,
                    observed_camera_frames=self.samples, streams=streams, rgbd=rgbd,
                    invalid_header_fallbacks=dict(self.fallbacks), action=dict(self.action))


def read_capture_metadata(path):
    from mcap.reader import make_reader
    from mcap.exceptions import McapError
    result = {}
    try:
        with path.open('rb') as stream:
            for item in make_reader(stream).iter_metadata():
                if item.name == WINDOW_METADATA:
                    result['capture_window'] = {
                        key: int(value) if key.endswith('_ns') else
                            json.loads(value) if key == 'buffered_topics' else value
                        for key, value in item.metadata.items()}
                elif item.name == AUDIT_METADATA:
                    result['capture_audit'] = json.loads(item.metadata['json'])
    except (OSError, ValueError, KeyError, McapError):
        pass
    return result

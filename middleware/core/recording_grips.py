"""Project optional binary hands from independent measured and commanded MCAP sources."""
from bisect import bisect_right

from .binary_grip import BinaryGrip

SIDES = ('left', 'right')
HAND_COMMANDS = {f'/action/{side}_hand/joint_states': side for side in SIDES}


def hand_selection(value):
    if value is True or value == 'both':
        return SIDES
    if value is False or value is None or value == 'none':
        return ()
    if isinstance(value, (list, tuple)):
        groups = {str(item).removesuffix('_hand') for item in value}
        if groups <= set(SIDES) and len(groups) == len(value):
            return tuple(side for side in SIDES if side in groups)
    if value in SIDES:
        return (value,)
    raise ValueError('record_hands must be none, left, right or both')


class GripTrack:
    def __init__(self, topic, *, bit=None, calibration=None, marker=False):
        self.topic, self.bit = topic, bit
        self.calibration, self.marker = calibration, marker
        self.samples, self.headers = [], []

    def observe(self, message, log_ns, header_ns):
        names, positions = message.name, message.position
        if self.bit:
            if self.bit not in names or len(names) != len(positions):
                return
            value = positions[names.index(self.bit)]
            bit = float(value) if value in (0., 1.) else None
        else:
            # Shared publishers can carry the opposite hand, or only the arms.
            if not any(n in names for n in self.calibration.command_names):
                return
            bit = self.calibration.decode(names, positions, use_marker=self.marker)
        self.samples.append((log_ns, bit))
        self.headers.append(header_ns)


class RecordingGrips:
    def __init__(self, sides, info, profile):
        self.names = [side+'_grip' for side in sides]
        self.candidates = {}
        self.by_topic = {}
        for side, name in zip(sides, self.names):
            group = info.get('groups', {}).get(side+'_hand', {})
            cal = group.get('grip_calibration')
            calibration = BinaryGrip.from_description(name, cal) if cal else None
            if calibration is None and profile != 'x1':
                raise ValueError(f'{name}: recording has no verified hand calibration')
            for kind in ('observation.state', 'action'):
                state = kind == 'observation.state'
                tracks = []
                if calibration:
                    public = (group.get('joint_states') if state else group.get('commanded_joint_states'))
                    if public:
                        tracks.append(GripTrack(public, bit=name))
                    topic = group['native_state_topic' if state else 'native_command_topic']
                    tracks.append(GripTrack(topic, calibration=calibration, marker=not state))
                elif not state:
                    # The historical X1 dedicated hand command carried a binary marker.
                    # The same name on the main arm command is a simulated angle.
                    tracks.append(GripTrack(f'/action/{side}_hand/joint_states', bit=side[0].upper()+'_ban'))
                self.candidates[kind, name] = tracks
                for track in tracks:
                    self.by_topic.setdefault(track.topic, []).append(track)

    def finish(self, clock):
        self.timeline, self.sources = {}, {}
        for (kind, name), candidates in self.candidates.items():
            track = next((t for t in candidates if any(v is not None for _, v in t.samples)), None)
            self.sources.setdefault(kind, {})[name] = track.topic if track else None
            samples = self.timeline[kind, name] = []
            if track:
                _, retimed = clock(track.samples, track.headers)
                last, value = -1, 0.
                for stamp, bit in retimed:
                    if bit is not None:
                        last, value = stamp, bit
                    samples.append((stamp, (value, last, bit is not None)))

    def append(self, row, stamp, max_age_ns):
        for kind, mask, times in (('observation.state', 'state_valid', 'observation.state_time_ns'),
                                  ('action', 'action_valid', 'action.source_time_ns')):
            values, validity, stamps = [], [], []
            for name in self.names:
                samples = self.timeline[kind, name]
                idx = bisect_right(samples, stamp, key=lambda sample: sample[0]) - 1
                value, last, active = samples[idx][1] if idx >= 0 else (0., -1, False)
                values.append(value)
                validity.append(active and 0 <= stamp-last <= max_age_ns)
                stamps.append(last)
            row['values'][kind] = row['values'][kind] + values
            row[mask] = row[mask] + validity
            row[times] = row[times] + stamps

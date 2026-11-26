"""Explicit shared training projection for native X1 captures before ServoJ discovery."""
from .x1_recording import X1_BODY_JOINTS


def historical_body(training_schema, recorded, ordinary, metadata):
    if training_schema not in ('auto', 'shared'):
        raise ValueError('training_schema must be auto or shared')
    if training_schema == 'auto':
        return None
    if not ordinary:
        raise ValueError('Shared training projection requires an ordinary recording; frozen and intervention captures retain their own contract')
    if recorded is not None:
        return None
    if (metadata.get('profile_id') or metadata.get('robot_name')) != 'x1':
        raise ValueError('Historical shared projection requires an explicitly identified native X1 recording')
    # Camera streams are discovered from the bag. A live robot profile cannot
    # describe which cameras or calibrations existed when the bag was written.
    return {'body': {'model_names': X1_BODY_JOINTS, 'units': ['rad'] * len(X1_BODY_JOINTS)}}


class HistoricalLabels:
    """Audit original labels independently of the stream chosen as command evidence."""
    def __init__(self, samples, headers, index, clock):
        self.origins = index.align(samples, headers, 'log_time')
        self.counts = {'published_action': 0, 'feedback_hold': 0, 'unverified': 0}
        timeline = []
        for ns, _ in samples:
            origin = self.origins.get(ns, 'unverified')
            self.counts[origin] += 1
            timeline.append((ns, origin))
        _, self.timeline = clock(timeline, headers, live=True)
        self.cursor = -1
        self.ranges = []

    def published(self, samples, headers):
        retained, retained_headers = [], []
        for sample, stamp in zip(samples, headers):
            if self.origins.get(sample[0]) == 'published_action':
                retained.append(sample)
                retained_headers.append(stamp)
        return retained, retained_headers

    def note_grid(self, frame, stamp, max_age):
        while self.cursor + 1 < len(self.timeline) and self.timeline[self.cursor + 1][0] <= stamp:
            self.cursor += 1
        origin = 'missing'
        if self.cursor >= 0:
            source_ns, kind = self.timeline[self.cursor]
            if stamp - source_ns <= max_age:
                origin = kind
        if self.ranges and self.ranges[-1]['source'] == origin:
            self.ranges[-1]['end_frame_exclusive'] = frame + 1
        else:
            self.ranges.append(dict(start_frame=frame, end_frame_exclusive=frame + 1, source=origin))

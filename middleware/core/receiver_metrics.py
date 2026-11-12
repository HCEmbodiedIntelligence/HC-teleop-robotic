"""Bounded, session-scoped receiver telemetry; all values are client reported.

No clock synchronization or physical display/capture timestamp is implied.
"""
from collections import deque
import math
import threading
import time


def percentiles(values):
    """Nearest-rank percentiles, including None for an empty window."""
    values = sorted(values)
    return {f'p{p}': round(values[max(0, math.ceil(len(values) * p / 100) - 1)], 2)
            if values else None for p in (50, 95, 99)}


COUNTERS = ('packets_received', 'packets_lost', 'frames_received', 'frames_decoded',
            'frames_dropped', 'frames_rendered', 'freeze_count', 'freeze_duration_ms',
            'jitter_buffer_delay_ms', 'jitter_buffer_emitted_count', 'decode_time_ms',
            'fec_packets_received', 'fec_packets_discarded')
SAMPLES = ('render_gap_ms', 'receive_to_render_ms', 'processing_ms')


class ReceiverMetrics:
    def __init__(self, window_seconds=30, stale_seconds=5, clock=None):
        self.clock = clock or time.monotonic
        self.window_seconds = window_seconds
        self.stale_seconds = stale_seconds
        self.lock = threading.Lock()
        self.reset(None)

    def reset(self, session_id):
        with self.lock:
            self.session_id = session_id
            self.sequence = -1
            self.previous = None
            self.received_at = None
            self.samples = deque(maxlen=120)

    @staticmethod
    def _number(value, name, maximum=1e15, minimum=0):
        if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value) or not minimum <= value <= maximum:
            raise ValueError(f'invalid {name}')
        return float(value)

    def ingest(self, payload):
        if not isinstance(payload, dict):
            raise ValueError('receiver telemetry must be an object')
        sequence = payload.get('sequence')
        if isinstance(sequence, bool) or not isinstance(sequence, int) or not 0 <= sequence <= 2**53-1:
            raise ValueError('invalid sequence')
        sample_ms = self._number(payload.get('sample_ms'), 'sample_ms')
        stats_id = payload.get('stats_id')
        if not isinstance(stats_id, str) or not 1 <= len(stats_id) <= 128:
            raise ValueError('invalid stats_id')
        counters = {name: self._number(payload[name], name, minimum=-1e9 if name == 'packets_lost' else 0)
                    for name in COUNTERS if payload.get(name) is not None}
        arrays = {}
        for name in SAMPLES:
            values = payload.get(name, [])
            if not isinstance(values, list) or len(values) > 240:
                raise ValueError(f'invalid {name} samples')
            arrays[name] = [self._number(v, name, maximum=60000) for v in values]
        with self.lock:
            if not self.session_id or payload.get('session_id') != self.session_id:
                raise ValueError('receiver session mismatch')
            if sequence <= self.sequence:
                return False
            now = self.clock()
            previous = self.previous
            elapsed = sample_ms - previous['sample_ms'] if previous else 0
            if previous and elapsed <= 0:
                raise ValueError('sample_ms must increase')
            # A new inbound SSRC/stat identity or reset invalidates counter deltas.
            deltas = {}
            if previous and previous['stats_id'] == stats_id and elapsed <= 60000:
                for name, value in counters.items():
                    old = previous['counters'].get(name)
                    if old is not None and value >= old:
                        deltas[name] = value - old
            self.previous = dict(sample_ms=sample_ms, stats_id=stats_id, counters=counters)
            self.sequence = sequence
            self.received_at = now
            self.samples.append(dict(at=now, elapsed_ms=elapsed, delta=deltas, arrays=arrays))
            self._trim(now)
            return True

    def _trim(self, now):
        while self.samples and self.samples[0]['at'] < now - self.window_seconds:
            self.samples.popleft()

    def snapshot(self):
        with self.lock:
            now = self.clock()
            self._trim(now)
            age = None if self.received_at is None else max(0, now - self.received_at)
            stale = age is None or age > self.stale_seconds
            samples = [] if stale else list(self.samples)
            result = dict(receiver_session_id=self.session_id,
                          receiver_reported=self.received_at is not None,
                          receiver_stale=stale, receiver_age_ms=round(age*1000) if age is not None else None,
                          receiver_sample_count=len(samples), receiver_metrics_source='client_reported',
                          receiver_loss_pattern='unknown: aggregate counters do not identify burst loss',
                          receiver_end_to_end_ms=None, receiver_physical_display_ms=None)
            def total(name):
                values = [s['delta'][name] for s in samples if name in s['delta']]
                return sum(values) if values else None
            def ratio(numerator, denominator):
                pairs = [(s['delta'][numerator], s['delta'][denominator]) for s in samples
                         if numerator in s['delta'] and denominator in s['delta']]
                count = sum(v[1] for v in pairs)
                return round(sum(v[0] for v in pairs)/count, 2) if count else None
            for name in ('received', 'decoded', 'render'):
                field = 'frames_rendered' if name == 'render' else f'frames_{name}'
                valid = [s for s in samples if field in s['delta'] and s['elapsed_ms'] > 0]
                elapsed = sum(s['elapsed_ms'] for s in valid)
                result[f'receiver_{name}_fps'] = round(sum(s['delta'][field] for s in valid)*1000/elapsed, 2) if elapsed else None
            pairs = [(s['delta']['packets_received'], s['delta']['packets_lost']) for s in samples
                     if 'packets_received' in s['delta'] and 'packets_lost' in s['delta']]
            packets = sum(a+b for a,b in pairs)
            result['receiver_loss_pct'] = round(sum(b for _,b in pairs)*100/packets, 2) if packets else None
            result['receiver_freeze_count'] = total('freeze_count')
            result['receiver_freeze_duration_ms'] = total('freeze_duration_ms')
            result['receiver_frames_dropped'] = total('frames_dropped')
            result['receiver_fec_packets_received'] = total('fec_packets_received')
            result['receiver_fec_packets_discarded'] = total('fec_packets_discarded')
            result['receiver_jitter_buffer_ms'] = ratio('jitter_buffer_delay_ms', 'jitter_buffer_emitted_count')
            result['receiver_decode_ms'] = ratio('decode_time_ms', 'frames_decoded')
            for name in SAMPLES:
                values = [v for s in samples for v in s['arrays'][name]]
                for p, value in percentiles(values).items():
                    result[f'receiver_{name.removesuffix("_ms")}_{p}_ms'] = value
            return result

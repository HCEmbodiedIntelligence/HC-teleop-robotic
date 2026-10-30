"""Camera stage rates sampled over one common monotonic time window."""
from collections import deque
import threading
import time


class CameraPipelineMetrics:
    def __init__(self, window_seconds=2.0, clock=None):
        self.window_seconds = float(window_seconds)
        self.clock = clock or (lambda: time.monotonic())
        self.started = self.clock()
        self.lock = threading.Lock()
        self.events = {name: deque() for name in ('capture', 'decode', 'submit', 'output')}
        self.totals = {name: 0 for name in self.events}
        self.durations = deque()

    def _trim(self, now):
        cutoff = now - self.window_seconds
        for samples in self.events.values():
            while samples and samples[0] < cutoff:
                samples.popleft()
        while self.durations and self.durations[0][0] < cutoff:
            self.durations.popleft()

    def record(self, stage):
        now = self.clock()
        with self.lock:
            self._trim(now)
            self.events[stage].append(now)
            self.totals[stage] += 1

    def output(self, submit_to_transport_ms, transport_ms):
        now = self.clock()
        with self.lock:
            self._trim(now)
            self.events['output'].append(now)
            self.totals['output'] += 1
            self.durations.append((now, submit_to_transport_ms, transport_ms))

    def reset_sender(self):
        with self.lock:
            for stage in ('submit', 'output'):
                self.events[stage].clear()
                self.totals[stage] = 0
            self.durations.clear()

    def snapshot(self):
        now = self.clock()
        with self.lock:
            self._trim(now)
            elapsed = min(self.window_seconds, max(0.0, now - self.started))
            rates = {stage: round(len(samples) / elapsed, 1) if elapsed > 0 else 0.0
                     for stage, samples in self.events.items()}
            durations = list(self.durations)
            return {
                'fps_window_seconds': self.window_seconds,
                'fps_window_elapsed_seconds': round(elapsed, 3),
                'pipeline_counts': dict(self.totals),
                'capture_fps': rates['capture'], 'decoded_fps': rates['decode'],
                'webrtc_submit_fps': rates['submit'], 'webrtc_output_fps': rates['output'],
                # Compatibility: this old field has always counted track handoffs.
                'webrtc_send_fps': rates['submit'],
                'webrtc_frames_sent': self.totals['submit'],
                'webrtc_frames_output': self.totals['output'],
                'webrtc_submit_to_transport_ms': round(sum(x[1] for x in durations)/len(durations), 2) if durations else None,
                'webrtc_transport_ms': round(sum(x[2] for x in durations)/len(durations), 2) if durations else None,
            }


def instrument_sender(pc, sender, track, metrics):
    """Observe this peer's completed frame submissions to its transport.

    aiortc has no public per-frame sent callback. Keep the adapter local to this
    sender/transport, and regression-test against the installed aiortc. An RTP
    output is the final primary-stream packet accepted by _send_rtp, not proof
    of receiver delivery or display. RTCP and retransmissions are excluded.
    """
    import struct
    from aiortc.rtp import is_rtcp
    original_next = sender._next_encoded_frame
    original_send = sender.transport._send_rtp
    pending = None

    async def next_encoded(codec):
        nonlocal pending
        frame = await original_next(codec)
        pending = ({'started': track._handoff_at, 'timestamp': None, 'transport_ms': 0.0}
                   if frame is not None else None)
        return frame

    async def send_rtp(data):
        nonlocal pending
        sample = pending
        is_primary = (sample is not None and len(data) >= 12 and not is_rtcp(data)
                      and struct.unpack_from('!I', data, 8)[0] == sender._ssrc)
        if is_primary:
            timestamp = struct.unpack_from('!I', data, 4)[0]
            if sample['timestamp'] is None:
                sample['timestamp'] = timestamp
            is_primary = timestamp == sample['timestamp']
        began = time.monotonic()
        await original_send(data)  # Includes SRTP protection and UDP submission.
        if is_primary and pending is sample:
            ended = time.monotonic()
            sample['transport_ms'] += (ended - began) * 1000
            if data[1] & 0x80:  # Last packet of this video frame.
                if pc.connectionState not in ('closed', 'failed'):
                    metrics.output((ended - sample['started']) * 1000, sample['transport_ms'])
                pending = None

    sender._next_encoded_frame = next_encoded
    sender.transport._send_rtp = send_rtp

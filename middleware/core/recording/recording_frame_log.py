"""Bounded, asynchronous per-frame diagnostics; never write on a DDS callback."""
import json
import os
from pathlib import Path
import queue
import threading
import time

from .capture_window import header_stamp_ns


class RecordingFrameLog:
    def __init__(self, stage, *, directory=None, capacity=8192, max_bytes=16*1024*1024):
        directory = directory if directory is not None else os.environ.get('HC_MIDDLEWARE_LOG_DIR')
        self.path = (Path(directory)/f'recording_frames_{stage}_{os.getpid()}.jsonl') if directory else None
        self.stage, self.max_bytes = stage, max_bytes
        self.queue = queue.Queue(maxsize=capacity)
        self.received = self.written = self.dropped = 0
        self.error = ''
        self._stop = threading.Event()
        self._thread = None
        if self.path is not None:
            self._thread = threading.Thread(target=self._run, name='recording-frame-log', daemon=True)
            self._thread.start()

    def offer(self, topic, source, kind, raw, wall_ns, mono_ns, *, active, generation=None,
              accepted=None):
        if self.path is None:
            return
        self.received += 1
        try:
            # Retain only the small standard CDR Header prefix, never image data.
            self.queue.put_nowait((self.received, topic, source, kind,
                raw[:12] if isinstance(raw, bytes) else b'', wall_ns, mono_ns,
                active, generation, accepted))
        except queue.Full:
            self.dropped += 1

    def status(self):
        return dict(enabled=self.path is not None, path=str(self.path or ''),
            received=self.received, written=self.written, dropped=self.dropped,
            queued=self.queue.qsize(), error=self.error,
            stopped=self._stop.is_set(), worker_alive=bool(self._thread and self._thread.is_alive()),
            clock='DDS: receive monotonic interval; published_action: actual publication wall interval',
            retention='current file + 2 rotated files, 16 MiB each by default')

    def close(self):
        self._stop.set()
        if self._thread is not None:
            self._thread.join(timeout=2.)
            if self._thread.is_alive():
                self.error = self.error or 'frame log drain incomplete'

    def _run(self):
        previous = {}
        stream = None
        try:
            self.path.parent.mkdir(parents=True, exist_ok=True)
            stream = self.path.open('a', buffering=65536)
            size, last_flush = self.path.stat().st_size, time.monotonic()
            while not self._stop.is_set() or not self.queue.empty():
                try:
                    seq, topic, source, kind, raw, wall, mono, active, generation, accepted = self.queue.get(timeout=.1)
                except queue.Empty:
                    stream.flush()
                    continue
                header = header_stamp_ns(raw, kind)
                key = topic, source, generation
                old = previous.get(key)
                receive_gap = (mono-old[1])/1e6 if old else None
                source_gap = (header-old[2])/1e6 if old and header is not None and old[2] is not None else None
                publication_gap = (wall-old[0])/1e6 if old and self.stage == 'published_action' else None
                gap = publication_gap if self.stage == 'published_action' else receive_gap
                row = dict(stage=self.stage, topic=topic, source_topic=source, sequence=seq,
                    level='WARNING' if (gap is not None and gap > 40 or source_gap is not None and source_gap > 40) else 'INFO',
                    accepting_snapshot=active, generation=generation, event_wall_ns=wall,
                    previous_event_wall_ns=old[0] if old else None,
                    diagnostic_monotonic_ns=mono, source_stamp_ns=header,
                    previous_source_stamp_ns=old[2] if old else None,
                    receive_interval_ms=receive_gap if self.stage != 'published_action' else None,
                    source_interval_ms=source_gap,
                    publication_ns=wall if self.stage == 'published_action' else None,
                    publication_interval_ms=publication_gap,
                    source_age_ms=(wall-header)/1e6 if header is not None else None,
                    source_age_basis='event wall minus original header; comparable wall epochs required',
                    ingress_accepted=accepted, log_queue_dropped_total=self.dropped,
                    interval_gt_40ms=bool(gap is not None and gap > 40),
                    source_gap_gt_40ms=bool(source_gap is not None and source_gap > 40))
                previous[key] = wall, mono, header
                # A capture generation is a boundary, not an ever-growing key.
                if len(previous) > 512:
                    previous = {key: previous[key]}
                line = json.dumps(row, ensure_ascii=False, separators=(',', ':'))+'\n'
                size += len(line.encode())
                stream.write(line)
                self.written += 1
                if size >= self.max_bytes:
                    stream.close()
                    for index in (2, 1):
                        source_path = self.path if index == 1 else Path(str(self.path)+'.1')
                        if source_path.exists():
                            source_path.replace(Path(str(self.path)+f'.{index}'))
                    stream = self.path.open('w', buffering=65536)
                    size = 0
                if time.monotonic()-last_flush >= .5:
                    stream.flush()
                    last_flush = time.monotonic()
        except Exception as exc:
            self.error = type(exc).__name__+': '+str(exc)
        finally:
            if stream is not None:
                stream.close()

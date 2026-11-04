"""Record the intersection of confirmed VR control and each MCAP capture."""
from concurrent.futures import ThreadPoolExecutor
from dataclasses import dataclass, field
import logging
from pathlib import Path

from .artifact_state import atomic_json


def takeover_path(source):
    return Path(source).with_suffix('.takeovers.json')


def takeover_file(source):
    """Optional companion of this MCAP for archives and dataset exports."""
    path = takeover_path(source)
    if Path(source).suffix == '.mcap' and path.is_file() and not path.is_symlink():
        return path


@dataclass
class _Capture:
    source: Path
    start_ns: int
    intervention_id: str | None
    intervals: list = field(default_factory=list)
    tagged: bool = False

    def open(self, token, stamp):
        if self.intervention_id is not None and token.intervention_id != self.intervention_id:
            return False
        stamp = max(self.start_ns, stamp)
        self.intervals.append(dict(session_id=token.session_id, intervention_id=token.intervention_id,
                                   start_ns=stamp, start_offset_s=(stamp - self.start_ns) / 1e9,
                                   end_ns=None, end_offset_s=None, end_reason=None))
        return True

    def close(self, stamp, reason):
        if not self.intervals or self.intervals[-1]['end_ns'] is not None:
            return False
        interval = self.intervals[-1]
        stamp = max(interval['start_ns'], stamp)
        interval.update(end_ns=stamp, end_offset_s=(stamp - self.start_ns) / 1e9, end_reason=reason)
        return True

    def save(self, *, complete=False):
        atomic_json(takeover_path(self.source), dict(schema='hc-dagger-takeovers/v1',
            tag='dagger', recording=self.source.name, clock='ros', recording_start_ns=self.start_ns,
            complete=complete, intervals=self.intervals))

    def checkpoint(self):
        try:
            self.save()
        except OSError:
            logging.getLogger(__name__).exception("Cannot write takeover times for %s", self.source)
            # Keep the intervals in memory. finish() retries and reports persistent errors.


class TakeoverLog:
    def __init__(self, clock):
        self.clock = clock
        self.worker = ThreadPoolExecutor(max_workers=1, thread_name_prefix='takeover-log')
        # Only the worker touches these fields. Disk I/O never holds a control lock.
        self.manual = None
        self.enabled = False
        self.captures = {}

    def dagger_changed(self, enabled):
        self.worker.submit(self._set_enabled, enabled)

    def _set_enabled(self, enabled):
        self.enabled = enabled
        if enabled:
            for capture in self.captures.values():
                if not capture.tagged:
                    capture.tagged = True
                    capture.checkpoint()

    def manual_changed(self, token, stamp, reason):
        self.worker.submit(self._change, token, stamp, reason)

    def _change(self, token, stamp, reason):
        self.manual = token
        for capture in self.captures.values():
            changed = capture.open(token, stamp) if token else capture.close(stamp, reason)
            if changed:
                capture.checkpoint()

    def begin(self, source, stamp=None, intervention_id=None):
        self.worker.submit(self._begin, Path(source), self.clock() if stamp is None else stamp, intervention_id)

    def _begin(self, source, stamp, intervention_id):
        capture = self.captures[source] = _Capture(source, stamp, intervention_id)
        if self.enabled:
            capture.tagged = True
            if self.manual:
                capture.open(self.manual, stamp)
            capture.checkpoint()

    def finish(self, source):
        return self.worker.submit(self._finish, Path(source), self.clock())

    def _finish(self, source, stamp):
        capture = self.captures.pop(source, None)
        if capture and capture.tagged:
            capture.close(stamp, 'recording_stopped')
            capture.save(complete=True)

    def shutdown(self):
        self.worker.shutdown()

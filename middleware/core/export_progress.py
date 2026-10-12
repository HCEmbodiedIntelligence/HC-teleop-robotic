"""Atomic, throttled cross-process export progress (no video/runtime dependencies)."""
from __future__ import annotations
import json
import os
import time
from pathlib import Path


def progress_path(destination: Path) -> Path:
    return destination.parent / f'.{destination.name}.progress.json'


def read_progress(path: Path) -> dict:
    try:
        value = json.loads(path.read_text(encoding='utf-8'))
        return value if isinstance(value, dict) else {}
    except (OSError, ValueError):
        return {}


class ExportProgress:
    def __init__(self, path: Path | None):
        self.path = path
        self.last_write = 0.0

    def update(self, stage, percent, current=0, total=0, unit='', state='exporting', error='', force=False, reset=False):
        if self.path is None:
            return
        now = time.monotonic()
        if not force and now - self.last_write < 0.5:
            return
        previous = {} if reset else read_progress(self.path)
        percent = max(float(previous.get('percent', 0)), float(percent))
        value = {'state': state, 'stage': stage, 'percent': round(min(100, max(0, percent)), 1),
                 'current': int(current), 'total': int(total), 'unit': unit,
                 'error': str(error), 'updated_at': time.time(), 'pid': os.getpid()}
        self.path.parent.mkdir(parents=True, exist_ok=True)
        temporary = self.path.with_suffix('.json.tmp')
        temporary.write_text(json.dumps(value, ensure_ascii=False), encoding='utf-8')
        temporary.replace(self.path)
        self.last_write = now

"""Bounded diagnostics, independent of control/recording and their locks.

Producers only enqueue references; callers must not mutate emitted event objects.
Formatting, redaction, causal indexing and disk I/O run on the worker. Existing
session log files are read from an allowlisted catalog, never from API paths.
"""
from __future__ import annotations

import asyncio
from collections import deque
import hashlib
import itertools
import json
import logging
import os
from pathlib import Path
import queue
import re
import threading
import time
import traceback
from datetime import datetime, timezone

from aiohttp import web

_SECRET = re.compile(r'(?:^|[_-])(token|password|passwd|secret|authorization|credential|api[_-]?key)(?:$|[_-])', re.I)
_TEXT_SECRET = re.compile(r'''(?ix)
 ((?:["']?(?:[\w-]*(?:token|password|passwd|secret|authorization|api[_-]?key))["']?\s*[:=]\s*)
 (?:"[^"\n]*"|'[^'\n]*'|[^\s,;}]+))
 ''')
_CLI_SECRET = re.compile(r'(?i)(--[\w-]*(?:token|password|secret|api-key)\s+)(\S+)')
_AUTH = re.compile(r'(?i)(\b(?:Bearer|Basic)\s+)\S+')
_URL_AUTH = re.compile(r'(https?://)[^\s/@:]+:[^\s/@]+@', re.I)
_LEVELS = {'debug': 10, 'info': 20, 'warning': 30, 'error': 40, 'critical': 50}
_SKIP = {'vr_pose', 'ros_message', 'camera_frame', 'camera_status', 'status'}
_CONTROL_DIAGNOSTICS = {'manual_input_timeout', 'heartbeat_timeout', 'recording_failed'}
_RECENT_ERROR_SECONDS = 300


def _subsystem_source(file, label=None):
    """`control/reload_1/v23_solver.log` -> ('control', 'control/v23_solver')."""
    parts = Path(file).parts
    subsystem = label or (parts[0] if len(parts) > 1 else 'middleware')
    return subsystem, f'{subsystem}/{Path(file).stem}'


def _entry_epoch(entry):
    try:
        return datetime.fromisoformat(str(entry.get('time'))).timestamp()
    except ValueError:
        return 0.


def _text(value, limit=16384):
    text = str(value)[:limit]
    text = _TEXT_SECRET.sub(lambda m: m.group(2) + '[REDACTED]', text)
    text = _CLI_SECRET.sub(r'\1[REDACTED]', text)
    text = _AUTH.sub(r'\1[REDACTED]', text)
    return _URL_AUTH.sub(r'\1[REDACTED]@', text)


def _clean(value, depth=0):
    if depth > 6:
        return '[truncated]'
    if isinstance(value, dict):
        return {str(k)[:128]: ('[REDACTED]' if _SECRET.search(str(k)) else _clean(v, depth + 1))
                for k, v in list(value.items())[:64]}
    if isinstance(value, (tuple, list)):
        return [_clean(v, depth + 1) for v in value[:64]]
    if value is None or type(value) in (bool, int):
        return value
    if isinstance(value, float):
        return value if float('-inf') < value < float('inf') else str(value)
    return _text(value, 16384 if depth < 2 else 4096)


def _stamp(value):
    try:
        stamp = float(value)
        if stamp > 1e12:
            stamp /= 1e9
        if not 0 < stamp < 32503680000:
            raise ValueError()
        return datetime.fromtimestamp(stamp, timezone.utc).isoformat(timespec='milliseconds')
    except (TypeError, ValueError, OverflowError):
        return datetime.now(timezone.utc).isoformat(timespec='milliseconds')


class DiagnosticLogHandler(logging.Handler):
    """No formatting, traceback work or file operations on the logging caller."""
    def __init__(self, service, level=logging.WARNING):
        super().__init__(level)
        self.service = service

    def emit(self, record):
        if not record.name.startswith(__name__):
            self.service._offer(('python', record))


class DiagnosticLogs:
    def __init__(self, project_root, log_dir=None, *, queue_size=1024,
                 max_entries=2000, max_bytes=5 * 1024 * 1024, backups=4,
                 poll_interval=.25, external_roots=()):
        self.project = Path(project_root).resolve()
        self.max_entries = max(10, min(int(max_entries), 10000))
        self.max_bytes = max(512, min(int(max_bytes), 20 * 1024 * 1024))
        self.backups = max(1, min(int(backups), 8))
        self.poll_interval = max(.02, float(poll_interval))
        if log_dir is None:
            label = datetime.now().strftime('session_%Y%m%d_%H%M%S') + '_' + os.urandom(3).hex()
            log_dir = self.project / 'runtime/middleware_logs' / label
        path = Path(log_dir).absolute()
        if path.is_symlink() or any(p.is_symlink() for p in path.parents):
            raise ValueError('log directory must not contain symlinks')
        self.log_dir = path
        # (label, base): the newest session_* folder under each base is tailed
        # too, e.g. the separately started HC_X1 hardware stack. Only warnings
        # and errors are kept from it so verbose drivers cannot evict evidence.
        self.external_roots = [(re.sub(r'[^a-zA-Z0-9_-]', '_', str(label))[:32], Path(base).absolute())
                               for label, base in external_roots]
        self.current_root = path.parent if path.name == 'middleware' else path
        self.current_session_id = self._session_id(self.current_root)
        self.journal = path / 'diagnostic_events.jsonl'
        self._queue = queue.Queue(maxsize=max(1, int(queue_size)))
        self._entries = deque(maxlen=self.max_entries)
        self._lock = threading.Lock()
        self._thread = None
        self._stop = threading.Event()
        self._closed = False
        self._drop_seq = itertools.count(1)
        self._dropped = 0
        self._reported_dropped = 0
        self.persistence_error = ''
        self._next_id = 1
        self._seen = set()
        self._tails = {}
        self._tail_paths = []
        self._next_catalog = 0.

    @staticmethod
    def _session_id(path):
        return hashlib.sha256(str(path).encode()).hexdigest()[:20]

    def _offer(self, item):
        if self._closed:
            return False
        try:
            self._queue.put_nowait(item)
            return True
        except queue.Full:
            # No other lock or formatting is allowed on the producer path.
            self._dropped = next(self._drop_seq)
            return False

    def emit(self, event):
        return self._offer(('event', event))

    def log(self, level, message, source='middleware', **details):
        return self._offer(('event', {'kind': 'log', 'source': source,
            'payload': {'level': level, 'message': message, **details}}))

    def start(self):
        if self._thread is not None:
            return self
        self.log_dir.mkdir(parents=True, mode=0o700, exist_ok=True)
        # Recovery is startup-only; the control threads are not running yet.
        recovered = self._read_journal(self.current_root)
        with self._lock:
            self._entries.extend(recovered[-self.max_entries:])
        self._next_id = max((e['id'] for e in recovered), default=0) + 1
        self._seen = {e.get('event_key') for e in recovered if e.get('event_key')}
        self._thread = threading.Thread(target=self._run, name='diagnostic-log-worker', daemon=True)
        self._thread.start()
        return self

    def close(self):
        self._closed = True
        self._stop.set()
        if self._thread is not None:
            self._thread.join(timeout=5)
            if self._thread.is_alive():
                self.persistence_error = 'diagnostic worker shutdown timed out; final queue may be incomplete'

    def _catalog(self):
        paths = {self.current_session_id: self.current_root}
        candidates = []
        for name in ('teleop_logs', 'middleware_logs'):
            base = self.project / 'runtime' / name
            if not self._safe(base, self.project / 'runtime'):
                continue
            try:
                candidates.extend(p for p in base.iterdir() if p.is_dir() and not p.is_symlink())
            except OSError:
                pass
        candidates.sort(key=lambda p: p.name, reverse=True)
        for path in candidates[:32]:
            paths[self._session_id(path)] = path
        return paths

    def sessions(self):
        return {'sessions': [{'id': key, 'label': path.name,
                              'current': key == self.current_session_id}
                             for key, path in self._catalog().items()],
                'current_session_id': self.current_session_id}

    @staticmethod
    def _safe(path, root):
        try:
            path.relative_to(root)
            if root.is_symlink():
                return False
            current = path
            while current != root:
                if current.is_symlink():
                    return False
                current = current.parent
            return True
        except ValueError:
            return False

    def _session(self, session):
        session = session or self.current_session_id
        if not isinstance(session, str) or not re.fullmatch(r'[0-9a-f]{20}', session):
            raise ValueError('unknown log session')
        path = self._catalog().get(session)
        if path is None:
            raise ValueError('unknown log session')
        return session, path

    def _journals(self, root):
        folder = self.log_dir if root == self.current_root else root / 'middleware'
        if not folder.exists():
            folder = root
        for index in range(self.backups, -1, -1):
            name = 'diagnostic_events.jsonl' + ('.' + str(index) if index else '')
            path = folder / name
            if self._safe(path, root):
                yield path

    def _read_journal(self, root):
        entries = deque(maxlen=self.max_entries)
        budget = self.max_bytes * (self.backups + 1)
        for path in self._journals(root):
            try:
                fd = os.open(path, os.O_RDONLY | os.O_NOFOLLOW)
                with os.fdopen(fd, 'rb') as stream:
                    # Bound even files maliciously enlarged outside this service.
                    size = os.fstat(stream.fileno()).st_size
                    if size > self.max_bytes:
                        stream.seek(size - self.max_bytes)
                        stream.readline(65536)
                    while budget > 0:
                        line = stream.readline(min(65536, budget))
                        if not line:
                            break
                        budget -= len(line)
                        try:
                            entry = json.loads(line)
                            if isinstance(entry, dict) and type(entry.get('id')) is int:
                                entries.append(_clean(entry))
                        except (ValueError, UnicodeDecodeError):
                            pass
            except OSError:
                continue
        return list(entries)

    def _raw_files(self, root):
        files = []
        # Session-local files only; do not traverse artifacts, other sessions or symlinks.
        stack = [(root, 0)]
        while stack and len(files) < 64:
            folder, depth = stack.pop()
            try:
                children = sorted(folder.iterdir())[:128]
            except OSError:
                continue
            for path in children:
                if not self._safe(path, root) or path.is_symlink():
                    continue
                if path.is_file() and path.suffix == '.log':
                    files.append(path)
                elif depth < 3 and path.is_dir() and path.name not in {'artifacts', 'topic_recordings'}:
                    stack.append((path, depth + 1))
        return files[:64]

    def _external_files(self):
        for label, base in self.external_roots:
            try:
                if base.is_symlink() or not base.is_dir():
                    continue
                sessions = sorted((p for p in base.iterdir()
                                   if p.name.startswith('session_') and p.is_dir() and not p.is_symlink()),
                                  key=lambda p: p.name)
            except OSError:
                continue
            if sessions:
                yield from ((path, sessions[-1], label) for path in self._raw_files(sessions[-1]))

    def _normalize(self, event):
        if not isinstance(event, dict):
            return None
        kind = str(event.get('kind', 'log'))
        if kind in _SKIP:
            return None
        wrapper = event
        payload = event.get('payload', {})
        payload = payload if isinstance(payload, dict) else {'message': payload}
        if kind == 'control_event':
            event = payload
            kind = str(event.get('kind', 'control_event'))
            payload = event
        level = str(payload.get('level', 'info')).lower().replace('warn', 'warning')
        if level not in _LEVELS:
            level = 'info'
        if kind in {'safety_stop', 'vr_input_fault'} or event.get('state') in {'FAULT', 'ESTOP'} or kind in _CONTROL_DIAGNOSTICS:
            level = 'error'
        message = payload.get('message') or payload.get('reason') or kind
        if kind == 'manual_input_timeout' and 'input_gap_ms' in payload:
            message = (f"{payload.get('reason', 'teleop input timeout')}: "
                       f"{payload['input_gap_ms']} ms > {payload.get('limit_ms', '?')} ms")
        owner = [event.get(k) for k in ('controller_instance_id', 'session_id', 'epoch')]
        control = all(v is not None and v != '' for v in owner)
        event_key = event.get('event_id') or wrapper.get('event_id')
        if control and 'sequence' in event:
            event_key = 'control:' + ':'.join(map(str, owner[:2])) + ':' + str(event['sequence'])
        return {'time': _stamp(event.get('stamp_ns', wrapper.get('timestamp', time.time()))),
                'level': level, 'source': _text(wrapper.get('source', 'middleware'), 128),
                'subsystem': 'middleware',
                'message': _text(message), 'kind': kind,
                'details': _clean(payload), 'file': None, 'line': None,
                'event_key': _text(event_key, 256) if event_key else None,
                'cause_key': payload.get('cause_event_id') or event.get('cause_event_id'),
                'owner_key': ':'.join(map(str, owner)) if control else None,
                'incident_id': event.get('incident_id') or payload.get('incident_id')}

    def _raw_entry(self, text, file, line, raw_key=None, label=None):
        # Structured control errors are also printed into middleware.log. Reuse
        # their original identity rather than count them twice.
        if '[control-event] ' in text:
            try:
                events = json.loads(text.split('[control-event] ', 1)[1])
                if isinstance(events, list):
                    return [self._normalize({'kind': 'control_event', 'source': 'ros_bridge', 'payload': e})
                            for e in events if isinstance(e, dict)]
            except ValueError:
                pass
        level_match = re.search(r'\[(DEBUG|INFO|WARN(?:ING)?|ERROR|FATAL|CRITICAL)\]', text, re.I)
        level = level_match[1].lower() if level_match else 'info'
        level = {'warn': 'warning', 'fatal': 'critical'}.get(level, level)
        if 'Traceback (most recent call last)' in text:
            level = 'error'
        timestamp = re.search(r'\[(\d{9,}\.\d+)\]', text)
        subsystem, source = _subsystem_source(file, label)
        entry = {'time': _stamp(timestamp[1] if timestamp else time.time()), 'level': level,
                 'source': source, 'subsystem': subsystem, 'message': _text(text),
                 'kind': 'control_rejection' if '[control-diag]' in text else 'saved_log',
                 'details': {}, 'file': file, 'line': line, 'event_key': raw_key,
                 'cause_key': None, 'owner_key': None, 'incident_id': None}
        return [entry]

    def _append(self, entry, *, persist=True):
        if entry is None or entry.get('event_key') in self._seen:
            return
        entry = _clean(entry)
        entry['id'] = self._next_id
        self._next_id += 1
        with self._lock:
            self._entries.append(entry)
            if entry.get('event_key'):
                self._seen.add(entry['event_key'])
            if len(self._seen) > self.max_entries * 2:
                self._seen = {e['event_key'] for e in self._entries if e.get('event_key')}
        if persist:
            self._write(entry)

    def _write(self, entry):
        try:
            if self.journal.is_symlink():
                raise OSError('journal symlink refused')
            if self.journal.exists() and self.journal.stat().st_size >= self.max_bytes:
                for index in range(self.backups, 0, -1):
                    source = self.journal if index == 1 else Path(str(self.journal) + '.' + str(index - 1))
                    destination = Path(str(self.journal) + '.' + str(index))
                    if source.is_symlink() or destination.is_symlink():
                        raise OSError('journal rotation symlink refused')
                    if source.exists():
                        source.replace(destination)
            fd = os.open(self.journal, os.O_WRONLY | os.O_CREAT | os.O_APPEND | os.O_NOFOLLOW, 0o600)
            with os.fdopen(fd, 'a', encoding='utf-8') as stream:
                stream.write(json.dumps(entry, ensure_ascii=False, allow_nan=False) + '\n')
            self.persistence_error = ''
        except OSError as exc:
            self.persistence_error = _text(f'{type(exc).__name__}: {exc}', 512)

    def _tail(self):
        now = time.monotonic()
        if now >= self._next_catalog:
            self._tail_paths = ([(path, self.current_root, None) for path in self._raw_files(self.current_root)]
                                + list(self._external_files()))
            self._next_catalog = now + 1.
        for path, root, label in self._tail_paths:
            try:
                fd = os.open(path, os.O_RDONLY | os.O_NOFOLLOW)
                with os.fdopen(fd, 'rb') as stream:
                    stat = os.fstat(stream.fileno())
                    state = self._tails.get(path)
                    if state is None or state['inode'] != stat.st_ino or stat.st_size < state['offset']:
                        state = {'inode': stat.st_ino, 'offset': max(0, stat.st_size - self.max_bytes),
                                 'line': 0, 'partial': b'', 'pending': [], 'pending_line': 1}
                        self._tails[path] = state
                        if state['offset']:
                            stream.seek(state['offset'])
                            skipped = stream.readline(65536)
                            state['offset'] += len(skipped)
                    stream.seek(state['offset'])
                    chunk = stream.read(65536)
                    state['offset'] += len(chunk)
                    lines = (state['partial'] + chunk).split(b'\n')
                    state['partial'] = lines.pop()[:65536]
                    for raw in lines:
                        state['line'] += 1
                        text = raw.decode('utf-8', 'replace')[:16384]
                        continuation = bool(text[:1].isspace() or text.startswith(('Traceback ', 'During handling', 'The above exception')))
                        if state['pending'] and not continuation:
                            self._flush_raw(path, state, root, label)
                        if not state['pending']:
                            state['pending_line'] = state['line']
                        state['pending'].append(text)
                        if sum(map(len, state['pending'])) > 32768:
                            self._flush_raw(path, state, root, label)
                    if not chunk and state['pending']:
                        self._flush_raw(path, state, root, label)
            except OSError:
                continue

    def _flush_raw(self, path, state, root=None, label=None):
        text = '\n'.join(state['pending'])
        state['pending'] = []
        relative = str(path.relative_to(root or self.current_root))
        shown = f'{label}/{relative}' if label else relative
        key = 'raw:' + hashlib.sha256(f"{shown}:{state['inode']}:{state['pending_line']}:{text}".encode()).hexdigest()
        for entry in self._raw_entry(text, relative, state['pending_line'], key, label):
            if label and entry is not None:
                if _LEVELS.get(entry['level'], 20) < _LEVELS['warning']:
                    continue
                entry['file'] = shown
            self._append(entry)

    def _run(self):
        while not self._stop.is_set() or not self._queue.empty():
            try:
                kind, value = self._queue.get(timeout=self.poll_interval)
                try:
                    if kind == 'python':
                        record = value
                        message = record.getMessage()
                        if record.exc_info:
                            message += '\n' + ''.join(traceback.format_exception(*record.exc_info))
                        entry = self._normalize({'kind': 'log', 'source': record.name,
                            'timestamp': record.created, 'payload': {'level': record.levelname.lower(), 'message': message}})
                    else:
                        entry = self._normalize(value)
                    self._append(entry)
                finally:
                    self._queue.task_done()
            except queue.Empty:
                pass
            except Exception as exc:
                # Never log recursively through DiagnosticLogHandler.
                self.persistence_error = _text(f'diagnostic processing failed: {exc}', 512)
            if self._dropped != self._reported_dropped:
                self._reported_dropped = self._dropped
                self._append(self._normalize({'kind': 'log_gap', 'source': 'diagnostics',
                    'payload': {'level': 'warning', 'message': f'日志队列已丢弃 {self._dropped} 条，溯源可能不完整',
                                'dropped': self._dropped}}))
            if not self._stop.is_set():
                self._tail()
        origins = {path: (root, label) for path, root, label in self._tail_paths}
        for path, state in self._tails.items():
            if state['partial']:
                state['pending'].append(state['partial'].decode('utf-8', 'replace'))
            if state['pending'] and path in origins:
                self._flush_raw(path, state, *origins[path])

    @staticmethod
    def _causes(entries, session):
        aliases = {e['event_key']: e['id'] for e in entries if e.get('event_key')}
        by_id = {e['id']: e for e in entries}
        incident_roots = {}
        for entry in entries:
            if entry['kind'] in _CONTROL_DIAGNOSTICS and entry.get('owner_key'):
                incident_roots[entry['owner_key']] = entry
        output = []
        incidents = {}
        for saved in entries:
            entry = dict(saved)
            explicit_cause = aliases.get(entry.get('cause_key'))
            entry.update(incident_id=entry.get('incident_id'), cause_id=explicit_cause,
                         root_cause_id=explicit_cause)
            seen = set()
            root = by_id.get(explicit_cause)
            while root and root['id'] not in seen:
                seen.add(root['id'])
                parent = by_id.get(aliases.get(root.get('cause_key')))
                if parent is None:
                    break
                root = parent
            is_fault = entry['kind'] == 'safety_stop' or (
                entry['kind'] == 'revoked' and entry.get('details', {}).get('state') in {'FAULT', 'ESTOP'})
            if entry.get('owner_key'):
                evidence = incident_roots.get(entry['owner_key'])
                if evidence and evidence.get('details', {}).get('reason') == entry.get('details', {}).get('reason'):
                    root = evidence
                    entry['cause_id'] = evidence['id'] if evidence['id'] != entry['id'] else None
            if root:
                entry['root_cause_id'] = root['id']
            if is_fault:
                key = entry.get('incident_id') or entry.get('owner_key') or f"event:{entry['id']}"
                incident_id = hashlib.sha256(f'{session}:{key}'.encode()).hexdigest()[:20]
                entry['incident_id'] = incident_id
                chain_ids = list(dict.fromkeys([*(seen or []), *((root['id'],) if root else ()), entry['id']]))
                chain = [dict(by_id[i]) for i in sorted(chain_ids) if i in by_id]
                incidents[incident_id] = {'id': incident_id, 'title': entry['message'], 'level': entry['level'],
                    'root_cause_id': root['id'] if root else None,
                    'root_message': root['message'] if root else '底层原因未确认；仅记录到停止结果',
                    'chain': chain, 'relation': 'explicit' if root else 'unknown'}
            output.append(entry)
        return output, list(incidents.values())[-40:]

    @staticmethod
    def _summary(entries, now=None):
        """Per-subsystem health for the overview cards; independent of filters."""
        now = time.time() if now is None else now
        groups = {}
        for entry in entries:
            name = entry.get('subsystem') or 'middleware'
            group = groups.setdefault(name, {'subsystem': name, 'errors': 0, 'warnings': 0, 'recent_errors': 0,
                                             'total': 0, 'sources': {}, 'latest_error': None})
            group['total'] += 1
            rank = _LEVELS.get(entry['level'], 20)
            group['errors'] += rank >= _LEVELS['error']
            group['warnings'] += _LEVELS['warning'] <= rank < _LEVELS['error']
            group['recent_errors'] += rank >= _LEVELS['error'] and now - _entry_epoch(entry) <= _RECENT_ERROR_SECONDS
            source = group['sources'].setdefault(entry['source'], {'errors': 0, 'warnings': 0})
            source['errors'] += rank >= _LEVELS['error']
            source['warnings'] += _LEVELS['warning'] <= rank < _LEVELS['error']
            if rank >= _LEVELS['error'] and (group['latest_error'] is None or entry['id'] > group['latest_error']['id']):
                group['latest_error'] = {key: entry.get(key) for key in ('id', 'time', 'source', 'message', 'kind')}
                group['latest_error']['message'] = _text(entry['message'], 600)
        for group in groups.values():
            latest = group['latest_error']
            group['state'] = ('error' if latest and now - _entry_epoch(latest) <= _RECENT_ERROR_SECONDS
                              else 'warning' if group['errors'] or group['warnings'] else 'ok')
        order = {'error': 0, 'warning': 1, 'ok': 2}
        return sorted(groups.values(), key=lambda g: (order[g['state']], g['subsystem']))

    def snapshot(self, session=None, after=0, limit=200, level='all', q='', priority=True, subsystem=''):
        session, root = self._session(session)
        after, limit = max(0, int(after)), max(1, min(int(limit), 500))
        if level not in {'all', *_LEVELS}:
            raise ValueError('invalid log level')
        if session == self.current_session_id:
            with self._lock:
                entries = list(self._entries)
        else:
            entries = self._read_journal(root)
            if not entries:
                for file in self._raw_files(root):
                    try:
                        fd = os.open(file, os.O_RDONLY | os.O_NOFOLLOW)
                        with os.fdopen(fd, 'rb') as stream:
                            size = os.fstat(stream.fileno()).st_size
                            if size > 262144:
                                stream.seek(size - 262144)
                                stream.readline(65536)
                            text = stream.read(262144).decode('utf-8', 'replace')
                        for number, line in enumerate(text.splitlines(), 1):
                            for entry in self._raw_entry(line, str(file.relative_to(root)), number):
                                if entry:
                                    entry['id'] = len(entries) + 1
                                    entries.append(entry)
                        entries = entries[-self.max_entries:]
                    except OSError:
                        continue
        entries, incidents = self._causes(entries, session)
        summary = self._summary(entries)
        subsystem = str(subsystem or '')[:64]
        q = str(q).lower()[:256]
        # Advance over nonmatching rows as well; cursor always follows ingest
        # order, never severity order, so priority cannot skip unseen records.
        remaining = [e for e in entries if e['id'] > after]
        scanned, selected = [], []
        for entry in remaining:
            scanned.append(entry)
            if (level == 'all' or _LEVELS.get(entry['level'], 20) >= _LEVELS[level]) and (
                    not subsystem or (entry.get('subsystem') or 'middleware') == subsystem) and (
                    not q or q in (entry['message'] + ' ' + entry['source']).lower()):
                selected.append(entry)
            if len(selected) >= limit:
                break
        cursor = scanned[-1]['id'] if scanned else after
        if priority:
            selected.sort(key=lambda e: (-_LEVELS.get(e['level'], 20), -e['id']))
        return {'session_id': session, 'entries': selected, 'incidents': incidents, 'summary': summary,
                'cursor': cursor, 'has_more': bool(remaining and cursor < remaining[-1]['id']),
                'dropped': self._dropped if session == self.current_session_id else 0,
                'persistence_error': self.persistence_error if session == self.current_session_id else ''}

    def export(self, session=None):
        result = self.snapshot(session, limit=500, priority=False)
        rows, cursor = list(result['entries']), result['cursor']
        while result['has_more'] and len(rows) < self.max_entries:
            result = self.snapshot(session, after=cursor, limit=500, priority=False)
            rows.extend(result['entries'])
            cursor = result['cursor']
        return ''.join(json.dumps(_clean(e), ensure_ascii=False, allow_nan=False) + '\n' for e in rows)


def register_log_routes(app, service):
    async def sessions(_request):
        return web.json_response(await asyncio.to_thread(service.sessions))

    async def entries(request):
        try:
            values = request.query
            result = await asyncio.to_thread(service.snapshot, values.get('session'),
                int(values.get('after', 0)), int(values.get('limit', 200)),
                values.get('level', 'all'), values.get('q', ''), values.get('priority', '1') != '0',
                values.get('subsystem', ''))
            return web.json_response(result)
        except (ValueError, TypeError):
            raise web.HTTPBadRequest(text='invalid log query or unknown session')

    async def export(request):
        try:
            session, _ = await asyncio.to_thread(service._session, request.query.get('session'))
            body = await asyncio.to_thread(service.export, session)
            return web.Response(text=body, content_type='application/x-ndjson',
                headers={'Content-Disposition': f'attachment; filename="logs-{session}.jsonl"'})
        except (ValueError, TypeError):
            raise web.HTTPBadRequest(text='unknown log session')

    app.router.add_get('/api/logs/sessions', sessions)
    app.router.add_get('/api/logs', entries)
    app.router.add_get('/api/logs/export', export)

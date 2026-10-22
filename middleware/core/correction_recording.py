"""Prearmed correction windows using main's TopicRecorder and TaskRecorder.

Writers and subscriptions live before AUTO. The ownership callback only claims
a prepared slot and publishes an ordered boundary; normal close never joins a
writer, starts a process, scans a directory or waits for an encoder.
"""
from collections import deque
import copy
from dataclasses import asdict
from datetime import datetime, timezone
import json
import multiprocessing as mp
from pathlib import Path
import queue
import shutil
import threading
import time
import uuid

from .topic_recorder import TopicRecorder, enqueue_record
from .ros_recording_executor import recording_subscriptions, _put_latest


class WindowRouter:
    """Source-time routing, first-observation snapshots and bounded lateness."""
    def __init__(self, sinks, bounds, *, max_age_ns=250_000_000, late_ns=200_000_000,
                 cache_bytes=64*1024**2, long_gap_ns=200_000_000, required_topics=()):
        self.sinks, self.bounds = sinks, bounds
        self.max_age_ns, self.late_ns = max_age_ns, late_ns
        self.byte_limit, self.bytes = cache_bytes, 0
        self.long_gap_ns = long_gap_ns
        self.required_topics = tuple(required_topics)
        self.latest = {}
        self.cache = deque()
        self.windows = {}
        self.receive_seq = 0
        self.rejected_time = 0

    def open(self, event):
        window = dict(event, counts={}, last={}, max_gaps={}, invalid_ranges=[],
                      copied={}, action_count=0, last_action_seq=0, missing_actions=0,
                      first_action_ns=None, last_action_ns=None, rejected=0, action_ranges=[])
        self.windows[event['key']] = window
        window['start_event_saved'] = self.sinks[event['slot']]({'topic': '/events/control/intervention',
            'stamp_ns': event['start_ns'], 'payload': {**event, 'kind': 'manual_open'}})
        initial = {}
        start, end = self.bounds(event['slot'])
        for sample in self.cache:
            stamp = sample['stamp_ns']
            if stamp <= start and start-stamp <= self.max_age_ns:
                previous = initial.get(sample['topic'])
                if previous is None or previous['stamp_ns'] < stamp:
                    initial[sample['topic']] = sample
        for sample in initial.values():
            self._write_observation(window, sample, 'initial_observation', replay=True)
        for sample in self.cache:
            if start < sample['stamp_ns'] and (not end or sample['stamp_ns'] < end):
                self._write_observation(window, sample, 'in_window', replay=True)

    def close(self, event):
        window = self.windows.get(event['key'])
        if window is None:
            raise ValueError('close without an ordered open boundary')
        window.update(end_ns=event['end_ns'], end_reason=event['reason'], last_command_seq=event['last_command_seq'])

    def observe(self, topic, kind, raw, stamp_ns, received_ns, *, time_quality='source_header', size=None):
        self.receive_seq += 1
        if stamp_ns <= 0 or stamp_ns > received_ns or received_ns-stamp_ns > self.max_age_ns+self.late_ns:
            self.rejected_time += 1
            return
        size = len(raw) if size is None else size
        sample = dict(topic=topic, msg_type=kind, _raw=raw, stamp_ns=stamp_ns, _sample_bytes=size,
                      received_ns=received_ns, receive_sequence=self.receive_seq, time_quality=time_quality)
        if size <= self.byte_limit:
            while self.cache and (self.bytes+size > self.byte_limit or
                    received_ns-self.cache[0]['received_ns'] > self.max_age_ns+self.late_ns):
                evicted = self.cache.popleft()
                self.bytes -= evicted['_sample_bytes']
            self.cache.append(sample)
            self.bytes += size
        for window in tuple(self.windows.values()):
            start, end = self.bounds(window['slot'])
            if start <= stamp_ns and (not end or stamp_ns < end) and (not end or received_ns <= end+self.late_ns):
                self._write_observation(window, sample, 'in_window')

    def _write_observation(self, window, sample, role, *, replay=False):
        topic, stamp = sample['topic'], sample['stamp_ns']
        if not replay and sample['receive_sequence'] <= window['copied'].get(topic, -1):
            return
        window['copied'][topic] = max(sample['receive_sequence'], window['copied'].get(topic, -1))
        accepted = self.sinks[window['slot']]({**sample, '_provenance': dict(
            topic=topic, role=role, source_stamp_ns=stamp, received_ns=sample['received_ns'],
            time_quality=sample['time_quality'], key=window['key'])})
        window['counts'][topic] = window['counts'].get(topic, 0)+int(accepted)
        window['rejected'] += not accepted
        if not accepted:
            return
        previous = window['last'].get(topic, window['start_ns'])
        if previous is not None:
            gap = max(0, stamp-previous)
            window['max_gaps'][topic] = max(gap, window['max_gaps'].get(topic, 0))
            if gap > self.long_gap_ns and len(window['invalid_ranges']) < 1024:
                window['invalid_ranges'].append(dict(topic=topic, start_ns=previous, end_ns=stamp, reason='long_source_gap'))
        window['last'][topic] = max(stamp, window['last'].get(topic, stamp))

    def action(self, evidence, make_action):
        window = self.windows.get(evidence['key'])
        if window is None:
            return False
        start, end = self.bounds(window['slot'])
        stamp, sequence = evidence['stamp_ns'], evidence['command_seq']
        if (evidence.get('control_source', evidence.get('source')) != 'vr' or
                not evidence.get('intervention') or stamp < start or (end and stamp >= end) or
                type(sequence) is not int or sequence < 1 or
                sequence > window.get('last_command_seq', sequence)):
            return False
        ranges = window['action_ranges']
        if any(lo <= sequence <= hi for lo, hi in ranges):
            return False
        ranges.append((sequence, sequence))
        merged = []
        for lo, hi in sorted(ranges):
            if merged and lo <= merged[-1][1]+1:
                merged[-1] = (merged[-1][0], max(hi, merged[-1][1]))
            else:
                merged.append((lo, hi))
        if len(merged) > 1024:
            ranges.pop()
            window['rejected'] += 1
            return False
        window['action_ranges'] = merged
        window['last_action_seq'] = max(sequence, window['last_action_seq'])
        window['action_count'] += 1
        window['first_action_ns'] = min(window['first_action_ns'] or stamp, stamp)
        window['last_action_ns'] = max(window['last_action_ns'] or stamp, stamp)
        accepted = self.sinks[window['slot']]({'topic': '/events/control/action_evidence',
            'stamp_ns': stamp, 'payload': evidence})
        accepted = make_action(window['slot'], evidence) and accepted
        window['rejected'] += not accepted
        return accepted

    def finish_due(self, now_ns):
        completed = []
        for key, window in tuple(self.windows.items()):
            if 'end_ns' not in window or now_ns < window['end_ns']+self.late_ns:
                continue
            missing_topics = [topic for topic in self.required_topics if not window['counts'].get(topic)]
            for topic in set(window['last']) | set(self.required_topics):
                previous = window['last'].get(topic, window['start_ns'])
                gap = max(0, window['end_ns']-previous)
                window['max_gaps'][topic] = max(gap, window['max_gaps'].get(topic, 0))
                if (gap > self.long_gap_ns or topic in missing_topics) and len(window['invalid_ranges']) < 1024:
                    window['invalid_ranges'].append(dict(topic=topic, start_ns=previous,
                        end_ns=window['end_ns'], reason='missing_source' if topic in missing_topics else 'long_tail_gap'))
            missing = max(0, window['last_command_seq']-window['action_count'])
            quality = dict(action_received=window['action_count'], action_emitted=window['last_command_seq'],
                missing_actions=missing, rejected=window['rejected'], counts=window['counts'],
                first_action_ns=window['first_action_ns'], last_action_ns=window['last_action_ns'],
                max_gap_ns=window['max_gaps'], invalid_ranges=window['invalid_ranges'], missing_topics=missing_topics,
                grade='review' if not window['action_count'] else 'warning' if missing or window['rejected'] or window['invalid_ranges'] else 'ok',
                source_frame_counter='unavailable; coverage uses source timestamps', outcome='unknown')
            quality['start_event_saved'] = window['start_event_saved']
            if not window['start_event_saved']:
                quality['grade'] = 'warning' if window['action_count'] else 'review'
            boundary = {k:window[k] for k in ('key', 'start_ns', 'end_ns', 'end_reason', 'last_command_seq')}
            if not self.sinks[window['slot']]({'topic': '/events/control/intervention',
                    'stamp_ns': window['end_ns'], 'payload': dict(boundary, kind='manual_close', quality=quality)}):
                # End metadata is also returned out-of-band and persisted by
                # TaskRecorder; a saturated data queue cannot erase it.
                quality['end_event_in_mcap'] = False
            else:
                quality['end_event_in_mcap'] = True
            completed.append(dict(slot=window['slot'], key=key, boundary=boundary, quality=quality))
            del self.windows[key]
        return completed


def _capture_process(config, resources, shared_bounds, boundaries, actions, completed, health, stopped):
    import rclpy
    from rclpy.context import Context
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.serialization import deserialize_message, serialize_message
    from rosidl_runtime_py.utilities import get_message
    from sensor_msgs.msg import JointState
    from .ros_qos import subscription_qos
    context = Context()
    node = executor = None
    try:
        rclpy.init(args=[], context=context, domain_id=int(config.get('domain_id', 14)))
        node = rclpy.create_node('hc_correction_capture', context=context)
        executor = SingleThreadedExecutor(context=context)
        executor.add_node(node)
        options = config.get('recording', {}).get('corrections', {})
        router = WindowRouter([lambda event, resource=resource: enqueue_record(resource, event) for resource in resources],
            lambda slot: tuple(shared_bounds[slot*2:slot*2+2]),
            max_age_ns=int(options.get('observation_max_age_sec', .25)*1e9),
            late_ns=int(options.get('late_message_sec', .2)*1e9),
            cache_bytes=int(options.get('observation_cache_bytes', 64*1024**2)),
            long_gap_ns=int(options.get('long_gap_sec', .2)*1e9))
        subscriptions = [item for item in recording_subscriptions(config)
                         if not item['topic'].startswith(('/action/', '/control/', '/events/', '/status/'))
                         and item['type'].startswith(('sensor_msgs/', 'geometry_msgs/'))]
        router.required_topics = tuple(item['topic'] for item in subscriptions if item.get('required', True))
        seen = {}
        handles = []

        def commands():
            while True:
                try:
                    event = boundaries.get_nowait()
                except queue.Empty:
                    break
                (router.open if event['op'] == 'open' else router.close)(event)

        for item in subscriptions:
            cls = get_message(item['type'])
            def receive(raw, item=item, cls=cls):
                commands()
                received = node.get_clock().now().nanoseconds
                message = deserialize_message(raw, cls)
                header = getattr(message, 'header', None)
                stamp = header.stamp.sec*1_000_000_000+header.stamp.nanosec if header else 0
                seen[item['topic']] = dict(stamp_ns=stamp, received_ns=received)
                router.observe(item['topic'], item['type'], bytes(raw), stamp, received)
            handles.append(node.create_subscription(cls, item['topic'], receive, subscription_qos(item), raw=True))

        def make_action(slot, evidence):
            msg = JointState(name=evidence['names'], position=evidence['positions'])
            msg.header.stamp.sec, msg.header.stamp.nanosec = divmod(evidence['stamp_ns'], 1_000_000_000)
            return enqueue_record(resources[slot], dict(topic='/action/joint_states', msg_type='sensor_msgs/msg/JointState',
                _raw=bytes(serialize_message(msg)), stamp_ns=evidence['stamp_ns'],
                _provenance=dict(key=evidence['key'], command_seq=evidence['command_seq'], action_boundary='hc_final')))

        last_health = 0.
        while not stopped.is_set() or router.windows:
            commands()
            while True:
                try:
                    router.action(actions.get_nowait(), make_action)
                except queue.Empty:
                    break
            executor.spin_once(timeout_sec=.002)
            now = node.get_clock().now().nanoseconds
            for result in router.finish_due(now):
                completed.put(result)
            if time.monotonic()-last_health >= .05:
                required = [item['topic'] for item in subscriptions if item.get('required', True)]
                issues = [topic for topic in required if topic not in seen or
                          not 0 <= now-seen[topic]['stamp_ns'] <= router.max_age_ns]
                _put_latest(health, dict(ready=bool(required) and not issues, issues=issues,
                    seen=seen, state='running', stamp_ns=now, monotonic_ns=time.monotonic_ns(), cache_bytes=router.bytes,
                    time_rejected=router.rejected_time))
                last_health = time.monotonic()
    except Exception as exc:
        _put_latest(health, dict(ready=False, state='error', error=str(exc)))
    finally:
        if executor:
            executor.shutdown()
        if node:
            node.destroy_node()
        context.try_shutdown()


class CorrectionSession:
    def __init__(self, service, metadata):
        self.service, self.metadata = service, copy.deepcopy(metadata)
        self.task_managed = hasattr(service, 'platform')
        self.jobs = {}
        self.selection = copy.deepcopy(service.platform.recording_selection()) if self.task_managed else None
        if self.selection: self.selection.pop('request_id', None)
        self.config = service.config.get('corrections', {})
        self.count = int(self.config.get('prepared_slots', 4))
        if not 2 <= self.count <= 16:
            raise ValueError('corrections.prepared_slots must be between 2 and 16')
        self.mp = mp.get_context('spawn')
        self.bounds = self.mp.Array('q', self.count*2)
        self.boundaries = self.mp.Queue(maxsize=self.count*2+1)
        self.actions = self.mp.Queue(maxsize=int(self.config.get('action_queue_items', 4096)))
        self.completed = self.mp.Queue(maxsize=self.count)
        self.health = self.mp.Queue(maxsize=4)
        self.stopped = self.mp.Event()
        self.lock = threading.RLock()
        self.slots, self.prepared = [], deque()
        self.active = None
        self.state = 'PREPARING'
        self.error = ''
        self.source_health = {}
        self.process = None
        self.closing = False
        self.registry_events = deque()
        self.worker = threading.Thread(target=self._monitor, name='correction-slots', daemon=True)

    def prepare(self):
        self.service.directory.mkdir(parents=True, exist_ok=True)
        if self.service.config.get('derive_action_from_next_state', False):
            raise ValueError('corrections cannot derive actions from the next state')
        for index in range(self.count):
            recorder = TopicRecorder({**self.service.config, 'reuse_writer_process': True,
                'action_sources': [],
                # Slot writers do not recover or publish the ordinary recorder's
                # legacy artifact jobs; TaskRecorder owns this entire session.
                'upload_job_state_dir': str(self.service.directory/'.correction-slots'/str(index)),
                'queue_bytes': int(self.config.get('total_queue_bytes', 256*1024**2))//self.count}, self.service.config_dir)
            # TaskRecorder owns preparation, sealing and uploading. Standalone
            # captures remain local too; an unclaimed warm slot is not an artifact.
            recorder.task_managed = True
            self.slots.append(dict(recorder=recorder, job=None, state='preparing'))
            self._prepare_slot(index)
        config = copy.deepcopy(self.service.ros_config)
        config['recording'] = copy.deepcopy(self.service.config)
        self.process = self.mp.Process(target=_capture_process,
            args=(config, [slot['recorder'].ipc_resources() for slot in self.slots], self.bounds,
                  self.boundaries, self.actions, self.completed, self.health, self.stopped),
            name='correction-capture', daemon=True)
        self.process.start()
        self.worker.start()

    def _prepare_slot(self, index):
        pending = len(self.service.pending) if self.task_managed else 0
        if pending >= self.service.config.get('max_pending_jobs', 8):
            if self.process is None:
                raise ValueError('wait for pending recordings before preparing corrections')
            self.slots[index]['state'] = 'waiting'
            return
        if shutil.disk_usage(self.service.directory).free < self.service.config.get('min_free_bytes', 268435456):
            raise RuntimeError('insufficient space to prepare correction slot')
        ident = uuid.uuid4().hex
        metadata = {**self.metadata, 'capture_mode': 'corrections_only', 'outcome': 'unknown'}
        slot = self.slots[index]
        filename = ident+'_correction.mcap'
        if self.task_managed:
            self.service.start(filename, metadata, selection=self.selection, _prepared_writer=slot['recorder'])
            job = next(job for job in self.service.jobs.values() if job['chosen_filename'] == filename)
        else:
            path = slot['recorder'].start(filename, {**metadata, 'recording_id': ident})
            job = dict(recording_id=ident, path=path, metadata=metadata, state='armed',
                       format=slot['recorder'].save_format, started_at=None, ended_at=None,
                       video_fps=slot['recorder'].lerobot_fps, binding=None)
        with self.lock:
            slot.update(job=job, state='prepared')
            self.prepared.append(index)

    def _save(self, job):
        current = self.jobs.get(job['recording_id'], {})
        if (current.get('capture_state') in {'source_complete', 'failed'} and
                job.get('capture_state') in {'capturing', 'draining'}):
            return  # A delayed lifecycle notification cannot reopen a closed file.
        self.jobs[job['recording_id']] = job
        if self.task_managed:
            with self.service.lock:
                self.service.jobs[job['recording_id']] = job
                self.service.save(job)

    def open(self, token, stamp_ns):
        with self.lock:
            if self.closing or self.active is not None or not self.prepared:
                raise RuntimeError('no prepared correction slot available')
            index = self.prepared.popleft()
            slot = self.slots[index]
            if slot['recorder'] is not None and not slot['recorder']._process.is_alive():
                raise RuntimeError('prepared correction writer exited')
            event = dict(op='open', slot=index, key=token.key(), start_ns=stamp_ns, **asdict(token))
            self.bounds[index*2:index*2+2] = [stamp_ns, 0]
            self.boundaries.put_nowait(event)
            slot['job']['metadata'].update(asdict(token))
            slot['job'].update(state='recording', capture_state='capturing', started_at=datetime.fromtimestamp(stamp_ns/1e9, timezone.utc).isoformat())
            slot['state'] = 'capturing'
            self.active = (index, token)
            self.registry_events.append(copy.deepcopy(slot['job']))
            self.state = 'CAPTURING'
            return slot['job']['recording_id']

    def close(self, token, stamp_ns, last_command_seq, reason):
        with self.lock:
            if self.active is None or self.active[1] != token:
                return
            index = self.active[0]
            self.bounds[index*2+1] = stamp_ns
            self.boundaries.put_nowait(dict(op='close', key=token.key(), end_ns=stamp_ns,
                last_command_seq=last_command_seq, reason=reason))
            slot = self.slots[index]
            slot['job'].update(state='stopping', capture_state='draining',
                ended_at=datetime.fromtimestamp(stamp_ns/1e9, timezone.utc).isoformat())
            slot['state'] = 'draining'
            self.registry_events.append(copy.deepcopy(slot['job']))
            self.active = None
            self.state = 'ARMED' if self.prepared else 'DEGRADED'

    def action(self, evidence):
        with self.lock:
            if self.active is None:
                return
            index, token = self.active
            if any(evidence.get(k) != v for k,v in asdict(token).items()):
                return
            try:
                self.actions.put_nowait({**evidence, 'key': token.key()})
            except queue.Full:
                job = self.slots[index]['job']
                job['action_queue_dropped'] = job.get('action_queue_dropped', 0)+1

    def status(self):
        with self.lock:
            ready = (not self.closing and bool(self.prepared) and self.source_health.get('ready', False)
                     and 0 <= time.monotonic_ns()-self.source_health.get('monotonic_ns', 0) <= 1_000_000_000
                     and self.process is not None and self.process.is_alive() and not self.error)
            state = self.state if self.active or self.closing else 'ARMED' if ready else 'PREPARING' if not self.error else 'DEGRADED'
            return dict(state=state, ready=ready, prepared_slots=len(self.prepared), error=self.error,
                        sources=copy.deepcopy(self.source_health),
                        active=copy.deepcopy(self.slots[self.active[0]]['job']) if self.active else None)

    def _monitor(self):
        try:
            while not self.closing or any(slot['state'] in {'capturing', 'draining'} for slot in self.slots):
                if not self.closing:
                    for index, slot in enumerate(self.slots):
                        if slot['state'] == 'waiting':
                            self._prepare_slot(index)
                while True:
                    try:
                        self.source_health = self.health.get_nowait()
                        if self.source_health.get('state') == 'error':
                            raise RuntimeError(self.source_health['error'])
                    except queue.Empty:
                        break
                with self.lock:
                    changes = list(self.registry_events)
                    self.registry_events.clear()
                for job in changes:
                    self._save(job)
                try:
                    result = self.completed.get(timeout=.02)
                except queue.Empty:
                    if self.process and not self.process.is_alive():
                        raise RuntimeError('correction receiver exited before shutdown')
                    continue
                index = result['slot']
                slot, job = self.slots[index], self.slots[index]['job']
                job['quality'] = result['quality']
                job['metadata']['correction'] = {**result['boundary'], 'quality': result['quality']}
                # The exporter started by stop() must see immutable correction
                # bounds before reading the closed MCAP.
                from .artifact_state import atomic_json
                atomic_json(Path(job['path']).with_suffix('.correction.json'),
                    {**job['metadata']['correction'], 'intervention': True, 'outcome': 'unknown'})
                writer = slot['recorder'].stop()
                if writer.get('error') or writer['messages'] != writer['accepted']:
                    job.update(capture_state='failed', state='interrupted', error=writer.get('error') or 'writer did not drain all accepted events')
                else:
                    job.update(capture_state='source_complete', state='finalizing', error=None)
                if self.task_managed:
                    saved = copy.deepcopy(job)
                    with self.service.lock:
                        self.service.jobs[job['recording_id']] = saved
                        self.service.save(saved)
                        if job['capture_state'] == 'source_complete':
                            self.service.submit(saved)
                    # Report source completion only after finalization is queued.
                    self.jobs[job['recording_id']] = saved
                else:
                    if job['capture_state'] == 'source_complete':
                        if job['format'] == 'mcap_mp4':
                            from .video_bundle import prepare
                            prepare(job['path'], self.service.directory/'bundles'/Path(job['path']).stem, job)
                        job['state'] = 'ready'
                    self._save(copy.deepcopy(job))
                slot['state'] = 'complete'
                if not self.closing:
                    self._prepare_slot(index)
        except Exception as exc:
            self.error, self.state = str(exc), 'DEGRADED'
            for slot in self.slots:
                if slot['state'] in {'capturing', 'draining'}:
                    slot['job'].update(capture_state='failed', state='interrupted', error=self.error)
                    self._save(copy.deepcopy(slot['job']))

    def stop(self):
        with self.lock:
            if self.active:
                raise ValueError('revoke manual control before stopping correction capture')
            self.closing, self.state = True, 'STOPPED'
        if self.worker.ident is not None:
            self.worker.join()
        self.stopped.set()
        if self.process:
            self.process.join(timeout=5.)
            if self.process.is_alive():
                self.process.terminate()
                self.process.join(timeout=2.)
        for slot in self.slots:
            writer = slot['recorder']
            # Empty preparation slots must never produce exports or uploads.
            if slot['state'] in {'preparing', 'prepared', 'waiting'}:
                writer.save_format = 'mcap'
            result = writer.stop()
            job = slot['job']
            if job and slot['state'] in {'prepared', 'waiting'} and result['messages'] == 0:
                writer.path.unlink(missing_ok=True)
                if self.task_managed:
                    with self.service.lock:
                        self.service.jobs.pop(job['recording_id'], None)
                        (self.service.state_dir/(job['recording_id']+'.json')).unlink(missing_ok=True)
            writer.close_writer()
            writer._artifacts.shutdown()

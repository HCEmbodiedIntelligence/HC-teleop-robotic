from __future__ import annotations

import multiprocessing as mp
import queue
import threading
import time
import traceback
from collections import deque
from typing import Any

from .topic_recorder import TopicRecorder, _counter_add, _lower_recording_priority
from .x1_recording import is_x1
from .capture_window import SensorPreroll, buffered_sensor
from .config import isolated_control_process_enabled


def recording_subscriptions(config: dict[str, Any]) -> list[dict[str, Any]]:
    """Return enabled ROS subscriptions explicitly selected for MCAP recording."""
    return [
        dict(item)
        for item in config.get("subscriptions", [])
        if item.get("enabled", True) and "record" in item.get("outputs", [])
    ]


def recording_reader_groups(config, subscriptions):
    """Keep large payload wait-sets out of the ordinary X1 state reader.

    This is part of the isolated control architecture. Other profiles and the
    previous middleware mode retain their existing process/QoS contract.
    """
    if not (is_x1(config) and isolated_control_process_enabled(config)):
        return [('all', subscriptions)]
    images, small = [], []
    for item in subscriptions:
        target = images if item.get('type') in {
            'sensor_msgs/msg/Image', 'sensor_msgs/msg/CompressedImage',
            'sensor_msgs/msg/PointCloud2',
        } else small
        target.append(item)
    return [(name, items) for name, items in [('small', small), ('images', images)] if items] or [('small', [])]


def recording_inputs(topic: str, config: dict[str, Any]) -> tuple[str, ...]:
    if is_x1(config) and topic in (
        '/observation/ee_poses', '/observation/left_arm/ee_pose', '/observation/right_arm/ee_pose'
    ):
        return ('/internal/recording' + topic,)
    # Public messages still arrive during ordinary teleoperation. The X1 v23
    # ownership path produces separate recording copies during interventions.
    if config.get("legacy_control_profile") and topic in (
        "/action/ee_poses", "/control/solver/ee_targets", "/control/solver/joint_targets"
    ):
        return topic, "/internal/recording" + topic
    return (topic,)


_IDLE_MONITOR_TOPICS = {
    # The dashboard needs the solver stage while capture is inactive.
    "/control/solver/joint_targets",
    # Camera frames arriving at capture start were exposed tens of ms earlier.
    # Keep only the two small proprioception streams warm for true pre-roll.
    "/observation/joint_states",
    "/observation/hands/joint_states",
}

_STATE_PREROLL_TOPICS = {
    "/observation/joint_states", "/observation/hands/joint_states",
}


class StatePreroll:
    """Bounded real raw-CDR state history; never interpolate or modify stamps.

    The raw recording process owns this object on its single executor thread.
    Snapshots may overlap consecutive recordings by design: those are actual
    samples from the 250 ms immediately preceding each capture.
    """

    def __init__(self, seconds: float = .25, byte_budget: int = 1_048_576):
        self.seconds = seconds
        self.byte_budget = byte_budget
        self.samples: deque[tuple[float, dict[str, Any]]] = deque()
        self.buffered_bytes = 0

    def _trim(self, now: float) -> None:
        while self.samples and (
            now - self.samples[0][0] > self.seconds
            or self.buffered_bytes > self.byte_budget
            or len(self.samples) > 128
        ):
            _, old = self.samples.popleft()
            self.buffered_bytes -= len(old['_raw'])

    def remember(self, now: float, event: dict[str, Any]) -> None:
        if event['topic'] not in _STATE_PREROLL_TOPICS:
            return
        self.samples.append((now, event))
        self.buffered_bytes += len(event['_raw'])
        self._trim(now)

    def snapshot(self, now: float) -> list[tuple[float, dict[str, Any]]]:
        self._trim(now)
        return list(self.samples)


def idle_monitoring_subscriptions(
    subscriptions: list[dict[str, Any]],
    *, buffer_sensors: bool = False,
) -> list[dict[str, Any]]:
    """Keep the dashboard stage and selected bounded sensor history warm.

    Keeping every small recording topic subscribed while idle used almost one
    CPU core on X1.  More importantly, their reliable depth-64 queues could
    back-pressure publishers whenever this low-priority process was delayed.
    Ordinary X1 also keeps selected cameras and four small evidence streams
    warm in this isolated process. These use explicitly non-blocking QoS and
    bounded raw-byte history. Other modes retain the configured capture QoS.
    """
    return [
        item
        for item in subscriptions
        if str(item.get("topic", "")) in _IDLE_MONITOR_TOPICS
        or buffer_sensors and buffered_sensor(item)
    ]


class RateGate:
    """Rate limit without halving a source that jitters around the configured cap."""

    def __init__(self) -> None:
        self._next_emit: dict[str, float] = {}
        self._lock = threading.Lock()

    def reset(self) -> None:
        with self._lock:
            self._next_emit.clear()

    def allow(self, topic: str, max_hz: float, now: float) -> bool:
        if max_hz <= 0:
            return True
        period = 1.0 / max_hz
        with self._lock:
            deadline = self._next_emit.get(topic)
            tolerance = min(0.002, period * 0.1)
            if deadline is not None and now + tolerance < deadline:
                return False
            self._next_emit[topic] = now + period
            return True


def _put_latest(target: Any, value: dict[str, Any]) -> None:
    try:
        target.put_nowait(value)
        return
    except queue.Full:
        pass
    try:
        target.get_nowait()
    except queue.Empty:
        pass
    try:
        target.put_nowait(value)
    except queue.Full:
        pass


def _health_snapshot(
    subscriptions: list[dict[str, Any]],
    timestamps: dict[str, deque[float]],
    message_counts: dict[str, int],
    now: float,
    active_topics: set[str] | None = None,
) -> dict[str, dict[str, Any]]:
    from .ros_bridge import DEFAULT_TOPIC_STANDARDS

    result: dict[str, dict[str, Any]] = {}
    for item in subscriptions:
        topic = str(item["topic"])
        if active_topics is not None and topic not in active_topics:
            continue
        samples = timestamps[topic]
        sample_times = list(samples)
        last_stamp = samples[-1] if samples else 0.0
        default = DEFAULT_TOPIC_STANDARDS.get(
            topic, {"target_hz": 10.0, "min_hz": 1.0}
        )
        target_hz = float(item.get("target_hz", 0.0) or default["target_hz"])
        min_hz = float(item.get("min_hz", 0.0) or default["min_hz"])
        if len(samples) >= 2 and now - last_stamp <= 1.8:
            elapsed = samples[-1] - samples[0]
            hz = (len(samples) - 1) / elapsed if elapsed > 0.0001 else 0.0
        else:
            hz = 0.0
        intervals = [
            later - earlier
            for earlier, later in zip(sample_times, sample_times[1:])
        ]
        max_gap_ms = round(max(intervals) * 1000.0, 1) if intervals else None
        if intervals:
            ordered = sorted(intervals)
            p95_gap_ms = round(
                ordered[int((len(ordered) - 1) * 0.95)] * 1000.0,
                1,
            )
        else:
            p95_gap_ms = None
        has_data = bool(samples) and now - last_stamp <= 1.8
        if not has_data:
            state = "no_data"
            message = "未检测到消息发布 (0 Hz)"
        elif min_hz > 0 and hz < min_hz:
            state = "low_rate"
            message = f"频率偏低 ({hz:.1f} Hz < 标准 {min_hz:.1f} Hz)"
        else:
            state = "ok"
            message = f"正常 ({hz:.1f} Hz)"
        result[topic] = {
            "topic": topic,
            "type": str(item["type"]),
            "messages": message_counts[topic],
            "hz": round(hz, 1),
            "target_hz": target_hz,
            "min_hz": min_hz,
            "record_enabled": True,
            "has_data": has_data,
            "state": state,
            "message": message,
            "last_received_age": round(now - last_stamp, 2) if last_stamp else None,
            "max_gap_ms": max_gap_ms,
            "p95_gap_ms": p95_gap_ms,
        }
    return result


def _should_release_subscriptions(
    *,
    has_subscriptions: bool,
    capture_seen: bool,
    accepting: bool,
    now: float,
    prepared_until: float,
) -> bool:
    """Release idle inputs after capture, or after an abandoned precheck."""
    return bool(
        has_subscriptions
        and not accepting
        and (capture_seen or now >= prepared_until)
    )


def _raw_recording_process(
    config: dict[str, Any],
    subscriptions: list[dict[str, Any]],
    event_queue: Any,
    accepting_event: Any,
    dropped_counter: Any,
    stop_event: Any,
    reset_event: Any,
    subscriptions_ready_event: Any,
    status_queue: Any,
    ingress_resources: Any,
    capture_window: Any = None,
) -> None:
    """Receive serialized ROS messages without deserializing them."""
    normal_priority = bool(config.get('_isolated_recording_reader', False))
    if not normal_priority:
        _lower_recording_priority()
    context = None
    executor = None
    node = None
    counters = {
        "received": 0,
        "recorded": 0,
        "throttled": 0,
        "rejected": 0,
        "serialization_errors": 0,
        "preroll_recorded": 0,
        "outside_capture_window": 0,
        "invalid_header_fallbacks": 0,
    }
    timestamps = {str(item["topic"]): deque(maxlen=120) for item in subscriptions}
    message_counts = {str(item["topic"]): 0 for item in subscriptions}
    gate = RateGate()
    sensor_window = is_x1(config) and capture_window is not None
    preroll = (SensorPreroll([str(item['topic']) for item in subscriptions if buffered_sensor(item)])
               if sensor_window else StatePreroll())
    capture_active = False
    max_rates = {str(item['topic']): float(item.get('max_hz', 0.))
                 for item in subscriptions}

    def record_event(event, received_at, *, historical=False):
        if sensor_window and capture_window.enabled():
            allowed, clock = capture_window.allows(event)
            if not allowed:
                counters['outside_capture_window'] += 1
                return
            if clock == 'invalid_header':
                counters['invalid_header_fallbacks'] += 1
        if not gate.allow(event['topic'], max_rates[event['topic']], received_at):
            counters['throttled'] += 1
            return
        try:
            if enqueue(event):
                counters['recorded'] += 1
                if historical:
                    counters['preroll_recorded'] += 1
        except queue.Full:
            counters['rejected'] += 1
            _counter_add(dropped_counter)

    def begin_capture():
        nonlocal capture_active
        if capture_active:
            return
        capture_active = True
        # Historical receive times also drive configured limits. Applying one
        # current time to the whole batch would incorrectly discard all but one.
        for received_at, event in preroll.snapshot(time.monotonic()):
            if (sensor_window and not capture_window.enabled()
                    and (event['topic'] not in _STATE_PREROLL_TOPICS
                         or time.monotonic()-received_at > .25)):
                continue
            record_event(event, received_at, historical=True)

    def enqueue(event):
        from .topic_recorder import enqueue_record
        if sensor_window:
            return enqueue_record(ingress_resources, event, capture_window=capture_window)
        return enqueue_record(ingress_resources, event)

    idle_subscriptions = idle_monitoring_subscriptions(subscriptions, buffer_sensors=sensor_window)
    idle_topics = {str(item["topic"]) for item in idle_subscriptions}
    ros_subscriptions: dict[str, Any] = {}
    subscription_modes: dict[str, str] = {}
    prepared_until = 0.0
    capture_seen = False
    has_capture_subscriptions = False

    def publish_status(state: str, error: str | None = None) -> None:
        now = time.monotonic()
        _put_latest(
            status_queue,
            {
                "state": state,
                "active": accepting_event.is_set(),
                "active_subscriptions": len(ros_subscriptions),
                "idle_monitor_subscriptions": len(
                    idle_topics.intersection(ros_subscriptions)
                ),
                "capture_subscriptions": len(
                    set(ros_subscriptions).difference(idle_topics)
                ),
                "subscriptions": [str(item["topic"]) for item in subscriptions],
                "topic_health": _health_snapshot(
                    subscriptions,
                    timestamps,
                    message_counts,
                    now,
                    {source.removeprefix('/internal/recording') for source in ros_subscriptions},
                ),
                "pid": __import__("os").getpid(),
                "priority": "normal" if normal_priority else "low",
                "reader_group": config.get('_recording_reader_group', 'all'),
                "state_preroll_ms": 250. if sensor_window else preroll.seconds * 1000,
                "history_retention_ms": preroll.seconds * 1000,
                "capture_window": capture_window.snapshot() if sensor_window else {},
                "preroll_byte_budget": preroll.byte_budget,
                "preroll_budget_evicted_bytes": getattr(preroll, 'evicted_bytes', 0),
                "warm_qos_override": {
                    'reliability': 'BEST_EFFORT',
                    'topics': sorted(source for source, mode in subscription_modes.items() if mode == 'warm'),
                    'scope': 'ordinary X1 warm readers only; other recording readers use configured QoS',
                } if sensor_window else None,
                "preroll_buffered_messages": len(preroll.samples),
                "error": error,
                **counters,
            },
        )

    try:
        import rclpy
        from rclpy.context import Context
        from rclpy.executors import SingleThreadedExecutor
        from rclpy.qos import qos_profile_sensor_data
        from .ros_qos import recording_subscription_qos
        if sensor_window:
            from .ros_qos import warm_recording_subscription_qos
        from rosidl_runtime_py.utilities import get_message

        domain_id = int(config.get("domain_id", 14))
        context = Context()
        rclpy.init(args=[], context=context, domain_id=domain_id)
        node_name = f"{config.get('node_name', 'hc_teleop_middleware')}_recorder"
        node = rclpy.create_node(node_name, context=context)
        executor = SingleThreadedExecutor(context=context)
        executor.add_node(node)

        def create_subscription(item: dict[str, Any], source: str, *, recording: bool,
                                warm: bool = False) -> None:
            topic = str(item["topic"])
            mode = 'warm' if warm else 'recording' if recording else 'idle'
            if source in ros_subscriptions:
                if subscription_modes[source] == mode:
                    return
                node.destroy_subscription(ros_subscriptions.pop(source))
                subscription_modes.pop(source, None)
            msg_type = str(item["type"])
            max_hz = float(item.get("max_hz", 0.0))
            message_type = get_message(msg_type)

            def callback(
                raw_data: bytes,
                *,
                topic: str = topic,
                msg_type: str = msg_type,
                max_hz: float = max_hz,
            ) -> None:
                nonlocal capture_active
                now = time.monotonic()
                counters["received"] += 1
                message_counts[topic] += 1
                timestamps[topic].append(now)
                if (topic == '/action/joint_states'
                        and config.get('record_action_at_publication', False)):
                    return  # Publication capture owns data; retain DDS health.
                if not isinstance(raw_data, bytes):
                    counters["serialization_errors"] += 1
                    return
                event = {
                    "kind": "ros_message",
                    "topic": topic,
                    "msg_type": msg_type,
                    "_raw": raw_data,
                    "record_only": source != topic,
                    "stamp_ns": time.time_ns(),
                }
                if accepting_event.is_set():
                    begin_capture()
                    # Append after taking the start snapshot, so this live
                    # packet cannot be emitted twice in the current capture.
                    preroll.remember(now, event)
                    record_event(event, now)
                else:
                    capture_active = False
                    preroll.remember(now, event)

            ros_subscriptions[source] = node.create_subscription(
                message_type,
                source,
                callback,
                warm_recording_subscription_qos(item) if warm else recording_subscription_qos(item)
                if recording
                else qos_profile_sensor_data,
                raw=True,
            )
            subscription_modes[source] = mode

        def destroy_subscriptions() -> None:
            for subscription in ros_subscriptions.values():
                node.destroy_subscription(subscription)
            ros_subscriptions.clear()
            subscription_modes.clear()

        def activate_subscriptions() -> None:
            nonlocal prepared_until, capture_seen, capture_active, has_capture_subscriptions
            prepared_until = time.monotonic() + 30.0
            capture_seen = accepting_event.is_set()
            gate.reset()
            capture_active = False
            for topic in timestamps:
                timestamps[topic].clear()
                message_counts[topic] = 0
            for key in counters:
                counters[key] = 0
            # Ordinary X1 reuses its persistent best-effort warm readers with
            # unchanged DDS identity across capture boundaries. Other streams
            # and snapshot/correction modes use configured recording QoS.
            ordinary = sensor_window and capture_window.enabled()
            for item in subscriptions:
                for source in recording_inputs(str(item["topic"]), config):
                    create_subscription(item, source, recording=True,
                                        warm=ordinary and buffered_sensor(item))
            has_capture_subscriptions = any(source not in idle_topics for source in ros_subscriptions)
            subscriptions_ready_event.set()
            publish_status("ready")

        def deactivate_capture_subscriptions() -> None:
            nonlocal capture_seen, has_capture_subscriptions
            for source in list(ros_subscriptions):
                if source not in idle_topics:
                    node.destroy_subscription(ros_subscriptions.pop(source))
                    subscription_modes.pop(source, None)
            for item in idle_subscriptions:
                create_subscription(item, str(item["topic"]), recording=False,
                                    warm=sensor_window and buffered_sensor(item))
            capture_seen = False
            has_capture_subscriptions = False
            subscriptions_ready_event.clear()
            publish_status("monitoring")

        for item in idle_subscriptions:
            create_subscription(item, str(item["topic"]), recording=False,
                                warm=sensor_window and buffered_sensor(item))
        publish_status("monitoring" if ros_subscriptions else "standby")
        last_status = time.monotonic()
        while not stop_event.is_set():
            if reset_event.is_set():
                reset_event.clear()
                activate_subscriptions()
            accepting = accepting_event.is_set()
            if accepting:
                capture_seen = True
                begin_capture()
            elif _should_release_subscriptions(
                has_subscriptions=has_capture_subscriptions,
                capture_seen=capture_seen,
                accepting=accepting,
                now=time.monotonic(),
                prepared_until=prepared_until,
            ):
                deactivate_capture_subscriptions()
            if not accepting:
                capture_active = False
            if ros_subscriptions:
                executor.spin_once(timeout_sec=0.02)
            else:
                stop_event.wait(0.05)
            now = time.monotonic()
            if now - last_status >= 0.5:
                publish_status(
                    "recording"
                    if accepting
                    else "ready"
                    if has_capture_subscriptions
                    else "monitoring"
                    if ros_subscriptions
                    else "standby"
                )
                last_status = now
    except Exception as exc:
        publish_status(
            "error",
            f"{type(exc).__name__}: {exc}\n{traceback.format_exc(limit=8)}",
        )
    finally:
        if executor is not None:
            try:
                if node is not None:
                    executor.remove_node(node)
                executor.shutdown(timeout_sec=2.0)
            except Exception:
                pass
        if node is not None:
            try:
                node.destroy_node()
            except Exception:
                pass
        if context is not None:
            try:
                context.try_shutdown()
            except Exception:
                pass
        publish_status("stopped")


class RosRecordingExecutor:
    """Independent raw readers share one immutable recording window/ingress."""

    def __init__(self, config: dict[str, Any], recorder: TopicRecorder):
        self.config = config
        self.recorder = recorder
        # The mode is known only after TaskRecorder freezes its capture plan.
        # Reconcile readers before opening ingress, never at the first frame.
        base = getattr(recorder, 'recorder', recorder)
        base.prepare_capture_window = self.prepare_recording
        window = getattr(base, '_capture_window', None)
        if window is not None and is_x1(config) and not getattr(base, '_reusable', False):
            # First ordinary preflight must not briefly replace the warm
            # readers with configured ones and then recreate them again when
            # the first writer freezes its ordinary-X1 metadata. Snapshot
            # metadata explicitly changes the mode before its final preflight.
            window.reset(True)
        self.domain_id = int(config.get("domain_id", 14))
        self.subscriptions = recording_subscriptions(config)
        self._mp = mp.get_context("spawn")
        self._stop_event = self._mp.Event()
        self._reset_event = self._mp.Event()
        self._subscriptions_ready_event = self._mp.Event()
        self._status_queue = self._mp.Queue(maxsize=4)
        self._process: Any = None
        self._isolated_readers = bool(is_x1(config) and isolated_control_process_enabled(config))
        self._workers = []
        for index, (name, items) in enumerate(recording_reader_groups(config, self.subscriptions)):
            self._workers.append(dict(name=name, subscriptions=items, process=None, status={},
                reset=self._reset_event if index == 0 else self._mp.Event(),
                ready=self._subscriptions_ready_event if index == 0 else self._mp.Event(),
                status_queue=self._status_queue if index == 0 else self._mp.Queue(maxsize=4)))
        self._gate = RateGate()
        self._lock = threading.Lock()
        self._status: dict[str, Any] = {
            "state": "disabled" if not config.get("enabled", True) else "starting",
            "domain_id": self.domain_id,
            "node_name": f"{config.get('node_name', 'hc_teleop_middleware')}_recorder",
            "subscriptions": [item["topic"] for item in self.subscriptions],
            "received": 0,
            "recorded": 0,
            "throttled": 0,
            "rejected": 0,
            "serialization_errors": 0,
            "preroll_recorded": 0,
            "state_preroll_ms": 250.0,
            "topic_health": {},
            "priority": "normal" if self._isolated_readers else "low",
            "error": None,
        }

    def start(self) -> None:
        if not self.config.get("enabled", True) or self._process is not None:
            return
        resources = self.recorder.ipc_resources()
        event_queue, accepting_event, dropped_counter = resources[:3]
        self._stop_event.clear()
        for worker in self._workers:
            worker['ready'].clear()
            worker_config = self.config
            if self._isolated_readers:
                worker_config = {**self.config,
                    'node_name': self.config.get('node_name', 'hc_teleop_middleware')+'_'+worker['name'],
                    '_isolated_recording_reader': True, '_recording_reader_group': worker['name']}
            worker['process'] = self._mp.Process(
                target=_raw_recording_process,
                args=(worker_config, worker['subscriptions'], event_queue, accepting_event,
                    dropped_counter, self._stop_event, worker['reset'], worker['ready'],
                    worker['status_queue'], resources, getattr(self.recorder, '_capture_window', None)),
                name='ros-cdr-recorder-'+worker['name'] if self._isolated_readers else 'ros-cdr-recorder',
                daemon=True)
            worker['process'].start()
            if self._process is None:
                self._process = worker['process']  # Existing primary-reader diagnostics.

    def stop(self) -> None:
        processes = [worker['process'] for worker in self._workers if worker['process'] is not None]
        if not processes:
            return
        self._stop_event.set()
        while any(process.is_alive() for process in processes):
            for process in processes:
                process.join(timeout=.05)
            self._refresh_status()
            # A multiprocessing feeder can still be flushing accepted image
            # bytes. Normal stop must not kill it and silently truncate them.
            with self._lock:
                self._status.update(state='draining', active=False)
            if self.recorder.status().get('error'):
                raise RuntimeError('writer failed while ROS input was draining; source is incomplete')
        self._process = None
        for worker in self._workers:
            worker['process'] = None
        with self._lock:
            self._status.update(state="stopped", active=False)

    def _refresh_status(self) -> None:
        for worker in self._workers:
            while True:
                try:
                    worker['status'] = worker['status_queue'].get_nowait()
                except queue.Empty:
                    break
            process = worker['process']
            if process is not None and not process.is_alive() and worker['status'].get('state') not in {'error', 'stopped'}:
                if self._stop_event.is_set() and process.exitcode == 0:
                    worker['status'].update(state='stopped', active=False)
                else:
                    worker['status'].update(state='error', error=(
                        f"{worker['name']} recording process exited with code {process.exitcode}"))
        states = [worker['status'] for worker in self._workers if worker['status']]
        if not states:
            return
        if len(self._workers) == 1:
            combined = dict(states[0])
        else:
            combined = dict(states[0])
            for key in ('received', 'recorded', 'throttled', 'rejected', 'serialization_errors',
                        'preroll_recorded', 'outside_capture_window', 'invalid_header_fallbacks',
                        'active_subscriptions', 'idle_monitor_subscriptions', 'capture_subscriptions',
                        'preroll_buffered_messages', 'preroll_budget_evicted_bytes'):
                combined[key] = sum(state.get(key, 0) for state in states)
            combined['topic_health'] = {topic: health for state in states for topic, health in state.get('topic_health', {}).items()}
            combined['subscriptions'] = [item['topic'] for item in self.subscriptions]
            errors = [state['error'] for state in states if state.get('error')]
            combined['error'] = '; '.join(errors) or None
            if errors:
                combined['state'] = 'error'
            elif all(state.get('state') == 'stopped' for state in states):
                combined['state'] = 'stopped'
            elif any(state.get('active') for state in states):
                combined['state'] = 'recording'
            elif len(states) != len(self._workers):
                combined['state'] = 'starting'
            elif all(state.get('state') == 'ready' for state in states):
                combined['state'] = 'ready'
            else:
                combined['state'] = 'monitoring'
            combined['active'] = any(state.get('active') for state in states)
            combined['warm_qos_override'] = (dict(reliability='BEST_EFFORT',
                topics=[topic for state in states for topic in (state.get('warm_qos_override') or {}).get('topics', [])],
                scope='ordinary X1 independent raw readers; configured QoS retained outside warm sensor streams')
                if any(state.get('warm_qos_override') is not None for state in states) else None)
        combined['readers'] = [{**worker['status'], 'group': worker['name']} for worker in self._workers]
        with self._lock:
            self._status.update(combined)

    def status(self) -> dict[str, Any]:
        self._refresh_status()
        with self._lock:
            result = dict(self._status)
            result["topic_health"] = dict(self._status.get("topic_health", {}))
            return result

    def get_topic_health(self) -> dict[str, Any]:
        return dict(self.status().get("topic_health", {}))

    def prepare_recording(self) -> None:
        if not self.config.get("enabled", True):
            return
        if self._process is None:
            self.start()
        self._gate.reset()
        for worker in self._workers:
            worker['ready'].clear()
            worker['reset'].set()
        with self._lock:
            self._status.update(
                received=0,
                recorded=0,
                throttled=0,
                rejected=0,
                serialization_errors=0,
                preroll_recorded=0,
            )
        deadline = time.monotonic()+5.
        for worker in self._workers:
            if not worker['ready'].wait(timeout=max(0., deadline-time.monotonic())):
                self._refresh_status()
                error = self._status.get('error')
                raise TimeoutError(f"ROS {worker['name']} recording subscriptions did not become ready within 5 seconds"
                    + (f": {error}" if error else ''))

    def _handle_message(
        self,
        raw_data: bytes,
        *,
        topic: str,
        msg_type: str,
        max_hz: float,
    ) -> None:
        """Exercise immediate live CDR ingress; pre-roll runs in the child."""
        if not self.recorder.is_recording():
            return
        now = time.monotonic()
        with self._lock:
            self._status["received"] += 1
        if not self._gate.allow(topic, max_hz, now):
            with self._lock:
                self._status["throttled"] += 1
            return
        if not isinstance(raw_data, bytes):
            with self._lock:
                self._status["serialization_errors"] += 1
            return
        event = {
                "kind": "ros_message",
                "topic": topic,
                "msg_type": msg_type,
                "_raw": raw_data,
                "stamp_ns": time.time_ns(),
            }
        window = getattr(self.recorder, '_capture_window', None)
        if window is not None and window.enabled():
            allowed, clock = window.allows(event)
            if not allowed:
                with self._lock:
                    self._status['outside_capture_window'] = self._status.get('outside_capture_window', 0)+1
                return
        accepted = self.recorder.record(event)
        with self._lock:
            self._status["recorded" if accepted else "rejected"] += 1

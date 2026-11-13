"""Capture accepted action publications without blocking the actuator send lock.

Reserve immediately before the final send lock and complete in a finally block
after that lock is released. Start capture after opening the recorder ingress;
end capture before closing ingress. Only this worker serializes or writes CDR.
"""
from collections import deque
from dataclasses import dataclass
import threading
import time


@dataclass(frozen=True)
class JointSnapshot:
    sec: int
    nanosec: int
    frame_id: str
    name: tuple
    position: tuple
    velocity: tuple
    effort: tuple
    publication_ns: int

    @classmethod
    def copy(cls, message, publication_ns):
        return cls(int(message.header.stamp.sec), int(message.header.stamp.nanosec),
                   str(message.header.frame_id), tuple(message.name),
                   tuple(message.position), tuple(message.velocity),
                   tuple(message.effort), publication_ns)


@dataclass
class CaptureTicket:
    generation: int | None
    completed: bool = False


def serialize_snapshot(snapshot):
    from rclpy.serialization import serialize_message
    from sensor_msgs.msg import JointState
    message = JointState(name=list(snapshot.name), position=list(snapshot.position),
                         velocity=list(snapshot.velocity), effort=list(snapshot.effort))
    message.header.stamp.sec = snapshot.sec
    message.header.stamp.nanosec = snapshot.nanosec
    message.header.frame_id = snapshot.frame_id
    return serialize_message(message)


class PublicationCapture:
    """Bounded asynchronous publication capture with generation drain barriers.

    ``reserve``/``complete`` are control-thread operations and must both run
    outside the bridge send lock. They never serialize, perform recorder IPC or
    wait for the worker. An optional 250 ms history contains only real successful
    publications and is copied into each new recording without changing stamps.
    Queue overflow and recording rejection remain visible in ``status``.
    """
    TOPIC = '/action/joint_states'
    TYPE = 'sensor_msgs/msg/JointState'

    def __init__(self, recorder, *, serializer=serialize_snapshot, max_queue=256,
                 pre_roll_s=.25, selected=None):
        if max_queue < 1 or not 0 <= pre_roll_s <= 1:
            raise ValueError('capture queue must be positive; pre-roll must be 0..1 s')
        self.recorder, self.serializer = recorder, serializer
        self.record_event = getattr(recorder, 'record_publication', recorder.record)
        self.max_queue, self.pre_roll_s = max_queue, pre_roll_s
        self.selected = selected or (lambda: True)
        self._condition = threading.Condition()
        self._queue, self._history = deque(), deque(maxlen=1024)
        self._inflight, self._pending = {}, {}
        self._tickets = {}
        self._generation, self._active, self._closed = 0, False, False
        self._captured = self._enqueued = self._dropped = self._failed = 0
        self._last_error = ''
        self._thread = threading.Thread(target=self._run,
                                        name='published-action-recorder', daemon=True)
        self._thread.start()

    def reserve(self):
        with self._condition:
            if self._closed:
                return None
            generation = self._generation if self._active else None
            if generation is not None:
                self._inflight[generation] = self._inflight.get(generation, 0) + 1
            ticket = CaptureTicket(generation)
            self._tickets[id(ticket)] = ticket
            return ticket

    def complete(self, ticket, message=None, *, publication_ns=None,
                 publication_monotonic=None):
        """Release one reservation; rejected/failed sends supply ``None``.

        Copies JointState fields before entering the short bookkeeping lock.
        The worker exclusively owns serialization and recorder admission.
        """
        if ticket is None:
            return
        snapshot, error = None, None
        try:
            if message is not None:
                snapshot = JointSnapshot.copy(message,
                    time.time_ns() if publication_ns is None else publication_ns)
        except Exception as exc:
            error = exc
        now = time.monotonic()
        published_at = now if publication_monotonic is None else publication_monotonic
        with self._condition:
            if ticket.completed:
                raise RuntimeError('capture reservation completed twice')
            ticket.completed = True
            self._tickets.pop(id(ticket), None)
            generation = ticket.generation
            if generation is not None:
                self._inflight[generation] -= 1
            if error is not None:
                self._failed += 1
                self._last_error = 'action snapshot failed: '+str(error)
            if snapshot is not None:
                self._history.append((published_at, snapshot))
                self._prune_history(now)
                if generation is not None:
                    self._enqueue(generation, snapshot)
            self._condition.notify_all()

    def _prune_history(self, now):
        # Completion order can differ from publication order across callbacks.
        self._history = deque((item for item in self._history
                               if 0 <= now-item[0] <= self.pre_roll_s), maxlen=1024)

    def _enqueue(self, generation, snapshot):
        if len(self._queue) >= self.max_queue:
            self._dropped += 1
            self._last_error = 'published action outbox full; recording incomplete'
            return
        self._queue.append((generation, snapshot))
        self._pending[generation] = self._pending.get(generation, 0) + 1
        self._captured += 1
        self._condition.notify_all()

    def begin_generation(self):
        """Open after recorder ingress; prior generation must already be drained."""
        selected = bool(self.selected())
        with self._condition:
            if self._closed or self._active or any(self._inflight.values()) or any(self._pending.values()):
                raise RuntimeError('previous publication capture generation not drained')
            self._generation += 1
            self._captured = self._enqueued = self._dropped = self._failed = 0
            self._last_error = ''
            self._active = selected
            if selected:
                # A send may already be between reserve and complete when a
                # recording starts. Include that in-flight real publication in
                # the new generation's stop barrier, rather than losing its
                # first action at the capture boundary.
                for ticket in self._tickets.values():
                    if ticket.generation is None:
                        ticket.generation = self._generation
                        self._inflight[self._generation] = self._inflight.get(self._generation, 0)+1
                self._prune_history(time.monotonic())
                for _, snapshot in sorted(self._history, key=lambda item: item[0]):
                    self._enqueue(self._generation, snapshot)
            return self._generation

    def end_generation(self, timeout=10.):
        """Close admission and drain this generation before recorder ingress closes.

        Idle publications can continue updating history but cannot extend this
        drain. A stalled writer raises on timeout without closing its ingress.
        """
        deadline = time.monotonic()+timeout
        with self._condition:
            self._active = False
            generation = self._generation
            while self._inflight.get(generation, 0) or self._pending.get(generation, 0):
                remaining = deadline-time.monotonic()
                if remaining <= 0:
                    self._last_error = 'published action drain timed out; keep recorder ingress open'
                    raise TimeoutError(self._last_error)
                self._condition.wait(remaining)

    def close_admission(self):
        """Fence new action tickets without waiting for serialization or IPC."""
        with self._condition:
            self._active = False

    def status(self):
        with self._condition:
            return dict(generation=self._generation, active=self._active,
                        captured=self._captured, enqueued=self._enqueued,
                        dropped=self._dropped, failed=self._failed,
                        queued=len(self._queue), inflight=sum(self._inflight.values()),
                        pending_ingress=sum(self._pending.values()),
                        error=self._last_error)

    def close(self):
        self.end_generation()
        with self._condition:
            self._closed = True
            self._condition.notify_all()
        self._thread.join(timeout=2)
        if self._thread.is_alive():
            raise RuntimeError('published action recorder thread did not stop')

    def _run(self):
        while True:
            with self._condition:
                while not self._queue and not self._closed:
                    self._condition.wait()
                if self._closed and not self._queue:
                    return
                generation, snapshot = self._queue.popleft()
            error = None
            try:
                raw = self.serializer(snapshot)
                if not self.record_event(dict(kind='ros_message', topic=self.TOPIC,
                        msg_type=self.TYPE, record_only=True, _raw=raw,
                        stamp_ns=snapshot.publication_ns)):
                    raise RuntimeError('published action rejected by recorder ingress')
            except Exception as exc:
                error = str(exc)
            with self._condition:
                if error is None:
                    self._enqueued += 1
                else:
                    self._failed += 1
                    self._last_error = error
                self._pending[generation] -= 1
                self._condition.notify_all()

"""Low-overhead SingleThreadedExecutor that reuses one WaitSet in steady state."""
from __future__ import annotations

import time
from typing import Any

from rclpy.clock import ClockType
from rclpy.executors import (
    ConditionReachedException,
    ExternalShutdownException,
    InvalidHandle,
    ShutdownException,
    SingleThreadedExecutor,
    TimeoutException,
    TimeoutObject,
    _rclpy,
)
from rclpy.utilities import timeout_sec_to_nsec


class CachedWaitSetExecutor(SingleThreadedExecutor):
    """Reuse an rclpy WaitSet across spins when entity counts stay unchanged."""

    def __init__(self, *, context: Any = None) -> None:
        super().__init__(context=context)
        self._cached_wait_set: Any = None
        self._cached_counts: tuple[int, int, int, int, int, int] | None = None

    def _enter_entities(self, entities: list[Any], entered: list[Any]) -> list[Any]:
        handles = []
        for entity in entities:
            handle = entity.handle
            try:
                handle.__enter__()
            except InvalidHandle:
                continue
            entered.append(handle)
            handles.append(handle)
        return handles

    def _get_wait_set(self, counts: tuple[int, int, int, int, int, int]) -> Any:
        if self._cached_wait_set is None or counts != self._cached_counts:
            self._cached_wait_set = _rclpy.WaitSet(*counts, self._context.handle)
            self._cached_counts = counts
        return self._cached_wait_set

    def _drain_pending_tasks(self, nodes_to_use: list[Any]):
        if hasattr(self, '_ready_tasks'):
            with self._tasks_lock:
                for task in list(self._pending_tasks.keys()):
                    if task.done() or task.cancelled():
                        del self._pending_tasks[task]
                ready_tasks_count = len(self._ready_tasks)
            for _ in range(ready_tasks_count):
                task = self._ready_tasks.popleft()
                task_data = self._pending_tasks[task]
                node = task_data.source_node
                if node is None or node in nodes_to_use:
                    yield task, task_data.source_entity, node
                else:
                    with self._tasks_lock:
                        self._ready_tasks.append(task)
            return
        with self._tasks_lock:
            tasks = list(self._tasks)
        if tasks:
            for task, entity, node in reversed(tasks):
                if (
                    not task.executing()
                    and not task.done()
                    and (node is None or node in nodes_to_use)
                ):
                    yield task, entity, node
            with self._tasks_lock:
                self._tasks = [t for t in self._tasks if not t[0].done() and not t[0].cancelled()]

    def _wait_for_ready_callbacks(
        self,
        timeout_sec: float | TimeoutObject | None = None,
        nodes: list[Any] | None = None,
        condition: Any = lambda: False,
    ):
        clock = getattr(self, '_clock', None)
        if getattr(clock, 'clock_type', None) != ClockType.STEADY_TIME:
            yield from super()._wait_for_ready_callbacks(
                timeout_sec=timeout_sec, nodes=nodes, condition=condition
            )
            return

        timeout_nsec = timeout_sec_to_nsec(
            timeout_sec.timeout if isinstance(timeout_sec, TimeoutObject) else timeout_sec
        )
        deadline_ns = (time.monotonic_ns() + timeout_nsec) if timeout_nsec > 0 else None
        yielded_work = False

        try:
            while not yielded_work and not self._is_shutdown and not condition():
                nodes_to_use = self.get_nodes() if nodes is None else nodes

                for item in self._drain_pending_tasks(nodes_to_use):
                    yielded_work = True
                    yield item

                subs: list[Any] = []
                tmrs: list[Any] = []
                clis: list[Any] = []
                srvs: list[Any] = []
                guards: list[Any] = []
                waitables: list[Any] = []
                can_execute = self.can_execute

                for node in nodes_to_use:
                    for s in node.subscriptions:
                        if can_execute(s):
                            subs.append(s)
                    for t in node.timers:
                        if can_execute(t):
                            tmrs.append(t)
                    for c in node.clients:
                        if can_execute(c):
                            clis.append(c)
                    for s in node.services:
                        if can_execute(s):
                            srvs.append(s)
                    for gc in node.guards:
                        if can_execute(gc):
                            if gc._executor_triggered:
                                try:
                                    gc.trigger()
                                except InvalidHandle:
                                    pass
                            guards.append(gc)
                    for w in node.waitables:
                        if can_execute(w):
                            waitables.append(w)

                guards.append(self._guard)
                guards.append(self._sigint_gc)

                entered: list[Any] = []
                try:
                    try:
                        self._context.handle.__enter__()
                        entered.append(self._context.handle)
                    except InvalidHandle:
                        raise ShutdownException()

                    sub_handles = self._enter_entities(subs, entered) if subs else ()
                    cli_handles = self._enter_entities(clis, entered) if clis else ()
                    srv_handles = self._enter_entities(srvs, entered) if srvs else ()
                    tmr_handles = self._enter_entities(tmrs, entered) if tmrs else ()
                    guard_handles = self._enter_entities(guards, entered)

                    active_waitables = []
                    n_subs = len(sub_handles)
                    n_guards = len(guard_handles)
                    n_tmrs = len(tmr_handles)
                    n_clis = len(cli_handles)
                    n_srvs = len(srv_handles)
                    n_events = 0
                    for wt in waitables:
                        try:
                            wt.__enter__()
                        except InvalidHandle:
                            continue
                        entered.append(wt)
                        active_waitables.append(wt)
                        ne = wt.get_num_entities()
                        n_subs += ne.num_subscriptions
                        n_guards += ne.num_guard_conditions
                        n_tmrs += ne.num_timers
                        n_clis += ne.num_clients
                        n_srvs += ne.num_services
                        n_events += ne.num_events

                    counts = (n_subs, n_guards, n_tmrs, n_clis, n_srvs, n_events)
                    wait_set = self._get_wait_set(counts)
                    wait_set.clear_entities()
                    for h in sub_handles:
                        wait_set.add_subscription(h)
                    for h in cli_handles:
                        wait_set.add_client(h)
                    for h in srv_handles:
                        wait_set.add_service(h)
                    for h in tmr_handles:
                        wait_set.add_timer(h)
                    for h in guard_handles:
                        wait_set.add_guard_condition(h)
                    for wt in active_waitables:
                        wt.add_to_wait_set(wait_set)

                    wait_nsec = (
                        timeout_nsec
                        if deadline_ns is None
                        else max(0, deadline_ns - time.monotonic_ns())
                    )
                    wait_set.wait(wait_nsec)
                    if self._is_shutdown:
                        raise ShutdownException()
                    if not self._context.ok():
                        raise ExternalShutdownException()

                    subs_ready = wait_set.get_ready_entities('subscription') if sub_handles else ()
                    guards_ready = wait_set.get_ready_entities('guard_condition') if guard_handles else ()
                    timers_ready = wait_set.get_ready_entities('timer') if tmr_handles else ()
                    clients_ready = wait_set.get_ready_entities('client') if cli_handles else ()
                    services_ready = wait_set.get_ready_entities('service') if srv_handles else ()

                    if guards_ready:
                        for gc in guards:
                            if gc.handle.pointer in guards_ready:
                                gc._executor_triggered = True

                    if active_waitables:
                        active_set = set(active_waitables)
                        for node in nodes_to_use:
                            for wt in node.waitables:
                                if wt in active_set and wt.is_ready(wait_set):
                                    if wt.callback_group.can_execute(wt):
                                        handler = self._make_handler(
                                            wt, node, lambda e: e.take_data(), self._execute_waitable
                                        )
                                        yielded_work = True
                                        yield handler, wt, node
                except InvalidHandle:
                    self._cached_wait_set = None
                    self._cached_counts = None
                    if self._is_shutdown:
                        raise ShutdownException()
                    if not self._context.ok():
                        raise ExternalShutdownException()
                    continue
                finally:
                    for h in reversed(entered):
                        h.__exit__(None, None, None)

                for node in nodes_to_use:
                    if timers_ready:
                        for tmr in node.timers:
                            if tmr.handle.pointer in timers_ready and tmr.handle.is_timer_ready():
                                if tmr.callback_group.can_execute(tmr):
                                    handler = self._make_handler(
                                        tmr, node, self._take_timer, self._execute_timer
                                    )
                                    yielded_work = True
                                    yield handler, tmr, node

                    if subs_ready:
                        for sub in node.subscriptions:
                            if sub.handle.pointer in subs_ready:
                                if sub.callback_group.can_execute(sub):
                                    handler = self._make_handler(
                                        sub, node, self._take_subscription, self._execute_subscription
                                    )
                                    yielded_work = True
                                    yield handler, sub, node

                    if guards_ready:
                        for gc in node.guards:
                            if gc._executor_triggered:
                                if gc.callback_group.can_execute(gc):
                                    handler = self._make_handler(
                                        gc, node, self._take_guard_condition, self._execute_guard_condition
                                    )
                                    yielded_work = True
                                    yield handler, gc, node

                    if clients_ready:
                        for cli in node.clients:
                            if cli.handle.pointer in clients_ready:
                                if cli.callback_group.can_execute(cli):
                                    handler = self._make_handler(
                                        cli, node, self._take_client, self._execute_client
                                    )
                                    yielded_work = True
                                    yield handler, cli, node

                    if services_ready:
                        for srv in node.services:
                            if srv.handle.pointer in services_ready:
                                if srv.callback_group.can_execute(srv):
                                    handler = self._make_handler(
                                        srv, node, self._take_service, self._execute_service
                                    )
                                    yielded_work = True
                                    yield handler, srv, node

                if not yielded_work and (
                    timeout_nsec == 0
                    or (deadline_ns is not None and time.monotonic_ns() >= deadline_ns)
                ):
                    raise TimeoutException()

            if self._is_shutdown:
                raise ShutdownException()
            if condition():
                raise ConditionReachedException()
        finally:
            if self._is_shutdown:
                self._cached_wait_set = None
                self._cached_counts = None

    def wait_for_ready_callbacks(self, *args: Any, **kwargs: Any):
        # Drain remaining handlers from an already-completed wait pass when the
        # caller switches from a blocking timeout_sec to a non-blocking poll.
        if (
            self._cb_iter is not None
            and not args
            and kwargs.get('timeout_sec') == 0
            and kwargs.get('nodes') is self._last_kwargs.get('nodes')
            and kwargs.get('condition') is self._last_kwargs.get('condition')
        ):
            try:
                return next(self._cb_iter)
            except StopIteration:
                self._cb_iter = None
        return super().wait_for_ready_callbacks(*args, **kwargs)

    def shutdown(self, timeout_sec: float | None = None) -> bool:
        try:
            return super().shutdown(timeout_sec=timeout_sec)
        finally:
            self._cached_wait_set = None
            self._cached_counts = None


def create_single_threaded_executor(
    executor_cls: Any = None, *, context: Any = None
) -> Any:
    """Return CachedWaitSetExecutor unless a mock SingleThreadedExecutor was injected."""
    if executor_cls is None:
        return CachedWaitSetExecutor(context=context)
    if executor_cls is SingleThreadedExecutor or (
        isinstance(executor_cls, type)
        and issubclass(executor_cls, SingleThreadedExecutor)
        and not issubclass(executor_cls, CachedWaitSetExecutor)
        and executor_cls._wait_for_ready_callbacks is SingleThreadedExecutor._wait_for_ready_callbacks
    ):
        return CachedWaitSetExecutor(context=context)
    return executor_cls(context=context)

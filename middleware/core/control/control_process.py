"""独立子进程控制宿主客户端（CoreProcessClient）与父进程存活探针（_ParentLiveness）。

本模块保留全部对外导出符号（含子进程入口 _control_worker 及父进程代理类），
底层执行端与代理层分别实现于 control_worker.py 与 control_proxies.py。
"""
from __future__ import annotations

import copy
import multiprocessing as mp
import os
import queue
import select
import threading
import time
import uuid

from .control_proxies import (
    _CONTROL_PREFIXES,
    _HOOKS,
    IsolatedRosBridge,
    RemoteOwnership,
    RemotePublicationCapture,
    publish_readonly_observation,
)
from .control_worker import (
    _BRIDGE_METHODS,
    _OWNER_METHODS,
    _POSITIVE_OWNER,
    _READONLY_ROS_TOPICS,
    ControlWorkerHost,
    _RecordingIngress,
    _generation,
    _negative_operation,
    _offer_latest,
    _parse_cpu_list,
    _pin_control_actor_cpu,
    run_control_worker,
)


class _ParentLiveness:
    """复用单个 poller 在每个检查点检测父进程 sentinel 存活状态。"""

    def __init__(self, expected_pid: int) -> None:
        self.expected_pid = expected_pid
        self.parent = mp.parent_process()
        self.poller = None
        if self.parent is not None and hasattr(select, "poll"):
            poller = select.poll()
            poller.register(
                self.parent.sentinel, select.POLLIN | select.POLLHUP | select.POLLERR
            )
            self.poller = poller

    def is_alive(self) -> bool:
        if os.getppid() != self.expected_pid:
            return False
        if self.poller is not None:
            try:
                # sentinel 就绪（含 HUP/ERR/NVAL）均表示父进程不再是已验证的活跃宿主。
                return not self.poller.poll(0)
            except (OSError, ValueError):
                return False
        return self.parent is None or self.parent.is_alive()


def _control_worker(config, channels, capture_spec, bridge_factory=None):
    """子进程入口（可被 spawn 序列化），透传本模块 _ParentLiveness 以便单测替换。"""
    return run_control_worker(
        config,
        channels,
        capture_spec,
        bridge_factory=bridge_factory,
        parent_liveness_cls=_ParentLiveness,
    )


class CoreProcessClient:
    """维护控制子进程的缓存观测快照及有界同步管理请求。"""

    def __init__(
        self,
        config,
        *,
        on_event=lambda *a: None,
        hooks=None,
        fallback_stop=lambda *a: None,
        bridge_factory=None,
        on_observer_ros=None,
    ):
        self.config = config
        self.context = mp.get_context("spawn")
        self.boot = uuid.uuid4().hex
        self.channels = {
            name: self.context.Queue(maxsize=size)
            for name, size in [
                ("requests", 64),
                ("urgent", 16),
                ("replay", 256),
                ("replies", 64),
                ("status", 2),
                ("observers", 256),
                ("observer_ros", 256),
                ("hooks", 2048),
                ("hook_replies", 8),
            ]
        }
        self.channels.update(
            boot=self.boot,
            parent_pid=os.getpid(),
            parent_generation=self.context.Value("Q", 0),
            core_epoch=self.context.Value("q", -1),
            ownership_active=self.context.Value("b", False),
            capture_ready=self.context.Value("b", False),
            capture_ready_at=self.context.Value("d", 0.0),
            failure_stop=self.context.Event(),
            observer_ros_enabled=on_observer_ros is not None,
        )
        self.on_event, self.hooks, self.fallback_stop = (
            on_event,
            hooks or {},
            fallback_stop,
        )
        self.bridge_factory = bridge_factory
        self.on_observer_ros = on_observer_ros
        self._observer_ros_errors = 0
        self._observer_ros_last_error = ""
        self.capture_spec = None
        self.process = None
        self._cache = dict(
            state="starting",
            ownership=dict(state="OFF"),
            command_mux=dict(output_enabled=False),
        )
        self._lock, self._pending_lock = threading.RLock(), threading.Lock()
        self._pending, self._request_seq = {}, 0
        self._failure = ""
        self._fallback_completed = threading.Event()
        self._fallback_error = ""
        self._failure_signal_thread = None
        self._closing = False
        self._stopped = threading.Event()
        self._threads = []
        self._hook_condition = threading.Condition()
        self._last_hook_done = 0
        self._hook_context = threading.local()
        self._last_snapshot_received = time.monotonic()

    def snapshot(self):
        with self._lock:
            result = copy.deepcopy(self._cache)
            failure = self._failure
            result.setdefault("control_process", {}).update(
                observer_ros_errors=self._observer_ros_errors,
                observer_ros_last_error=self._observer_ros_last_error,
                fallback_stop_completed=self._fallback_completed.is_set(),
                fallback_stop_error=self._fallback_error,
            )
        if failure:
            result.update(state="error", error=failure)
            result.setdefault("command_mux", {})["output_enabled"] = False
        result["isolated_control_process"] = True
        return result

    def authorization_fence(self):
        if self._failure:
            return self.boot, -1, -1
        try:
            result = (
                self.boot,
                int(self._shared_value("parent_generation")),
                int(self._shared_value("core_epoch")),
            )
        except RuntimeError:
            return self.boot, -1, -1
        return (self.boot, -1, -1) if self._failure else result

    def _shared_value(self, name, value=None, *, write=False):
        """超时保护共享值访问，防止被强杀的子进程在孤儿锁上拖死父进程。"""
        if self._failure:
            raise RuntimeError(self._failure)
        shared = self.channels[name]
        lock = shared.get_lock()
        if not lock.acquire(timeout=0.02):
            self._failed(
                "control shared " + name + " lock timed out; explicit restart required"
            )
            raise RuntimeError(self._failure)
        try:
            if self._failure:
                raise RuntimeError(self._failure)
            if write:
                shared.get_obj().value = value
            return shared.get_obj().value
        finally:
            lock.release()

    def revoke_intent(self):
        if self._failure:
            return -1
        shared = self.channels["parent_generation"]
        lock = shared.get_lock()
        if not lock.acquire(timeout=0.02):
            self._failed(
                "control shared parent_generation lock timed out; explicit restart required"
            )
            return -1
        try:
            if self._failure:
                return -1
            shared.get_obj().value += 1
            generation = int(shared.get_obj().value)
        finally:
            lock.release()
        with self._lock:
            self._cache.setdefault("command_mux", {})["output_enabled"] = False
        return generation

    def _failed(self, reason):
        with self._lock:
            first = not self._failure
            self._failure = self._failure or reason
        if first:
            try:
                self.fallback_stop(reason)
            except Exception as exc:
                with self._lock:
                    self._fallback_error = str(exc)
            finally:
                self._fallback_completed.set()

            def signal_owner():
                self.channels["failure_stop"].set()
                shared = self.channels["parent_generation"]
                with shared.get_lock():
                    shared.get_obj().value += 1

            self._failure_signal_thread = threading.Thread(
                target=signal_owner, name="control-failure-signal", daemon=True
            )
            self._failure_signal_thread.start()

    def _cache_snapshot(self, value):
        if value:
            with self._lock:
                self._last_snapshot_received = time.monotonic()
                if value.get("snapshot_monotonic_ns", 0) >= self._cache.get(
                    "snapshot_monotonic_ns", 0
                ):
                    self._cache = value

    def start(self, timeout=15.0):
        if self.process is not None:
            return
        self.process = self.context.Process(
            target=_control_worker,
            args=(
                copy.deepcopy(self.config),
                self.channels,
                self.capture_spec,
                self.bridge_factory,
            ),
            name="teleop-control-owner",
            daemon=True,
        )
        self.process.start()
        for name, target in [
            ("control-ipc-reader", self._receive),
            ("control-record-hooks", self._record_hooks),
            ("control-observers", self._observers),
            ("control-observer-ros", self._observer_ros),
            ("control-capture-ready", self._capture_readiness),
            ("control-watchdog", self._watchdog),
        ]:
            thread = threading.Thread(target=target, name=name, daemon=True)
            self._threads.append(thread)
            thread.start()
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            status = self.snapshot()
            if status["state"] == "running":
                return
            if self._failure or status["state"] == "error":
                raise RuntimeError(status.get("error") or self._failure)
            time.sleep(0.01)
        self._failed("control process startup timed out")
        raise RuntimeError(self._failure)

    def call(
        self,
        op,
        *args,
        expected_fence=None,
        urgent=False,
        allow_failed=False,
        timeout=1.0,
        negative_generation=None,
        **kwargs,
    ):
        if self.process is None or not self.process.is_alive():
            self._failed("control process unavailable; explicit restart required")
            raise RuntimeError(self._failure)
        if self._failure:
            raise RuntimeError(self._failure)
        with self._pending_lock:
            self._request_seq += 1
            identifier = self._request_seq
            event = threading.Event()
            self._pending[identifier] = [event, None]
        request = dict(
            id=identifier,
            op=op,
            args=args,
            kwargs=kwargs,
            fence=(
                tuple(expected_fence)
                if expected_fence is not None
                else self.authorization_fence()
            ),
        )
        if _negative_operation(op, args):
            request["negative_generation"] = (
                request["fence"][1]
                if negative_generation is None
                else negative_generation
            )
        try:
            try:
                self.channels["urgent" if urgent else "requests"].put_nowait(request)
            except queue.Full:
                self._failed("control administrative queue full")
                raise RuntimeError(self._failure)
            if not event.wait(timeout):
                self._failed("control request timed out; authorization fenced")
                raise RuntimeError(self._failure)
            with self._pending_lock:
                reply = self._pending[identifier][1]
            if reply.get("error"):
                error = (
                    ValueError
                    if reply.get("error_type") in {"ValueError", "OwnershipConflict"}
                    else RuntimeError
                )
                raise error(reply["error"])
            return reply.get("result")
        finally:
            with self._pending_lock:
                self._pending.pop(identifier, None)

    def notify(self, op, *args, urgent=True, negative_generation=None, **kwargs):
        """负向输入边沿绝不在 VR 接收/超时线程上阻塞等待。"""
        if self.process is None or not self.process.is_alive() or self._failure:
            self._failed("control process unavailable; explicit restart required")
            return False
        request = dict(
            id=0, op=op, args=args, kwargs=kwargs, fence=self.authorization_fence()
        )
        if _negative_operation(op, args):
            request["negative_generation"] = (
                request["fence"][1]
                if negative_generation is None
                else negative_generation
            )
        try:
            self.channels["urgent" if urgent else "requests"].put_nowait(request)
            return True
        except queue.Full:
            self._failed("control administrative queue full")
            return False

    def submit_replay_raw(self, topic, msg_type, raw_data, *, expected_fence=None):
        """非阻塞投递带栅栏的回放数据包。"""
        if self.process is None or not self.process.is_alive() or self._failure:
            self._failed("control process unavailable; explicit restart required")
            return False
        fence = (
            tuple(expected_fence)
            if expected_fence is not None
            else self.authorization_fence()
        )
        try:
            self.channels["replay"].put_nowait(
                dict(topic=topic, msg_type=msg_type, raw_data=raw_data, fence=fence)
            )
            return True
        except queue.Full:
            return False

    def _receive(self):
        while not self._stopped.is_set():
            try:
                reply = self.channels["replies"].get(timeout=0.01)
                self._cache_snapshot(reply.get("snapshot"))
                with self._lock:
                    if "ownership" in reply:
                        self._cache["ownership"] = reply["ownership"]
                    if "authorization_fence" in reply:
                        self._cache["authorization_fence"] = reply[
                            "authorization_fence"
                        ]
                    self._cache["snapshot_monotonic_ns"] = reply.get(
                        "snapshot_monotonic_ns",
                        self._cache.get("snapshot_monotonic_ns", 0),
                    )
                with self._pending_lock:
                    pending = self._pending.get(reply["id"])
                    if pending is not None:
                        pending[1] = reply
                        pending[0].set()
            except queue.Empty:
                pass
            while True:
                try:
                    self._cache_snapshot(self.channels["status"].get_nowait())
                except queue.Empty:
                    break
            if (
                not self._closing
                and self.process is not None
                and not self.process.is_alive()
            ):
                self._failed("control process exited; explicit restart required")
            if (
                not self._closing
                and self._cache.get("state") == "running"
                and time.monotonic() - self._last_snapshot_received > 1.0
            ):
                self._failed(
                    "control process heartbeat timed out; explicit restart required"
                )

    def _capture_readiness(self):
        while not self._stopped.is_set():
            callback = self.hooks.get("capture_ready")
            if callback is not None:
                try:
                    ready = bool(callback())
                except Exception:
                    ready = False
                try:
                    self._shared_value("capture_ready", ready, write=True)
                    self._shared_value("capture_ready_at", time.monotonic(), write=True)
                except RuntimeError:
                    return
            self._stopped.wait(0.05)

    def _watchdog(self):
        while not self._stopped.wait(0.05):
            if self._closing:
                continue
            if self.process is not None and not self.process.is_alive():
                self._failed("control process exited; explicit restart required")
            elif (
                self._cache.get("state") == "running"
                and time.monotonic() - self._last_snapshot_received > 1.0
            ):
                self._failed(
                    "control process heartbeat timed out; explicit restart required"
                )

    def _observers(self):
        while not self._stopped.is_set():
            try:
                event, outputs = self.channels["observers"].get(timeout=0.05)
                self.on_event(event, outputs)
            except queue.Empty:
                pass
            except Exception:
                pass

    def _observer_ros(self):
        while not self._stopped.is_set():
            try:
                topic, payload = self.channels["observer_ros"].get(timeout=0.05)
                if topic not in _READONLY_ROS_TOPICS or not isinstance(payload, dict):
                    raise ValueError("unsupported read-only control observation")
                if self.on_observer_ros is not None:
                    if self.on_observer_ros(topic, payload) is False:
                        raise RuntimeError(
                            "read-only ROS observer rejected by telemetry queue"
                        )
            except queue.Empty:
                continue
            except Exception as exc:
                with self._lock:
                    self._observer_ros_errors += 1
                    self._observer_ros_last_error = str(exc)

    def _record_hooks(self):
        opened = {}
        while not self._stopped.is_set():
            try:
                item = self.channels["hooks"].get(timeout=0.05)
            except queue.Empty:
                continue
            sequence, kind, name = item["sequence"], item["kind"], item["name"]
            self._hook_context.clock_ns = item["clock_ns"]
            reply = dict(sequence=sequence)
            try:
                if kind in {"hook", "hook_request"}:
                    callback = self.hooks.get(name)
                    if callback is None and kind == "hook_request":
                        raise RuntimeError("recording hook unavailable: " + name)
                    if (
                        kind == "hook_request"
                        and name == "open_segment"
                        and tuple(item["fence"]) != self.authorization_fence()
                    ):
                        raise RuntimeError(
                            "recording hook belongs to revoked authorization"
                        )
                    result = callback(*item["args"]) if callback else None
                    reply["result"] = result
                    if name == "open_segment":
                        opened[sequence] = item
                elif kind == "hook_ack":
                    opened.pop(item["request_sequence"], None)
                elif kind == "hook_cancel":
                    original = opened.pop(item["request_sequence"], None)
                    if original is not None:
                        callback = self.hooks.get("close_segment")
                        if callback:
                            callback(
                                original["args"][0],
                                item["clock_ns"],
                                0,
                                "control revoked or recording preparation timed out",
                            )
            except Exception as exc:
                reply["error"] = str(exc)
            finally:
                self._hook_context.clock_ns = None
                if kind == "hook_request":
                    try:
                        self.channels["hook_replies"].put_nowait(reply)
                    except queue.Full:
                        self._failed(
                            "recording hook acknowledgements could not be delivered"
                        )
                with self._hook_condition:
                    self._last_hook_done = max(self._last_hook_done, sequence)
                    self._hook_condition.notify_all()

    def flush_recording_events(self, timeout=2.0):
        sequence = self.call("record_barrier", timeout=timeout, allow_failed=True)
        deadline = time.monotonic() + timeout
        with self._hook_condition:
            while self._last_hook_done < sequence:
                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    raise RuntimeError(
                        "recording event drain timed out; capture incomplete"
                    )
                self._hook_condition.wait(remaining)

    def stop(self):
        if self.process is None:
            return
        self._closing = True
        try:
            if self.process.is_alive():
                if not self._failure:
                    try:
                        self.call("shutdown", urgent=True)
                    except RuntimeError as exc:
                        self._failed(str(exc))
                self.process.join(3.0)
            if self.process.is_alive():
                self.process.terminate()
                self.process.join(2.0)
        finally:
            self._stopped.set()
            for value in self.channels.values():
                if hasattr(value, "cancel_join_thread"):
                    value.cancel_join_thread()
            for thread in self._threads:
                thread.join(1.0)

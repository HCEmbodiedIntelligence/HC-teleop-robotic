"""隔离控制子进程权威执行端（ControlWorkerHost）、CPU 绑核与队列辅助工具。"""
from __future__ import annotations

import copy
import os
import queue
import select
import sys
import threading
import time
from typing import Any

_BRIDGE_METHODS = {
    "publish",
    "publish_raw",
    "request_manual_home",
    "request_replay_home",
    "begin_replay",
    "end_replay",
    "emergency_stop",
    "set_control_source",
    "set_command_output_enabled",
    "recover_intervention",
    "control_ready",
    "start_replay_alignment",
    "replay_alignment_status",
    "pause_replay_alignment",
    "resume_replay_alignment",
    "cancel_replay_alignment",
}
_OWNER_METHODS = {
    "enable",
    "disable",
    "start_model",
    "set_dagger",
    "start_auto",
    "takeover",
    "release",
    "hold",
    "fault",
    "recover",
    "input_lost",
    "finish_capture_preparation",
}
_POSITIVE_OWNER = {
    "enable",
    "start_model",
    "start_auto",
    "takeover",
    "recover",
    "set_dagger",
}
_READONLY_ROS_TOPICS = frozenset(
    {
        "/status/control/ownership",
        "/events/control/intervention",
        "/internal/control/action_evidence",
    }
)


def _negative_operation(op: str, args: tuple[Any, ...]) -> bool:
    return (
        op in {"emergency_stop", "end_replay", "shutdown", "owner.disable", "owner.hold", "owner.fault"}
        or (op == "set_command_output_enabled" and bool(args) and not args[0])
    )


def _offer_latest(target: Any, value: Any) -> None:
    """非阻塞投递最新快照：队列满时丢弃最旧元素再写入。"""
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


def _generation(shared: Any) -> int:
    with shared.get_lock():
        return int(shared.value)


def _parse_cpu_list(text: str) -> set[int]:
    cpus: set[int] = set()
    for part in str(text).split(","):
        part = part.strip()
        if not part:
            continue
        if "-" in part:
            start, end = (int(item) for item in part.split("-", 1))
            cpus.update(range(start, end + 1))
        else:
            cpus.add(int(part))
    return cpus


def _pin_control_actor_cpu() -> tuple[set[int] | None, set[int] | None]:
    """可用时将控制 Actor 主循环绑定到隔离核 CPU 3，并让辅助线程远离该核。"""
    if not hasattr(os, "sched_getaffinity") or not hasattr(os, "sched_setaffinity"):
        return None, None
    choice = os.environ.get("HC_CONTROL_CPU", "auto").strip().lower()
    if choice in {"off", "none", "false", ""}:
        return None, None
    try:
        orig = os.sched_getaffinity(0)
        with open("/sys/devices/system/cpu/online", encoding="utf-8") as f:
            online = _parse_cpu_list(f.read())
        with open("/sys/devices/system/cpu/isolated", encoding="utf-8") as f:
            isolated = _parse_cpu_list(f.read())
    except OSError:
        return None, None
    if choice == "auto":
        if 3 not in isolated or 3 not in online:
            return None, None
        target = 3
    else:
        try:
            target = int(choice)
        except ValueError:
            return None, None
        if target not in online:
            return None, None
    if target not in orig and target not in isolated:
        return None, None
    non_isolated = (online - isolated) & orig
    if not non_isolated:
        non_isolated = orig - {target}
    if target not in orig:
        non_isolated = non_isolated | orig
    try:
        os.sched_setaffinity(0, {target})
    except OSError:
        return None, None
    return orig, (non_isolated or None)


class _RecordingIngress:
    def __init__(self, resources: Any, finish: Any):
        from ..recording.topic_recorder import enqueue_record

        self.resources, self.finish = resources, finish
        self._enqueue_record = enqueue_record

    def record(self, event: dict[str, Any]) -> Any:
        return self.record_publication(event)

    def record_publication(self, event: dict[str, Any]) -> Any:
        return self._enqueue_record(
            self.resources, event, allow_closed=True, finished=self.finish
        )


class ControlWorkerHost:
    """运行在隔离控制子进程中的权威宿主，封装 RPC 分发、栅栏校验与 IPC 唤醒。"""

    def __init__(
        self,
        config: dict[str, Any],
        channels: dict[str, Any],
        capture_spec: dict[str, Any] | None,
        *,
        bridge_factory: Any = None,
        parent_liveness_cls: Any,
    ) -> None:
        from .ros_bridge import RosBridge

        self.config = config
        self.channels = channels
        self.counters = {
            "observer_dropped": 0,
            "observer_ros_dropped": 0,
            "observer_ros_rejected": 0,
            "recording_events_dropped": 0,
            "rpc_replies_dropped": 0,
            "stale_queued_commands": 0,
        }
        self.hook_sequence = 0
        self.pumping = False
        self.hard_stopped = False
        self.ipc_drained = threading.Event()
        self.ipc_stop_r: int | None = None
        self.ipc_stop_w: int | None = None
        self.ipc_wake_thread: threading.Thread | None = None
        self.parent_liveness = parent_liveness_cls(channels["parent_pid"])
        self.seen_parent = _generation(channels["parent_generation"])
        self.applied_parent = self.seen_parent
        self.last_snapshot = 0.0
        self.last_ownership_active: bool | None = None
        self.child_selected = True
        self.non_isolated_cpus: set[int] | None = None

        core_config = copy.deepcopy(config)
        core_config.update(_control_actor_mode=True, subscriptions=[])
        core_config["node_name"] = (
            config.get("node_name", "hc_teleop_middleware") + "_control_owner"
        )
        core_config["dataset_topics"] = {"enabled": False}

        factory = bridge_factory or RosBridge
        self.bridge = factory(core_config, self.emit)
        self.owner = self.bridge.ownership
        self._wire_bridge(capture_spec)

    def _wire_bridge(self, capture_spec: dict[str, Any] | None) -> None:
        if self.channels["observer_ros_enabled"]:
            self.bridge._observer_ros_publish = self.observer_ros_publish
        self._original_revoke = self.bridge._revoke_locked
        self.bridge._revoke_locked = self.revoke
        self.bridge._command_output_enabled = False
        self.bridge._revoke_locked()

        self.owner.manual_changed = lambda *args: self.async_hook("manual_changed", *args)
        self.owner.dagger_changed = lambda *args: self.async_hook("dagger_changed", *args)
        self.owner.open_segment = lambda *args: self.blocking_hook("open_segment", *args)
        self.owner.close_segment = lambda *args: self.blocking_hook("close_segment", *args)
        self.owner.capture_ready = lambda: bool(
            self.channels["capture_ready"].value
            and 0 <= time.monotonic() - self.channels["capture_ready_at"].value <= 0.25
        )
        self.bridge._action_sink = lambda evidence: self.async_hook("action", evidence)
        if capture_spec:
            from ..recording.publication_capture import PublicationCapture

            self.bridge._publication_capture = PublicationCapture(
                _RecordingIngress(capture_spec["resources"], capture_spec["finish"]),
                selected=lambda: self.child_selected,
            )

        self.bridge._actor_pump = self.pump
        self.bridge._actor_pump_checkpoint = lambda allow_snapshot=False: self.pump(
            allow_snapshot=allow_snapshot
        )
        self.bridge._actor_urgent_check = self.urgent_check
        self.bridge._actor_authority_blocked = self.authority_blocked
        self.bridge._actor_ready = self.start_ipc_wake
        self.bridge._actor_command_fence = lambda: (
            _generation(self.channels["parent_generation"]),
            self.bridge._output_epoch,
        )
        self.bridge._actor_command_allowed = self._allowed_command

    def emit(self, event: dict[str, Any], outputs: list[str]) -> None:
        try:
            self.channels["observers"].put_nowait((event, outputs))
        except queue.Full:
            self.counters["observer_dropped"] += 1

    def observer_ros_publish(self, topic: str, payload: Any) -> bool:
        if topic not in _READONLY_ROS_TOPICS or not isinstance(payload, dict):
            self.counters["observer_ros_rejected"] += 1
            return False
        try:
            self.channels["observer_ros"].put_nowait((topic, payload))
            return True
        except queue.Full:
            self.counters["observer_ros_dropped"] += 1
            return False

    def revoke(self, *, keep_replay: bool = False) -> None:
        self._original_revoke(keep_replay=keep_replay)
        with self.channels["core_epoch"].get_lock():
            self.channels["core_epoch"].value = self.bridge._output_epoch

    def fence(self) -> tuple[Any, int, int]:
        return (
            self.channels["boot"],
            _generation(self.channels["parent_generation"]),
            self.bridge._output_epoch,
        )

    def authority_blocked(self) -> bool:
        return (
            self.channels["failure_stop"].is_set()
            or _generation(self.channels["parent_generation"]) > self.applied_parent
        )

    def verify(self, expected: Any) -> None:
        if expected is None or tuple(expected) != self.fence():
            raise ValueError(
                "authorization revoked while request was waiting; explicit fresh authorization required"
            )
        if self.channels["failure_stop"].is_set():
            raise RuntimeError(
                "control host failed; restart and explicit authorization required"
            )
        if self.authority_blocked():
            raise ValueError(
                "negative safety request is pending; fresh authorization required after stop completes"
            )

    def record_event(self, kind: str, name: str = "", args: tuple[Any, ...] = (), **fields: Any) -> dict[str, Any]:
        self.hook_sequence += 1
        event = dict(
            sequence=self.hook_sequence,
            kind=kind,
            name=name,
            args=args,
            clock_ns=self.owner.ros_clock(),
            fence=self.fence(),
            **fields,
        )
        try:
            self.channels["hooks"].put_nowait(event)
            return event
        except queue.Full:
            self.counters["recording_events_dropped"] += 1
            self.owner.recording_error = (
                "control recording observer queue full; capture incomplete"
            )
            raise RuntimeError(self.owner.recording_error)

    def async_hook(self, name: str, *args: Any) -> None:
        try:
            self.record_event("hook", name, args)
        except RuntimeError:
            pass  # 显式标记证据不完整，绝不阻塞活跃发送线程。

    def blocking_hook(self, name: str, *args: Any) -> Any:
        if self.bridge._command_output_enabled:
            raise RuntimeError("recording preparation requires closed control output")
        request = self.record_event("hook_request", name, args)
        deadline = time.monotonic() + 0.5
        try:
            while time.monotonic() < deadline:
                if (
                    self.channels["failure_stop"].is_set()
                    or _generation(self.channels["parent_generation"]) != request["fence"][1]
                ):
                    raise RuntimeError("recording preparation revoked")
                try:
                    reply = self.channels["hook_replies"].get(timeout=0.005)
                except queue.Empty:
                    continue
                if reply.get("sequence") != request["sequence"]:
                    continue
                if reply.get("error"):
                    raise RuntimeError(reply["error"])
                self.record_event("hook_ack", request_sequence=request["sequence"])
                return reply.get("result")
            raise RuntimeError("recording preparation acknowledgement timed out")
        except Exception:
            try:
                self.record_event("hook_cancel", request_sequence=request["sequence"])
            except RuntimeError:
                pass
            raise

    def snapshot(self) -> dict[str, Any]:
        result = self.bridge.status()
        result["control_process"] = dict(
            pid=os.getpid(),
            boot_id=self.channels["boot"],
            actor=True,
            parent_pid=self.channels["parent_pid"],
            python_switch_interval_ms=sys.getswitchinterval() * 1000,
            applied_negative_generation=self.applied_parent,
            pending_negative_generation=(
                _generation(self.channels["parent_generation"])
                if self.authority_blocked()
                else None
            ),
            **self.counters,
        )
        result["control_ready"] = self.bridge.control_ready()
        capture = getattr(self.bridge, "_publication_capture", None)
        result["publication_capture"] = capture.status() if capture else None
        result["authorization_fence"] = self.fence()
        result["snapshot_monotonic_ns"] = time.monotonic_ns()
        return result

    def update_snapshot(self, force: bool = False) -> None:
        active_now = bool(self.owner.active)
        if active_now != self.last_ownership_active:
            self.channels["ownership_active"].value = active_now
            self.last_ownership_active = active_now
        if force or time.monotonic() - self.last_snapshot >= 0.15:
            _offer_latest(self.channels["status"], self.snapshot())
            self.last_snapshot = time.monotonic()

    def guarded(self, operation: Any, expected: Any) -> Any:
        # 录制准备可能等待父进程钩子，不可跨越该等待持有父进程紧急锁；执行后再复核共享栅栏。
        self.verify(expected)
        try:
            return operation()
        finally:
            with self.channels["parent_generation"].get_lock():
                changed = expected[1] != self.channels["parent_generation"].value
            if changed or self.channels["failure_stop"].is_set():
                self.bridge._command_output_enabled = False
                self.bridge._revoke_locked()
                raise ValueError(
                    "authorization revoked during request; explicit fresh authorization required"
                )

    def intervention(
        self,
        name: str,
        data: dict[str, Any],
        recording_ready: bool,
        preparing_capture: bool,
        expected: Any,
    ) -> dict[str, Any]:
        operations = {
            "enable": lambda: self.owner.enable(preparing_capture=preparing_capture),
            "start_model": lambda: self.owner.start_model(
                control_ready=self.bridge.control_ready(),
                recording_ready=recording_ready,
            ),
            "enable_dagger": lambda: self.owner.set_dagger(
                True, preparing_capture=preparing_capture
            ),
            "disable_dagger": lambda: self.owner.set_dagger(False),
            "takeover": self.owner.takeover,
            "hold": self.owner.hold,
            "disable": self.owner.disable,
            "recover": self.bridge.recover_intervention,
            "return": lambda: self.owner.start_auto(
                recording_ready=recording_ready,
                control_ready=self.bridge.control_ready(),
            ),
        }
        if name not in operations:
            raise ValueError("unknown intervention operation")
        applied = [False]

        def apply() -> None:
            if name in {"enable", "enable_dagger"} and not self.owner.required_peers <= self.owner.peers.keys():
                raise ValueError(
                    "DAgger requires control participants: "
                    + ", ".join(sorted(self.owner.required_peers))
                )
            if name in {"enable", "start_model", "enable_dagger", "takeover", "recover", "return"}:
                self.guarded(operations[name], expected)
            else:
                operations[name]()
            applied[0] = True

        result = self.owner.request(
            data.get("request_id"),
            data.get("session_id", ""),
            data.get("expected_epoch"),
            apply,
        )
        return dict(ownership=result, applied=applied[0], request_id=data.get("request_id"))

    def _dispatch_capture(self, op: str, kwargs: dict[str, Any]) -> Any:
        if op == "capture_begin":
            self.child_selected = bool(kwargs.pop("selected", True))
            return self.bridge._publication_capture.begin_generation(**kwargs)
        if op == "capture_fence":
            return self.bridge._publication_capture.close_admission()
        if op == "capture_status":
            return self.bridge._publication_capture.status()
        if self.bridge._command_output_enabled:
            raise RuntimeError("publication capture shutdown requires closed control output")
        state = self.bridge._publication_capture.status()
        if state["inflight"] or state["pending_ingress"]:
            raise RuntimeError("publication capture must be drained before close")
        self.bridge._publication_capture.close()
        return None

    def dispatch(self, request: dict[str, Any]) -> Any:
        op, args, kwargs = request["op"], request.get("args", ()), request.get("kwargs", {})
        expected = request.get("fence")
        if op == "clock":
            return self.owner.ros_clock()
        if op == "record_barrier":
            return self.record_event("barrier")["sequence"]
        if op in {"capture_begin", "capture_fence", "capture_status", "capture_close"}:
            return self._dispatch_capture(op, kwargs)
        if op == "shutdown":
            self.bridge._command_output_enabled = False
            self.bridge._revoke_locked()
            self.update_snapshot(True)
            self.bridge._stop.set()
            return None
        if op == "resume_output":
            def resume() -> dict[str, Any]:
                self.bridge.set_command_output_enabled(True)
                self.bridge.publish(args[0], "std_msgs/msg/Bool", {"data": False})
                self.bridge.publish(
                    "/control/teleop/enabled",
                    "std_msgs/msg/Bool",
                    {"data": self.bridge._command_source == "vr"},
                )
                flush = getattr(self.bridge, "_actor_flush_commands", None)
                if flush is not None:
                    flush()
                return dict(authorization_fence=self.fence(), ownership=self.owner.snapshot())

            return self.guarded(resume, expected)
        if op == "apply_intervention":
            name, data = args
            return self.intervention(name, data, expected=expected, **kwargs)
        if op.startswith("owner."):
            method = op[6:]
            if method not in _OWNER_METHODS:
                raise ValueError("unsupported ownership operation")
            if method in {"start_model", "start_auto", "recover"}:
                kwargs["control_ready"] = self.bridge.control_ready()
            if method in _POSITIVE_OWNER and not (method == "set_dagger" and not args[0]):
                return self.guarded(lambda: getattr(self.owner, method)(*args, **kwargs), expected)
            return getattr(self.owner, method)(*args, **kwargs)
        if op not in _BRIDGE_METHODS:
            raise ValueError("unsupported control operation")
        positive = (
            op
            in {
                "begin_replay",
                "request_replay_home",
                "recover_intervention",
                "set_control_source",
                "start_replay_alignment",
                "resume_replay_alignment",
            }
            or (op == "set_command_output_enabled" and args[0])
        )
        data_request = op in {"publish", "publish_raw"}
        if positive or data_request:
            return self.guarded(lambda: getattr(self.bridge, op)(*args, **kwargs), expected)
        return getattr(self.bridge, op)(*args, **kwargs)

    def urgent_check(self) -> bool:
        if not self.parent_liveness.is_alive():
            self.channels["failure_stop"].set()
        if self.channels["failure_stop"].is_set():
            if not self.hard_stopped:
                self.hard_stopped = True  # 负向冲刷可能触发递归 checkpoint。
                self.bridge.emergency_stop(
                    self.config.get("stop_topic", "/control/emergency_stop"),
                    "control process host or IPC failed; manual restart required",
                )
                flush = getattr(self.bridge, "_actor_flush_commands", None)
                if flush is not None:
                    flush()
                self.bridge._stop.set()
                self.update_snapshot(True)
            return False
        parent_gen = _generation(self.channels["parent_generation"])
        if parent_gen != self.seen_parent or (
            parent_gen > self.applied_parent and self.bridge._command_output_enabled
        ):
            self.bridge._command_output_enabled = False
            # 保留 replay 标志以便正常结束回放后允许回零；安全停机/切换源时在 dispatch 清零。
            self.bridge._revoke_locked(keep_replay=True)
            self.seen_parent = parent_gen
        return not self.bridge._stop.is_set()

    def _drain_replay_channel(self) -> None:
        replay_channel = self.channels.get("replay")
        if replay_channel is None:
            return
        replay_reader = getattr(replay_channel, "_reader", None)
        replay_ready = True
        if replay_reader is not None:
            try:
                replay_ready = bool(replay_reader.poll(0))
            except (OSError, ValueError):
                replay_ready = True
        if not replay_ready:
            return
        replay_deadline = time.monotonic() + 0.002
        for _ in range(32):
            if time.monotonic() >= replay_deadline:
                break
            try:
                replay_item = replay_channel.get_nowait()
            except queue.Empty:
                break
            expected = replay_item.get("fence")
            if expected is None or tuple(expected) != self.fence() or self.authority_blocked():
                self.counters["stale_queued_commands"] += 1
                continue
            try:
                self.bridge.publish_raw(
                    replay_item["topic"], replay_item["msg_type"], replay_item["raw_data"]
                )
            except Exception:
                pass

    def pump(self, *, allow_snapshot: bool = True) -> None:
        if self.pumping:
            return
        self.pumping = True
        try:
            if not self.urgent_check():
                return
            deadline = time.monotonic() + 0.001
            for source in ("urgent", "requests"):
                reader = getattr(self.channels[source], "_reader", None)
                if reader is not None:
                    try:
                        if not reader.poll(0):
                            continue
                    except (OSError, ValueError):
                        pass
                for _ in range(4 if source == "urgent" else 2):
                    if time.monotonic() >= deadline:
                        break
                    try:
                        request = self.channels[source].get_nowait()
                    except queue.Empty:
                        break
                    try:
                        value = self.dispatch(request)
                        if _negative_operation(request["op"], request.get("args", ())):
                            generation = request.get("negative_generation")
                            if (
                                type(generation) is int
                                and 0 <= generation <= request["fence"][1]
                                and request["fence"][0] == self.channels["boot"]
                            ):
                                self.applied_parent = max(self.applied_parent, generation)
                        response = dict(id=request["id"], result=value)
                    except Exception as exc:
                        response = dict(
                            id=request["id"], error=str(exc), error_type=type(exc).__name__
                        )
                    if not request["id"]:
                        continue
                    response["ownership"] = self.owner.snapshot()
                    response["authorization_fence"] = self.fence()
                    response["snapshot_monotonic_ns"] = time.monotonic_ns()
                    try:
                        self.channels["replies"].put_nowait(response)
                    except queue.Full:
                        self.counters["rpc_replies_dropped"] += 1
                        self.channels["failure_stop"].set()
            if allow_snapshot:
                self.update_snapshot()
            self._drain_replay_channel()
        finally:
            self.pumping = False
            self.ipc_drained.set()

    def start_ipc_wake(self) -> None:
        self.update_snapshot(True)
        wake = getattr(self.bridge, "_actor_wake", None)
        if self.ipc_wake_thread is not None or not callable(wake) or not hasattr(select, "poll"):
            return
        poller = select.poll()
        queue_fds: set[int] = set()
        for source in ("urgent", "requests", "replay"):
            chan = self.channels.get(source)
            reader = getattr(chan, "_reader", None) if chan is not None else None
            fd = reader.fileno() if reader is not None and hasattr(reader, "fileno") else None
            if isinstance(fd, int):
                poller.register(fd, select.POLLIN)
                queue_fds.add(fd)
        sentinel = getattr(self.parent_liveness.parent, "sentinel", None)
        sentinel_fd = sentinel if isinstance(sentinel, int) else None
        if sentinel_fd is not None:
            poller.register(sentinel_fd, select.POLLIN | select.POLLHUP | select.POLLERR)
        if not queue_fds and sentinel_fd is None:
            return
        self.ipc_stop_r, self.ipc_stop_w = os.pipe()
        poller.register(self.ipc_stop_r, select.POLLIN)

        def wake_loop() -> None:
            nonlocal sentinel_fd
            if self.non_isolated_cpus is not None and hasattr(os, "sched_setaffinity"):
                try:
                    os.sched_setaffinity(0, self.non_isolated_cpus)
                except OSError:
                    pass
            while True:
                try:
                    events = poller.poll(500)
                except (OSError, ValueError):
                    break
                if not events:
                    continue
                if any(fd == self.ipc_stop_r for fd, _ in events):
                    break
                for fd, mask in list(events):
                    if fd == sentinel_fd or (mask & select.POLLNVAL):
                        try:
                            poller.unregister(fd)
                        except (OSError, KeyError, ValueError):
                            pass
                        queue_fds.discard(fd)
                        if fd == sentinel_fd:
                            sentinel_fd = None
                if any(fd in queue_fds for fd, _ in events):
                    self.ipc_drained.clear()
                    wake()
                    self.ipc_drained.wait(0.005)
                else:
                    wake()

        self.ipc_wake_thread = threading.Thread(
            target=wake_loop, name="control-ipc-wake", daemon=True
        )
        self.ipc_wake_thread.start()

    def _allowed_command(self, expected: Any, command: Any, data: Any) -> bool:
        result = tuple(expected) == self.bridge._actor_command_fence()
        if self.authority_blocked():
            negative = getattr(self.bridge, "_actor_negative_command", lambda c, d: c == "stop")
            if not negative(command, data):
                result = False
        if not result:
            self.counters["stale_queued_commands"] += 1
        return result

    def run(self) -> None:
        prev_switch_interval = (
            sys.getswitchinterval() if hasattr(sys, "getswitchinterval") else None
        )
        if hasattr(sys, "setswitchinterval"):
            try:
                sys.setswitchinterval(0.001)
            except ValueError:
                pass
        orig_affinity, self.non_isolated_cpus = _pin_control_actor_cpu()
        if (
            orig_affinity is not None
            and hasattr(os, "sched_getaffinity")
            and hasattr(os, "sched_setaffinity")
        ):
            try:
                actor_cpus = os.sched_getaffinity(0)
            except OSError:
                actor_cpus = None
            if actor_cpus:
                setup_cpus = self.non_isolated_cpus or orig_affinity
                self.bridge._actor_setup_affinity = lambda pin_actor: os.sched_setaffinity(
                    0, actor_cpus if pin_actor else setup_cpus
                )
        try:
            self.bridge._run()
        except BaseException as exc:
            _offer_latest(
                self.channels["status"],
                dict(state="error", error=type(exc).__name__ + ": " + str(exc)),
            )
        finally:
            if prev_switch_interval is not None and hasattr(sys, "setswitchinterval"):
                try:
                    sys.setswitchinterval(prev_switch_interval)
                except ValueError:
                    pass
            if orig_affinity is not None and hasattr(os, "sched_setaffinity"):
                try:
                    os.sched_setaffinity(0, orig_affinity)
                except OSError:
                    pass
            if self.ipc_stop_w is not None:
                try:
                    os.write(self.ipc_stop_w, b"x")
                except OSError:
                    pass
            if self.ipc_wake_thread is not None:
                self.ipc_wake_thread.join(timeout=1.0)
            for pipe_fd in (self.ipc_stop_r, self.ipc_stop_w):
                if pipe_fd is not None:
                    try:
                        os.close(pipe_fd)
                    except OSError:
                        pass
            capture = getattr(self.bridge, "_publication_capture", None)
            if capture is not None:
                state = capture.status()
                if not state.get("pending_ingress") and not state.get("inflight"):
                    try:
                        capture.close()
                    except Exception:
                        pass
            for name in ("observers", "observer_ros", "status", "replay"):
                if name in self.channels:
                    self.channels[name].cancel_join_thread()


def run_control_worker(
    config: dict[str, Any],
    channels: dict[str, Any],
    capture_spec: dict[str, Any] | None,
    bridge_factory: Any = None,
    *,
    parent_liveness_cls: Any,
) -> None:
    """运行隔离控制子进程宿主循环。"""
    ControlWorkerHost(
        config,
        channels,
        capture_spec,
        bridge_factory=bridge_factory,
        parent_liveness_cls=parent_liveness_cls,
    ).run()


"""实时指令多路复用转发、Actor 最新目标合并与 ROS 命令队列排空分发器。"""
from __future__ import annotations

import copy
import math
import queue
import threading
import time
from typing import Any, Callable

from .handover_hold import _HANDOVER_HOLD_SPEC, _HOLD_ATTR_BY_SOURCE


def _copy_actor_message(message: Any) -> Any:
    """浅层复制 JointState 目标消息，避免 deepcopy 遍历开销。"""
    name = getattr(message, "name", None)
    position = getattr(message, "position", None)
    header = getattr(message, "header", None)
    stamp = getattr(header, "stamp", None)
    velocity = getattr(message, "velocity", None)
    effort = getattr(message, "effort", None)
    if (
        header is None
        or stamp is None
        or name is None
        or position is None
        or velocity is None
        or effort is None
    ):
        return copy.deepcopy(message)
    try:
        cloned = type(message)()
        cloned.header.stamp.sec = stamp.sec
        cloned.header.stamp.nanosec = stamp.nanosec
        cloned.header.frame_id = header.frame_id
        cloned.name = list(name)
        cloned.position = list(position)
        cloned.velocity = list(velocity)
        cloned.effort = list(effort)
        return cloned
    except (AttributeError, TypeError, ValueError):
        return copy.deepcopy(message)


class CommandDispatcher:
    """封装控制指令通道路由、发布点采集记账、实时目标合并转发及命令队列排空。"""

    @staticmethod
    def complete_publication_capture(
        capture: Any,
        ticket: Any,
        published: Any,
        wire_cdr: bytes | None,
        publication_ns: int | None,
        publication_monotonic: float | None,
    ) -> None:
        if ticket is None or capture is None:
            return
        if publication_ns is not None and wire_cdr is not None:
            capture.complete(
                ticket,
                serialized_cdr=wire_cdr,
                publication_ns=publication_ns,
                publication_monotonic=publication_monotonic,
            )
        else:
            capture.complete(
                ticket,
                published,
                publication_ns=publication_ns,
                publication_monotonic=publication_monotonic,
            )

    @staticmethod
    def record_forwarded_metrics_locked(
        bridge: Any, message: Any, publication_monotonic: float
    ) -> None:
        channel = bridge._command_channel(getattr(message, "name", ()))
        bridge._mux_forwarded += 1
        bridge._mux_forwarded_times.append(publication_monotonic)
        bridge._mux_channel_forwarded[channel] += 1
        bridge._mux_channel_forwarded_times[channel].append(publication_monotonic)

    @staticmethod
    def count_live_input_locked(
        bridge: Any, source: str, channel: str, now: float
    ) -> None:
        bridge._mux_received[source] += 1
        bridge._mux_last_received[source] = now
        bridge._mux_received_times[source].append(now)
        bridge._mux_channel_received[channel] += 1
        bridge._mux_channel_received_times[channel].append(now)

    @staticmethod
    def live_target_allowed_locked(
        bridge: Any, source: str, channel: str, token: Any, stamp: float
    ) -> bool:
        owner = bridge.ownership
        if bridge._legacy_enabled and channel == "right_hand" and owner.state == "AUTO":
            if stamp <= owner.right_gripper_cutoff_ns * 1e-9:
                return False
            if owner.right_gripper_source != "motion":
                return (
                    bridge._command_source == "motion"
                    and source == "vr"
                    and token is not None
                    and token == owner.right_gripper_token
                    and owner.dagger_enabled
                    and owner._input_fresh()
                )
        return bridge._command_source == source and (
            not owner.active or owner.allows(token)
        )

    @staticmethod
    def command_channel(bridge: Any, names: Any) -> str:
        if bridge._command_channel_boundary is not bridge._boundary:
            bridge._command_channel_boundary = bridge._boundary
            bridge._command_channel_cache.clear()
        try:
            key = names if isinstance(names, tuple) else tuple(names or ())
            cached = bridge._command_channel_cache.get(key)
            if cached is not None:
                return cached
            boundary = bridge._boundary
            arms = getattr(boundary, "arms", ())
            hands = getattr(boundary, "hands", {})
            if not isinstance(arms, (tuple, list, set, frozenset)):
                arms = ()
            if not isinstance(hands, dict):
                hands = {}
            if any(name in arms for name in key):
                channel = "arms"
            else:
                sides = [
                    side
                    for side in ("right", "left")
                    if any(name in hands.get(side, ()) for name in key)
                ]
                channel = sides[0] + "_hand" if len(sides) == 1 else "other"
            if len(bridge._command_channel_cache) < 64:
                bridge._command_channel_cache[key] = channel
            return channel
        except (TypeError, ValueError):
            return "other"

    @staticmethod
    def queue_actor_target(
        bridge: Any, source: str, message: Any, publish: Callable, token: Any
    ) -> bool:
        if source not in bridge._mux_received:
            return False
        now = time.monotonic()
        channel = bridge._command_channel(getattr(message, "name", ()))
        with bridge._lock:
            bridge._count_live_input_locked(source, channel, now)
        try:
            target = _copy_actor_message(message)
            names = tuple(target.name)
            stamp = target.header.stamp.sec + target.header.stamp.nanosec * 1e-9
            valid = (
                bool(names)
                and len(names) == len(target.position)
                and len(set(names)) == len(names)
                and all(names)
                and all(math.isfinite(float(v)) for v in target.position)
                and math.isfinite(stamp)
                and stamp > 0
                and -0.05 <= bridge._ros_now() - stamp <= 0.3
            )
        except (AttributeError, TypeError, ValueError):
            return False
        key = source, channel
        stream = (source, names) if bridge._legacy_enabled else source
        with bridge._lock:
            bridge._actor_counts["offered"] += 1
            revision = (
                bridge.ownership.right_gripper_revision
                if channel == "right_hand"
                else None
            )
            pending = bridge._actor_targets.get(key)
            if pending is not None and pending[5] != revision:
                pending = None
            if (
                not valid
                or bridge._stop.is_set()
                or not bridge._ready_locked(time.monotonic())
                or not bridge._command_output_enabled
                or bridge._actor_output_blocked()
                or bridge._replay_active
                or stamp <= max(bridge._source_cutoff, bridge._source_stamps.get(stream, 0))
                or (pending is not None and stamp <= pending[0])
                or not bridge._live_target_allowed_locked(source, channel, token, stamp)
            ):
                bridge._actor_counts["discarded"] += 1
                return False
            if pending is not None:
                bridge._actor_counts["coalesced"] += 1
            bridge._actor_targets[key] = (
                stamp,
                bridge._output_epoch,
                target,
                publish,
                token,
                revision,
            )
            bridge._actor_callback_count += 1
            return True

    @staticmethod
    def drain_actor_targets(bridge: Any) -> None:
        if not bridge._control_actor_mode:
            return
        with bridge._lock:
            if not bridge._actor_targets or bridge._stop.is_set():
                return
            ready = []
            for source in ("vr", "exoskeleton", "motion"):
                for channel in bridge.COMMAND_CHANNELS:
                    pending = bridge._actor_targets.pop((source, channel), None)
                    if pending is not None and pending[1] == bridge._output_epoch:
                        ready.append((source, pending))
        for source, pending in ready:
            with bridge._lock:
                if pending[1] != bridge._output_epoch or bridge._stop.is_set():
                    continue
            _, _, target, publish, token, revision = pending
            bridge._actor_dispatching = True
            bridge._actor_dispatch_epoch = pending[1]
            bridge._actor_dispatch_gripper_revision = revision
            try:
                bridge.forward_live_command(source, target, publish, token)
            finally:
                bridge._actor_dispatching = False
                bridge._actor_dispatch_epoch = None
                bridge._actor_dispatch_gripper_revision = None

    @staticmethod
    def forward_live_command(
        bridge: Any, source: str, message: Any, publish: Callable, token: Any = None
    ) -> bool:
        if bridge._control_actor_mode and not (
            bridge._actor_dispatching
            and bridge._actor_thread_ident == threading.get_ident()
        ):
            return bridge._queue_actor_target(source, message, publish, token)
        bridge._command_timing_sequence += 1
        if bridge._command_timing_sequence % 8 != 1:
            return bridge._forward_live_command(source, message, publish, token)
        started = time.monotonic_ns(), time.thread_time_ns()
        timing = {"_previous": started}
        boundary = bridge._boundary
        channel = (
            "arms"
            if boundary is not None
            and set(getattr(message, "name", ())).intersection(
                getattr(boundary, "arms", ())
            )
            else "hands"
        )
        try:
            return bridge._forward_live_command(
                source, message, publish, token, timing=timing
            )
        finally:
            finished = time.monotonic_ns(), time.thread_time_ns()
            timing.pop("_previous", None)
            timing["total"] = dict(
                wall_ms=(finished[0] - started[0]) / 1e6,
                cpu_ms=(finished[1] - started[1]) / 1e6,
            )
            with bridge._callback_timing_lock:
                for stage, elapsed in timing.items():
                    for key in (source + ":" + stage, source + ":" + channel + ":" + stage):
                        value = bridge._command_stage_timings.setdefault(
                            key, dict(samples=0, max_ms=0.0, max_cpu_ms=0.0)
                        )
                        value.update(
                            samples=value["samples"] + 1,
                            last_ms=elapsed["wall_ms"],
                            last_cpu_ms=elapsed["cpu_ms"],
                            max_ms=max(value["max_ms"], elapsed["wall_ms"]),
                            max_cpu_ms=max(value["max_cpu_ms"], elapsed["cpu_ms"]),
                        )
                if timing["total"]["wall_ms"] >= 10.0:
                    bridge._slow_command_stages.append(
                        dict(
                            source=source,
                            channel=channel,
                            stamp_ns=time.time_ns(),
                            stages=timing,
                        )
                    )

    @staticmethod
    def forward_live_command_body(
        bridge: Any,
        source: str,
        message: Any,
        publish: Callable,
        token: Any = None,
        *,
        timing: dict[str, Any] | None = None,
    ) -> bool:
        bridge._actor_checkpoint(allow_snapshot=False)
        now = time.monotonic()
        stream: Any = source
        try:
            names, positions = message.name, message.position
            stream = (source, tuple(names)) if bridge._legacy_enabled else source
            stamp = message.header.stamp.sec + message.header.stamp.nanosec * 1e-9
            valid = (
                bool(names)
                and len(names) == len(positions)
                and len(set(names)) == len(names)
                and all(names)
                and all(math.isfinite(float(v)) for v in positions)
                and math.isfinite(stamp)
                and stamp > 0
                and -0.05 <= bridge._ros_now() - stamp <= 0.3
            )
            if valid and bridge._public_api is not None:
                bridge._public_api.convert(
                    message.name,
                    message.position,
                    message.velocity,
                    message.effort,
                    to_model=True,
                )
        except (AttributeError, TypeError, ValueError):
            valid, stamp = False, 0.0
        channel = bridge._command_channel(
            stream[1] if isinstance(stream, tuple) else getattr(message, "name", ())
        )
        initial_attempt = bridge._command_instant(timing)
        with bridge._lock:
            initial_acquired = bridge._command_instant(timing)
            if not bridge._control_actor_mode:
                bridge._count_live_input_locked(source, channel, now)
            ready = bridge._ready_locked(now)
            revision = (
                bridge.ownership.right_gripper_revision
                if channel == "right_hand"
                else None
            )
            receipt_epoch_changed = bridge._control_actor_mode and (
                bridge._actor_dispatch_epoch != bridge._output_epoch
                or bridge._actor_dispatch_gripper_revision != revision
            )
            if (
                not valid
                or receipt_epoch_changed
                or stamp <= max(bridge._source_cutoff, bridge._source_stamps.get(stream, 0))
                or not ready
                or not bridge._command_output_enabled
                or bridge._actor_output_blocked()
                or bridge._replay_active
                or not bridge._live_target_allowed_locked(source, channel, token, stamp)
            ):
                bridge._mux_rejected += 1
                reason = (
                    "invalid_target"
                    if not valid
                    else "receipt_epoch_changed"
                    if receipt_epoch_changed
                    else "stale_or_reordered"
                    if stamp <= max(bridge._source_cutoff, bridge._source_stamps.get(stream, 0))
                    else "hardware_not_ready"
                    if not ready
                    else "output_closed"
                    if not bridge._command_output_enabled
                    else "replay_active"
                    if bridge._replay_active
                    else "source_mismatch"
                    if bridge._command_source != source
                    else "ownership_denied"
                )
                bridge._mux_rejection_reasons[source + ":" + reason] += 1
                return False
            epoch = bridge._output_epoch
        bridge._command_interval(
            timing, "initial_lock_wait", initial_attempt, initial_acquired
        )
        bridge._command_checkpoint(timing, "initial_gate")
        prepared = bridge._boundary.prepare(message) if bridge._boundary else message
        bridge._command_checkpoint(timing, "prepare")
        send, wire_cdr = publish, None
        prepare_publication = getattr(bridge._boundary, "prepare_publication", None)
        if bridge._legacy_enabled and prepared is not None and callable(prepare_publication):
            try:
                send, wire_cdr = prepare_publication(prepared[0], publish)
            except Exception as exc:
                bridge._boundary.reject("command serialization failed: " + str(exc))
                prepared = None
        bridge._command_checkpoint(timing, "prepare_publication")
        bridge._actor_checkpoint(allow_snapshot=False)
        capture = getattr(bridge, "_publication_capture", None)
        arm_target = (
            bridge._legacy_enabled
            and bridge._boundary
            and prepared is not None
            and set(prepared[0].name).intersection(bridge._boundary.arms)
        )
        ticket = capture.reserve() if capture is not None and arm_target else None
        bridge._command_checkpoint(timing, "capture_reserve")
        published, publication_ns, publication_monotonic = None, None, None
        evidence = None
        send_attempt = bridge._command_instant(timing)
        send_acquired = commit_done = publish_done = None
        try:
            with bridge._lock:
                send_acquired = bridge._command_instant(timing)
                if (
                    prepared is None
                    or epoch != bridge._output_epoch
                    or not bridge._command_output_enabled
                    or bridge._actor_output_blocked()
                    or (
                        bridge._control_actor_mode
                        and (
                            bridge._stop.is_set()
                            or bridge._actor_targets.get((source, channel), (0,))[0] > stamp
                        )
                    )
                    or (
                        revision is not None
                        and revision != bridge.ownership.right_gripper_revision
                    )
                    or not bridge._ready_locked(time.monotonic())
                    or bridge._replay_active
                    or stamp <= bridge._source_stamps.get(stream, 0)
                    or not bridge._live_target_allowed_locked(source, channel, token, stamp)
                ):
                    bridge._mux_rejected += 1
                    bridge._mux_rejection_reasons[
                        source
                        + (
                            ":boundary_rejected"
                            if prepared is None
                            else ":authority_changed_during_prepare"
                        )
                    ] += 1
                    return False
                hold_attr = _HOLD_ATTR_BY_SOURCE.get(source)
                hold = getattr(bridge, hold_attr) if hold_attr is not None else None
                closes_hold = (
                    hold is not None
                    and hold.token == token
                    and arm_target
                    and source == _HANDOVER_HOLD_SPEC[hold.mode]["source"]
                )
                if (
                    closes_hold
                    and _HANDOVER_HOLD_SPEC[hold.mode]["fault_on_deadline"]
                    and bridge.ownership.clock() >= hold.deadline_ns
                ):
                    bridge.ownership.fault(_HANDOVER_HOLD_SPEC[hold.mode]["deadline_reason"])
                    return False
                if bridge._boundary and not (
                    bridge._boundary.commit(prepared, intervention=True)
                    if bridge.ownership.active
                    else bridge._boundary.commit(prepared)
                ):
                    bridge._mux_rejected += 1
                    bridge._mux_rejection_reasons[source + ":commit_rejected"] += 1
                    return False
                bridge._source_stamps[stream] = stamp
                if bridge._legacy_enabled and bridge._boundary:
                    message = prepared[0]
                commit_done = bridge._command_instant(timing)
                send(message)
                if closes_hold:
                    setattr(bridge, hold_attr, None)
                    if hold.sent:
                        spec = _HANDOVER_HOLD_SPEC[hold.mode]
                        bridge.ownership._event(
                            spec["close_event"], end_reason=spec["close_reason"]
                        )
                publish_done = bridge._command_instant(timing)
                published, publication_ns = message, time.time_ns()
                publication_monotonic = time.monotonic()
                if (
                    bridge.ownership.active
                    and token != bridge.ownership.right_gripper_token
                ):
                    evidence = bridge.ownership.sent(
                        token,
                        message,
                        int(bridge._ros_now() * 1e9),
                        arm_command=not bridge._legacy_enabled or arm_target,
                    )
                    if evidence is not None:
                        evidence["command_stamp_ns"] = (
                            message.header.stamp.sec * 1_000_000_000
                            + message.header.stamp.nanosec
                        )
                        if not bridge._control_actor_mode:
                            bridge._action_sink(evidence)
                bridge._mux_forwarded += 1
                bridge._mux_forwarded_times.append(time.monotonic())
                bridge._mux_channel_forwarded[channel] += 1
                bridge._mux_channel_forwarded_times[channel].append(
                    publication_monotonic
                )
                return True
        finally:
            released = bridge._command_instant(timing)
            bridge._command_interval(timing, "send_lock_wait", send_attempt, send_acquired)
            bridge._command_interval(timing, "commit_validate", send_acquired, commit_done)
            bridge._command_interval(timing, "dds_publish", commit_done, publish_done)
            bridge._command_interval(timing, "send_accounting", publish_done, released)
            bridge._command_checkpoint(timing, "commit_publish")
            bridge._complete_publication_capture(
                capture, ticket, published, wire_cdr, publication_ns, publication_monotonic
            )
            bridge._command_checkpoint(timing, "capture_complete")
            if bridge._control_actor_mode and evidence is not None:
                bridge._action_sink(evidence)

    @staticmethod
    def actor_negative_command(bridge: Any, command: str, args: Any) -> bool:
        if (
            command == "stop"
            or (command == "command_output" and not args[0])
            or (command == "_actor_permit" and (not args[3] or args[1] == "revoke"))
        ):
            return True
        if command != "publish":
            return False
        topic, kind, data = args
        return kind == "std_msgs/msg/Bool" and (
            (
                topic == bridge.config.get("stop_topic", "/control/emergency_stop")
                and data.get("data") is True
            )
            or (
                topic
                in {
                    "/control/teleop/enabled",
                    bridge.config.get("command_mux", {}).get(
                        "hardware_enable_topic", "/control/hardware/enabled"
                    ),
                }
                and data.get("data") is False
            )
        )

    @staticmethod
    def actor_queue_item_allowed(bridge: Any, item: tuple[Any, ...]) -> bool:
        if not bridge._control_actor_mode:
            return True
        if bridge._stop.is_set() and item[0] != "stop":
            return False
        if bridge._actor_output_blocked() and not bridge._actor_negative_command(
            *item[:2]
        ):
            return False
        if len(item) != 3:
            return False
        allowed = getattr(bridge, "_actor_command_allowed", None)
        return (
            allowed(item[2], item[0], item[1])
            if callable(allowed)
            else item[2] == bridge._output_epoch
        )

    @staticmethod
    def drain_commands(
        bridge: Any,
        node: Any,
        publishers: dict[tuple[str, str], Any],
        get_message: Callable[[str], Any],
        set_message_fields: Callable[[Any, dict[str, Any]], None],
        wait: float = 0.0,
    ) -> None:
        if (
            bridge._control_actor_mode
            and bridge._actor_thread_ident != threading.get_ident()
        ):
            raise RuntimeError("control DDS commands must be drained by the actor thread")
        for index in range(16 if bridge._control_actor_mode else 100):
            try:
                if bridge._control_actor_mode and bridge._actor_urgent:
                    item = bridge._actor_urgent.popleft()
                elif index == 0 and wait > 0:
                    item = bridge._commands.get(timeout=wait)
                else:
                    try:
                        item = bridge._commands.get_nowait()
                    except queue.Empty:
                        if not bridge._control_actor_mode or not bridge._actor_replay:
                            raise
                        _, item = bridge._actor_replay.popitem()
                command, args = item[:2]
                if bridge._control_actor_mode:
                    if not bridge._actor_queue_item_allowed(item):
                        bridge._actor_counts["discarded_commands"] += 1
                        continue
            except queue.Empty:
                break
            try:
                if command == "_actor_permit":
                    if bridge._permit_sender is not None:
                        bridge._permit_sender(*args)
                elif command == "manual_home":
                    with bridge._lock:
                        if (
                            args != bridge.ownership.token
                            or bridge.ownership.state != "MANUAL"
                            or not bridge._command_output_enabled
                            or not bridge._ready_locked(time.monotonic())
                        ):
                            continue
                        from std_msgs.msg import Header

                        message = Header(
                            stamp=node.get_clock().now().to_msg(), frame_id=args.key()
                        )
                        bridge._publisher(
                            node,
                            publishers,
                            "/internal/vr/home_request",
                            "std_msgs/msg/Header",
                            Header,
                        ).publish(message)
                elif command == "replay_home":
                    with bridge._lock:
                        epoch, output_resumed = args
                        if epoch != bridge._output_epoch or not bridge._replay_home_allowed_locked(
                            output_resumed=output_resumed
                        ):
                            continue
                        from std_msgs.msg import Bool

                        bridge._publisher(
                            node, publishers, "/teleop/arm/home", "std_msgs/msg/Bool", Bool
                        ).publish(Bool(data=True))
                        bridge._command_output_enabled = True
                elif command == "ownership_recover":
                    with bridge._lock:
                        if (
                            args != getattr(bridge, "_recovery", None)
                            or args[0] != bridge.ownership.token
                        ):
                            continue
                        from std_msgs.msg import Bool

                        bridge._publisher(
                            node,
                            publishers,
                            "/control/emergency_stop",
                            "std_msgs/msg/Bool",
                            Bool,
                        ).publish(Bool(data=False))
                        bridge._hardware_enable_sent_epoch = None
                        bridge._publish_hardware_enable_locked(node, publishers, True)
                elif command == "publish":
                    topic, msg_type_name, data = args
                    if str(topic) == "/control/joint_cmd" or "/msg/" not in str(
                        msg_type_name
                    ):
                        continue
                    if bridge.ownership.active and bridge._ownership_control_topic(topic):
                        continue
                    message_type = get_message(msg_type_name)
                    publisher = bridge._publisher(
                        node, publishers, topic, msg_type_name, message_type
                    )
                    message = message_type()
                    set_message_fields(message, data)
                    bridge._actor_checkpoint()
                    if not bridge._actor_queue_item_allowed(item):
                        continue
                    publisher.publish(message)
                elif command in {"publish_raw", "replay_raw"}:
                    topic, msg_type_name, raw_bytes = args[:3]
                    if str(topic) == "/control/joint_cmd" or "/msg/" not in str(
                        msg_type_name
                    ):
                        continue
                    if not bridge._raw_replay_destination_allowed(topic):
                        continue
                    if bridge.ownership.active and bridge._ownership_control_topic(topic):
                        continue
                    if command == "replay_raw":
                        with bridge._lock:
                            if not bridge._replay_allowed_locked(args[3]):
                                continue
                    message_type = get_message(msg_type_name)
                    from rclpy.serialization import deserialize_message

                    message = deserialize_message(raw_bytes, message_type)
                    if command == "replay_raw":
                        bridge._publish_replay_joint_message(
                            node,
                            publishers,
                            topic,
                            msg_type_name,
                            message_type,
                            message,
                            epoch=args[3],
                            convert_public=True,
                        )
                    else:
                        publisher = bridge._publisher(
                            node, publishers, topic, msg_type_name, message_type
                        )
                        bridge._actor_checkpoint()
                        if bridge._actor_queue_item_allowed(item):
                            publisher.publish(message)
                elif command == "stop":
                    topic, reason = args
                    from std_msgs.msg import Bool

                    publisher = bridge._publisher(
                        node, publishers, topic, "std_msgs/msg/Bool", Bool
                    )
                    publisher.publish(Bool(data=True))
                    if not bridge._control_actor_mode:
                        node.get_logger().warning(f"Emergency stop: {reason}")
                elif command == "control_source":
                    if bridge.ownership.active:
                        continue
                    mux_config = bridge.config.get("command_mux", {})
                    topic = str(
                        mux_config.get("control_source_topic", "/control/source")
                    )
                    from std_msgs.msg import String

                    publisher = bridge._publisher(
                        node, publishers, topic, "std_msgs/msg/String", String
                    )
                    publisher.publish(String(data=str(args)))
                    if not bridge._control_actor_mode:
                        node.get_logger().info(f"Joint command source: {args}")
                elif command == "command_output":
                    if bridge.ownership.active:
                        continue
                    enabled, epoch = args
                    with bridge._lock:
                        if (
                            epoch != bridge._output_epoch
                            or enabled != bridge._command_output_enabled
                        ):
                            continue
                        bridge._publish_hardware_enable_locked(node, publishers, enabled)
                    if not bridge._control_actor_mode:
                        node.get_logger().info(
                            "Joint command output %s"
                            % ("enabled" if enabled else "disabled")
                        )
            except Exception as exc:
                bridge._set_status(error=f"publish failed: {type(exc).__name__}: {exc}")

        with bridge._lock:
            bridge._ready_locked(time.monotonic())
            if (
                not getattr(bridge, "_recovery", None)
                and not bridge._command_output_enabled
                and (
                    not bridge.ownership.active
                    or bridge.ownership.state in {"FAULT", "ESTOP"}
                )
            ):
                bridge._publish_hardware_enable_locked(node, publishers, False)

    @staticmethod
    def publish_hardware_enable_locked(
        bridge: Any, node: Any, publishers: dict[tuple[str, str], Any], enabled: bool
    ) -> None:
        if enabled and bridge._actor_output_blocked():
            return
        now = time.monotonic()
        if bridge._hardware_enable_sent_epoch == bridge._output_epoch and (
            enabled
            or now - getattr(bridge, "_hardware_enable_last_sent", float("-inf")) < 0.1
        ):
            return
        from std_msgs.msg import Bool

        topic = str(
            bridge.config.get("command_mux", {}).get(
                "hardware_enable_topic", "/control/hardware/enabled"
            )
        )
        publisher = bridge._publisher(node, publishers, topic, "std_msgs/msg/Bool", Bool)
        publisher.publish(Bool(data=enabled))
        bridge._hardware_enable_sent_epoch = bridge._output_epoch
        bridge._hardware_enable_last_sent = now

    @staticmethod
    def publisher(
        node: Any,
        publishers: dict[tuple[str, str], Any],
        topic: str,
        msg_type_name: str,
        message_type: Any,
    ) -> Any:
        key = (topic, msg_type_name)
        if key not in publishers:
            qos: Any = 10
            if topic in {
                "/status/control/ownership",
                "/events/control/intervention",
                "/internal/control/action_evidence",
            }:
                from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

                qos = QoSProfile(
                    depth=1 if topic == "/status/control/ownership" else 100,
                    reliability=ReliabilityPolicy.RELIABLE,
                    durability=(
                        DurabilityPolicy.TRANSIENT_LOCAL
                        if topic == "/status/control/ownership"
                        else DurabilityPolicy.VOLATILE
                    ),
                )
            publishers[key] = node.create_publisher(message_type, topic, qos)
        return publishers[key]


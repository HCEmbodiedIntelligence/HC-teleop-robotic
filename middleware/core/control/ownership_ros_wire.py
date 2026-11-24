"""控制权状态机（ControlOwnership）与 ROS 2 话题/定时器的接线模块。"""
from __future__ import annotations

from collections import Counter, deque
import copy
import json
import threading
import time
from typing import Any

from ..protocol import envelope


def setup_ownership_ros(
    bridge: Any,
    node: Any,
    publishers: dict[tuple[str, str], Any],
    permit_pub: Any,
    output_pub: Any,
    subscriptions: list[Any],
    *,
    command_node: Any = None,
    status_node: Any = None,
) -> None:
    """在私有 HC 控制链路上挂载控制权许可、确认、语义输入、受控目标和状态观测。"""
    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
    from sensor_msgs.msg import JointState
    from std_msgs.msg import Header, String
    from .control_ownership import ControlToken

    observer_hook = (
        getattr(bridge, "_observer_ros_publish", None)
        if bridge._control_actor_mode
        else None
    )
    observer_hook = observer_hook if callable(observer_hook) else None
    state_pub = events_pub = action_pub = None
    if observer_hook is None:
        state_pub = node.create_publisher(
            String,
            "/status/control/ownership",
            QoSProfile(
                depth=1,
                reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.TRANSIENT_LOCAL,
            ),
        )
        events_pub = node.create_publisher(String, "/events/control/intervention", 100)
        action_pub = node.create_publisher(
            String, "/internal/control/action_evidence", 100
        )

    def publish_observer(topic: str, payload: dict[str, Any], publisher: Any) -> None:
        if observer_hook is not None:
            if observer_hook(topic, payload) is False:
                bridge._actor_counts["observer_ros_dropped"] += 1
        else:
            publisher.publish(String(data=json.dumps(payload, allow_nan=False)))

    previous_sink = bridge._action_sink
    actor_evidence: deque[dict[str, Any]] = deque(maxlen=128)

    def evidence_sink(evidence: dict[str, Any]) -> None:
        if bridge._control_actor_mode:
            previous_sink(evidence)
            if len(actor_evidence) == actor_evidence.maxlen:
                bridge._actor_counts["dds_evidence_dropped"] += 1
            actor_evidence.append(evidence)
        else:
            previous_sink(evidence)
            publish_observer("/internal/control/action_evidence", evidence, action_pub)

    bridge._action_sink = evidence_sink

    def send(token: Any, phase: str, cutoff_ns: int, active: bool) -> None:
        if (
            bridge._control_actor_mode
            and bridge._actor_thread_ident != threading.get_ident()
        ):
            bridge._enqueue("_actor_permit", (token, phase, cutoff_ns, active))
            return
        if active and phase == "grant" and bridge._actor_output_blocked():
            return
        peers = bridge.ownership.peers
        key = "#".join(
            (
                token.key(),
                phase if active else "off",
                peers.get("solver" if bridge._legacy_enabled else "motion", ""),
                peers.get("teleop", ""),
            )
        )
        permit_pub.publish(Header(stamp=node.get_clock().now().to_msg(), frame_id=key))
        if phase == "revoke" and bridge.ownership.state in {"OFF", "FAULT", "ESTOP"}:
            bridge._publish_hardware_enable_locked(node, publishers, False)

    bridge._permit_sender = send

    def tick() -> None:
        with bridge._lock:
            ownership = bridge.ownership
            ownership.watchdog()
            recovery = getattr(bridge, "_recovery", None)
            if recovery:
                if (
                    ownership.token != recovery[0]
                    or time.monotonic() - recovery[1] > 2.0
                ):
                    bridge._recovery = None
                    bridge._hardware_enable_sent_epoch = None
                    bridge._publish_hardware_enable_locked(node, publishers, False)
                elif bridge._hardware_ready_last > recovery[1] and bridge.control_ready():
                    bridge._recovery = None
                    ownership.recover(control_ready=True)
            if (
                ownership.state in {"AUTO", "MANUAL"}
                and bridge._boundary
                and not bridge._boundary.feedback_fresh()
            ):
                if bridge._legacy_enabled:
                    bridge._status["last_feedback_failure"] = dict(
                        stamp_ns=node.get_clock().now().nanoseconds,
                        **bridge._boundary.status(),
                    )
                ownership.fault("necessary robot feedback lost")
            ownership.advance(
                control_ready=bridge._handover_control_ready_locked(),
                manual_ready=ownership.input_valid and ownership.gripped,
            )
            if (
                ownership.state in {"AUTO", "MANUAL"}
                and not bridge._command_output_enabled
                and not bridge._actor_output_blocked()
            ):
                bridge._command_source = ownership.token.source
                bridge._source_cutoff = ownership.cutoff_ns * 1e-9
                bridge._command_output_enabled = True
        bridge._tick_handover_holds()

    def ack(msg: Any) -> None:
        try:
            value = json.loads(msg.data)
            participant, instance = value["participant"], value["instance"]
            if (
                participant not in bridge.ownership.required_peers
                or not isinstance(instance, str)
                or not instance
            ):
                return
            age = node.get_clock().now().nanoseconds - int(value["stamp_ns"])
            if not -50_000_000 <= age <= 300_000_000:
                return
            bridge.ownership.peer(participant, instance)
            if value.get("token"):
                bridge.ownership.acknowledge(
                    ControlToken.from_key(value["token"]),
                    value["phase"],
                    participant,
                    instance,
                    value.get("success") is True,
                    value.get("reason", ""),
                )
            tick()
        except (ValueError, KeyError, TypeError):
            return

    def semantic_input(msg: Any) -> None:
        entered_ns = node.get_clock().now().nanoseconds
        try:
            value = json.loads(msg.data)
            bridge.receive_intervention_input(value, callback_entry_ns=entered_ns)
            tick()
        except (ValueError, KeyError, TypeError) as error:
            with bridge._lock:
                bridge._status["last_input_callback_error"] = dict(
                    stamp_ns=node.get_clock().now().nanoseconds,
                    error=f"{type(error).__name__}: {error}",
                )
            return

    def owned_target(msg: Any) -> None:
        try:
            key, frame = msg.header.frame_id.split("#", 1)
            token = ControlToken.from_key(key)
        except (ValueError, AttributeError):
            return
        if bridge._legacy_enabled:
            bridge._boundary.receive_owned(msg, token, frame_id=frame)
            return
        target = copy.deepcopy(msg)
        target.header.frame_id = frame
        if bridge._public_api:
            allowed = {
                bridge._public_api.joints[n]["public_name"]
                for group, names in bridge._public_api.groups.items()
                if group.endswith(("_arm", "_gripper"))
                for n in names
            }
            if bridge._boundary:
                allowed.update(
                    bridge._public_api.joints[n]["public_name"]
                    for n in bridge._boundary.gripper_publishers
                )
            if not set(target.name) <= allowed:
                return
        bridge.forward_live_command(token.source, target, output_pub.publish, token)

    event_cursor = 0
    state_last = 0.0
    state_sequence = -1
    diagnostic_last = time.monotonic()

    def rejection_counts() -> Counter:
        counts = Counter(
            {
                reason: count
                for reason, count in bridge._mux_rejection_reasons.items()
                if reason.partition(":")[2]
                in {
                    "invalid_target",
                    "stale_or_reordered",
                    "boundary_rejected",
                    "commit_rejected",
                }
            }
        )
        if bridge._legacy_enabled and bridge._boundary:
            counts.update(
                {
                    reason: count
                    for reason, count in bridge._boundary.diagnostics.items()
                    if reason.startswith("rejected:")
                }
            )
        return counts

    with bridge._lock:
        diagnostic_previous = rejection_counts()
        clip_previous = (
            bridge._boundary.diagnostics.get("arm_velocity_clipped", 0)
            if bridge._legacy_enabled and bridge._boundary
            else 0
        )

    def publish_status() -> None:
        nonlocal event_cursor, state_last, state_sequence
        nonlocal diagnostic_last, diagnostic_previous, clip_previous
        wall_ns = bridge.ownership.ros_clock()
        with bridge._lock:
            ownership = bridge.ownership
            events = [
                event for event in ownership.events if event["sequence"] > event_cursor
            ]
            now = time.monotonic()
            state = (
                ownership.snapshot(wall_ns=wall_ns)
                if now - state_last >= 0.1 or state_sequence != ownership.event_seq
                else None
            )
            sequence = ownership.event_seq
            if state is not None:
                state["recording_output"] = dict(
                    allowed=bool(
                        bridge._command_output_enabled
                        and not bridge._actor_output_blocked()
                        and bridge._hardware_ready
                        and not bridge._replay_active
                        and bridge._command_source == "vr"
                        and ownership.state not in {"FAULT", "ESTOP"}
                    ),
                    output_epoch=bridge._output_epoch,
                    source=bridge._command_source,
                    stamp_ns=wall_ns,
                )
            diagnostic = None
            clip_diagnostic = None
            if now - diagnostic_last >= 1.0:
                counts = rejection_counts()
                delta = counts - diagnostic_previous
                if delta:
                    diagnostic = dict(
                        state=ownership.state,
                        epoch=ownership.epoch,
                        counts=dict(delta),
                    )
                    if any(key.startswith("rejected:") for key in delta):
                        diagnostic.update(
                            recent_rejection=bridge._boundary.recent_rejection,
                            rejection_stamp_ns=bridge._boundary.last_rejection_ns,
                        )
                diagnostic_previous, diagnostic_last = counts, now
                if bridge._legacy_enabled and bridge._boundary:
                    arm_clips = int(
                        bridge._boundary.diagnostics.get("arm_velocity_clipped", 0)
                    )
                    if arm_clips > clip_previous:
                        clip_diagnostic = dict(
                            state=ownership.state,
                            epoch=ownership.epoch,
                            count=arm_clips - clip_previous,
                            total=arm_clips,
                            last_clip=dict(bridge._boundary.last_clip),
                        )
                        clip_previous = arm_clips
                    bridge._boundary.arm_publish_gap_max_ms = 0.0
        while actor_evidence:
            evidence = actor_evidence.popleft()
            publish_observer("/internal/control/action_evidence", evidence, action_pub)
        errors = []
        for event in events:
            bridge.on_event(envelope("control_event", "ros_bridge", event), [])
            publish_observer("/events/control/intervention", event, events_pub)
            if event["kind"] in {
                "recording_failed",
                "heartbeat_timeout",
                "manual_input_timeout",
            } or (event["kind"] == "revoked" and event["state"] in {"FAULT", "ESTOP"}):
                errors.append(event)
            event_cursor = event["sequence"]
        if errors and not bridge._control_actor_mode:
            node.get_logger().error(
                "[control-event] " + json.dumps(errors, allow_nan=False)
            )
        if diagnostic is not None:
            bridge.on_event(envelope("control_diagnostic", "ros_bridge", diagnostic), [])
            if not bridge._control_actor_mode:
                node.get_logger().error(
                    "[control-diag] " + json.dumps(diagnostic, allow_nan=False)
                )
        if clip_diagnostic is not None:
            bridge.on_event(envelope("control_clip", "ros_bridge", clip_diagnostic), [])
        if state is not None:
            publish_observer("/status/control/ownership", state, state_pub)
            state_last = now
            state_sequence = sequence

    status_executor_node = status_node if status_node is not None else node
    status_executor_node.create_timer(
        0.1 if bridge._control_actor_mode else 0.05,
        bridge._timed_callback("ownership_status", publish_status),
        callback_group=(
            bridge._telemetry_callback_group
            if status_node is not None
            else bridge._ownership_callback_group
        ),
    )

    subscriptions.extend(
        [
            node.create_subscription(
                String,
                "/status/control/ack",
                bridge._timed_callback("ownership_ack", ack),
                100,
                callback_group=bridge._ownership_callback_group,
            ),
            node.create_subscription(
                String,
                "/events/teleop/clutch",
                bridge._timed_callback("ownership_input", semantic_input),
                100,
                callback_group=bridge._ownership_callback_group,
            ),
            (command_node or node).create_subscription(
                JointState,
                "/internal/control/joint_targets",
                bridge._timed_callback("owned_target", owned_target),
                10,
                callback_group=bridge._command_callback_group,
            ),
        ]
    )
    node.create_timer(
        0.02,
        bridge._timed_callback("ownership_tick", tick),
        callback_group=bridge._ownership_callback_group,
    )


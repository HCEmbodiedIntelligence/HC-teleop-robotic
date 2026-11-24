"""RosBridge 轨迹回放授权、平滑对齐规划与回放消息发布控制器。"""
from __future__ import annotations

import math
import time
from typing import Any, Callable

from ..recording.replay_alignment import HandAlignmentTrack, ReplayAlignmentPlanner


class ReplayBridgeController:
    """集中管理回放起止门控、起始位姿平滑对齐及回放后回零授权。"""

    @staticmethod
    def raw_replay_destination_allowed(topic: str) -> bool:
        topic = str(topic)
        return not (
            topic == "/vrdata"
            or topic.startswith(("/control/", "/status/", "/events/", "/internal/"))
            or (topic.startswith("/observation/") and topic.endswith("/joint_states"))
        )

    @staticmethod
    def replay_allowed_locked(bridge: Any, epoch: int) -> bool:
        return (
            not (bridge._control_actor_mode and bridge._stop.is_set())
            and epoch == bridge._output_epoch
            and bridge._replay_active
            and not bridge.ownership.active
            and bridge._command_output_enabled
            and not bridge._actor_output_blocked()
            and bridge._ready_locked(time.monotonic())
        )

    @staticmethod
    def begin_replay(bridge: Any) -> bool:
        with bridge._lock:
            if bridge.ownership.active:
                return False
            if not bridge.config.get("enabled", True):
                bridge._replay_active = True
                return True
            if (
                not bridge._command_output_enabled
                or bridge._actor_output_blocked()
                or not bridge._ready_locked(time.monotonic())
            ):
                return False
            bridge._actor_targets.clear()
            bridge._replay_home_available = False
            bridge._replay_active = True
            return True

    @staticmethod
    def end_replay(bridge: Any, require_reset: bool = True) -> None:
        with bridge._lock:
            can_return_home = (
                bridge._replay_active
                and require_reset
                and bridge._ready_locked(time.monotonic())
            )
            bridge._revoke_locked()
            bridge._replay_active = False
            if require_reset:
                # 必须显式恢复控制后，实时控制源才能重新获得执行器输出权。
                bridge._command_output_enabled = False
            bridge._replay_home_available = can_return_home

    @staticmethod
    def clear_alignment_locked(bridge: Any, reason: str = "") -> None:
        planner = bridge._replay_alignment
        bridge._replay_alignment = None
        if planner is not None and planner.state in {"aligning", "paused"}:
            if reason:
                planner.abort(reason)
            bridge._replay_alignment_status = planner.status_dict()

    @staticmethod
    def start_alignment(
        bridge: Any,
        arm_raw: bytes,
        hand_raws: dict[str, bytes] | None = None,
    ) -> dict[str, Any]:
        with bridge._lock:
            epoch = bridge._output_epoch
            if not bridge._replay_allowed_locked(epoch):
                raise ValueError("replay output is not active or hardware is not ready")
            boundary = bridge._boundary
            legacy = bridge._legacy_enabled
        if not legacy or boundary is None:
            with bridge._lock:
                bridge._replay_alignment = None
                bridge._replay_alignment_status = {
                    "state": "aligned",
                    "error": "",
                    "max_diff_rad": 0.0,
                }
                return dict(bridge._replay_alignment_status)

        from rclpy.serialization import deserialize_message
        from sensor_msgs.msg import JointState

        arm_msg = deserialize_message(arm_raw, JointState)
        if bridge._public_api is not None:
            bridge._public_api.convert_message(arm_msg, to_model=True)

        wall_ns = boundary.clock_now_ns()
        with bridge._lock:
            last_stamp = max(
                (boundary.last_command_stamp.get(n, 0) for n in boundary.limits),
                default=0,
            )
        stamp_ns = max(wall_ns, last_stamp + 1)
        arm_msg.header.stamp.sec, arm_msg.header.stamp.nanosec = divmod(
            stamp_ns, 1_000_000_000
        )
        prepared_arm = boundary.prepare(arm_msg)
        if prepared_arm is None:
            raise ValueError(
                f"replay initial pose rejected: {boundary.last_rejection or 'invalid pose'}"
            )
        arm_target_msg = prepared_arm[0]
        arm_names = tuple(arm_target_msg.name)
        goal_positions = {
            name: float(pos)
            for name, pos in zip(arm_target_msg.name, arm_target_msg.position)
        }

        control = boundary.teleop.get("control", {})
        configured_home = control.get("home_joint_names")
        candidates = (
            list(configured_home)
            if configured_home is not None
            else list(boundary.arms)
        )
        align_joints = tuple(
            n
            for n in candidates
            if n in goal_positions and math.isfinite(goal_positions[n])
        )
        if not align_joints:
            align_joints = tuple(
                n
                for n in boundary.arms
                if n in goal_positions and math.isfinite(goal_positions[n])
            )

        hand_tracks: dict[str, HandAlignmentTrack] = {}
        grippers = boundary.teleop.get("grippers", {})
        topic_to_side = {
            str(grippers[s]["finger_topic"]): s
            for s in boundary.hands
            if s in grippers and "finger_topic" in grippers[s]
        }
        for h_topic, h_raw in (hand_raws or {}).items():
            side = topic_to_side.get(str(h_topic))
            if side is None:
                continue
            h_msg = deserialize_message(h_raw, JointState)
            if bridge._public_api is not None:
                bridge._public_api.convert_message(h_msg, to_model=True)
            h_msg.header.stamp.sec, h_msg.header.stamp.nanosec = divmod(
                stamp_ns, 1_000_000_000
            )
            prepared_hand = boundary.prepare(h_msg)
            if prepared_hand is None:
                raise ValueError(
                    f"replay initial {side} hand pose rejected: {boundary.last_rejection or 'invalid pose'}"
                )
            hand_target = prepared_hand[0]
            h_names = tuple(hand_target.name)
            hand_set = set(boundary.hands[side])
            h_goal: dict[str, float] = {}
            h_extras: dict[str, float] = {}
            for name, pos in zip(h_names, hand_target.position):
                val = float(pos)
                if name in hand_set and math.isfinite(val):
                    h_goal[name] = val
                else:
                    h_extras[name] = val
            with bridge._lock:
                h_cur = {}
                for name, g_val in h_goal.items():
                    last_cmd = boundary.last_command.get(name)
                    if last_cmd is not None and math.isfinite(float(last_cmd)):
                        h_cur[name] = float(last_cmd)
                    elif name in boundary.positions and boundary.fresh(
                        (name,), now=time.monotonic(), wall_ns=boundary.clock_now_ns()
                    ):
                        h_cur[name] = float(boundary.positions[name])
                    else:
                        h_cur[name] = g_val
            hand_tracks[side] = HandAlignmentTrack(
                side=side,
                topic=str(h_topic),
                joint_names=h_names,
                goal=h_goal,
                current=h_cur,
                extras=h_extras,
            )

        now_mono = time.monotonic()
        with bridge._lock:
            if not bridge._replay_allowed_locked(epoch):
                raise ValueError("replay output revoked while preparing alignment")
            if not boundary.fresh(
                set(align_joints), now=now_mono, wall_ns=boundary.clock_now_ns()
            ):
                raise ValueError("required feedback missing/stale for replay alignment")
            start_positions = {n: float(boundary.positions[n]) for n in align_joints}
            home_vel = float(control.get("home_joint_velocity", 0.5))
            home_acc = float(control.get("home_joint_acceleration", 1.0))
            settle_tol = float(control.get("home_tolerance", 0.05))
            planner = ReplayAlignmentPlanner(
                epoch=epoch,
                arm_topic=bridge._mux_output_topic,
                arm_joint_names=arm_names,
                goal_positions=goal_positions,
                align_joints=align_joints,
                start_positions=start_positions,
                hand_tracks=hand_tracks,
                max_velocity=min(home_vel, 0.5),
                max_acceleration=min(home_acc, 1.0),
                hand_velocity=min(float(grippers.get("max_velocity", 3.5)), 2.5),
                start_tolerance=min(settle_tol, 0.03),
                settle_tolerance=settle_tol,
                max_tracking_error=boundary.max_error,
                settle_timeout=float(control.get("home_feedback_timeout", 2.0)),
                now_mono=now_mono,
            )
            bridge._replay_alignment_status = planner.status_dict()
            bridge._replay_alignment = None if planner.state == "aligned" else planner
            wake = bridge._actor_wake if bridge._replay_alignment is not None else None
        if callable(wake):
            wake()
        return dict(bridge._replay_alignment_status)

    @staticmethod
    def alignment_status(bridge: Any) -> dict[str, Any]:
        with bridge._lock:
            if bridge._replay_alignment is not None:
                return bridge._replay_alignment.status_dict()
            return dict(bridge._replay_alignment_status)

    @staticmethod
    def pause_alignment(bridge: Any) -> dict[str, Any]:
        with bridge._lock:
            if bridge._replay_alignment is not None:
                bridge._replay_alignment.pause()
                bridge._replay_alignment_status = bridge._replay_alignment.status_dict()
            return dict(bridge._replay_alignment_status)

    @staticmethod
    def resume_alignment(bridge: Any) -> dict[str, Any]:
        with bridge._lock:
            if bridge._replay_alignment is not None:
                bridge._replay_alignment.resume(time.monotonic())
                bridge._replay_alignment_status = bridge._replay_alignment.status_dict()
                wake = bridge._actor_wake
            else:
                wake = None
        if callable(wake):
            wake()
        return dict(bridge._replay_alignment_status)

    @staticmethod
    def cancel_alignment(bridge: Any) -> dict[str, Any]:
        with bridge._lock:
            bridge._clear_replay_alignment_locked("replay alignment cancelled")
            return dict(bridge._replay_alignment_status)

    @staticmethod
    def replay_home_allowed_locked(bridge: Any, *, output_resumed: bool) -> bool:
        return (
            not bridge._stop.is_set()
            and not bridge._actor_output_blocked()
            and not bridge._replay_active
            and not bridge.ownership.active
            and bridge._command_output_enabled == output_resumed
            and bridge._command_source == "vr"
            and bridge._ready_locked(time.monotonic())
            and (
                not bridge._legacy_enabled
                or (bridge._boundary is not None and bridge._boundary.feedback_fresh())
            )
        )

    @staticmethod
    def request_replay_home(bridge: Any, *, output_resumed: bool = False) -> bool:
        with bridge._lock:
            if not bridge._replay_home_allowed_locked(output_resumed=output_resumed):
                raise ValueError(
                    "回初始位姿需要 VR/IK 控制源、可用的控制权限和新鲜的硬件反馈"
                )
            if not output_resumed and not bridge._replay_home_available:
                raise ValueError("回放已结束，请先人工恢复控制，再回初始位姿")
            if (
                bridge._replay_home_publisher is None
                or not bridge._replay_home_publisher.get_subscription_count()
            ):
                raise ValueError("回零控制节点尚未连接，请稍后重试")
            if not bridge._enqueue("replay_home", (bridge._output_epoch, output_resumed)):
                return False
            bridge._replay_home_available = False
            return True

    @staticmethod
    def select_replay_publisher(
        bridge: Any,
        node: Any,
        publishers: dict[tuple[str, str], Any],
        topic: str,
        msg_type_name: str,
        message_type: Any,
    ) -> Any:
        boundary = bridge._boundary
        if bridge._legacy_enabled and boundary is not None:
            if (
                topic == bridge._mux_output_topic
                and getattr(boundary, "output", None) is not None
            ):
                return boundary.output
            hand_pubs = getattr(boundary, "hand_pubs", None)
            grippers = getattr(boundary, "teleop", {}).get("grippers", {})
            if isinstance(hand_pubs, dict):
                for side in boundary.hands:
                    cfg = grippers.get(side)
                    if (
                        isinstance(cfg, dict)
                        and str(cfg.get("finger_topic", "")) == str(topic)
                        and hand_pubs.get(side) is not None
                    ):
                        return hand_pubs[side]
        return bridge._publisher(node, publishers, topic, msg_type_name, message_type)

    @staticmethod
    def publish_replay_joint_message(
        bridge: Any,
        node: Any,
        publishers: dict[tuple[str, str], Any],
        topic: str,
        msg_type_name: str,
        message_type: Any,
        message: Any,
        *,
        epoch: int,
        convert_public: bool = True,
    ) -> bool:
        if convert_public and bridge._public_api is not None:
            bridge._public_api.convert_message(message, to_model=True)
        boundary = bridge._boundary
        if node is not None:
            clock_now = node.get_clock().now()
            if hasattr(clock_now, "to_msg"):
                message.header.stamp = clock_now.to_msg()
            else:
                ns = int(getattr(clock_now, "nanoseconds", time.time_ns()))
                message.header.stamp.sec, message.header.stamp.nanosec = divmod(
                    ns, 1_000_000_000
                )
        elif boundary is not None:
            wall_ns = boundary.clock_now_ns()
            with bridge._lock:
                last_stamp = max(
                    (boundary.last_command_stamp.get(n, 0) for n in boundary.limits),
                    default=0,
                )
            stamp_ns = max(wall_ns, last_stamp + 1)
            message.header.stamp.sec, message.header.stamp.nanosec = divmod(
                stamp_ns, 1_000_000_000
            )

        publisher = bridge._select_replay_publisher(
            node, publishers, topic, msg_type_name, message_type
        )
        prepared = boundary.prepare(message) if boundary else message
        out_msg = (
            (
                prepared[0]
                if (bridge._legacy_enabled and boundary is not None)
                else prepared
            )
            if prepared is not None
            else None
        )

        send, wire_cdr = publisher.publish, None
        prepare_publication = getattr(boundary, "prepare_publication", None)
        if bridge._legacy_enabled and out_msg is not None and callable(prepare_publication):
            try:
                send, wire_cdr = prepare_publication(out_msg, publisher.publish)
            except Exception as exc:
                boundary.reject("command serialization failed: " + str(exc))
                prepared = out_msg = None

        capture = getattr(bridge, "_publication_capture", None)
        capture_target = (
            capture is not None
            and out_msg is not None
            and (
                str(topic) == getattr(capture, "TOPIC", "/action/joint_states")
                or (
                    bridge._legacy_enabled
                    and boundary is not None
                    and bool(set(getattr(out_msg, "name", ())).intersection(boundary.arms))
                )
            )
        )
        if capture_target and wire_cdr is None:
            try:
                from rclpy.serialization import serialize_message

                wire_cdr = serialize_message(out_msg)
            except Exception:
                wire_cdr = None

        bridge._actor_checkpoint(allow_snapshot=False)
        ticket = capture.reserve() if capture_target else None
        published, publication_ns, publication_monotonic = None, None, None
        try:
            with bridge._lock:
                if prepared is None or not bridge._replay_allowed_locked(epoch):
                    return False
                if boundary and not boundary.commit(prepared):
                    return False
                send(out_msg)
                published = out_msg
                publication_ns = time.time_ns()
                publication_monotonic = time.monotonic()
                bridge._record_forwarded_metrics_locked(out_msg, publication_monotonic)
                return True
        finally:
            bridge._complete_publication_capture(
                capture, ticket, published, wire_cdr, publication_ns, publication_monotonic
            )

    @staticmethod
    def tick_alignment(
        bridge: Any,
        node: Any,
        publishers: dict[tuple[str, str], Any],
        get_message: Callable[[str], Any] | None = None,
    ) -> None:
        with bridge._lock:
            planner = bridge._replay_alignment
            if planner is None:
                return
            boundary = bridge._boundary
            now_mono = time.monotonic()
            measured = dict(boundary.positions) if boundary is not None else {}
            feedback_fresh = bool(
                boundary is not None
                and boundary.fresh(
                    set(planner.align_joints),
                    now=now_mono,
                    wall_ns=boundary.clock_now_ns(),
                )
            )
            replay_allowed = bridge._replay_allowed_locked(planner.epoch)
            result = planner.step(
                now_mono,
                measured,
                feedback_fresh=feedback_fresh,
                replay_allowed=replay_allowed,
            )
            bridge._replay_alignment_status = planner.status_dict()
            if planner.state in {"aligned", "error"}:
                bridge._replay_alignment = None
        if not result.get("publish"):
            return
        from sensor_msgs.msg import JointState

        arm_msg = JointState(
            name=list(planner.arm_joint_names),
            position=[float(v) for v in result["arm_positions"]],
        )
        bridge._publish_replay_joint_message(
            node,
            publishers,
            planner.arm_topic,
            "sensor_msgs/msg/JointState",
            JointState,
            arm_msg,
            epoch=planner.epoch,
            convert_public=False,
        )
        for hand_topic, (hand_names, hand_pos) in result.get(
            "hand_commands", {}
        ).items():
            hand_msg = JointState(
                name=list(hand_names),
                position=[float(v) for v in hand_pos],
            )
            bridge._publish_replay_joint_message(
                node,
                publishers,
                hand_topic,
                "sensor_msgs/msg/JointState",
                JointState,
                hand_msg,
                epoch=planner.epoch,
                convert_public=False,
            )


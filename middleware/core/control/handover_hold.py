"""DAgger 控制权交接期间的机械臂位姿保持控制器。"""
from __future__ import annotations

from dataclasses import dataclass
import time
from typing import Any

from .control_ownership import ControlToken


@dataclass
class _GripReturnHold:
    token: ControlToken
    output_epoch: int
    deadline_ns: int
    positions: dict[str, float]
    mode: str = "return"
    next_send_ns: int = 0
    sent: bool = False


_HANDOVER_HOLD_SPEC = {
    "return": dict(
        attr="_return_hold",
        begin_state="RETURNING",
        states=frozenset({"RETURNING", "AUTO"}),
        source="motion",
        frame_id="hc-dagger/return-hold",
        open_event="grip_return_hold_open",
        close_event="grip_return_hold_closed",
        close_reason="first model arm command published",
        require_grip=False,
        fault_on_deadline=True,
        deadline_reason="Grip return timed out waiting for the first model arm command",
        feedback_fault="Grip return hold requires fresh arm feedback",
        pose_fault="Grip return hold pose is outside the joint limits",
        stale_fault="Grip return hold feedback stale or outside the arm safety envelope",
    ),
    "takeover": dict(
        attr="_takeover_hold",
        begin_state="TAKEOVER",
        states=frozenset({"TAKEOVER", "MANUAL"}),
        source="vr",
        frame_id="hc-dagger/takeover-hold",
        open_event="grip_takeover_hold_open",
        close_event="grip_takeover_hold_closed",
        close_reason="first manual arm command published",
        require_grip=True,
        fault_on_deadline=False,
        deadline_reason="Grip takeover timed out waiting for the first manual arm command",
        feedback_fault="Grip takeover hold requires fresh arm feedback",
        pose_fault="Grip takeover hold pose is outside the joint limits",
        stale_fault="Grip takeover hold feedback stale or outside the arm safety envelope",
    ),
}
_HOLD_ATTR_BY_SOURCE = {"motion": "_return_hold", "vr": "_takeover_hold"}


class HandoverHoldController:
    """在 DAgger 接管/归还窗口内以 50 Hz 维持实测机械臂姿态，直至首帧新指令到来。"""

    @staticmethod
    def begin_hold(bridge: Any, mode: str) -> None:
        owner, boundary = bridge.ownership, bridge._boundary
        spec = _HANDOVER_HOLD_SPEC[mode]
        if (
            not bridge._legacy_enabled
            or boundary is None
            or not owner.dagger_enabled
            or bridge._actor_output_blocked()
            or not bridge._ready_locked(time.monotonic())
            or owner.state != spec["begin_state"]
            or not owner._input_fresh()
            or (spec["require_grip"] and not owner.gripped)
        ):
            return
        if not boundary.feedback_fresh():
            owner.fault(spec["feedback_fault"])
            return
        positions = {name: boundary.positions[name] for name in boundary.arms}
        if not boundary.return_hold_valid(positions):
            owner.fault(spec["pose_fault"])
            return
        setattr(
            bridge,
            spec["attr"],
            _GripReturnHold(
                owner.token,
                bridge._output_epoch,
                owner.deadline_ns,
                positions,
                mode=mode,
            ),
        )

    @staticmethod
    def hold_allowed_locked(bridge: Any, hold: _GripReturnHold) -> bool:
        owner, boundary = bridge.ownership, bridge._boundary
        spec = _HANDOVER_HOLD_SPEC[hold.mode]
        attr = spec["attr"]
        if (
            hold is not getattr(bridge, attr)
            or bridge._stop.is_set()
            or hold.token != owner.token
            or hold.output_epoch != bridge._output_epoch
            or owner.state not in spec["states"]
            or not owner.dagger_enabled
            or bridge._replay_active
            or bridge._actor_output_blocked()
            or not bridge._ready_locked(time.monotonic())
        ):
            return False
        if owner.clock() >= hold.deadline_ns:
            if spec["fault_on_deadline"] or owner.state == spec["begin_state"]:
                owner.fault(spec["deadline_reason"])
            else:
                setattr(bridge, attr, None)
                if hold.sent:
                    owner._event(spec["close_event"], end_reason=spec["deadline_reason"])
            return False
        if (
            not owner._input_fresh()
            or (spec["require_grip"] and not owner.gripped)
            or any(
                owner.clock() - owner.peer_times.get(peer, 0)
                > getattr(owner, "peer_heartbeat_timeout_ns", owner.input_timeout_ns)
                for peer in owner.required_peers
            )
        ):
            setattr(bridge, attr, None)
            return False
        if not boundary.return_hold_valid(hold.positions):
            owner.fault(spec["stale_fault"])
            return False
        return True

    @staticmethod
    def tick_hold(bridge: Any, attr: str) -> None:
        hold = getattr(bridge, attr)
        if hold is None or bridge.ownership.clock() < hold.next_send_ns:
            return
        boundary = bridge._boundary
        spec = _HANDOVER_HOLD_SPEC[hold.mode]
        message = boundary.return_hold_message(hold.positions, frame_id=spec["frame_id"])
        send, wire_cdr = boundary.prepare_publication(message, boundary.output.publish)
        capture = getattr(bridge, "_publication_capture", None)
        ticket = capture.reserve() if capture is not None else None
        published_ns = published_mono = None
        try:
            with bridge._lock:
                if (
                    not bridge._return_hold_allowed_locked(hold)
                    or bridge.ownership.clock() < hold.next_send_ns
                ):
                    return
                send(message)
                published_ns, published_mono = time.time_ns(), time.monotonic()
                hold.next_send_ns = bridge.ownership.clock() + 19_000_000
                boundary._arm_published()
                bridge._mux_channel_forwarded["arms"] += 1
                bridge._mux_channel_forwarded_times["arms"].append(published_mono)
                if not hold.sent:
                    hold.sent = True
                    bridge.ownership._event(spec["open_event"])
        finally:
            bridge._complete_publication_capture(
                capture,
                ticket,
                message if published_ns is not None else None,
                wire_cdr if published_ns is not None else None,
                published_ns,
                published_mono,
            )

    @staticmethod
    def handover_control_ready_locked(bridge: Any) -> bool:
        if not (bridge._ready_locked(time.monotonic()) and bridge._boundary):
            return False
        if bridge.ownership.dagger_enabled and bridge.ownership.state in {
            "TAKEOVER",
            "RETURNING",
        }:
            return bool(bridge._boundary.feedback_fresh())
        return bool(bridge._boundary.handover_ready())


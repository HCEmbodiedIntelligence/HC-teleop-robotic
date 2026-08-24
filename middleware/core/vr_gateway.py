from __future__ import annotations

import select
import socket
import threading
import time
from typing import Any, Callable

from .protocol import (
    DISCOVERY_REQUEST,
    PacketError,
    decode_pose_packet,
    encode_json_packet,
    envelope,
    is_sequence_newer,
    packet_loss_count,
)


class VrGateway:
    def __init__(
        self,
        config: dict[str, Any],
        on_pose: Callable[[Any], None],
        on_event: Callable[[dict[str, Any], list[str]], None],
        on_timeout: Callable[[str], None],
    ):
        self.config = config
        self.on_pose = on_pose
        self.on_event = on_event
        self.on_timeout = on_timeout
        self._stop = threading.Event()
        self._thread: threading.Thread | None = None
        self._outbound: socket.socket | None = None
        self._lock = threading.Lock()
        self._status: dict[str, Any] = {
            "state": "disabled" if not config.get("enabled", True) else "starting",
            "peer": None,
            "last_packet_at": None,
            "tracking": {"head": False, "left": False, "right": False},
            "inputs": {
                "left": {"held": [], "trigger": 0.0, "grip": 0.0},
                "right": {"held": [], "trigger": 0.0, "grip": 0.0},
            },
            "protocol_version": None,
            "controller_events": 0,
            "received": 0,
            "lost": 0,
            "old": 0,
            "invalid": 0,
            "sent": 0,
            "send_errors": 0,
            "timeout": False,
            "error": None,
        }

    def start(self) -> None:
        if not self.config.get("enabled", True):
            return
        self._thread = threading.Thread(target=self._run, name="vr-gateway", daemon=True)
        self._thread.start()

    def stop(self) -> None:
        self._stop.set()
        if self._thread is not None:
            self._thread.join(timeout=3)
        if self._outbound is not None:
            self._outbound.close()
            self._outbound = None

    def status(self) -> dict[str, Any]:
        with self._lock:
            result = dict(self._status)
            result["tracking"] = dict(self._status["tracking"])
            result["inputs"] = {
                side: dict(value) for side, value in self._status["inputs"].items()
            }
            return result

    def send_event(self, event: dict[str, Any]) -> bool:
        host = self.config.get("outbound_host") or ""
        if not host:
            current = self.status().get("peer")
            host = current[0] if current else ""
        if not host:
            return False
        try:
            payload = encode_json_packet(event)
            if len(payload) > 65507:
                raise ValueError("JSON event exceeds UDP datagram limit")
            if self._outbound is None:
                self._outbound = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self._outbound.sendto(payload, (host, int(self.config["outbound_port"])))
            with self._lock:
                self._status["sent"] += 1
            return True
        except (OSError, ValueError) as exc:
            with self._lock:
                self._status["send_errors"] += 1
                self._status["error"] = f"send failed: {exc}"
            return False

    def _run(self) -> None:
        pose_socket = discovery_socket = None
        try:
            pose_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            pose_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            pose_socket.bind((self.config["listen_host"], int(self.config["pose_port"])))
            pose_socket.setblocking(False)

            discovery_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            discovery_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            discovery_socket.bind(
                (self.config["listen_host"], int(self.config["discovery_port"]))
            )
            discovery_socket.setblocking(False)
            response = f"PICO_RECEIVER_V1|{self.config['pose_port']}".encode("ascii")
            self._set_status(state="running", error=None)

            last_sequence = None
            last_sender = None
            last_packet_time = time.monotonic()
            ever_received = False
            head_invalid_active = False
            timeout_seconds = float(self.config["pose_timeout_ms"]) / 1000.0

            while not self._stop.is_set():
                readable, _, _ = select.select(
                    [pose_socket, discovery_socket], [], [], min(0.05, timeout_seconds / 2)
                )
                for current_socket in readable:
                    packet, sender = current_socket.recvfrom(65535)
                    if current_socket is discovery_socket:
                        if packet.strip() == DISCOVERY_REQUEST:
                            discovery_socket.sendto(response, sender)
                        continue

                    now = time.monotonic()
                    try:
                        pose = decode_pose_packet(packet)
                    except PacketError:
                        self._increment("invalid")
                        continue
                    if last_sender is not None and sender != last_sender:
                        last_sequence = None
                    last_sender = sender
                    if not is_sequence_newer(pose.sequence, last_sequence):
                        self._increment("old")
                        continue
                    lost = packet_loss_count(pose.sequence, last_sequence)
                    last_sequence = pose.sequence
                    last_packet_time = now
                    ever_received = True
                    tracking = pose.as_dict()["tracking"]
                    inputs = pose.as_dict()["inputs"]
                    with self._lock:
                        self._status.update(
                            peer=[sender[0], sender[1]],
                            last_packet_at=time.time(),
                            tracking=tracking,
                            inputs=inputs,
                            protocol_version=pose.protocol_version,
                            received=self._status["received"] + 1,
                            lost=self._status["lost"] + lost,
                            timeout=False,
                        )
                    self.on_pose(pose)
                    self.on_event(
                        envelope("vr_pose", "vr_udp", pose.as_dict()), ["websocket"]
                    )
                    for side, controller_input in (
                        ("left", pose.left_input),
                        ("right", pose.right_input),
                    ):
                        if controller_input.pressed_mask or controller_input.released_mask:
                            event_payload = {
                                "side": side,
                                "sequence": pose.sequence,
                                "vr_timestamp": pose.vr_timestamp,
                                "held": controller_input.decode_buttons(
                                    controller_input.held_mask
                                ),
                                "pressed": controller_input.decode_buttons(
                                    controller_input.pressed_mask
                                ),
                                "released": controller_input.decode_buttons(
                                    controller_input.released_mask
                                ),
                                "pressed_mask": controller_input.pressed_mask,
                                "released_mask": controller_input.released_mask,
                            }
                            self._increment("controller_events")
                            self.on_event(
                                envelope(
                                    "vr_controller_event", "vr_udp", event_payload
                                ),
                                ["websocket"],
                            )

                    if not tracking["head"] and not head_invalid_active:
                        head_invalid_active = True
                        self.on_timeout("head tracking invalid")
                    elif tracking["head"]:
                        head_invalid_active = False

                elapsed = time.monotonic() - last_packet_time
                if ever_received and elapsed > timeout_seconds and not self.status()["timeout"]:
                    self._set_status(timeout=True)
                    self.on_timeout(f"no VR pose data for {elapsed * 1000:.0f} ms")
        except OSError as exc:
            self._set_status(state="error", error=f"{type(exc).__name__}: {exc}")
        finally:
            if pose_socket is not None:
                pose_socket.close()
            if discovery_socket is not None:
                discovery_socket.close()
            if self.status()["state"] != "error":
                self._set_status(state="stopped")

    def _set_status(self, **changes: Any) -> None:
        with self._lock:
            self._status.update(changes)

    def _increment(self, key: str) -> None:
        with self._lock:
            self._status[key] += 1

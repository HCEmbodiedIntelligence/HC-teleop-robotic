"""Separate adapter processes: VR -> core -> IK -> core -> backend.

Run with Python 3.10 from a ROS-enabled environment. No old middleware or
controller module is imported. Every process exits on core disconnect.
"""

import argparse
import asyncio
from collections import deque
from concurrent.futures import ThreadPoolExecutor
import json
import os
from pathlib import Path
import signal
import socket
import stat
import threading
import time

from .ipc import CoreConnection
from .mapping import BaseMapper, TeleopSolver
from .protocol import PRIMARY, SECONDARY
from .vr import VrGateway


class ObservationEmitter:
    """Optional datagram mirror; recording traffic never blocks motion IPC."""

    def __init__(self, directory):
        self.target = str(Path(directory) / "observations.sock") if directory else None
        self.socket = socket.socket(socket.AF_UNIX, socket.SOCK_DGRAM)
        self.socket.setblocking(False)
        self.sent = self.dropped = 0

    def emit(self, kind, payload):
        if self.target is None:
            return
        try:
            data = json.dumps(
                {"kind": kind, "payload": payload},
                separators=(",", ":"),
                allow_nan=False,
            ).encode()
            if len(data) > 65536:
                self.dropped += 1
                return
            self.socket.sendto(data, self.target)
            self.sent += 1
        except (OSError, ValueError):
            self.dropped += 1

    def status(self):
        return {"sent": self.sent, "dropped": self.dropped}

    def close(self):
        self.socket.close()


def write_status(directory, name, value):
    if directory is None:
        return
    path = Path(directory)
    path.mkdir(parents=True, exist_ok=True, mode=0o700)
    target = path / (name + ".json")
    temporary = path / (name + ".json.tmp")
    value = dict(
        value,
        pid=os.getpid(),
        updated_unix=time.time(),
        updated_mono_ns=time.monotonic_ns(),
    )
    temporary.write_text(
        json.dumps(value, ensure_ascii=False, allow_nan=False), encoding="utf-8"
    )
    temporary.replace(target)


class _Events(asyncio.DatagramProtocol):
    def __init__(self, receive):
        self.receive = receive
        self.errors = 0

    def datagram_received(self, data, address):
        try:
            if len(data) > 8192:
                return
            event = json.loads(data)
            if isinstance(event, dict) and isinstance(event.get("kind"), str):
                self.receive(event)
        except (ValueError, TypeError):
            self.errors += 1


async def event_channel(directory, receive):
    if directory is None:
        return None, None
    path = Path(directory) / "vr-events.sock"
    if len(os.fsencode(path)) >= 108:
        raise ValueError("Unix socket path too long; use a short runtime_state_dir")
    path.parent.mkdir(parents=True, exist_ok=True, mode=0o700)
    if path.exists() or path.is_symlink():
        info = path.lstat()
        if not stat.S_ISSOCK(info.st_mode) or info.st_uid != os.getuid():
            raise ValueError("refusing to replace non-owned event socket")
        path.unlink()
    transport, _ = await asyncio.get_running_loop().create_datagram_endpoint(
        lambda: _Events(receive), family=socket.AF_UNIX, local_addr=str(path)
    )
    path.chmod(0o600)
    return transport, path


async def run_vr(config, socket_path, state_dir, shutdown):
    queue, errors = deque(), deque()
    core = None
    event_transport = None
    outgoing_sequence = 0
    previous_held = {"left": 0, "right": 0}
    last_home_gesture = False
    body_mapper = BaseMapper(config)
    control = config.get("teleop", {}).get("control", {})
    grip_threshold = float(control.get("clutch_threshold", 0.55))
    safety = None
    management_state = {}
    observations = ObservationEmitter(state_dir)

    def frame_received(frame):
        if len(queue) >= 64:
            queue.clear()
            errors.append("VR_EVENT_QUEUE_OVERFLOW")
        else:
            queue.append(frame)

    def disconnected(reason):
        if len(errors) < 8:
            errors.append(reason)

    gateway = VrGateway(
        frame_received,
        disconnected,
        host=config.get("vr_bind_host", "0.0.0.0"),
        pose_port=int(config.get("vr_udp_port", 5005)),
        discovery_port=int(config.get("discovery_port", 5006)),
        event_port=int(config.get("status_port", 5007)),
        input_timeout_s=float(config.get("vr_timeout_ms", 200)) / 1000,
    )

    def core_message(message):
        nonlocal safety, management_state
        if message.get("type") == "state":
            state = message.get("state", message)
            management_state = state
            new = state.get("safety")
            if new != safety:
                safety = new
                gateway.send_event(
                    "safety_resume" if new == "ready" else "safety_stop",
                    {"reason": state.get("reason", new or "stopped")},
                )

    def api_event(event):
        kind = event["kind"]
        if not kind.startswith(("recording_", "replay_", "safety_")):
            return
        payload = dict(event.get("payload", {}))
        if kind == "recording_started":
            payload["recording"] = True
        elif kind == "recording_stopped":
            payload["recording"] = False
        gateway.send_event(
            kind,
            payload,
            latch="recording"
            if kind in ("recording_started", "recording_stopped", "recording_status")
            else "replay"
            if kind in ("replay_started", "replay_stopped", "replay_status", "replay_reset")
            else None,
        )

    def api_command(kind, accepted):
        if event_transport is None:
            return
        event = {
            "kind": kind,
            "source": "vr",
            "sequence": outgoing_sequence,
            "sample_mono_ns": accepted.received_ns,
            "peer": accepted.peer,
            "payload": {},
            "boot_id": management_state.get("boot_id"),
            "epoch": management_state.get("epoch"),
        }
        try:
            event_transport.sendto(
                json.dumps(event).encode(), str(Path(state_dir) / "api-events.sock")
            )
        except (OSError, RuntimeError):
            gateway.send_event(
                "recording_error" if kind.startswith("record_") else "safety_stop",
                {
                    "message": "management endpoint unavailable",
                    "reason": "management endpoint unavailable",
                },
            )

    core = CoreConnection(socket_path, "vr", core_message)
    await core.start()
    try:
        await gateway.start()
        event_transport, event_path = await event_channel(state_dir, api_event)
        last_status = 0.0
        while not shutdown.is_set() and not core.closed.is_set():
            while errors:
                reason = errors.popleft()
                await core.send({"op": "stop", "reason": reason})
                gateway.send_event("safety_stop", {"reason": reason})
            if not queue:
                await asyncio.sleep(0.002)
            while queue:
                accepted = queue.popleft()
                frame = accepted.frame
                outgoing_sequence += 1
                left = frame.left_input.grip >= grip_threshold
                right = frame.right_input.grip >= grip_threshold
                velocity, independent = body_mapper.map(frame, left)
                normalized = {
                    "op": "vr",
                    "seq": outgoing_sequence,
                    "sample_seq": accepted.sample_seq,
                    "sample_mono_ns": accepted.sample_mono_ns,
                    "source_frame": "unity_tracking",
                    "tracked": {
                        side: frame.tracked(side) for side in ("head", "left", "right")
                    },
                    "grip": {"left": left, "right": right},
                    "trigger": {
                        "left": frame.left_input.trigger,
                        "right": frame.right_input.trigger,
                    },
                    "head_pose": frame.head.as_list(),
                    "left_pose": frame.left.as_list(),
                    "right_pose": frame.right.as_list(),
                    "base_velocity": velocity,
                    "base_independent": independent,
                }
                await core.send(normalized)
                observations.emit(
                    "vr",
                    dict(
                        normalized,
                        sample_unix_ns=time.time_ns(),
                        left_input={
                            "held": frame.left_input.held,
                            "pressed": frame.left_input.pressed,
                            "released": frame.left_input.released,
                            "trigger": frame.left_input.trigger,
                            "grip": frame.left_input.grip,
                            "primary_axis": frame.left_input.primary_axis,
                        },
                        right_input={
                            "held": frame.right_input.held,
                            "pressed": frame.right_input.pressed,
                            "released": frame.right_input.released,
                            "trigger": frame.right_input.trigger,
                            "grip": frame.right_input.grip,
                            "primary_axis": frame.right_input.primary_axis,
                        },
                    ),
                )
                # First packet of a new IP lease cannot replay queued UI edges.
                if accepted.new_session:
                    previous_held = {
                        "left": frame.left_input.held,
                        "right": frame.right_input.held,
                    }
                    continue
                for side, input_value in (
                    ("left", frame.left_input),
                    ("right", frame.right_input),
                ):
                    pressed = input_value.pressed | (
                        input_value.held & ~previous_held[side]
                    )
                    previous_held[side] = input_value.held
                    if not accepted.fresh_sample or not frame.tracked(side):
                        continue
                    if side == "right":
                        if pressed & SECONDARY:
                            await core.send({"op": "stop", "reason": "VR B button"})
                        elif pressed & PRIMARY:
                            api_command("control_recover", accepted)
                    else:
                        if pressed & SECONDARY:
                            api_command("record_stop", accepted)
                        elif pressed & PRIMARY:
                            api_command("record_toggle", accepted)
                    if pressed & (1 << 5):
                        api_command("record_mark", accepted)
                home_gesture = (
                    frame.left_input.primary_axis[0] < -0.7
                    and frame.right_input.primary_axis[0] > 0.7
                )
                if home_gesture and not last_home_gesture:
                    api_command("home", accepted)
                last_home_gesture = home_gesture
            if time.monotonic() - last_status > 0.25:
                state = gateway.status()
                state.update(
                    source_frame="unity_tracking",
                    core_connected=True,
                    observation_mirror=observations.status(),
                )
                write_status(state_dir, "vr", state)
                last_status = time.monotonic()
    finally:
        await gateway.close()
        await core.close()
        observations.close()
        if event_transport:
            event_transport.close()
        if "event_path" in locals() and event_path:
            event_path.unlink(missing_ok=True)
        write_status(state_dir, "vr", dict(gateway.status(), core_connected=False))


async def run_backend(config, socket_path, state_dir, shutdown, backend_kind):
    lock = threading.Lock()
    latest = None
    acks = deque()
    overflow = False
    outputs = {}
    observations = ObservationEmitter(
        state_dir if backend_kind == "simulation" else None
    )

    def feedback(message):
        nonlocal latest
        with lock:
            latest = message

    def applied(seq, accepted, reason=""):
        nonlocal overflow
        with lock:
            if len(acks) >= 128:
                overflow = True
            else:
                acks.append(
                    {
                        "op": "output_ack",
                        "seq": seq,
                        "accepted": accepted,
                        "reason": reason,
                        "sample_mono_ns": time.monotonic_ns(),
                    }
                )

    if backend_kind == "simulation":
        from .simulation import SimulationAdapter

        backend = SimulationAdapter(config, feedback)
    else:
        from .x1 import X1RosAdapter

        backend = X1RosAdapter(config, feedback)
    backend.on_output = applied
    if backend_kind == "simulation":
        backend.start()
    else:
        await asyncio.to_thread(backend.start)

    async def message_received(message):
        if message.get("type") == "fence":
            if backend_kind == "simulation":
                backend.fence(message["boot_id"], message["epoch"])
            else:
                await asyncio.to_thread(
                    backend.fence, message["boot_id"], message["epoch"]
                )
            await core.send(
                {
                    "op": "fence_ack",
                    "boot_id": message["boot_id"],
                    "epoch": message["epoch"],
                    "stopped": True,
                }
            )
        elif message.get("type") == "output":
            outputs[message["seq"]] = message
            while len(outputs) > 128:
                outputs.pop(next(iter(outputs)))
            backend.submit(message)

    core = CoreConnection(
        socket_path,
        "backend",
        message_received,
        {"fence_ack": True, "physical_stop_ack": False},
    )
    try:
        await core.start()
        last_status = 0.0
        while not shutdown.is_set() and not core.closed.is_set():
            started = time.monotonic()
            if backend_kind == "simulation":
                backend.step()
            with lock:
                current, latest = latest, None
                pending = list(acks)
                acks.clear()
                failed = overflow
            if failed:
                await core.send({"op": "stop", "reason": "BACKEND_AUDIT_OVERFLOW"})
                raise RuntimeError("backend output audit queue overflow")
            if current:
                await core.send(current)
                observations.emit("feedback", current)
            for ack in pending:
                await core.send(ack)
                output = outputs.pop(ack["seq"], None)
                if output:
                    observations.emit(
                        "action",
                        {
                            "output": output,
                            "ack": ack,
                            "sample_unix_ns": time.time_ns(),
                        },
                    )
            if started - last_status > 0.25:
                write_status(
                    state_dir,
                    "backend",
                    dict(
                        backend.status(),
                        core_connected=True,
                        observation_mirror=observations.status(),
                    ),
                )
                last_status = started
            period = backend.dt if backend_kind == "simulation" else 0.005
            await asyncio.sleep(max(0.0001, period - (time.monotonic() - started)))
    finally:
        if backend_kind == "simulation":
            backend.stop()
            backend.close()
        else:
            await asyncio.to_thread(backend.close)
        await core.close()
        observations.close()
        write_status(state_dir, "backend", dict(backend.status(), core_connected=False))


async def run_ik(config, socket_path, state_dir, shutdown):
    solver = TeleopSolver(config)
    latest = None
    wakeup = asyncio.Event()

    def receive(message):
        nonlocal latest
        if message.get("type") == "ik_request":
            latest = message
            wakeup.set()

    core = CoreConnection(socket_path, "ik", receive)
    executor = ThreadPoolExecutor(max_workers=1, thread_name_prefix="bounded-ik")
    await core.start()
    try:
        last_status = 0.0
        while not shutdown.is_set() and not core.closed.is_set():
            if latest is None:
                try:
                    await asyncio.wait_for(wakeup.wait(), 0.1)
                except asyncio.TimeoutError:
                    pass
            wakeup.clear()
            request, latest = latest, None
            if request:
                try:
                    result = await asyncio.get_running_loop().run_in_executor(
                        executor, solver.solve, request
                    )
                    if result and time.monotonic_ns() < request["deadline_mono_ns"]:
                        await core.send(result)
                except (ValueError, KeyError, ArithmeticError) as error:
                    solver.last_status = {
                        "ready": False,
                        "reason": "INVALID_IK_REQUEST",
                        "error": str(error),
                    }
                    await core.send({"op": "stop", "reason": "IK_WORKER_ERROR"})
            if time.monotonic() - last_status > 0.25:
                write_status(
                    state_dir, "ik", dict(solver.last_status, core_connected=True)
                )
                last_status = time.monotonic()
    finally:
        executor.shutdown(wait=True, cancel_futures=True)
        solver.close()
        await core.close()
        write_status(state_dir, "ik", dict(solver.last_status, core_connected=False))


async def main_async(args):
    config = json.loads(Path(args.config).read_text())
    state_dir = args.state_dir or config.get("runtime_state_dir")
    shutdown = asyncio.Event()
    for signum in (signal.SIGINT, signal.SIGTERM):
        asyncio.get_running_loop().add_signal_handler(signum, shutdown.set)
    if args.role == "vr":
        await run_vr(config, args.socket, state_dir, shutdown)
    elif args.role == "ik":
        await run_ik(config, args.socket, state_dir, shutdown)
    else:
        await run_backend(config, args.socket, state_dir, shutdown, args.backend)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--role", required=True, choices=("vr", "ik", "backend"))
    parser.add_argument("--config", required=True)
    parser.add_argument("--socket", required=True)
    parser.add_argument("--state-dir")
    parser.add_argument("--backend", choices=("simulation", "x1"), default="simulation")
    args = parser.parse_args()
    asyncio.run(main_async(args))


if __name__ == "__main__":
    main()

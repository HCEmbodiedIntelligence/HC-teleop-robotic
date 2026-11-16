#!/usr/bin/env python3
"""Real process/socket regression; no ROS or robot hardware required."""
import json
import os
import select
import socket
import subprocess
import sys
import tempfile
import time
from pathlib import Path


def main(binary):
    with tempfile.TemporaryDirectory(prefix="hc-control-test-") as tmp:
        root = Path(tmp)
        config = root / "robot.json"
        config.write_text(json.dumps({
            "robot_id": "fixture", "config_hash": "fixture-v1",
            "joint_names": ["left", "right"], "lower": [-2, -2], "upper": [2, 2],
            "velocity": [0.6, 0.6], "acceleration": [2.4, 2.4], "home": [0, 0],
            "groups": {"left_arm": ["left"], "right_arm": ["right"]},
            "feedback_timeout_ms": 500, "command_timeout_ms": 150,
        }))
        path = root / "control.sock"
        process = subprocess.Popen([binary, "--socket", str(path), "--config", str(config)], stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
        sockets = []
        try:
            deadline = time.monotonic() + 5
            while not path.exists():
                if process.poll() is not None:
                    raise AssertionError(process.communicate())
                if time.monotonic() > deadline:
                    raise TimeoutError("socket startup")
                time.sleep(.01)
            assert path.stat().st_mode & 0o777 == 0o600
            def connect():
                sock = socket.socket(socket.AF_UNIX, socket.SOCK_SEQPACKET)
                sock.connect(str(path)); sock.setblocking(False); sockets.append(sock); return sock
            api, backend, program, duplicate, recorder = [connect() for _ in range(5)]
            def send(sock, obj): sock.send(json.dumps(obj).encode())
            seq = 0
            outputs = []
            audit = []
            def receive_until(sock, request_id, timeout=2):
                nonlocal seq
                end = time.monotonic() + timeout
                feedback_at = 0
                while time.monotonic() < end:
                    now = time.monotonic()
                    if feedback_at and now >= feedback_at:
                        seq += 1
                        send(backend, {"op":"feedback", "seq":seq, "joint_names":["left","right"], "positions":[0,0], "ready":True, "sample_mono_ns":time.monotonic_ns()})
                        feedback_at = now + .05
                    ready, _, _ = select.select(sockets, [], [], .02)
                    for current in ready:
                        data = current.recv(65536)
                        if not data: raise AssertionError("unexpected disconnect")
                        event = json.loads(data)
                        if event.get("type") == "fence" and current is backend:
                            send(backend, {"op":"fence_ack", "boot_id":event["boot_id"], "epoch":event["epoch"], "stopped":True})
                        if current is recorder:
                            if event.get("type") == "output": outputs.append(event)
                            if event.get("type") == "ownership_transition": audit.append(event)
                        if event.get("type") == "response" and event.get("id") == "backend":
                            feedback_at = now + .05
                        if current is sock and event.get("type") == "response" and event.get("id") == request_id:
                            return event
                raise TimeoutError(request_id)
            for sock, role in [(api,"api"),(backend,"backend"),(program,"program"),(recorder,"recorder")]:
                msg = {"op":"hello", "id":role, "role":role}
                if role == "backend": msg["capabilities"]={"fence_ack":True,"physical_stop_ack":False}
                send(sock,msg); assert receive_until(sock,role)["ok"]
            send(duplicate,{"op":"hello","id":"duplicate","role":"backend","capabilities":{"fence_ack":True}})
            assert receive_until(duplicate,"duplicate")["code"] == "ROLE_ALREADY_OWNED"
            def feedback():
                nonlocal seq
                seq += 1
                send(backend,{"op":"feedback","id":"fb","seq":seq,"joint_names":["left","right"],"positions":[0,0],"ready":True,"sample_mono_ns":time.monotonic_ns()})
                assert receive_until(backend,"fb")["ok"]
            feedback()
            send(api,{"op":"recover","id":"recover"}); assert receive_until(api,"recover")["ok"]
            def state_until(predicate):
                for i in range(100):
                    send(api,{"op":"status","id":f"state{i}"})
                    state = receive_until(api,f"state{i}")["state"]
                    if predicate(state): return state
                    time.sleep(.005)
                raise AssertionError(state)
            state_until(lambda s:s["safety"]=="ready" and not s["transition_pending"])
            send(api,{"op":"session_start","id":"start","mode":"program"})
            assert receive_until(api,"start")["ok"]
            state = state_until(lambda s:s["owner"]=="program")
            command={"op":"command","id":"command","source":"program","seq":1,"ttl_ms":100,"joint_names":["left"],"positions":[0.3]}
            for field in ["boot_id","epoch","lease_id","reference_epoch"]: command[field]=state[field]
            send(program,command); assert receive_until(program,"command")["ok"]
            send(program,{**command,"id":"wrong","positions":[100],"seq":2})
            assert receive_until(program,"wrong")["code"] == "JOINT_LIMIT"
            send(api,{"op":"stop","id":"stop"}); assert receive_until(api,"stop")["ok"]
            state_until(lambda s:s["safety"]=="stopped")
            send(program,{**command,"id":"stale","seq":3})
            assert not receive_until(program,"stale")["ok"]
            # Parser rejects duplicate names before they can reach authority.
            send(backend,{"op":"feedback","id":"duplicate-joints","seq":seq+1,"joint_names":["left","left"],"positions":[0,0],"ready":True})
            assert receive_until(backend,"duplicate-joints")["code"] == "DUPLICATE_JOINT"
            # Unprivileged role cannot bypass the core's management permissions.
            send(program,{"op":"recover","id":"forbidden"})
            assert receive_until(program,"forbidden")["code"] == "FORBIDDEN_ROLE"
            state_until(lambda s:s["safety"]=="stopped")
            assert outputs and all(o["deadline_mono_ns"] >= o["issued_mono_ns"] for o in outputs)
            assert any(e["new_owner"] == "program" for e in audit)
            # Another process cannot steal/unlink the live socket.
            other=subprocess.run([binary,"--socket",str(path),"--config",str(config)],capture_output=True,text=True,timeout=3)
            assert other.returncode != 0 and path.exists()
            print("IPC: permissions, exclusive backend, fencing, roles, recording, malformed data passed")
        finally:
            for sock in sockets: sock.close()
            process.terminate()
            try: process.wait(timeout=3)
            except subprocess.TimeoutExpired: process.kill(); process.wait()
            if process.returncode not in (0,-15):
                raise AssertionError(process.communicate())


if __name__ == "__main__": main(sys.argv[1])

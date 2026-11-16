"""Small asyncio client. Each submitted action is a new, short-lived target."""

from __future__ import annotations
import math
import os
import time
import uuid
import httpx


class RobotError(RuntimeError):
    def __init__(self, response):
        self.response = response
        super().__init__(str(response))


class RobotClient:
    def __init__(self, url="http://127.0.0.1:7876", token=None):
        token = token if token is not None else os.environ.get("HC_PLATFORM_TOKEN", "")
        self.http = httpx.AsyncClient(
            base_url=url,
            headers={"Authorization": "Bearer " + token} if token else {},
            timeout=5,
            trust_env=False,
        )
        self.sequence = 0
        self.clock_anchor = None

    async def __aenter__(self):
        return self

    async def __aexit__(self, *args):
        await self.http.aclose()

    async def call(self, method, path, payload=None):
        response = await self.http.request(method, path, json=payload)
        try:
            result = response.json()
        except ValueError:
            result = {"code": "INVALID_SERVER_RESPONSE", "status": response.status_code}
        if response.is_error:
            raise RobotError(result)
        return result

    async def state(self):
        state = await self.call("GET", "/api/control/state")
        # Taking the server sample at response receipt is a conservative clock
        # estimate: network delay makes a target older, never newer.
        self.clock_anchor = (
            state["issued_mono_ns"],
            time.monotonic_ns(),
            state["boot_id"],
        )
        self.sequence = max(self.sequence, state.get("last_command_seq", 0))
        return state

    async def operation(self, op, **params):
        if op != "stop":
            state = await self.state()
            params.setdefault("expected_boot_id", state["boot_id"])
            params.setdefault("expected_epoch", state["epoch"])
        return await self.call(
            "POST",
            "/api/control/operations",
            {"op": op, "request_id": uuid.uuid4().hex, **params},
        )

    async def info(self):
        return await self.call("GET", "/api/robot/info")

    async def observations(self):
        return await self.call("GET", "/api/observations")

    async def joint_target(
        self,
        joint_names,
        positions,
        *,
        state,
        source="program",
        ttl_ms=100,
        sample_local_ns=None,
    ):
        """Supply a recently read state; stale authority is rejected, not retried.

        Pass sample_local_ns captured when the model produced the action. Reusing
        a result does not renew that result's original sample time.
        """
        if (
            len(joint_names) != len(positions)
            or not positions
            or not all(math.isfinite(v) for v in positions)
        ):
            raise ValueError("Invalid named joint target")
        if self.clock_anchor is None or self.clock_anchor[2] != state["boot_id"]:
            raise ValueError("Read fresh server state before sending targets")
        local = time.monotonic_ns() if sample_local_ns is None else int(sample_local_ns)
        sample_server_ns = self.clock_anchor[0] + local - self.clock_anchor[1]
        self.sequence = max(self.sequence, state.get("last_command_seq", 0)) + 1
        payload = {
            key: state[key]
            for key in ("boot_id", "epoch", "lease_id", "reference_epoch")
        }
        payload.update(
            source=source,
            seq=self.sequence,
            ttl_ms=ttl_ms,
            sample_mono_ns=sample_server_ns,
            deadline_mono_ns=sample_server_ns + int(ttl_ms * 1e6),
            joint_names=list(joint_names),
            positions=list(positions),
        )
        return await self.call("POST", "/api/control/command", payload)

    async def motion(self, command, **payload):
        paths = {
            "ServoJ": "servo-j",
            "ServoP": "servo-p",
            "MoveJ": "move-j",
            "MoveL": "move-l",
            "MoveP": "move-p",
        }
        if command not in paths:
            raise ValueError("Unknown Motion command")
        return await self.call("POST", "/api/motion/" + paths[command], payload)

    async def task(self, identity):
        return await self.call("GET", "/api/motion/tasks/" + identity)

    async def cancel_task(self, identity):
        return await self.call("POST", "/api/motion/tasks/" + identity + "/cancel", {})

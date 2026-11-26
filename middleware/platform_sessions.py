"""Async clients for the HC v1 robot/session API, independent of ROS, VR and WebRTC.

Both clients own their HTTP connection pool inside an async context manager.
An operator (GUI or policy actor) reserves a device, posts its SDP offer and
polls sync() within the platform lease until it releases the device in finally.
Use the same request_id and revision when retrying a capture command.
"""

from urllib.parse import quote, urlsplit

import aiohttp


def platform_url(value: str) -> str:
    base = value.rstrip("/")
    url = urlsplit(base)
    loopback = url.scheme == "http" and url.hostname in {
        "localhost",
        "127.0.0.1",
        "::1",
    }
    if (
        not url.hostname
        or (url.scheme != "https" and not loopback)
        or url.username is not None
        or url.password is not None
        or url.query
        or url.fragment
    ):
        raise ValueError("base_url must be an HTTPS API origin or loopback test server")
    return base


def _segment(value: str) -> str:
    if not value or value in {".", ".."} or "/" in value or "\\" in value:
        raise ValueError("expected one non-empty platform identifier")
    return quote(value, safe="")


class PlatformHTTPError(RuntimeError):
    def __init__(self, status: int):
        self.status = status
        super().__init__(f"platform session API returned HTTP {status}")


class _PlatformClient:
    def __init__(self, base_url: str, token: str):
        self._base = platform_url(base_url) + "/api/v1"
        if not token or not token.strip():
            raise ValueError("a platform bearer token is required")
        self._headers = {"Authorization": "Bearer " + token}
        self._http = None

    async def __aenter__(self):
        if self._http is not None:
            raise RuntimeError("platform client is already open")
        self._http = aiohttp.ClientSession(
            headers=self._headers, timeout=aiohttp.ClientTimeout(total=3)
        )
        return self

    async def __aexit__(self, *exc):
        await self._http.close()
        self._http = None

    async def _request(self, method, path, *, body=None, params=None):
        if self._http is None:
            raise RuntimeError("open the platform client with async with")
        async with self._http.request(
            method, self._base + path, json=body, params=params, allow_redirects=False
        ) as response:
            if not 200 <= response.status < 300:
                raise PlatformHTTPError(response.status)
            if response.status != 204:
                return await response.json()


class RobotPlatform(_PlatformClient):
    """Robot identity is derived from its credential by the platform."""

    async def sync(self, instance_id: str, name: str, **fields) -> dict:
        return await self._request(
            "POST",
            "/robot-ingest/teleoperation/sync",
            body=dict(instance_id=instance_id, name=name, **fields),
        )


class OperatorPlatform(_PlatformClient):
    """Organization-scoped discovery, signaling, leases and task-bound capture."""

    def __init__(self, base_url: str, token: str, organization_id: str):
        super().__init__(base_url, token)
        self._robots = (
            f"/organizations/{_segment(organization_id)}/teleoperation/robots"
        )
        self._headers["X-Organization-Id"] = organization_id

    def _robot(self, robot_id):
        return self._robots + "/" + _segment(robot_id)

    async def robots(self) -> list:
        return await self._request("GET", self._robots)

    async def reserve(self, robot_id: str) -> dict:
        return await self._request("POST", self._robot(robot_id) + "/sessions")

    async def sync(
        self, robot_id: str, session_id: str, offer: str | None = None
    ) -> dict:
        path = self._robot(robot_id) + "/sessions/" + _segment(session_id) + "/sync"
        return await self._request(
            "POST", path, body={} if offer is None else {"offer": offer}
        )

    async def release(self, robot_id: str, session_id: str) -> None:
        path = self._robot(robot_id) + "/sessions/" + _segment(session_id)
        await self._request("DELETE", path)

    async def capture_state(self, robot_id: str, project_id: str) -> dict:
        return await self._request(
            "GET", self._robot(robot_id) + "/capture", params={"project_id": project_id}
        )

    async def command_capture(
        self,
        robot_id: str,
        *,
        session_id: str,
        project_id: str,
        task_id: str,
        action: str,
        revision: int,
        request_id: str,
    ) -> dict:
        return await self._request(
            "POST",
            self._robot(robot_id) + "/capture",
            body={
                "session_id": session_id,
                "project_id": project_id,
                "task_id": task_id,
                "action": action,
                "revision": revision,
                "request_id": request_id,
            },
        )

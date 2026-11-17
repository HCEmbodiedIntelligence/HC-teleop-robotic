"""Private Unix-domain JSON RPC for the single data-service owner."""

import asyncio
import inspect
import json
import os
import time
import uuid
from pathlib import Path

from .artifacts import canonical
from .errors import DataError
from .service import DataService

MAX_RPC_BYTES = 24 * 1024 * 1024
METHODS = frozenset(
    {
        "status",
        "start_session",
        "append_sample",
        "append_samples",
        "set_intervention",
        "stop_session",
        "get_session",
        "list_sessions",
        "list_datasets",
        "get_dataset",
        "import_mcap",
        "acquire_artifact",
        "release_artifact",
        "resolve_asset",
        "prepare_download",
        "delete_dataset",
        "enqueue_conversion",
        "enqueue_video",
        "enqueue_upload",
        "get_job",
        "list_jobs",
        "job_events",
        "cancel_job",
        "retry_job",
        "configure_platform",
        "platform_status",
        "platform_tasks",
        "platform_preflight",
        "platform_prepare_recording",
        "connect_platform",
        "refresh_upload",
    }
)


class DataClient:
    def __init__(self, socket_path, *, timeout=60):
        self.socket_path = str(socket_path)
        self.timeout = timeout
        self.lock = asyncio.Lock()
        self.reader = None
        self.writer = None

    async def close(self):
        if self.writer:
            self.writer.close()
            try:
                await self.writer.wait_closed()
            except (ConnectionError, OSError):
                pass
        self.reader = self.writer = None

    async def call(self, method, params=None):
        if method not in METHODS:
            raise DataError("METHOD_NOT_FOUND")
        if method == "append_sample":
            # Timestamp before waiting for this connection's request lock.
            # Capture/core timestamps carried in the payload take precedence.
            params = dict(params or {})
            params.setdefault("sample_monotonic_ns", time.monotonic_ns())
        elif method == "append_samples":
            params = dict(params or {})
            now_mono = time.monotonic_ns()
            params["samples"] = [
                {"sample_monotonic_ns": now_mono, **item}
                if "sample_monotonic_ns" not in item
                else item
                for item in params.get("samples", ())
            ]
        async with self.lock:
            request_id = str(uuid.uuid4())
            request = (
                canonical(
                    {"id": request_id, "method": method, "params": params or {}}
                ).encode()
                + b"\n"
            )
            if len(request) > MAX_RPC_BYTES:
                raise DataError("RPC_REQUEST_TOO_LARGE")

            async def exchange():
                if self.writer is None or self.writer.is_closing():
                    self.reader, self.writer = await asyncio.open_unix_connection(
                        self.socket_path, limit=MAX_RPC_BYTES
                    )
                self.writer.write(request)
                await self.writer.drain()
                raw = await self.reader.readline()
                if not raw:
                    raise ConnectionError("data service disconnected")
                response = json.loads(raw)
                if response.get("id") != request_id:
                    raise DataError("RPC_RESPONSE_MISMATCH")
                if "error" in response:
                    value = response["error"]
                    raise DataError(
                        value["code"],
                        value.get("message", ""),
                        retryable=value.get("retryable", False),
                    )
                return response["result"]

            try:
                return await asyncio.wait_for(exchange(), self.timeout)
            except DataError:
                raise
            except (
                OSError,
                ConnectionError,
                ValueError,
                TimeoutError,
                asyncio.TimeoutError,
            ) as error:
                await self.close()
                raise DataError(
                    "DATA_SERVICE_UNAVAILABLE", type(error).__name__, retryable=True
                ) from None
            except asyncio.CancelledError:
                await self.close()
                raise

    def __getattr__(self, name):
        if name not in METHODS:
            raise AttributeError(name)

        async def invoke(*args, **kwargs):
            bound = inspect.signature(getattr(DataService, name)).bind(
                None, *args, **kwargs
            )
            return await self.call(
                name,
                {key: value for key, value in bound.arguments.items() if key != "self"},
            )

        return invoke


class DataServer:
    def __init__(self, service, socket_path):
        self.service, self.path = service, Path(socket_path)
        self.server = None
        self.writers = set()

    async def start(self):
        self.path.parent.mkdir(parents=True, exist_ok=True)
        if self.path.exists():
            try:
                _, writer = await asyncio.wait_for(
                    asyncio.open_unix_connection(str(self.path)), 1
                )
            except (OSError, TimeoutError, asyncio.TimeoutError):
                self.path.unlink()
            else:
                writer.close()
                await writer.wait_closed()
                raise DataError("DATA_SOCKET_IN_USE")
        self.server = await asyncio.start_unix_server(
            self._handle, path=str(self.path), limit=MAX_RPC_BYTES
        )
        os.chmod(self.path, 0o600)
        return self

    async def close(self):
        if self.server:
            self.server.close()
            await self.server.wait_closed()
        for writer in list(self.writers):
            writer.close()
        self.path.unlink(missing_ok=True)

    async def _handle(self, reader, writer):
        self.writers.add(writer)
        try:
            while True:
                try:
                    line = await reader.readline()
                except (ValueError, asyncio.LimitOverrunError):
                    break
                if not line:
                    break
                identity = None
                try:
                    request = json.loads(line)
                    identity = request.get("id")
                    method = request.get("method")
                    if method not in METHODS or not isinstance(
                        request.get("params", {}), dict
                    ):
                        raise DataError("METHOD_NOT_FOUND")
                    result = await getattr(self.service, method)(
                        **request.get("params", {})
                    )
                    response = {"id": identity, "result": result}
                except DataError as error:
                    response = {"id": identity, "error": error.as_dict()}
                except (TypeError, ValueError, KeyError):
                    response = {
                        "id": identity,
                        "error": {
                            "code": "RPC_REQUEST_INVALID",
                            "message": "Invalid method parameters",
                        },
                    }
                except Exception as error:
                    response = {
                        "id": identity,
                        "error": {
                            "code": "DATA_OPERATION_FAILED",
                            "message": type(error).__name__,
                        },
                    }
                writer.write(canonical(response).encode() + b"\n")
                await writer.drain()
        except (ConnectionError, OSError):
            pass
        finally:
            self.writers.discard(writer)
            writer.close()
            try:
                await writer.wait_closed()
            except (ConnectionError, OSError):
                pass

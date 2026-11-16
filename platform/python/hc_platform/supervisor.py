"""Supervise only owned process groups; no shell commands or global process kills."""

from __future__ import annotations

import asyncio
from collections import deque
import contextlib
from dataclasses import dataclass
import os
from pathlib import Path
import signal
import time
import json
import uuid


@dataclass
class Child:
    name: str
    process: asyncio.subprocess.Process
    argv: list[str]
    started_at: float
    log_task: asyncio.Task | None = None
    wait_task: asyncio.Task | None = None
    stopping: bool = False


class Supervisor:
    def __init__(self, cwd: Path, log_dir: Path | None = None):
        self.cwd = cwd
        self.children: dict[str, Child] = {}
        self.logs = deque(maxlen=10000)
        self.sequence = 0
        self.lock = asyncio.Lock()
        self.session_id = time.strftime("%Y%m%dT%H%M%S") + "-" + uuid.uuid4().hex[:8]
        self.log_dir = log_dir
        self.log_queue = asyncio.Queue(maxsize=2048)
        self.log_task = None
        self.logs_dropped = 0

    def start_logging(self):
        if self.log_dir and self.log_task is None:
            self.log_dir.mkdir(parents=True, exist_ok=True)
            self.log_task = asyncio.create_task(self._persist_logs())

    async def _persist_logs(self):
        path = self.log_dir / (self.session_id + ".jsonl")

        def append(batch):
            with path.open("a") as stream:
                for item in batch:
                    stream.write(json.dumps(item, ensure_ascii=False) + "\n")

        while True:
            first = await self.log_queue.get()
            batch = [first]
            while not self.log_queue.empty() and len(batch) < 256:
                batch.append(self.log_queue.get_nowait())
            final = None in batch
            batch = [item for item in batch if item is not None]
            try:
                await asyncio.to_thread(append, batch)
            except OSError:
                self.logs_dropped += len(batch)
            if final:
                return

    def query_logs(
        self, *, session="", after=0, level="", subsystem="", q="", limit=1000
    ):
        if session and session != self.session_id:
            # Session identifiers are generated names, never caller-supplied paths.
            files = (
                {p.stem: p for p in self.log_dir.glob("*.jsonl")}
                if self.log_dir
                else {}
            )
            path = files.get(session)
            if path is None:
                return []

            def entries():
                with path.open() as stream:
                    for line in stream:
                        try:
                            yield json.loads(line)
                        except ValueError:
                            continue

            source = entries()
        else:
            source = list(self.logs)
        result = deque(maxlen=min(max(int(limit), 1), 10000))
        for item in source:
            if (
                item["seq"] > after
                and (not level or item["level"] == level)
                and (not subsystem or item["subsystem"] == subsystem)
                and (not q or q.lower() in item["message"].lower())
            ):
                result.append(item)
        return list(result)

    def log(self, subsystem, message, level="INFO"):
        self.sequence += 1
        item = {
            "seq": self.sequence,
            "session": self.session_id,
            "time": time.time(),
            "subsystem": subsystem,
            "level": level,
            "message": str(message)[:8192],
        }
        self.logs.append(item)
        if self.log_task:
            try:
                self.log_queue.put_nowait(item)
            except asyncio.QueueFull:
                self.logs_dropped += 1

    async def start(self, name: str, argv: list[str], env=None):
        async with self.lock:
            previous = self.children.get(name)
            if previous and previous.process.returncode is None:
                raise RuntimeError(f"Process already running: {name}")
            process = await asyncio.create_subprocess_exec(
                *argv,
                cwd=str(self.cwd),
                env=env or os.environ.copy(),
                stdout=asyncio.subprocess.PIPE,
                stderr=asyncio.subprocess.STDOUT,
                start_new_session=True,
                limit=65536,
            )
            child = Child(name, process, argv, time.time())
            self.children[name] = child
            child.log_task = asyncio.create_task(self._read_logs(child))
            child.wait_task = asyncio.create_task(self._wait(child))
            self.log(name, f"Started pid={process.pid}")
            return child

    async def _read_logs(self, child):
        try:
            while True:
                data = await child.process.stdout.read(4096)
                if not data:
                    break
                for line in data.decode(errors="replace").splitlines():
                    self.log(child.name, line)
        except (asyncio.CancelledError, OSError):
            pass

    async def _wait(self, child):
        code = await child.process.wait()
        self.log(
            child.name,
            f"Exited code={code} expected={child.stopping}",
            "INFO" if child.stopping else "ERROR",
        )

    async def stop(self, name, timeout=4):
        child = self.children.get(name)
        if not child:
            return
        child.stopping = True
        # Children created with start_new_session own exactly this process group.
        if child.process.returncode is None:
            with contextlib.suppress(ProcessLookupError):
                os.killpg(child.process.pid, signal.SIGTERM)
            try:
                await asyncio.wait_for(child.process.wait(), timeout)
            except asyncio.TimeoutError:
                with contextlib.suppress(ProcessLookupError):
                    os.killpg(child.process.pid, signal.SIGKILL)
                await child.process.wait()
        for task in (child.log_task, child.wait_task):
            if task:
                with contextlib.suppress(asyncio.CancelledError):
                    await task

    async def close(self):
        for name in reversed(tuple(self.children)):
            await self.stop(name)
        if self.log_task:
            await self.log_queue.put(None)
            await self.log_task
            self.log_task = None

    def status(self):
        return [
            {
                "name": c.name,
                "pid": c.process.pid,
                "running": c.process.returncode is None,
                "exit_code": c.process.returncode,
                "started_at": c.started_at,
            }
            for c in self.children.values()
        ]

    async def wait_path(self, path: Path, child: Child, timeout=10):
        deadline = asyncio.get_running_loop().time() + timeout
        while asyncio.get_running_loop().time() < deadline:
            if child.process.returncode is not None:
                raise RuntimeError(
                    f"{child.name} failed to start (exit {child.process.returncode})"
                )
            if path.exists():
                return
            await asyncio.sleep(0.05)
        raise TimeoutError(f"{child.name} did not create {path.name}")

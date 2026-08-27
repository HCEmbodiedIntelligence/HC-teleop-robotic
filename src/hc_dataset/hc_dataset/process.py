from __future__ import annotations

import os
import signal
import subprocess
import threading
import time
from pathlib import Path
from typing import IO

from .catalog import DatasetError, DatasetRecord


class BagCommandBuilder:
    """Build argv-only rosbag commands; shell parsing is never used."""

    def __init__(self, ros2_executable: str = "ros2", storage_id: str = "mcap"):
        self.ros2_executable = ros2_executable
        self.storage_id = storage_id

    def record(self, record: DatasetRecord, *, max_cache_size: int = 16 * 1024 * 1024) -> list[str]:
        if max_cache_size < 0:
            raise DatasetError("max_cache_size cannot be negative")
        return [
            self.ros2_executable,
            "bag",
            "record",
            "--storage",
            self.storage_id,
            "--output",
            str(record.bag_directory),
            "--max-cache-size",
            str(max_cache_size),
            "--log-level",
            "warn",
            *record.manifest["topics"],
        ]

    def replay(
        self,
        record: DatasetRecord,
        *,
        rate: float = 1.0,
        start_offset: float = 0.0,
        publish_clock: bool = False,
        loop: bool = False,
    ) -> list[str]:
        if not (0.01 <= rate <= 100.0):
            raise DatasetError("replay rate must be in [0.01, 100]")
        if start_offset < 0:
            raise DatasetError("replay start offset cannot be negative")
        if not record.bag_directory.is_dir():
            raise DatasetError(f"dataset has no readable bag: {record.dataset_id}")
        replay_root = f"/replay/{record.dataset_id}"
        remaps = [
            f"{topic}:={replay_root}/{topic.lstrip('/')}"
            for topic in record.manifest["topics"]
        ]
        command = [
            self.ros2_executable,
            "bag",
            "play",
            str(record.bag_directory),
            "--rate",
            str(rate),
            "--start-offset",
            str(start_offset),
            "--disable-keyboard-controls",
            "--remap",
            *remaps,
        ]
        if publish_clock:
            command.append("--clock")
        if loop:
            command.append("--loop")
        return command


class ManagedBagProcess:
    """Own one child process and leave no subprocess or thread behind while idle."""

    def __init__(self) -> None:
        self._process: subprocess.Popen[bytes] | None = None
        self._log: IO[bytes] | None = None
        self._lock = threading.Lock()

    @property
    def running(self) -> bool:
        with self._lock:
            return self._process is not None and self._process.poll() is None

    @property
    def pid(self) -> int | None:
        with self._lock:
            return self._process.pid if self._process is not None else None

    def start(self, command: list[str], log_path: Path) -> int:
        if not command or any(not isinstance(item, str) or not item for item in command):
            raise DatasetError("process command contains an invalid argument")
        with self._lock:
            if self._process is not None and self._process.poll() is None:
                raise DatasetError("a bag process is already running")
            log_path.parent.mkdir(parents=True, exist_ok=True)
            self._log = log_path.open("ab", buffering=0)
            try:
                self._process = subprocess.Popen(
                    command,
                    stdin=subprocess.DEVNULL,
                    stdout=self._log,
                    stderr=subprocess.STDOUT,
                    shell=False,
                    start_new_session=True,
                )
            except Exception:
                self._log.close()
                self._log = None
                raise
            return self._process.pid

    def poll(self) -> int | None:
        with self._lock:
            return self._process.poll() if self._process is not None else None

    def stop(self, timeout: float = 5.0) -> int | None:
        with self._lock:
            process = self._process
        if process is None:
            return None
        if process.poll() is None:
            try:
                os.killpg(process.pid, signal.SIGINT)
                process.wait(timeout=max(0.1, timeout))
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGTERM)
                try:
                    process.wait(timeout=2.0)
                except subprocess.TimeoutExpired:
                    os.killpg(process.pid, signal.SIGKILL)
                    process.wait(timeout=2.0)
            except ProcessLookupError:
                pass
        code = process.poll()
        with self._lock:
            if self._log is not None:
                self._log.close()
            self._log = None
            self._process = None
        return code

    def wait(self, poll_interval: float = 0.1) -> int:
        while True:
            code = self.poll()
            if code is not None:
                self.stop()
                return code
            time.sleep(poll_interval)

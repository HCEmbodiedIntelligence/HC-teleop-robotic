from __future__ import annotations

import asyncio
import json
import os
import threading
import time
from pathlib import Path
from typing import Any

import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from hc_teleop_interfaces.action import RecordDataset, ReplayDataset
from hc_teleop_interfaces.msg import Status

from .catalog import DatasetCatalog, DatasetError, DatasetRecord
from .process import BagCommandBuilder, ManagedBagProcess


def _seconds(duration: Any) -> float:
    return float(duration.sec) + float(duration.nanosec) * 1.0e-9


def _set_duration(message: Any, seconds: float) -> None:
    whole = max(0, int(seconds))
    message.sec = whole
    message.nanosec = int(max(0.0, seconds - whole) * 1_000_000_000)


def _status(code: int, message: str, severity: int | None = None) -> Status:
    value = Status()
    value.code = code
    value.message = message
    if severity is None:
        severity = Status.SEVERITY_OK if code == Status.OK else Status.SEVERITY_ERROR
    value.severity = severity
    return value


def _directory_size(path: Path) -> int:
    total = 0
    for item in path.rglob("*"):
        try:
            if item.is_file():
                total += item.stat().st_size
        except OSError:
            continue
    return total


class DatasetNode(Node):
    def __init__(self) -> None:
        super().__init__("hc_dataset")
        root = self.declare_parameter(
            "dataset_root",
            str(Path.home() / ".local" / "share" / "hc_teleop" / "datasets"),
        ).value
        storage_id = str(self.declare_parameter("storage_id", "mcap").value)
        ros2_executable = str(self.declare_parameter("ros2_executable", "ros2").value)
        self._max_cache_size = int(
            self.declare_parameter("max_cache_size", 16 * 1024 * 1024).value
        )
        self._catalog = DatasetCatalog(str(root))
        self._commands = BagCommandBuilder(ros2_executable, storage_id)
        self._record_process = ManagedBagProcess()
        self._replay_process = ManagedBagProcess()
        self._reservation_lock = threading.Lock()
        self._reserved: str | None = None

        self._record_server = ActionServer(
            self,
            RecordDataset,
            "dataset/record",
            execute_callback=self._record,
            goal_callback=lambda request: self._reserve("record", request),
            cancel_callback=lambda _: CancelResponse.ACCEPT,
        )
        self._replay_server = ActionServer(
            self,
            ReplayDataset,
            "dataset/replay",
            execute_callback=self._replay,
            goal_callback=lambda request: self._reserve("replay", request),
            cancel_callback=lambda _: CancelResponse.ACCEPT,
        )
        self.get_logger().info(
            f"Dataset service ready at {self._catalog.root}; no recorder process is running"
        )

    def destroy_node(self) -> bool:
        self._record_process.stop()
        self._replay_process.stop()
        self._record_server.destroy()
        self._replay_server.destroy()
        return super().destroy_node()

    def _reserve(self, kind: str, request: Any) -> GoalResponse:
        del request
        with self._reservation_lock:
            if self._reserved is not None:
                self.get_logger().warning(
                    f"Rejected {kind}: dataset operation '{self._reserved}' is active"
                )
                return GoalResponse.REJECT
            self._reserved = kind
            return GoalResponse.ACCEPT

    def _release(self, kind: str) -> None:
        with self._reservation_lock:
            if self._reserved == kind:
                self._reserved = None

    async def _record(self, goal_handle: Any) -> RecordDataset.Result:
        request = goal_handle.request
        result = RecordDataset.Result()
        record: DatasetRecord | None = None
        started = time.monotonic()
        try:
            metadata = self._metadata(request.metadata_json)
            record = self._catalog.create(request.dataset_name, list(request.topics), metadata)
            command = self._commands.record(record, max_cache_size=self._max_cache_size)
            self._record_process.start(command, record.directory / "record.log")
            self.get_logger().info(f"Recording dataset {record.dataset_id}")
            maximum = _seconds(request.max_duration)
            while True:
                elapsed = time.monotonic() - started
                if goal_handle.is_cancel_requested:
                    self._record_process.stop()
                    final = self._finalize(record, "canceled")
                    goal_handle.canceled()
                    self._fill_record_result(result, final, Status.CANCELED, "recording canceled")
                    return result
                if maximum > 0.0 and elapsed >= maximum:
                    self._record_process.stop()
                    break
                code = self._record_process.poll()
                if code is not None:
                    if code != 0:
                        raise DatasetError(f"rosbag recorder exited with code {code}")
                    break
                feedback = RecordDataset.Feedback()
                feedback.progress = min(1.0, elapsed / maximum) if maximum > 0.0 else 0.0
                _set_duration(feedback.elapsed, elapsed)
                feedback.bytes_written = _directory_size(record.bag_directory)
                feedback.message_count = 0
                feedback.status = _status(Status.OK, "recording", Status.SEVERITY_INFO)
                goal_handle.publish_feedback(feedback)
                await asyncio.sleep(0.2)

            final = self._finalize(record, "complete")
            goal_handle.succeed()
            self._fill_record_result(result, final, Status.OK, "recording complete")
            return result
        except Exception as error:
            self._record_process.stop()
            if record is not None:
                try:
                    record = self._finalize(record, "error", str(error))
                except Exception:
                    pass
            goal_handle.abort()
            result.status = _status(Status.IO_ERROR, str(error))
            if record is not None:
                result.dataset_id = record.dataset_id
                result.uri = str(record.directory)
                result.bytes_written = _directory_size(record.directory)
            self.get_logger().error(f"Dataset recording failed: {error}")
            return result
        finally:
            self._release("record")

    async def _replay(self, goal_handle: Any) -> ReplayDataset.Result:
        request = goal_handle.request
        result = ReplayDataset.Result()
        started = time.monotonic()
        try:
            record = self._catalog.get(request.dataset_id)
            if record.manifest.get("state") != "complete":
                raise DatasetError("only complete datasets may be replayed")
            start_offset = _seconds(request.start_offset)
            end_offset = _seconds(request.end_offset)
            if end_offset > 0.0 and end_offset <= start_offset:
                raise DatasetError("end_offset must be greater than start_offset")
            command = self._commands.replay(
                record,
                rate=float(request.rate or 1.0),
                start_offset=start_offset,
                publish_clock=bool(request.publish_clock),
                loop=bool(request.loop),
            )
            self._replay_process.start(command, record.directory / "replay.log")
            self.get_logger().warning(
                f"Replaying {record.dataset_id} only below /replay/{record.dataset_id}/..."
            )
            wall_limit = (
                (end_offset - start_offset) / float(request.rate or 1.0)
                if end_offset > 0.0
                else 0.0
            )
            while True:
                elapsed = time.monotonic() - started
                if goal_handle.is_cancel_requested:
                    self._replay_process.stop()
                    goal_handle.canceled()
                    result.status = _status(Status.CANCELED, "replay canceled")
                    return result
                if wall_limit > 0.0 and elapsed >= wall_limit:
                    self._replay_process.stop()
                    break
                code = self._replay_process.poll()
                if code is not None:
                    if code != 0:
                        raise DatasetError(f"rosbag replay exited with code {code}")
                    break
                feedback = ReplayDataset.Feedback()
                feedback.progress = min(1.0, elapsed / wall_limit) if wall_limit > 0.0 else 0.0
                _set_duration(
                    feedback.current_offset,
                    start_offset + elapsed * float(request.rate or 1.0),
                )
                feedback.messages_replayed = 0
                feedback.status = _status(Status.OK, "replaying", Status.SEVERITY_INFO)
                goal_handle.publish_feedback(feedback)
                await asyncio.sleep(0.2)

            goal_handle.succeed()
            result.status = _status(Status.OK, "replay complete")
            _set_duration(
                result.replayed_duration,
                (time.monotonic() - started) * float(request.rate or 1.0),
            )
            result.messages_replayed = 0
            return result
        except Exception as error:
            self._replay_process.stop()
            goal_handle.abort()
            result.status = _status(Status.IO_ERROR, str(error))
            self.get_logger().error(f"Dataset replay failed: {error}")
            return result
        finally:
            self._release("replay")

    @staticmethod
    def _metadata(encoded: str) -> dict[str, Any]:
        if not encoded:
            return {}
        if len(encoded.encode("utf-8")) > 64 * 1024:
            raise DatasetError("metadata_json exceeds 64 KiB")
        try:
            value = json.loads(encoded)
        except json.JSONDecodeError as error:
            raise DatasetError(f"metadata_json is invalid: {error}") from error
        if not isinstance(value, dict):
            raise DatasetError("metadata_json must encode an object")
        return value

    def _finalize(
        self, record: DatasetRecord, state: str, error: str | None = None
    ) -> DatasetRecord:
        return self._catalog.finalize(
            record.dataset_id,
            state=state,
            message_count=0,
            bytes_written=_directory_size(record.bag_directory),
            error=error,
        )

    @staticmethod
    def _fill_record_result(
        result: RecordDataset.Result,
        record: DatasetRecord,
        code: int,
        message: str,
    ) -> None:
        result.status = _status(code, message)
        result.dataset_id = record.dataset_id
        result.uri = str(record.directory)
        result.message_count = int(record.manifest.get("message_count", 0))
        result.bytes_written = int(record.manifest.get("bytes_written", 0))


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = DatasetNode()
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()

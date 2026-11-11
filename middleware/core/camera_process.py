"""Isolate camera decode, encoding and RTP from control's Python GIL.

Only camera subscriptions exist in the child. A private pipe carries signaling,
cached status and on-demand snapshots; live images do not pass through the
middleware process or an unbounded IPC queue.
"""
from __future__ import annotations

import asyncio
from concurrent.futures import Future
import multiprocessing
import threading
import time
from typing import Any


class CameraProcessService:
    def __init__(self, config: dict[str, Any], domain_id: int = 14):
        self.config = dict(config)
        self.domain_id = int(domain_id)
        self._process = None
        self._connection = None
        self._reader = None
        self._closed = threading.Event()
        self._lock = threading.Lock()
        self._send_lock = threading.Lock()
        self._pending: dict[int, Future] = {}
        self._request_id = 0
        self._status_at = 0.0
        from .camera import CameraService
        self._status = CameraService(config, domain_id).status()
        self._status.update(video_process=True, video_pid=None)

    def start(self) -> None:
        if not self.config.get("enabled", False) or self._process is not None:
            return
        context = multiprocessing.get_context("spawn")
        parent, child = context.Pipe()
        self._connection = parent
        self._closed.clear()
        self._process = context.Process(
            target=_camera_worker, args=(child, self.config, self.domain_id),
            name="camera-" + str(self.config.get("id", "main")), daemon=True,
        )
        try:
            self._process.start()
        except BaseException:
            parent.close()
            self._process = None
            self._connection = None
            raise
        finally:
            child.close()
        self._status["video_pid"] = self._process.pid
        self._reader = threading.Thread(target=self._read_responses,
                                        name="camera-status", daemon=True)
        self._reader.start()

    def _read_responses(self) -> None:
        error = "camera process exited"
        try:
            while not self._closed.is_set():
                if not self._connection.poll(.2):
                    if not self._process.is_alive():
                        break
                    continue
                message = self._connection.recv()
                if message[0] == "status":
                    with self._lock:
                        self._status.update(message[1])
                        self._status_at = time.monotonic()
                else:
                    _, request_id, success, value = message
                    with self._lock:
                        future = self._pending.pop(request_id, None)
                    if future is not None and not future.done():
                        if success:
                            future.set_result(value)
                        else:
                            future.set_exception(RuntimeError(value))
        except (EOFError, OSError) as exc:
            error = f"camera process disconnected: {exc}"
        finally:
            with self._lock:
                pending = list(self._pending.values())
                self._pending.clear()
                if not self._closed.is_set() and self._status.get("state") != "error":
                    self._status.update(state="error", error=error, peers=0)
            for future in pending:
                if not future.done():
                    future.set_exception(RuntimeError(error))

    def _request(self, operation: str, value=None, timeout: float = 15):
        with self._lock:
            if self._closed.is_set() or self._process is None or not self._process.is_alive():
                raise RuntimeError("camera process is not running")
            self._request_id += 1
            request_id = self._request_id
            future = Future()
            self._pending[request_id] = future
        try:
            with self._send_lock:
                self._connection.send((request_id, operation, value))
            return future.result(timeout)
        finally:
            with self._lock:
                self._pending.pop(request_id, None)

    def handle_ros_message(self, topic: str, msg: Any) -> None:
        # The parent still monitors ROS topic health. The child independently
        # subscribes with depth=1, so control callbacks never copy video frames.
        pass

    def status(self) -> dict[str, Any]:
        with self._lock:
            status = dict(self._status)
            stale = time.monotonic() - self._status_at > 2
        if stale:
            status['receiver_stale'] = True
            status['receiver_age_ms'] = None
            for key in tuple(status):
                if key.startswith('receiver_') and key not in {
                        'receiver_stale', 'receiver_reported', 'receiver_session_id',
                        'receiver_metrics_source', 'receiver_loss_pattern'}:
                    status[key] = None
            for key in ("capture_fps", "decoded_fps", "webrtc_send_fps", "webrtc_submit_fps", "webrtc_output_fps"):
                status[key] = 0.0
        return status

    def latest(self):
        if self._process is None or self._closed.is_set():
            return None
        return self._request("snapshot", timeout=3)

    async def offer(self, data: dict[str, Any]) -> dict[str, str]:
        return await asyncio.to_thread(self._request, "offer", data)

    async def receiver_report(self, payload):
        return await asyncio.to_thread(self._request, "receiver_report", payload, 3)

    async def stop(self) -> None:
        if self._process is None:
            return
        try:
            if self._process.is_alive():
                await asyncio.to_thread(self._request, "stop", None, 8)
        except (RuntimeError, TimeoutError, EOFError, OSError):
            pass
        self._closed.set()
        await asyncio.to_thread(self._process.join, 2)
        if self._process.is_alive():
            self._process.terminate()
            await asyncio.to_thread(self._process.join, 2)
        if self._process.is_alive():
            self._process.kill()
            await asyncio.to_thread(self._process.join, 2)
        if self._reader is not None:
            await asyncio.to_thread(self._reader.join, 1)
        self._connection.close()
        with self._lock:
            self._status.update(state="stopped", peers=0, capture_fps=0,
                                decoded_fps=0, webrtc_send_fps=0)
        self._process = None


def _camera_worker(connection, config: dict[str, Any], domain_id: int):
    """Spawn entrypoint; inherit no live ROS contexts or locks from the parent."""
    import os
    from .camera import CameraService
    camera = CameraService(config, domain_id)
    context = node = executor = ros_thread = None
    send_lock = threading.Lock()

    def send(message):
        with send_lock:
            connection.send(message)

    async def run():
        nonlocal context, node, executor, ros_thread
        camera.start()
        if str(config.get("source", "ros")).lower() == "ros":
            import rclpy
            from rclpy.context import Context
            from rclpy.executors import SingleThreadedExecutor
            from rclpy.qos import QoSProfile, ReliabilityPolicy
            from sensor_msgs.msg import CompressedImage, Image
            context = Context()
            rclpy.init(args=[], context=context, domain_id=domain_id)
            node = rclpy.create_node(f"hc_camera_video_{os.getpid()}", context=context)
            topic = config.get("custom_topic") or config.get("topic", "/observation/cameras/head/color/compressed")
            image_type = CompressedImage if "compressed" in topic else Image
            node.create_subscription(image_type, topic,
                lambda msg: camera.handle_ros_message(topic, msg),
                QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT))
            executor = SingleThreadedExecutor(context=context)
            executor.add_node(node)
            ros_thread = threading.Thread(target=executor.spin, name="camera-source", daemon=True)
            ros_thread.start()

        async def report():
            while True:
                status = camera.status()
                status.update(video_process=True, video_pid=os.getpid())
                send(("status", status))
                await asyncio.sleep(.5)

        reporter = asyncio.create_task(report())
        try:
            while True:
                try:
                    request_id, operation, value = await asyncio.to_thread(connection.recv)
                except (EOFError, OSError):
                    break  # Parent exit closes the only other pipe endpoint.
                try:
                    if operation == "stop":
                        send(("response", request_id, True, None))
                        break
                    if operation == "offer":
                        result = await camera.offer(value)
                    elif operation == "receiver_report":
                        result = camera.receiver_report(value)
                    elif operation == "snapshot":
                        result = camera.latest()
                    else:
                        raise ValueError("unknown camera operation")
                    send(("response", request_id, True, result))
                except Exception as exc:
                    send(("response", request_id, False, f"{type(exc).__name__}: {exc}"))
        finally:
            reporter.cancel()
            await asyncio.gather(reporter, return_exceptions=True)
            await camera.stop()

    try:
        asyncio.run(run())
    except (EOFError, BrokenPipeError):
        pass
    except Exception as exc:
        try:
            send(("status", {"state": "error", "error": f"{type(exc).__name__}: {exc}", "peers": 0}))
        except (EOFError, OSError):
            pass
    finally:
        if executor is not None:
            executor.shutdown(timeout_sec=2)
        if ros_thread is not None:
            ros_thread.join(timeout=2)
        if node is not None:
            node.destroy_node()
        if context is not None and context.ok():
            context.shutdown()
        connection.close()

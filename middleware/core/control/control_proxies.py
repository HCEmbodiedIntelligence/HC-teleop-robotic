"""隔离控制父进程代理对象：RemoteOwnership、RemotePublicationCapture 与 IsolatedRosBridge。"""
from __future__ import annotations

import copy
import json
import threading
import time
from types import SimpleNamespace
from typing import Any

from .control_worker import _OWNER_METHODS, _READONLY_ROS_TOPICS

_HOOKS = {"manual_changed", "dagger_changed", "open_segment", "close_segment", "capture_ready"}
_CONTROL_PREFIXES = ("/control/", "/internal/", "/teleop/", "/action/", "/hc_teleop/")


def publish_readonly_observation(telemetry: Any, topic: str, payload: Any) -> Any:
    """仅限父进程执行的只读观测序列化发布，绝不可触及执行器指令。"""
    if topic not in _READONLY_ROS_TOPICS or not isinstance(payload, dict):
        raise ValueError("unsupported read-only control observation")
    return telemetry.publish(
        topic,
        "std_msgs/msg/String",
        {"data": json.dumps(payload, ensure_ascii=False, allow_nan=False)},
    )


class RemoteOwnership:
    def __init__(self, client: Any) -> None:
        object.__setattr__(self, "_client", client)

    def __setattr__(self, name: str, value: Any) -> None:
        if name in _HOOKS:
            self._client.hooks[name] = value
        else:
            raise AttributeError("remote ownership attributes are read-only")

    def snapshot(self, **_kwargs: Any) -> dict[str, Any]:
        return self._client.snapshot().get("ownership", {"state": "OFF"})

    def ros_clock(self) -> int:
        stamp = getattr(self._client._hook_context, "clock_ns", None)
        if stamp is not None:
            return stamp
        if not self._client._closing and not self._client._failure:
            return self._client.call("clock")
        stamp = self.snapshot().get("published_stamp_ns")
        if not stamp:
            raise RuntimeError("control clock unavailable")
        return stamp

    @property
    def active(self) -> bool:
        if self._client._failure:
            return False
        try:
            return bool(self._client._shared_value("ownership_active"))
        except RuntimeError:
            return False

    def __getattr__(self, name: str) -> Any:
        if name in _OWNER_METHODS:
            def invoke(*args: Any, expected_fence: Any = None, **kwargs: Any) -> Any:
                urgent = name in {"disable", "hold", "fault", "input_lost"}
                negative_generation = None
                if name in {"disable", "hold", "fault"}:
                    negative_generation = self._client.revoke_intent()
                if name == "input_lost":
                    return self._client.notify("owner." + name, *args, **kwargs)
                return self._client.call(
                    "owner." + name,
                    *args,
                    expected_fence=expected_fence,
                    urgent=urgent,
                    negative_generation=negative_generation,
                    **kwargs,
                )

            return invoke
        aliases = {
            "instance": "controller_instance_id",
            "session": "session_id",
            "epoch": "epoch",
        }
        value = self.snapshot()
        if name == "token":
            from .control_ownership import ControlToken

            return ControlToken(
                *(
                    value.get(k, 0 if k == "epoch" else "")
                    for k in (
                        "controller_instance_id",
                        "session_id",
                        "epoch",
                        "source",
                        "intervention_id",
                    )
                )
            )
        if name == "required_peers":
            return frozenset(value.get(name, ("solver", "teleop")))
        if name == "recording_enabled":
            return value.get(
                name,
                self._client.config.get("intervention", {}).get(
                    "automatic_recording", True
                ),
            )
        if name == "peers":
            return value.get(name, {})
        if name in value or name in aliases:
            return value.get(aliases.get(name, name), "" if name != "epoch" else 0)
        raise AttributeError(name)


class RemotePublicationCapture:
    def __init__(self, client: Any, selected: Any) -> None:
        self.client, self.selected = client, selected
        self._closed = False
        self._error = ""
        self._admission_fence = None
        self._last_status_at = 0
        self._last = dict(
            generation=0,
            active=False,
            captured=0,
            enqueued=0,
            dropped=0,
            failed=0,
            outside_preroll=0,
            queued=0,
            inflight=0,
            pending_ingress=0,
            error="",
        )

    def status(self) -> dict[str, Any]:
        snapshot = self.client.snapshot()
        current = snapshot.get("publication_capture")
        observed_at = snapshot.get("snapshot_monotonic_ns", 0)
        if current and not self._closed and observed_at >= self._last_status_at:
            self._last = current
        result = dict(self._last)
        if self._error or self.client._failure:
            result["error"] = self._error or self.client._failure
        return result

    def begin_generation(self, **kwargs: Any) -> Any:
        if self._closed:
            raise RuntimeError("publication capture is closed")
        generation = self.client.call(
            "capture_begin", selected=bool(self.selected()), **kwargs
        )
        self._admission_fence = None
        self._error = ""
        return generation

    def close_admission(self) -> tuple[int, int]:
        if self._admission_fence is not None:
            return self._admission_fence
        if self._closed:
            self._admission_fence = time.time_ns(), time.monotonic_ns()
            return self._admission_fence
        try:
            self._admission_fence = tuple(
                self.client.call("capture_fence", urgent=True, allow_failed=True)
            )
        except RuntimeError as exc:
            self._error = (
                str(exc) + "; publication fence unavailable, capture incomplete"
            )
            self._admission_fence = time.time_ns(), time.monotonic_ns()
        return self._admission_fence

    def end_generation(self, timeout: float = 10.0) -> None:
        self.close_admission()
        deadline = time.monotonic() + timeout
        while not self._closed:
            try:
                self._last = self.client.call("capture_status", allow_failed=True)
                self._last_status_at = self.client.snapshot().get(
                    "snapshot_monotonic_ns", 0
                )
            except RuntimeError as exc:
                self._error = str(exc) + "; published action drain incomplete"
                return
            if not self._last["inflight"] and not self._last["pending_ingress"]:
                return
            if time.monotonic() >= deadline:
                self._error = "published action drain timed out; capture incomplete"
                return
            time.sleep(0.005)

    def close(self) -> None:
        if self._closed:
            return
        self.end_generation()
        if (
            not self._error
            and not self._last["inflight"]
            and not self._last["pending_ingress"]
        ):
            try:
                self.client.call("capture_close", allow_failed=True)
            except RuntimeError as exc:
                self._error = str(exc) + "; publication capture incomplete"
        self._closed = True


class IsolatedRosBridge:
    """父进程门面：遥测保留在父进程，所有控制权与执行器发布均隔离在子进程内。"""

    isolated_control_process = True
    supports_replay_alignment = True

    def __init__(
        self,
        config: dict[str, Any],
        on_event: Any,
        on_frame: Any = None,
        camera_topic: str = "",
        camera_topics: list[str] | None = None,
        *,
        bridge_factory: Any = None,
    ) -> None:
        from .control_process import CoreProcessClient
        from .ros_bridge import RosBridge

        self.config, self.on_event = config, on_event
        self._legacy_enabled = bool(config.get("legacy_control_profile"))
        telemetry_config = copy.deepcopy(config)
        telemetry_config.pop("legacy_control_profile", None)
        telemetry_config.pop("robot_api_profile", None)
        telemetry_config["command_mux"] = {
            **telemetry_config.get("command_mux", {}),
            "enabled": False,
        }
        self._telemetry = RosBridge(
            telemetry_config, on_event, on_frame, camera_topic, camera_topics
        )
        self._action_sink = lambda evidence: None
        hooks = {"action": lambda evidence: self._action_sink(evidence)}
        self._client = CoreProcessClient(
            config,
            on_event=on_event,
            hooks=hooks,
            fallback_stop=self._fallback_stop,
            bridge_factory=bridge_factory,
            on_observer_ros=lambda topic, payload: publish_readonly_observation(
                self._telemetry, topic, payload
            ),
        )
        self.ownership = RemoteOwnership(self._client)
        self._boundary = SimpleNamespace(
            status=lambda: self._client.snapshot().get("legacy_control", {})
        )
        self._publication_capture = None

    def _fallback_stop(self, reason: str) -> None:
        # 仅做负向急停，任何降级路径都绝不重新开启输出。
        self._telemetry.publish(
            self.config.get("stop_topic", "/control/emergency_stop"),
            "std_msgs/msg/Bool",
            {"data": True},
        )

        def log_failure() -> None:
            self.on_event(
                {
                    "kind": "log",
                    "source": "control_process",
                    "payload": {"level": "error", "message": reason},
                },
                ["websocket"],
            )

        threading.Thread(
            target=log_failure, name="control-failure-log", daemon=True
        ).start()

    def create_publication_capture(self, recorder: Any, selected: Any) -> RemotePublicationCapture:
        if self._client.process is not None:
            raise RuntimeError(
                "publication ingress must be configured before control process starts"
            )
        self._client.capture_spec = dict(
            resources=recorder.ipc_resources(), finish=recorder._finish_event
        )
        self._publication_capture = RemotePublicationCapture(self._client, selected)
        return self._publication_capture

    def start(self) -> None:
        self._telemetry.start()
        try:
            self._client.start()
        except Exception:
            self._client.stop()
            self._telemetry.stop()
            raise

    def stop(self) -> None:
        try:
            if self._client.process is not None and self._client.process.is_alive():
                try:
                    self.set_command_output_enabled(False)
                    self.flush_recording_events()
                except (ValueError, RuntimeError):
                    pass
            if self._publication_capture is not None:
                self._publication_capture.close()
        finally:
            self._client.stop()
            self._telemetry.stop()

    def status(self) -> dict[str, Any]:
        result = self._telemetry.status()
        core = self._client.snapshot()
        for key in (
            "command_mux",
            "ownership",
            "legacy_control",
            "control_actor",
            "callback_timings",
            "slow_callbacks",
            "command_stage_timings",
            "slow_command_stages",
            "control_contention_samples",
            "control_process",
            "authorization_fence",
            "snapshot_monotonic_ns",
        ):
            if key in core:
                result[key] = core[key]
        result["isolated_control_process"] = True
        if core.get("state") != "running":
            result.update(state=core.get("state", "starting"), error=core.get("error"))
        return result

    def get_topic_health(self) -> dict[str, Any]:
        return self._telemetry.get_topic_health()

    def authorization_fence(self) -> tuple[Any, int, int]:
        return self._client.authorization_fence()

    def publish(self, topic: str, msg_type: str, data: dict[str, Any], *, expected_fence: Any = None) -> Any:
        if str(topic).startswith(_CONTROL_PREFIXES) or str(topic) == "/io_teleop/joint_cmd":
            if str(topic).endswith("/emergency_stop") and data.get("data") is True:
                return self.emergency_stop(topic, "explicit emergency stop publication")
            return self._client.call(
                "publish", topic, msg_type, data, expected_fence=expected_fence
            )
        return self._telemetry.publish(topic, msg_type, data)

    def publish_raw(self, topic: str, msg_type: str, raw_data: Any, *, expected_fence: Any = None) -> Any:
        if str(topic).startswith(_CONTROL_PREFIXES) or str(topic) == "/io_teleop/joint_cmd":
            if expected_fence is not None:
                return self._client.call(
                    "publish_raw", topic, msg_type, raw_data, expected_fence=expected_fence
                )
            return self._client.submit_replay_raw(topic, msg_type, raw_data)
        return self._telemetry.publish_raw(topic, msg_type, raw_data)

    def start_replay_alignment(self, arm_raw: Any, hand_raws: Any = None, *, expected_fence: Any = None) -> Any:
        return self._client.call(
            "start_replay_alignment", arm_raw, hand_raws, expected_fence=expected_fence
        )

    def replay_alignment_status(self) -> Any:
        return self._client.call("replay_alignment_status")

    def pause_replay_alignment(self) -> Any:
        return self._client.call("pause_replay_alignment")

    def resume_replay_alignment(self) -> Any:
        return self._client.call("resume_replay_alignment")

    def cancel_replay_alignment(self) -> Any:
        return self._client.call("cancel_replay_alignment")

    def emergency_stop(self, topic: str, reason: str) -> bool:
        generation = self._client.revoke_intent()
        return self._client.notify(
            "emergency_stop", topic, reason, negative_generation=generation
        )

    def set_command_output_enabled(self, enabled: bool, *, expected_fence: Any = None) -> Any:
        generation = None
        if not enabled:
            generation = self._client.revoke_intent()
        return self._client.call(
            "set_command_output_enabled",
            bool(enabled),
            expected_fence=expected_fence,
            urgent=not enabled,
            negative_generation=generation,
        )

    def resume_output(self, stop_topic: str, *, expected_fence: Any) -> Any:
        return self._client.call(
            "resume_output", stop_topic, expected_fence=expected_fence
        )

    def recover_intervention(self, *, expected_fence: Any = None) -> Any:
        return self._client.call("recover_intervention", expected_fence=expected_fence)

    def set_control_source(self, source: str) -> Any:
        source = str(source).strip().lower()
        if source not in {"vr", "exoskeleton", "motion"}:
            raise ValueError("control source must be vr, exoskeleton or motion")
        return self._client.call("set_control_source", source, urgent=True)

    def begin_replay(self) -> Any:
        return self._client.call("begin_replay")

    def request_replay_home(self, *, output_resumed: bool, expected_fence: Any) -> Any:
        return self._client.call(
            "request_replay_home",
            output_resumed=output_resumed,
            expected_fence=expected_fence,
        )

    def end_replay(self, require_reset: bool = True) -> Any:
        generation = self._client.revoke_intent()
        return self._client.call(
            "end_replay", require_reset, urgent=True, negative_generation=generation
        )

    def request_manual_home(self) -> Any:
        return self._client.call("request_manual_home")

    def control_ready(self) -> bool:
        return bool(self._client.snapshot().get("control_ready"))

    def apply_intervention(
        self,
        name: str,
        data: dict[str, Any],
        *,
        recording_ready: bool = False,
        preparing_capture: bool = False,
        expected_fence: Any = None,
    ) -> Any:
        return self._client.call(
            "apply_intervention",
            name,
            data,
            recording_ready=recording_ready,
            preparing_capture=preparing_capture,
            expected_fence=expected_fence,
            urgent=name in {"disable", "hold", "disable_dagger"},
        )

    def flush_recording_events(self, timeout: float = 2.0) -> None:
        return self._client.flush_recording_events(timeout)


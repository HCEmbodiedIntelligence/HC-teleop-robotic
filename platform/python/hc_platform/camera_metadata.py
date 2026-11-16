"""Provenance-preserving camera metadata and bounded TF fallback cache."""

from collections import deque
import json
import math
import threading
import time

import numpy as np

from .adapters.model import pose_matrix


class CameraMetadata:
    def __init__(self, max_age_s=2.0):
        self.max_age_s = float(max_age_s)
        self.entries = {}
        self.transforms = {}
        self.errors = {}
        self.lock = threading.Lock()

    def json_message(self, stream, kind, topic, text):
        try:
            if len(text) > 131072:
                raise ValueError("metadata exceeds 128KiB")
            value = json.loads(
                text,
                parse_constant=lambda x: (_ for _ in ()).throw(
                    ValueError("nonfinite metadata")
                ),
            )
            if not isinstance(value, dict):
                raise ValueError("metadata must be a JSON object")
            pending = [(value, 0)]
            while pending:
                item, depth = pending.pop()
                if depth > 32:
                    raise ValueError("metadata nesting exceeds 32 levels")
                if isinstance(item, float) and not math.isfinite(item):
                    raise ValueError("nonfinite metadata")
                if isinstance(item, dict):
                    pending.extend((child, depth + 1) for child in item.values())
                elif isinstance(item, list):
                    pending.extend((child, depth + 1) for child in item)
            source = str(value.get("source", "hardware_json"))
            calibrated = (
                value.get("calibrated")
                if isinstance(value.get("calibrated"), bool)
                else None
            )
            if "model" in source.lower() or "static_transform_config" in source.lower():
                calibrated = False
            record = {
                "source": source,
                "source_topic": topic,
                "calibrated": calibrated,
                "calibration_verified": False,
                "calibration_claim": "source_declared" if calibrated is True else None,
                "static": kind in ("calibration", "intrinsics")
                or "static_transform_config" in source.lower(),
                "kind": kind,
                "payload": value,
                "received_mono": time.monotonic(),
            }
            with self.lock:
                self.entries[(stream, kind, topic)] = record
                self.errors.pop(topic, None)
        except (ValueError, TypeError) as error:
            with self.lock:
                self.errors[topic] = str(error)

    def camera_info(self, stream, topic, message):
        k = list(map(float, message.k))
        p = list(map(float, message.p))
        r = list(map(float, message.r))
        d = list(map(float, message.d))
        if (
            len(k) != 9
            or len(p) != 12
            or len(r) != 9
            or not all(math.isfinite(x) for x in k + p + r + d)
        ):
            with self.lock:
                self.errors[topic] = "invalid CameraInfo matrices"
            return
        record = {
            "source": "camera_info",
            "source_topic": topic,
            "kind": "intrinsics",
            "calibrated": k[0] != 0,
            "calibration_scope": "intrinsics",
            "physical_calibration_verified": False,
            "calibration_verified": False,
            "static": True,
            "payload": {
                "width": int(message.width),
                "height": int(message.height),
                "K": k,
                "P": p,
                "R": r,
                "D": d,
                "distortion_model": message.distortion_model,
                "frame_id": message.header.frame_id,
                "stamp_ns": message.header.stamp.sec * 1_000_000_000
                + message.header.stamp.nanosec,
            },
            "received_mono": time.monotonic(),
        }
        with self.lock:
            self.entries[(stream, "intrinsics", topic)] = record
            self.errors.pop(topic, None)

    def tf_message(self, message, static=False):
        now = time.monotonic()
        records = []
        for transform in message.transforms[:512]:
            parent, child = (
                transform.header.frame_id.lstrip("/"),
                transform.child_frame_id.lstrip("/"),
            )
            if (
                not parent
                or not child
                or parent == child
                or max(len(parent), len(child)) > 256
            ):
                continue
            value = transform.transform
            try:
                matrix = pose_matrix(
                    [
                        value.translation.x,
                        value.translation.y,
                        value.translation.z,
                        value.rotation.x,
                        value.rotation.y,
                        value.rotation.z,
                        value.rotation.w,
                    ]
                )
            except ValueError:
                continue
            records.append(((parent, child), (matrix, now, bool(static))))
        with self.lock:
            for key, value in records:
                if key in self.transforms or len(self.transforms) < 512:
                    self.transforms[key] = value

    def _lookup(self, parent, child, now):
        if not parent or not child:
            return None
        parent, child = parent.lstrip("/"), child.lstrip("/")
        graph = {}
        for (a, b), (matrix, stamp, static) in self.transforms.items():
            if not static and now - stamp > self.max_age_s:
                continue
            graph.setdefault(a, []).append((b, matrix, stamp, static))
            graph.setdefault(b, []).append((a, np.linalg.inv(matrix), stamp, static))
        queue = deque([(parent, np.eye(4), [], [])])
        seen = {parent}
        while queue and len(seen) <= 512:
            frame, matrix, stamps, static_flags = queue.popleft()
            if frame == child:
                return {
                    "source": "tf",
                    "source_topic": "/tf_static"
                    if static_flags and all(static_flags)
                    else "/tf",
                    "calibrated": None,
                    "calibration_verified": False,
                    "kind": "extrinsics",
                    "payload": {
                        "parent_frame": parent,
                        "child_frame": child,
                        "T_parent_child": matrix.tolist(),
                        "dynamic": not all(static_flags),
                    },
                    "received_mono": min(stamps) if stamps else now,
                    "static": all(static_flags),
                }
            for next_frame, transform, stamp, static in graph.get(frame, []):
                if next_frame not in seen:
                    seen.add(next_frame)
                    queue.append(
                        (
                            next_frame,
                            matrix @ transform,
                            stamps + [stamp],
                            static_flags + [static],
                        )
                    )
        return None

    def status(self, stream, parent_frame=None, camera_frame=None):
        now = time.monotonic()
        with self.lock:
            entries = [
                dict(value)
                for (identifier, _, _), value in self.entries.items()
                if identifier == stream
            ]
            if not camera_frame:
                infos = [entry for entry in entries if entry["source"] == "camera_info"]
                if infos:
                    camera_frame = max(infos, key=lambda entry: entry["received_mono"])[
                        "payload"
                    ].get("frame_id")
            tf = self._lookup(parent_frame, camera_frame, now)
            if tf:
                entries.append(tf)
            errors = dict(self.errors)
        for entry in entries:
            entry["age_ms"] = (now - entry.pop("received_mono")) * 1000
            entry["fresh"] = (
                entry.get("static", False) or entry["age_ms"] <= self.max_age_s * 1000
            )
        fresh = [entry for entry in entries if entry["fresh"]]
        # Preserve all candidates. Selection does not reclassify configured/model
        # transforms as physical calibration evidence.
        preferred = {}
        for kind in ("intrinsics", "calibration", "extrinsics"):
            choices = [entry for entry in fresh if entry["kind"] == kind]
            if choices:
                preferred[kind] = min(
                    choices, key=lambda e: (e["source"] == "tf", e["age_ms"])
                )
        return {
            "available": bool(fresh),
            "selected": preferred,
            "sources": entries,
            "errors": errors,
        }

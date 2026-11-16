"""Fixed subprocess protocol for causal MCAP → LeRobot v3 export.

This module intentionally does not implement a fake dataset when the training
dependencies are absent. A separately pinned interpreter may be configured as
the service's converter_command. Command arguments never come from an API call.
"""

import argparse
import importlib.metadata
import inspect
import json
import traceback
from pathlib import Path

from .artifacts import atomic_json
from .errors import DataError
from .sample_codec import decode_image, field, iter_samples


def joint_vector(payload, names):
    if (
        isinstance(payload, dict)
        and isinstance(payload.get("state"), dict)
        and not any(key in payload for key in ("name", "names", "joint_names"))
    ):
        payload = payload["state"]
    source_names = field(
        payload, "name", field(payload, "names", field(payload, "joint_names"))
    )
    positions = field(
        payload, "position", field(payload, "positions", field(payload, "values"))
    )
    if (
        source_names is None
        and isinstance(payload, dict)
        and all(name in payload for name in names)
    ):
        values = [payload[name] for name in names]
    else:
        if (
            source_names is None
            or positions is None
            or len(source_names) != len(positions)
            or len(set(source_names)) != len(source_names)
        ):
            raise DataError("JOINT_NAME_MAPPING_REQUIRED")
        mapping = dict(zip(source_names, positions))
        if not all(name in mapping for name in names):
            raise DataError("JOINT_MISSING")
        values = [mapping[name] for name in names]
    import numpy as np

    vector = np.asarray(values, dtype=np.float32)
    if vector.shape != (len(names),) or not np.isfinite(vector).all():
        raise DataError("JOINT_SAMPLE_INVALID")
    return vector


def convert(source, output, options):
    try:
        import numpy as np
        from lerobot.datasets.lerobot_dataset import LeRobotDataset

        version = importlib.metadata.version("lerobot")
    except ImportError:
        raise DataError(
            "LEROBOT_UNAVAILABLE", "Run this worker in a pinned LeRobot v3 environment"
        ) from None
    if options.get("lerobot_version") and version != options["lerobot_version"]:
        raise DataError("LEROBOT_VERSION_MISMATCH")
    names = options.get("joint_names")
    if not isinstance(names, list) or not names or len(names) != len(set(names)):
        raise DataError("JOINT_EXPORT_ORDER_REQUIRED", "Supply explicit joint_names")
    fps = options.get("fps", 30)
    if type(fps) is not int or not 1 <= fps <= 120:
        raise DataError("EXPORT_FPS_INVALID")
    state_topic, action_topic = (
        options.get("state_topic", "/platform/control/state"),
        options.get("action_topic", "/platform/control/commands"),
    )
    cameras = options.get("cameras", {})
    if not isinstance(cameras, dict) or len(cameras) > 8:
        raise DataError("CAMERA_EXPORT_MAPPING_INVALID")
    streams = {"observation.state": state_topic, "action": action_topic, **cameras}
    if len(set(streams.values())) != len(streams):
        raise DataError("EXPORT_TOPIC_MAPPING_AMBIGUOUS")
    latest, dataset = {}, None
    period, next_tick = round(1e9 / fps), None
    samples = skipped = episode_frames = episode_index = ticks = 0
    max_age = int(options.get("max_age_ns", 200_000_000))
    if max_age <= 0:
        raise DataError("EXPORT_MAX_AGE_INVALID")
    interventions_only = options.get("only_interventions", False)
    intervention_active = False
    frame_limit = int(options.get("max_output_frames", 2_000_000))
    max_episode = int(options.get("max_episode_frames", 18000))
    audit_path = Path(output).parent / "sample-provenance.jsonl"
    audit = audit_path.open("w")
    task = str(options.get("task", "teleoperation"))

    def close_episode(reason):
        nonlocal episode_frames, episode_index
        if dataset is not None and episode_frames:
            dataset.save_episode()
            audit.write(
                json.dumps(
                    {
                        "event": "episode_end",
                        "episode_index": episode_index,
                        "reason": reason,
                    }
                )
                + "\n"
            )
            episode_frames = 0
            episode_index += 1

    def emit(tick):
        nonlocal dataset, samples, skipped, episode_frames, ticks
        ticks += 1
        if ticks > frame_limit:
            raise DataError("EXPORT_FRAME_BUDGET_EXCEEDED")
        source_status = {
            key: {
                "present": key in latest,
                "valid": key in latest
                and 0 <= tick - latest[key]["timestamp_ns"] <= max_age,
                "age_ns": tick - latest[key]["timestamp_ns"] if key in latest else None,
                "reason": "missing"
                if key not in latest
                else "fresh"
                if 0 <= tick - latest[key]["timestamp_ns"] <= max_age
                else "stale",
            }
            for key in streams
        }
        valid = (not interventions_only or intervention_active) and all(
            value["valid"] for value in source_status.values()
        )
        if not valid:
            skipped += 1
            close_episode("missing_or_stale_sample")
            audit.write(
                json.dumps(
                    {
                        "event": "skipped_tick",
                        "export_time_ns": tick,
                        "valid": False,
                        "intervention_active": intervention_active,
                        "sources": source_status,
                    }
                )
                + "\n"
            )
            return
        frame = {
            "observation.state": joint_vector(
                latest["observation.state"]["payload"], names
            ),
            "action": joint_vector(latest["action"]["payload"], names),
            "task": task,
            "observation.source_valid": np.ones(len(streams), dtype=np.bool_),
            "observation.source_age_s": np.asarray(
                [source_status[key]["age_ns"] / 1e9 for key in streams],
                dtype=np.float32,
            ),
        }
        for key in cameras:
            frame[key] = np.asarray(
                decode_image(latest[key]["payload"]), dtype=np.uint8
            )
        if dataset is None:
            features = {
                "observation.state": {
                    "dtype": "float32",
                    "shape": (len(names),),
                    "names": names,
                },
                "action": {"dtype": "float32", "shape": (len(names),), "names": names},
                "observation.source_valid": {
                    "dtype": "bool",
                    "shape": (len(streams),),
                    "names": list(streams),
                },
                "observation.source_age_s": {
                    "dtype": "float32",
                    "shape": (len(streams),),
                    "names": list(streams),
                },
            }
            if options.get("joint_units"):
                if len(options["joint_units"]) != len(names) or any(
                    unit not in {"rad", "m"} for unit in options["joint_units"]
                ):
                    raise DataError("JOINT_UNITS_INVALID")
                for key in ("observation.state", "action"):
                    features[key]["units"] = options["joint_units"]
            for key in cameras:
                features[key] = {
                    "dtype": "video",
                    "shape": frame[key].shape,
                    "names": ["height", "width", "channels"],
                }
            kwargs = dict(
                repo_id=options.get("repo_id", "local/hc_export"),
                root=Path(output),
                fps=fps,
                robot_type=options.get("robot_type", "x1"),
                features=features,
                use_videos=bool(cameras),
                vcodec="h264",
                video_backend="pyav",
            )
            # Preserve compatibility with a fixed v3 release without requiring
            # recently introduced optional streaming encoder arguments.
            if (
                "streaming_encoding"
                in inspect.signature(LeRobotDataset.create).parameters
            ):
                kwargs["streaming_encoding"] = True
            dataset = LeRobotDataset.create(**kwargs)
        dataset.add_frame(frame)
        audit.write(
            json.dumps(
                {
                    "event": "frame",
                    "valid": True,
                    "episode_index": episode_index,
                    "frame_index": episode_frames,
                    "export_time_ns": tick,
                    "sources": {
                        key: {
                            "recorded_time_ns": sample["timestamp_ns"],
                            "source_time_ns": sample["source_ns"],
                            "age_ns": tick - sample["timestamp_ns"],
                            "topic": sample["topic"],
                            "present": True,
                            "valid": True,
                            "control": {
                                name: sample["payload"][name]
                                for name in (
                                    "boot_id",
                                    "epoch",
                                    "control_epoch",
                                    "lease_id",
                                    "reference_epoch",
                                    "seq",
                                    "stage",
                                    "owner",
                                    "source",
                                    "mode",
                                )
                                if isinstance(sample["payload"], dict)
                                and name in sample["payload"]
                            },
                        }
                        for key, sample in latest.items()
                    },
                }
            )
            + "\n"
        )
        samples += 1
        episode_frames += 1
        if episode_frames >= max_episode:
            close_episode("configured_episode_frame_limit")

    try:
        topics = list(streams.values()) + ["/hc/control/intervention"]
        reverse = {value: key for key, value in streams.items()}
        last_stamp = None
        for sample in iter_samples(source, topics=topics):
            stamp = sample["timestamp_ns"]
            if next_tick is None:
                next_tick = stamp
            while next_tick < stamp:
                emit(next_tick)
                next_tick += period
            if sample["topic"] == "/hc/control/intervention":
                payload = sample["payload"]
                if payload.get("confirmed") is not True:
                    raise DataError("UNCONFIRMED_INTERVENTION")
                if not payload["active"]:
                    close_episode("intervention_end")
                intervention_active = bool(payload["active"])
            elif sample["topic"] in reverse:
                latest[reverse[sample["topic"]]] = sample
            last_stamp = stamp
        if last_stamp is not None and next_tick <= last_stamp:
            emit(next_tick)
        if dataset is None or samples == 0:
            raise DataError(
                "NO_VALID_ALIGNED_SAMPLES",
                "Check explicit topics, joint names and maximum sample age",
            )
        close_episode("capture_end")
        dataset.finalize()
        # Reopening with the official reader catches broken writers/footers and
        # incompatible features before the parent can publish the artifact.
        loaded = LeRobotDataset(
            repo_id=options.get("repo_id", "local/hc_export"),
            root=Path(output),
            download_videos=False,
            video_backend="pyav",
        )
        if len(loaded) != samples:
            raise DataError("LEROBOT_FRAME_COUNT_MISMATCH")
        result = {
            "schema_version": "hc-conversion/v1",
            "writer_version": version,
            "fps": fps,
            "joint_names": names,
            "topics": streams,
            "exported_frames": samples,
            "skipped_ticks": skipped,
            "sync_rule": "causal_hold",
            "max_age_ns": max_age,
            "missing_rule": "split_episode",
            "action_semantics": "commanded_target_not_hardware_execution",
            "task_success": None,
        }
        atomic_json(Path(output) / "conversion.json", result)
        audit.close()
        audit_path.replace(Path(output) / "sample-provenance.jsonl")
        return result
    finally:
        if not audit.closed:
            audit.close()


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--input")
    parser.add_argument("--output")
    parser.add_argument("--options")
    parser.add_argument("--validate-only")
    args = parser.parse_args()
    if args.validate_only:
        from .artifacts import validate_lerobot

        print(json.dumps(validate_lerobot(Path(args.validate_only))), flush=True)
        return
    if not args.input or not args.output or not args.options:
        parser.error("--input, --output and --options are required for conversion")
    try:
        result = convert(
            args.input, args.output, json.loads(Path(args.options).read_text())
        )
        print(json.dumps(result), flush=True)
    except BaseException as error:
        atomic_json(
            Path(args.output).parent / "conversion-error.json",
            {
                "code": getattr(error, "code", "CONVERSION_FAILED"),
                "type": type(error).__name__,
            },
        )
        traceback.print_exc()
        raise SystemExit(1)


if __name__ == "__main__":
    main()

"""CPU-heavy artifact sealing functions executed outside the media process."""
import os
from pathlib import Path

from humanoid_upload.bundle import checksum
from humanoid_upload.common import UploadError, atomic_json, read_json

from .capture_formats import artifact_identity, artifact_snapshot, upload_formats


def lower_artifact_priority() -> None:
    try:
        os.nice(10)
    except OSError:
        pass


def seal_task_artifact(job, fmt, directory, state_dir, cfg):
    from .task_recording import now

    directory, state_dir = Path(directory), Path(state_dir)
    identity = artifact_identity(job, fmt)
    source = Path(job["path"])
    snapshot = artifact_snapshot(job, fmt)
    if fmt == "lerobot":
        root = Path(job.get("dataset_path") or directory / "lerobot" / source.stem)
        if snapshot:
            from humanoid_upload.capture_plan import validate_lerobot_info

            presence = validate_lerobot_info(
                snapshot, read_json(root / "meta/info.json")
            )
            context = {
                "schema_version": "hc-capture-context/v1",
                "recording_id": identity,
                "capture_snapshot": snapshot,
                "feature_presence": presence,
            }
            context_path = root / "capture-context.json"
            if context_path.exists() and read_json(context_path) != context:
                raise UploadError(
                    "CAPTURE_SNAPSHOT_MISMATCH", "已有 LeRobot 采集快照不能覆盖"
                )
            if not context_path.exists():
                atomic_json(context_path, context)
        from .recording_datasets import dataset_files

        files = dataset_files(root)
        kind = "hc_lerobot"
    elif fmt == "mcap_mp4":
        from .video_bundle import prepare
        from .recording_datasets import dataset_files

        root = prepare(
            source,
            directory / "bundles" / source.stem,
            {**job, "capture_snapshot": snapshot},
        )
        if not job.get("binding"):
            return "ready", False, identity, None
        files, kind = dataset_files(root), "platform_raw"
    else:
        root, files, kind = source.parent, [source], "hc_mcap"

    descriptor_path = state_dir / (identity + ".json")
    if descriptor_path.exists():
        descriptor = read_json(descriptor_path)
        if descriptor.get("capture_snapshot") != snapshot:
            raise UploadError(
                "CAPTURE_SNAPSHOT_MISMATCH",
                "已封存方案快照不能改变或由当前默认值补写",
            )
        if job["binding"]:
            binding_id = job["binding"]["binding_id"]
            if descriptor.get("binding_id") not in (None, binding_id):
                raise UploadError("TASK_BINDING_MISMATCH", "已封存归属不能改变")
            if not descriptor.get("binding_id"):
                descriptor.update(
                    binding_id=binding_id,
                    ownership_snapshot=job["binding"]["task"]["ownership"],
                )
                atomic_json(descriptor_path, descriptor)
    else:
        descriptor = {
            "schema_version": "hc-artifact-ready/v1",
            "recording_id": identity,
            "artifact_id": identity,
            "kind": kind,
            "root": str(root),
            "files": [
                {"path": str(path.relative_to(root)), **checksum(path)}
                for path in files
            ],
            "profile_id": job["local_robot"]["profile_id"],
            "config_fingerprint": job["local_robot"]["config_fingerprint"],
            "source_artifact_id": (
                job["recording_id"] if identity != job["recording_id"] else None
            ),
            "native_session_id": (
                job["recording_id"] if fmt == "mcap_mp4" else None
            ),
            "native_completion_marker": (
                "recording-complete.json" if fmt == "mcap_mp4" else None
            ),
            "binding_id": job["binding"]["binding_id"] if job["binding"] else None,
            "capture_started_at": job["started_at"],
            "capture_ended_at": job["ended_at"],
            "completed_at": now(),
        }
        if job["binding"]:
            descriptor["ownership_snapshot"] = job["binding"]["task"]["ownership"]
        if snapshot:
            descriptor["capture_snapshot"] = snapshot
        from humanoid_upload.artifacts import inspect_ready

        inspect_ready(descriptor, cfg)
        atomic_json(descriptor_path, descriptor)

    upload = (
        job.get("upload_requested")
        and fmt in job.get("upload_formats", upload_formats(job))
    ) or bool(
        job["binding"]
        and job["binding"]["auto_upload"]
        and fmt in upload_formats(job)
    )
    return (
        "queued" if upload else "ready",
        upload,
        identity,
        job["binding"]["binding_id"] if job["binding"] else None,
    )

"""Immutable artifact seals, bounded inspection and filesystem safety."""

import hashlib
import json
import os
import stat
import subprocess
import zipfile
from pathlib import Path, PurePosixPath

from .errors import DataError


def canonical(value):
    return json.dumps(
        value,
        sort_keys=True,
        separators=(",", ":"),
        ensure_ascii=False,
        allow_nan=False,
    )


def atomic_json(path, value):
    path = Path(path)
    temporary = path.with_name(path.name + ".tmp")
    with temporary.open("w", encoding="utf-8") as stream:
        stream.write(canonical(value))
        stream.flush()
        os.fsync(stream.fileno())
    os.replace(temporary, path)
    sync_directory(path.parent)


def sync_directory(path):
    descriptor = os.open(path, os.O_RDONLY | os.O_DIRECTORY)
    try:
        os.fsync(descriptor)
    finally:
        os.close(descriptor)


def safe_asset(root, relative):
    if (
        not isinstance(relative, str)
        or not relative
        or "\\" in relative
        or "\0" in relative
        or PurePosixPath(relative).is_absolute()
        or any(p in {"", ".", ".."} for p in relative.split("/"))
    ):
        raise DataError("UNSAFE_ARTIFACT_PATH")
    root = Path(root).resolve()
    result = root / relative
    current = root
    for component in relative.split("/"):
        current /= component
        if current.is_symlink():
            raise DataError("ARTIFACT_SYMLINK_REJECTED")
    if not result.resolve().is_relative_to(root):
        raise DataError("UNSAFE_ARTIFACT_PATH")
    return result


def freeze_robot_package(source, destination, expected_sha256=None, package_hash=None):
    """Copy a supervisor-owned archive without trusting a mutable pathname.

    The HTTP layer must replace the source with its registry export. No package
    is extracted here; path validation also keeps the portable copy safe for a
    future importer. Run in a worker thread before arming the recorder.
    """
    destination = Path(destination)
    temporary = destination.with_suffix(".zip.partial")
    try:
        descriptor = os.open(source, os.O_RDONLY | os.O_NOFOLLOW | os.O_NONBLOCK)
        with os.fdopen(descriptor, "rb") as src, temporary.open("xb") as dst:
            before = os.fstat(src.fileno())
            if not stat.S_ISREG(before.st_mode):
                raise DataError(
                    "ROBOT_PACKAGE_INVALID",
                    "Configuration archive must be a regular file",
                )
            digest, size = hashlib.sha256(), 0
            for block in iter(lambda: src.read(1024 * 1024), b""):
                digest.update(block)
                dst.write(block)
                size += len(block)
            after = os.fstat(src.fileno())
            if (
                (before.st_size, before.st_mtime_ns, before.st_ctime_ns)
                != (after.st_size, after.st_mtime_ns, after.st_ctime_ns)
            ) or size != before.st_size:
                raise DataError("ROBOT_PACKAGE_CHANGED")
            sha256 = digest.hexdigest()
            if expected_sha256 is not None and expected_sha256 != sha256:
                raise DataError("ROBOT_PACKAGE_HASH_MISMATCH")
            dst.flush()
            os.fsync(dst.fileno())
        with zipfile.ZipFile(temporary) as archive:
            entries = archive.infolist()
            if not entries:
                raise DataError(
                    "ROBOT_PACKAGE_INVALID", "Configuration archive is empty"
                )
            for entry in entries:
                name = entry.filename.rstrip("/")
                if (
                    not name
                    or name.startswith("/")
                    or "\\" in name
                    or "\0" in name
                    or any(part in {"", ".", ".."} for part in name.split("/"))
                    or stat.S_ISLNK(entry.external_attr >> 16)
                ):
                    raise DataError("ROBOT_PACKAGE_INVALID", "Unsafe archive entry")
        os.replace(temporary, destination)
        sync_directory(destination.parent)
        return {
            "path": destination.name,
            "sha256": sha256,
            "size_bytes": size,
            "package_hash": package_hash,
        }
    except (OSError, ValueError, zipfile.BadZipFile) as error:
        raise DataError("ROBOT_PACKAGE_INVALID", str(error)) from error
    finally:
        temporary.unlink(missing_ok=True)


def _crc_table():
    result = []
    for byte in range(256):
        for _ in range(8):
            byte = (byte >> 1) ^ (0xC96C5795D7870F42 if byte & 1 else 0)
        result.append(byte)
    return result


CRC64 = _crc_table()


def checksum(path):
    sha, crc, size = hashlib.sha256(), (1 << 64) - 1, 0
    with Path(path).open("rb") as stream:
        before = os.fstat(stream.fileno())
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            sha.update(block)
            for byte in block:
                crc = CRC64[(crc ^ byte) & 255] ^ (crc >> 8)
            size += len(block)
        after = os.fstat(stream.fileno())
    if (before.st_size, before.st_mtime_ns, before.st_ctime_ns) != (
        after.st_size,
        after.st_mtime_ns,
        after.st_ctime_ns,
    ):
        raise DataError("ARTIFACT_CHANGED")
    return {
        "size_bytes": size,
        "sha256": sha.hexdigest(),
        "crc64": str(crc ^ ((1 << 64) - 1)),
    }


def seal_directory(root, manifest):
    root = Path(root)
    inventory = []
    for path in sorted(root.rglob("*")):
        if path.is_symlink():
            raise DataError("ARTIFACT_SYMLINK_REJECTED")
        if not path.is_file() or path.name in {"manifest.json", "manifest.json.tmp"}:
            continue
        relative = path.relative_to(root).as_posix()
        inventory.append({"path": relative, **checksum(path)})
    if not inventory:
        raise DataError("EMPTY_ARTIFACT")
    sealed = {**manifest, "schema_version": "hc-artifact/v1", "assets": inventory}
    sealed["content_sha256"] = hashlib.sha256(canonical(inventory).encode()).hexdigest()
    atomic_json(root / "manifest.json", sealed)
    return sealed


def verify_directory(root, manifest):
    inventory = manifest.get("assets", [])
    if not inventory:
        raise DataError("EMPTY_ARTIFACT")
    names = set()
    for item in inventory:
        if item["path"] in names:
            raise DataError("DUPLICATE_ARTIFACT_PATH")
        names.add(item["path"])
        path = safe_asset(root, item["path"])
        if not path.is_file() or checksum(path) != {
            k: item[k] for k in ("size_bytes", "sha256", "crc64")
        }:
            raise DataError("ARTIFACT_CHANGED")
    expected = hashlib.sha256(canonical(inventory).encode()).hexdigest()
    if manifest.get("content_sha256") != expected:
        raise DataError("MANIFEST_CHANGED")


def inspect_mcap(path):
    try:
        from mcap.reader import make_reader
    except ImportError:
        raise DataError(
            "MCAP_UNAVAILABLE", "Install the optional mcap dependency"
        ) from None
    with Path(path).open("rb") as stream:
        reader = make_reader(stream, validate_crcs=True)
        topics, count, first, last = {}, 0, None, None
        for schema, channel, message in reader.iter_messages():
            count += 1
            first = message.log_time if first is None else min(first, message.log_time)
            last = message.log_time if last is None else max(last, message.log_time)
            topics[channel.topic] = topics.get(channel.topic, 0) + 1
    return {
        "message_count": count,
        "topics": topics,
        "started_ns": first,
        "ended_ns": last,
        "duration_ns": 0 if first is None else last - first,
    }


def validate_lerobot(root):
    root = Path(root)
    try:
        info = json.loads((root / "meta/info.json").read_text())
    except (OSError, ValueError):
        raise DataError(
            "LEROBOT_INVALID", "Missing or invalid meta/info.json"
        ) from None
    if (
        info.get("codebase_version") != "v3.0"
        or not info.get("features")
        or int(info.get("total_frames", 0)) <= 0
    ):
        raise DataError("LEROBOT_INVALID", "Expected nonempty LeRobot v3.0 data")
    if not list((root / "data").rglob("*.parquet")) or not list(
        (root / "meta/episodes").rglob("*.parquet")
    ):
        raise DataError("LEROBOT_INVALID", "Missing frame or episode shards")
    try:
        import pyarrow.parquet as pq
    except ImportError:
        raise DataError(
            "LEROBOT_VALIDATOR_UNAVAILABLE",
            "Install pyarrow to validate converter output",
        ) from None
    frames = sum(
        pq.ParquetFile(path).metadata.num_rows
        for path in (root / "data").rglob("*.parquet")
    )
    if frames != int(info["total_frames"]):
        raise DataError("LEROBOT_FRAME_COUNT_MISMATCH")
    for key, feature in info["features"].items():
        if feature.get("dtype") == "video" and not list(
            (root / "videos" / key).rglob("*.mp4")
        ):
            raise DataError("LEROBOT_VIDEO_MISSING", key)
    for video in (root / "videos").rglob("*.mp4"):
        try:
            probe = subprocess.run(
                [
                    "ffprobe",
                    "-v",
                    "error",
                    "-show_entries",
                    "format=duration",
                    "-of",
                    "json",
                    str(video),
                ],
                capture_output=True,
                timeout=30,
                check=True,
            )
            if float(json.loads(probe.stdout)["format"]["duration"]) <= 0:
                raise ValueError("empty")
        except FileNotFoundError:
            raise DataError(
                "VIDEO_VALIDATOR_UNAVAILABLE", "ffprobe is required for video datasets"
            ) from None
        except (ValueError, KeyError, subprocess.SubprocessError):
            raise DataError("LEROBOT_VIDEO_INVALID", video.name) from None
    return info

"""Check the A→D seal, byte inventory and identity without a training stack."""
import hashlib
import json
import os
import stat
import struct
from datetime import datetime
from pathlib import Path, PurePosixPath

from .common import Limiter, UploadError, canonical, read_json, validate


def _crc_table():
    table = []
    for value in range(256):
        for _ in range(8):
            value = (value >> 1) ^ (0xC96C5795D7870F42 if value & 1 else 0)
        table.append(value)
    return table


CRC_TABLE = _crc_table()

import ctypes
_fast_crc64 = None
try:
    _so_path = Path(__file__).parent / "_crc64.so"
    if not _so_path.exists():
        _c_path = Path(__file__).parent / "_crc64.c"
        if _c_path.exists():
            import subprocess
            subprocess.run(["gcc", "-O3", "-shared", "-fPIC", str(_c_path), "-o", str(_so_path)],
                           check=True, capture_output=True)
    if _so_path.exists():
        _clib = ctypes.CDLL(str(_so_path))
        _clib.update_crc64.argtypes = [ctypes.c_uint64, ctypes.c_char_p, ctypes.c_size_t]
        _clib.update_crc64.restype = ctypes.c_uint64
        _fast_crc64 = _clib.update_crc64
except Exception:
    _fast_crc64 = None


def checksum(path, limiter=None):
    sha, crc, size = hashlib.sha256(), (1 << 64)-1, 0
    with open(path, "rb") as stream:
        before = os.fstat(stream.fileno())
        for block in iter(lambda: stream.read(65536), b""):
            sha.update(block)
            if _fast_crc64 is not None:
                crc = _fast_crc64(crc, block, len(block))
            else:
                for byte in block:
                    crc = CRC_TABLE[(crc ^ byte) & 255] ^ (crc >> 8)
            size += len(block)
            if limiter:
                limiter.account(len(block))
        after = os.fstat(stream.fileno())
    if (before.st_size, before.st_mtime_ns, before.st_ctime_ns) != (after.st_size, after.st_mtime_ns, after.st_ctime_ns):
        raise UploadError("LOCAL_CONTENT_CHANGED")
    return dict(size_bytes=size, sha256=sha.hexdigest(), crc64=str(crc ^ ((1 << 64)-1)))


def safe_path(value):
    if (not isinstance(value, str) or not value or PurePosixPath(value).is_absolute()
            or "\\" in value or ":" in value or "\x00" in value
            or any(p in {"", ".", ".."} for p in value.split("/"))):
        raise UploadError("UNSAFE_PATH")


def finalized_mp4(path):
    """Reject unclosed/truncated ISO BMFF containers; A/B own media decoding."""
    size, seen = path.stat().st_size, set()
    with path.open("rb") as stream:
        while stream.tell() < size:
            start = stream.tell()
            raw = stream.read(8)
            if len(raw) != 8:
                raise UploadError("MP4_INCOMPLETE")
            length, kind = struct.unpack(">I4s", raw)
            header = 8
            if length == 1:
                raw = stream.read(8)
                if len(raw) != 8:
                    raise UploadError("MP4_INCOMPLETE")
                length, header = struct.unpack(">Q", raw)[0], 16
            if length < header or start + length > size:
                raise UploadError("MP4_INCOMPLETE")
            seen.add(kind)
            stream.seek(start + length)
    if not {b"ftyp", b"moov", b"mdat"} <= seen:
        raise UploadError("MP4_INCOMPLETE")


def inspect(root, cfg):
    root = Path(root).absolute()
    if any(p.is_symlink() for p in [root, *root.parents]):
        raise UploadError("SYMLINK_ASSET")
    root = root.resolve()
    data = Path(cfg["data_directory"])
    if root != data and data not in root.parents:
        raise UploadError("EXPORT_OUTSIDE_DATA_DIRECTORY")
    if any(p.startswith(".export-") for p in root.parts):
        raise UploadError("EXPORT_INCOMPLETE")
    marker = read_json(root / "export-complete.json")
    validate(marker, "export-complete")
    entries = marker["assets"]
    paths = [a["path"] for a in entries]
    if paths != sorted(set(paths)) or "export-complete.json" in paths:
        raise UploadError("ASSET_INVENTORY")
    actual = []
    for p in root.rglob("*"):
        mode = p.lstat().st_mode
        if stat.S_ISLNK(mode) or not (stat.S_ISREG(mode) or stat.S_ISDIR(mode)):
            raise UploadError("SYMLINK_OR_SPECIAL_ASSET")
        rel = p.relative_to(root).as_posix()
        safe_path(rel)
        if any(part.startswith(".export-") or part.endswith((".part", ".incomplete", ".tmp")) or
               part == "FAILED.json" for part in p.parts):
            raise UploadError("EXPORT_INCOMPLETE")
        if p.is_file() and rel != "export-complete.json":
            actual.append(rel)
    if sorted(actual) != paths or hashlib.sha256(canonical(entries)).hexdigest() != marker["content_sha256"]:
        raise UploadError("ASSET_INVENTORY")
    limiter = Limiter(cfg["read_bytes_per_second"])
    for a in entries:
        safe_path(a["path"])
        if {"path": a["path"], **checksum(root / a["path"], limiter)} != a:
            raise UploadError("ASSET_INTEGRITY", "本地资产已变化；保留旧队列并使用新 export_id 重新导出")
        if a["path"].endswith(".mp4"):
            finalized_mp4(root / a["path"])
    context = read_json(root / "capture-context.json")
    validate(context, "capture-context")
    for key in ("export_id", "session_id", "robot_id", "collection_task_id", "synthetic", "contract_version"):
        if marker[key] != context[key]:
            raise UploadError("IDENTITY_MISMATCH")
    if context["robot_id"] != cfg["robot_id"] or context["local_robot_id"] != cfg["local_robot_id"]:
        raise UploadError("ROBOT_IDENTITY_MISMATCH", "核对平台与本地机器人映射；不能改写历史包身份")
    if context["collection_task_id"] != cfg["task_id"]:
        raise UploadError("TASK_BINDING_MISMATCH", "使用包内冻结任务；换任务须显式重新导出为新 export_id")
    if datetime.fromisoformat(context["capture_ended_at"].replace("Z", "+00:00")) <= datetime.fromisoformat(context["capture_started_at"].replace("Z", "+00:00")):
        raise UploadError("CAPTURE_TIME_INVALID")
    episodes, profile = context["episodes"], context["profile"]
    for i, ep in enumerate(episodes):
        if ep["episode_index"] != i or not ep["export_qualified"] or ep["end_frame"] <= ep["start_frame"]:
            raise UploadError("EPISODE_NOT_QUALIFIED")
        if any(x["start_frame"] < ep["end_frame"] and x["end_frame"] > ep["start_frame"] for x in context["invalid_intervals"]):
            raise UploadError("EPISODE_NOT_QUALIFIED")
    info = read_json(root / "meta/info.json")
    axes = [a["name"] for a in profile["axes"]]
    if info.get("codebase_version") != "v3.0" or info.get("fps") != profile["fps"] or info.get("total_episodes") != len(episodes):
        raise UploadError("LEROBOT_METADATA_INVALID")
    if len(set(axes)) != len(axes):
        raise UploadError("AXES_INVALID")
    for name in ("action", "observation.state"):
        feature = info["features"][name]
        if feature["names"] != axes or feature["shape"] != [len(axes)] or feature["dtype"] != "float32":
            raise UploadError("FEATURE_IDENTITY")
    count = 0
    with (root / "source_mapping.jsonl").open() as stream:
        for ep in episodes:
            for frame in range(ep["end_frame"] - ep["start_frame"]):
                line = stream.readline(1024*1024)
                try:
                    mapping = json.loads(line)
                except ValueError:
                    raise UploadError("SOURCE_MAPPING_INVALID") from None
                validate(mapping, "source-mapping")
                if (mapping["episode_index"] != ep["episode_index"] or mapping["frame_index"] != frame
                        or mapping["source_episode_id"] != ep["source_episode_id"]
                        or mapping["session_id"] != context["session_id"]
                        or mapping["source_frame"] != ep["start_frame"] + frame):
                    raise UploadError("SOURCE_MAPPING_INVALID")
                for key in ("action_valid", "state_valid"):
                    if len(mapping[key]) != len(axes) or not all(mapping[key]):
                        raise UploadError("SOURCE_VALIDITY_INVALID")
                stamps = mapping["action_source_ns"]
                if len(stamps) != len(axes) or any(t is None or not 0 <= mapping["target_ns"]-t <=
                        profile["validity"][axis["group"]]["max_age_ms"]*1000000 for t, axis in zip(stamps, profile["axes"])):
                    raise UploadError("ACTION_TIMING_INVALID")
                count += 1
        if stream.read(1) or count != info["total_frames"]:
            raise UploadError("SOURCE_MAPPING_COUNT")
    marker_asset = {"path": "export-complete.json", **checksum(root / "export-complete.json", limiter)}
    return root, marker, context, info, [*entries, marker_asset]


def manifest_for(root, cfg, client_id):
    if (Path(root)/'recording-complete.json').is_file():
        from .recording_bundle import manifest_for_recording
        return manifest_for_recording(root,cfg,client_id)
    root, marker, context, info, inventory = inspect(root, cfg)
    cameras, assets = [], []
    for index, item in enumerate(inventory):
        a = {**item, "asset_id": f"asset-{index:03d}", "role": "RAW", "metadata": {},
             "media_type": "video/mp4" if item["path"].endswith(".mp4") else "application/octet-stream"}
        for camera in context["profile"]["cameras"]:
            if item["path"].startswith("videos/observation.images." + camera["id"] + "/"):
                a["camera_id"] = camera["id"]
        assets.append(a)
    for camera in context["profile"]["cameras"]:
        matches = [a for a in assets if a.get("camera_id") == camera["id"]]
        if not matches:
            raise UploadError("CAMERA_ASSET_MISSING")
        feature = info["features"]["observation.images." + camera["id"]]
        fps = context["profile"]["fps"]
        cameras.append(dict(camera_id=camera["id"], path=matches[0]["path"],
                            codec=feature["info"]["video.codec"], width=camera["width"], height=camera["height"],
                            declared_fps={"numerator": fps, "denominator": 1},
                            declared_frame_count=info["total_frames"],
                            declared_duration_ns=round(info["total_frames"] * 1e9 / fps),
                            clock_domain="capture-context:" + context["session_id"],
                            capture_started_at=context["capture_started_at"], capture_ended_at=context["capture_ended_at"]))
    manifest = dict(schema_version="robot-ingest/v1", client_upload_id=client_id,
                    collection_task_id=context["collection_task_id"], robot_id=context["robot_id"],
                    source_format="LEROBOT_V3", source_format_version="v3.0", capture_mode="PRESEGMENTED",
                    capture_started_at=context["capture_started_at"], capture_ended_at=context["capture_ended_at"],
                    declared_episode_count=len(context["episodes"]), cameras=cameras, assets=assets,
                    format_metadata={"openarm_capture": dict(contract_version=marker["contract_version"],
                      content_sha256=marker["content_sha256"], export_id=marker["export_id"],
                      synthetic=marker["synthetic"], capture_context_path="capture-context.json")})
    validate(manifest, "robot-ingest-manifest")
    return root, marker, manifest

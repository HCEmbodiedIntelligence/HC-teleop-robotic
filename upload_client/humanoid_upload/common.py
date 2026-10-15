from __future__ import annotations

import json
import os
import re
import time
from functools import lru_cache
from pathlib import Path
from urllib.parse import urlsplit

from jsonschema import Draft202012Validator, FormatChecker


class UploadError(Exception):
    """Only public codes/actions: never log a transport exception or response body."""
    def __init__(self, code, action="检查导出并重新生成完成包", *, retry=False, after=0, conflict=False):
        self.code, self.action, self.retry, self.after = code, action, retry, after
        self.conflict = conflict
        super().__init__(code)


def canonical(value):
    return json.dumps(value, sort_keys=True, separators=(",", ":"), ensure_ascii=False,
                      allow_nan=False).encode()


def read_json(path):
    try:
        with Path(path).open("rb") as stream:
            value = stream.read(16*1024**2+1)
        if len(value) > 16*1024**2:
            raise UploadError("JSON_TOO_LARGE", "单个 JSON 上限 16 MiB；请拆分导出包")
        return json.loads(value)
    except (ValueError, OSError):
        raise UploadError("JSON_UNREADABLE") from None


@lru_cache(maxsize=8)
def validator(name):
    return Draft202012Validator(read_json(Path(__file__).parent / "schemas" /
                                         (name + ".schema.json")), format_checker=FormatChecker())


def validate(value, name):
    if not validator(name).is_valid(value):
        raise UploadError("SCHEMA_INVALID", "检查 " + name + " 与 G0 公共契约是否一致")


def atomic_json(path, value):
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True, mode=0o700)
    import tempfile
    fd, temp = tempfile.mkstemp(dir=path.parent, prefix=".write-")
    try:
        with os.fdopen(fd, "wb") as stream:
            stream.write(canonical(value) + b"\n")
            stream.flush()
            os.fsync(stream.fileno())
        os.replace(temp, path)
        fd = os.open(path.parent, os.O_RDONLY | os.O_DIRECTORY)
        try:
            os.fsync(fd)
        finally:
            os.close(fd)
    finally:
        if os.path.exists(temp):
            os.unlink(temp)


def valid_url(url):
    try:
        p = urlsplit(url)
        if p.scheme not in {"http", "https"} or not p.hostname or p.username or p.password or p.fragment:
            raise ValueError()
        _ = p.port
    except (ValueError, TypeError):
        raise UploadError("INVALID_URL", "配置平台公开 HTTP/HTTPS 地址") from None
    return p


DEFAULTS = dict(concurrency=1, part_size_bytes=8*1024**2, bandwidth_bytes_per_second=4*1024**2,
                read_bytes_per_second=16*1024**2, retry_base_seconds=1, retry_max_seconds=60,
                max_attempts=0, poll_seconds=5, scan_seconds=30, timeout_seconds=30,
                processing_endpoint=False, auto_enqueue=True)


def load_config(path):
    return validate_config(read_json(path))


def validate_config(value):
    required = {"platform_base_url", "credential_ref", "robot_id", "local_robot_id",
                "task_id", "data_directory", "queue_directory"}
    if not isinstance(value, dict) or not required <= value.keys() or value.keys() - required - DEFAULTS.keys():
        raise UploadError("CONFIG_INVALID", "使用 config.example.json 的字段；凭据仅填写引用")
    cfg = {**DEFAULTS, **value}
    for key in required:
        if not isinstance(cfg[key], str) or not cfg[key].strip():
            raise UploadError("CONFIG_INVALID")
    base = valid_url(cfg["platform_base_url"])
    if base.query:
        raise UploadError("CONFIG_INVALID")
    cfg["platform_base_url"] = cfg["platform_base_url"].rstrip("/").removesuffix("/api/v1")
    if not re.fullmatch(r"env:[A-Za-z_][A-Za-z0-9_]*|file:/[^\r\n]+", cfg["credential_ref"]):
        raise UploadError("CREDENTIAL_REFERENCE_INVALID", "使用 env:变量名 或 file:/绝对路径")
    for key in ("data_directory", "queue_directory"):
        cfg[key] = str(Path(cfg[key]).expanduser().resolve())
    data, queue = Path(cfg["data_directory"]), Path(cfg["queue_directory"])
    if data == queue or data in queue.parents or queue in data.parents:
        raise UploadError("QUEUE_INSIDE_DATA", "队列与导出目录必须分开")
    if cfg["concurrency"] != 1:
        raise UploadError("CONCURRENCY_LIMIT", "v1 仅支持一个上传 worker，concurrency 设为 1")
    for key in DEFAULTS.keys() - {"processing_endpoint", "concurrency", "auto_enqueue"}:
        val = cfg[key]
        if isinstance(val, bool) or not isinstance(val, (int, float)) or not 0 <= val < 1e12:
            raise UploadError("CONFIG_INVALID")
    if not 5*1024**2 <= cfg["part_size_bytes"] <= 64*1024**2 or not isinstance(cfg["part_size_bytes"], int):
        raise UploadError("PART_SIZE_INVALID", "分片大小为 5–64 MiB 整数")
    if not (0 < cfg["retry_base_seconds"] <= cfg["retry_max_seconds"] <= 60):
        raise UploadError("RETRY_CONFIG_INVALID")
    if any(cfg[k] <= 0 for k in ("timeout_seconds", "scan_seconds", "poll_seconds",
                                "bandwidth_bytes_per_second", "read_bytes_per_second")):
        raise UploadError("RATE_CONFIG_INVALID")
    if not isinstance(cfg["processing_endpoint"], bool) or not isinstance(cfg["auto_enqueue"],bool) or not isinstance(cfg["max_attempts"], int):
        raise UploadError("CONFIG_INVALID")
    return cfg


def credential(ref):
    if ref.startswith("env:"):
        value = os.environ.get(ref[4:], "")
    else:
        p = Path(ref[5:])
        try:
            if p.is_symlink() or p.stat().st_mode & 0o077 or p.stat().st_uid != os.getuid():
                raise OSError()
            value = p.read_text().strip()
        except OSError:
            raise UploadError("CREDENTIAL_FILE_PERMISSIONS", "凭据文件须由当前用户拥有且权限为 0600") from None
    if not value or "\n" in value or "\r" in value:
        raise UploadError("CREDENTIAL_MISSING", "配置环境变量或受限凭据文件后执行 retry")
    return value


class Limiter:
    def __init__(self, rate):
        self.rate, self.start, self.count = rate, time.monotonic(), 0

    def account(self, size):
        self.count += size
        delay = self.count / self.rate - (time.monotonic() - self.start)
        if delay > 0:
            time.sleep(delay)

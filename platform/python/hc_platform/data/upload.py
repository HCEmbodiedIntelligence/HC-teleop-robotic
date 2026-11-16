"""Independent asynchronous adapter for HC robot-ingest/v1 multipart HTTP.

Only the public protocol is shared with the legacy uploader. No legacy queue,
worker or runtime module is imported here.
"""

import asyncio
import importlib.util
import math
import mimetypes
import os
from datetime import datetime, timezone
from fractions import Fraction
from pathlib import Path
from urllib.parse import quote, urlencode, urlsplit

from .artifacts import safe_asset, verify_directory
from .errors import DataError


def validate_config(config):
    config = dict(config)
    base = config.get("platform_base_url", "").rstrip("/").removesuffix("/api/v1")
    parsed = urlsplit(base)
    if (
        parsed.scheme not in {"https", "http"}
        or not parsed.hostname
        or parsed.username
        or parsed.password
        or parsed.query
        or parsed.fragment
    ):
        raise DataError("PLATFORM_URL_INVALID")
    if (
        parsed.scheme == "http"
        and parsed.hostname not in {"localhost", "127.0.0.1", "::1"}
        and not config.get("allow_insecure_lan")
    ):
        raise DataError("PLATFORM_TLS_REQUIRED")
    if not config.get("credential_env") and not config.get("credential_file"):
        raise DataError("PLATFORM_CREDENTIAL_REFERENCE_REQUIRED")
    if any(
        key in config for key in ("token", "password", "credential", "authorization")
    ):
        raise DataError("PLATFORM_SECRET_INLINE_REJECTED")
    part_size = int(config.get("part_size_bytes", 8 * 1024 * 1024))
    if not 5 * 1024 * 1024 <= part_size <= 64 * 1024 * 1024:
        raise DataError("PART_SIZE_INVALID")
    return {
        **config,
        "platform_base_url": base,
        "part_size_bytes": part_size,
        "timeout_seconds": min(120, max(1, float(config.get("timeout_seconds", 30)))),
    }


def utc(ns):
    return datetime.fromtimestamp(ns / 1e9, timezone.utc).isoformat()


def make_upload_manifest(artifact, job_id, target):
    local = artifact["manifest"]
    session = local.get("session", {})
    if not target.get("robot_id") or not target.get("collection_task_id"):
        raise DataError("UPLOAD_TARGET_REQUIRED")
    assets = []
    for index, item in enumerate(local["assets"]):
        if item["size_bytes"] <= 0:
            raise DataError("UPLOAD_EMPTY_ASSET", item["path"])
        assets.append(
            {
                "asset_id": f"asset-{index:06d}",
                **{k: item[k] for k in ("path", "sha256", "crc64", "size_bytes")},
                "media_type": "application/vnd.mcap"
                if item["path"].endswith(".mcap")
                else mimetypes.guess_type(item["path"])[0]
                or "application/octet-stream",
                "role": "RAW",
                "metadata": {},
            }
        )
    cameras = []
    for camera in local.get("cameras", []):
        asset = next((a for a in assets if a["path"] == camera["path"]), None)
        if not asset:
            raise DataError("CAMERA_ASSET_MISSING")
        asset.update(role="RAW_VIDEO", camera_id=camera["camera_id"])
        fps = Fraction(str(camera["fps"])).limit_denominator(1_000_000)
        cameras.append(
            {
                "camera_id": camera["camera_id"],
                "asset_id": asset["asset_id"],
                "path": camera["path"],
                "codec": camera["codec"],
                "width": camera["width"],
                "height": camera["height"],
                "declared_fps": {
                    "numerator": fps.numerator,
                    "denominator": fps.denominator,
                },
                "declared_frame_count": camera["frame_count"],
                "declared_duration_ns": camera["ended_ns"] - camera["started_ns"],
                "clock_domain": camera["clock_domain"],
                "capture_started_at": utc(camera["started_ns"]),
                "capture_ended_at": utc(camera["ended_ns"]),
            }
        )
    source_formats = {
        "mcap": ("MCAP", "0"),
        "mcap_mp4": ("CAPTURE_BUNDLE", "2"),
        "lerobot": ("LEROBOT_V3", "v3.0"),
    }
    if artifact["format"] not in source_formats:
        raise DataError("UPLOAD_FORMAT_UNSUPPORTED")
    source_format, source_version = source_formats[artifact["format"]]
    manifest = {
        "schema_version": "robot-ingest/v1",
        "client_upload_id": job_id,
        "collection_task_id": target["collection_task_id"],
        "robot_id": target["robot_id"],
        "capture_mode": "PRESEGMENTED"
        if artifact["format"] == "lerobot"
        else "CONTINUOUS",
        "source_format": source_format,
        "source_format_version": source_version,
        "capture_started_at": utc(
            session.get("started_ns", local.get("started_ns", 0))
        ),
        "capture_ended_at": utc(session.get("ended_ns", local.get("ended_ns", 0))),
        "assets": assets,
        "cameras": cameras,
        "format_metadata": {
            "artifact_id": artifact["id"],
            "content_sha256": local["content_sha256"],
        },
    }
    for field in ("organization_id", "project_id", "collection_job_id", "capture_plan"):
        if target.get(field) is not None:
            manifest[field] = target[field]
    return manifest


def _contains(expected, actual, key=""):
    if isinstance(expected, dict):
        return isinstance(actual, dict) and all(
            k in actual and _contains(v, actual[k], k) for k, v in expected.items()
        )
    if isinstance(expected, list):
        return (
            isinstance(actual, list)
            and len(expected) == len(actual)
            and all(_contains(a, b, key) for a, b in zip(expected, actual))
        )
    if key.endswith("_at") and isinstance(expected, str) and isinstance(actual, str):
        try:
            return datetime.fromisoformat(
                expected.replace("Z", "+00:00")
            ) == datetime.fromisoformat(actual.replace("Z", "+00:00"))
        except ValueError:
            pass
    return expected == actual


class PlatformAdapter:
    def __init__(self, config):
        self.config = validate_config(config)
        if importlib.util.find_spec("httpx") is None:
            raise DataError(
                "HTTP_CLIENT_UNAVAILABLE", "Install httpx for platform access"
            )

    def _token(self):
        if self.config.get("credential_env"):
            token = os.environ.get(self.config["credential_env"], "")
        else:
            path = Path(self.config["credential_file"])
            if path.is_symlink() or path.stat().st_mode & 0o077:
                raise DataError(
                    "CREDENTIAL_FILE_PERMISSIONS",
                    "Credential file must be private (0600)",
                )
            token = path.read_text().strip()
        if not token or "\n" in token or "\r" in token:
            raise DataError("PLATFORM_CREDENTIAL_UNAVAILABLE")
        return token

    async def request(self, client, method, endpoint, payload=None, *, anonymous=False):
        import httpx

        try:
            response = await client.request(
                method,
                self.config["platform_base_url"] + "/api/v1/robot-ingest/" + endpoint,
                json=payload,
                headers={}
                if anonymous
                else {"Authorization": "Bearer " + self._token()},
            )
        except httpx.RequestError:
            raise DataError("PLATFORM_NETWORK_UNCERTAIN", retryable=True) from None
        if response.status_code >= 400:
            raise DataError(
                f"PLATFORM_HTTP_{response.status_code}",
                retryable=response.status_code in {408, 429}
                or response.status_code >= 500,
            )
        if response.is_redirect or len(response.content) > 16 * 1024 * 1024:
            raise DataError("PLATFORM_RESPONSE_INVALID")
        try:
            return response.json()
        except ValueError:
            raise DataError("PLATFORM_RESPONSE_INVALID", retryable=True) from None

    def client(self):
        import httpx

        return httpx.AsyncClient(
            timeout=self.config["timeout_seconds"],
            follow_redirects=False,
            limits=httpx.Limits(max_connections=4, max_keepalive_connections=4),
            trust_env=False,
        )

    async def capabilities(self):
        async with self.client() as client:
            value = await self.request(client, "GET", "uploads:capabilities")
        if value.get("schema_version") != "robot-ingest-capabilities/v1":
            raise DataError("PLATFORM_CAPABILITIES_INVALID")
        if (
            self.config.get("robot_id")
            and value.get("robot_id") != self.config["robot_id"]
        ):
            raise DataError("PLATFORM_ROBOT_MISMATCH")
        return value

    async def tasks(self, query=None):
        async with self.client() as client:
            return await self.request(client, "GET", "tasks?" + urlencode(query or {}))

    async def preflight(self, task_id, source_format, local_robot):
        async with self.client() as client:
            value = await self.request(
                client,
                "POST",
                "tasks/" + quote(task_id, safe="") + ":preflight",
                {"source_format": source_format, "local_robot": local_robot},
            )
        if (
            value.get("collection_task_id") != task_id
            or value.get("selectable") is not True
        ):
            raise DataError("PLATFORM_TASK_UNAVAILABLE")
        return value

    async def prepare_recording(self, task_id, command):
        import hashlib
        from .artifacts import canonical

        async with self.client() as client:
            value = await self.request(
                client,
                "POST",
                "tasks/" + quote(task_id, safe="") + ":prepare-recording",
                command,
            )
        expected = {
            "schema_version": "capture-recording-snapshot/v1",
            "collection_task_id": task_id,
            "recording_id": command.get("recording_id"),
            "source_format": command.get("source_format"),
            "plan_revision": command.get("expected_plan_revision"),
        }
        if self.config.get("robot_id"):
            expected["robot_id"] = self.config["robot_id"]
        if any(value.get(key) != item for key, item in expected.items()):
            raise DataError("CAPTURE_SNAPSHOT_IDENTITY_MISMATCH")
        digest = hashlib.sha256(
            canonical(
                {key: item for key, item in value.items() if key != "snapshot_sha256"}
            ).encode()
        ).hexdigest()
        output_hash = hashlib.sha256(
            canonical(value.get("output")).encode()
        ).hexdigest()
        if digest != value.get("snapshot_sha256") or output_hash != value.get(
            "output_contract_sha256"
        ):
            raise DataError("CAPTURE_SNAPSHOT_HASH_MISMATCH")
        return value

    async def refresh(self, manifest, saved):
        if not saved.get("upload_id"):
            raise DataError(
                "UPLOAD_REMOTE_ID_UNAVAILABLE",
                "Retry transfer with the original job UUID to reconcile creation",
            )
        async with self.client() as client:
            endpoint = "uploads/" + quote(saved["upload_id"], safe="")
            remote = self._validate_upload(
                await self.request(client, "GET", endpoint), manifest
            )
            result = {
                **saved,
                "remote_state": remote["state"],
                "processing_status": remote.get("processing_status", "UNKNOWN"),
                "quality_status": remote.get("quality_status", "UNKNOWN"),
                "raw_source_id": remote.get("raw_source_id"),
            }
            if (
                remote["state"] == "COMMITTED"
                and self.config.get("processing_endpoint")
                and manifest["source_format"] == "LEROBOT_V3"
            ):
                processing = await self.request(client, "GET", endpoint + "/processing")
                if processing.get("upload_id") != saved["upload_id"] or processing.get(
                    "raw_source_id"
                ) != remote.get("raw_source_id"):
                    raise DataError("PROCESSING_IDENTITY_MISMATCH")
                result["processing"] = processing
                result["processing_status"] = processing.get(
                    "processing_status", "UNKNOWN"
                )
                result["quality_status"] = processing.get("quality_status", "UNKNOWN")
            return result

    def _validate_upload(self, response, manifest):
        remote = response.get("data", {})
        if (
            remote.get("client_upload_id") != manifest["client_upload_id"]
            or remote.get("authenticated_robot_id") != manifest["robot_id"]
            or remote.get("request_robot_id") != manifest["robot_id"]
            or remote.get("target", {}).get("collection_task_id")
            != manifest["collection_task_id"]
            or not _contains(manifest, remote.get("manifest"))
            or not remote.get("upload_id")
        ):
            raise DataError("PLATFORM_UPLOAD_IDENTITY_MISMATCH")
        items = {a["asset_id"]: a for a in manifest["assets"]}
        if len(remote.get("assets", [])) != len(items):
            raise DataError("PLATFORM_ASSET_MISMATCH")
        for asset in remote["assets"]:
            expected = items.get(asset.get("asset_id"))
            if (
                expected is None
                or asset.get("path") != expected["path"]
                or any(
                    asset.get("expected_" + k) != expected[k]
                    for k in ("sha256", "crc64", "size_bytes")
                )
            ):
                raise DataError("PLATFORM_ASSET_MISMATCH")
        return remote

    async def upload(self, artifact, job_id, target, saved, checkpoint, cancelled):
        """Reconcile on every attempt. POST uses the same immutable UUID/manifest."""
        import httpx

        await asyncio.to_thread(
            verify_directory, artifact["root"], artifact["manifest"]
        )
        manifest = make_upload_manifest(artifact, job_id, target)
        if saved.get("upload_manifest") and saved["upload_manifest"] != manifest:
            raise DataError("UPLOAD_MANIFEST_CHANGED")
        saved = {
            **saved,
            "upload_manifest": manifest,
            "part_size_bytes": saved.get(
                "part_size_bytes", self.config["part_size_bytes"]
            ),
        }
        await checkpoint(saved)
        async with self.client() as client:
            await cancelled()
            response = await self.request(client, "POST", "uploads", manifest)
            remote = self._validate_upload(response, manifest)
            endpoint = "uploads/" + quote(remote["upload_id"], safe="")
            saved.update(
                upload_id=remote["upload_id"],
                remote_state=remote["state"],
                processing_status=remote.get("processing_status", "UNKNOWN"),
                quality_status=remote.get("quality_status", "UNKNOWN"),
            )
            await checkpoint(saved)
            if remote["state"] == "COMMITTED":
                return saved
            if remote["state"] not in {"UPLOADING", "READY_TO_COMMIT"}:
                raise DataError("PLATFORM_UPLOAD_NOT_WRITABLE", remote["state"])
            remote_assets = {a["asset_id"]: a for a in remote["assets"]}
            for asset in manifest["assets"]:
                await cancelled()
                if remote_assets[asset["asset_id"]]["state"] == "COMPLETED":
                    continue
                part_size = max(
                    saved["part_size_bytes"], math.ceil(asset["size_bytes"] / 10000)
                )
                if part_size > 64 * 1024 * 1024:
                    raise DataError(
                        "ASSET_TOO_LARGE",
                        "Multipart plan exceeds the bounded part buffer",
                    )
                count = math.ceil(asset["size_bytes"] / part_size)
                asset_endpoint = (
                    endpoint + "/assets/" + quote(asset["asset_id"], safe="")
                )
                for number in range(1, count + 1):
                    await cancelled()
                    grant = await self.request(
                        client,
                        "POST",
                        asset_endpoint + ":authorize-parts",
                        {"part_numbers": [number]},
                    )
                    if (
                        grant.get("upload_id") != remote["upload_id"]
                        or grant.get("asset_id") != asset["asset_id"]
                    ):
                        raise DataError("PART_IDENTITY_MISMATCH")
                    parts = {
                        p["part_number"]: p for p in grant.get("uploaded_parts", [])
                    }
                    if len(parts) != len(grant.get("uploaded_parts", [])):
                        raise DataError("PART_LIST_INVALID")
                    length = min(
                        part_size, asset["size_bytes"] - (number - 1) * part_size
                    )
                    if number in parts:
                        if parts[number].get("size_bytes") != length:
                            raise DataError("PART_SIZE_MISMATCH")
                        continue
                    grants = [
                        p
                        for p in grant.get("authorizations", [])
                        if p.get("part_number") == number
                    ]
                    if len(grants) != 1:
                        raise DataError("PART_AUTHORIZATION_INVALID")
                    url = grants[0]["url"]
                    parsed, origin = (
                        urlsplit(url),
                        urlsplit(self.config["platform_base_url"]),
                    )
                    if (
                        parsed.scheme not in {"https", "http"}
                        or not parsed.hostname
                        or parsed.username
                        or parsed.password
                    ):
                        raise DataError("PART_URL_INVALID")
                    if parsed.scheme == "http" and origin.scheme == "https":
                        raise DataError("PART_TLS_DOWNGRADE")
                    if parsed.hostname in {
                        "localhost",
                        "127.0.0.1",
                        "::1",
                    } and origin.hostname not in {"localhost", "127.0.0.1", "::1"}:
                        raise DataError("PART_URL_LOOPBACK_REJECTED")

                    def read_part():
                        with safe_asset(artifact["root"], asset["path"]).open(
                            "rb"
                        ) as stream:
                            stream.seek((number - 1) * part_size)
                            return stream.read(length)

                    body = await asyncio.to_thread(read_part)
                    if len(body) != length:
                        raise DataError("ARTIFACT_CHANGED")
                    try:
                        # Never send the platform bearer to a presigned object URL.
                        put = await client.put(
                            url, content=body, headers={"Content-Length": str(length)}
                        )
                    except httpx.RequestError:
                        raise DataError("PART_PUT_UNCERTAIN", retryable=True) from None
                    if (
                        put.is_redirect
                        or put.status_code >= 400
                        or not put.headers.get("etag")
                    ):
                        raise DataError("PART_PUT_UNCERTAIN", retryable=True)
                    saved.update(last_asset=asset["asset_id"], last_part=number)
                    await checkpoint(saved)
                grant = await self.request(
                    client,
                    "POST",
                    asset_endpoint + ":authorize-parts",
                    {"part_numbers": [1]},
                )
                if (
                    grant.get("upload_id") != remote["upload_id"]
                    or grant.get("asset_id") != asset["asset_id"]
                ):
                    raise DataError("PART_IDENTITY_MISMATCH")
                parts = sorted(
                    grant.get("uploaded_parts", []), key=lambda p: p["part_number"]
                )
                if [p["part_number"] for p in parts] != list(
                    range(1, count + 1)
                ) or any(
                    p.get("size_bytes")
                    != min(
                        part_size,
                        asset["size_bytes"] - (p["part_number"] - 1) * part_size,
                    )
                    or not p.get("etag")
                    for p in parts
                ):
                    raise DataError("PART_LIST_INCOMPLETE", retryable=True)
                self._validate_upload(
                    await self.request(
                        client,
                        "POST",
                        asset_endpoint + ":complete",
                        {
                            "parts": [
                                {"part_number": p["part_number"], "etag": p["etag"]}
                                for p in parts
                            ]
                        },
                    ),
                    manifest,
                )
            await cancelled()
            await asyncio.to_thread(
                verify_directory, artifact["root"], artifact["manifest"]
            )
            remote = self._validate_upload(
                await self.request(client, "POST", endpoint + ":commit"), manifest
            )
            saved.update(
                remote_state=remote["state"],
                processing_status=remote.get("processing_status", "UNKNOWN"),
                quality_status=remote.get("quality_status", "UNKNOWN"),
                raw_source_id=remote.get("raw_source_id"),
            )
            if remote["state"] != "COMMITTED":
                raise DataError("COMMIT_UNCERTAIN", retryable=True)
            await checkpoint(saved)
            return saved

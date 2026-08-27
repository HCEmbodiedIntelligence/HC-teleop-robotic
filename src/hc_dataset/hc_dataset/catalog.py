from __future__ import annotations

import json
import os
import re
import secrets
from dataclasses import dataclass
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Mapping


DATASET_ID = re.compile(r"^[a-z0-9][a-z0-9_-]{0,95}$")


class DatasetError(ValueError):
    """Invalid dataset metadata or an unsafe catalog path."""


@dataclass(frozen=True)
class DatasetRecord:
    dataset_id: str
    directory: Path
    bag_directory: Path
    manifest_path: Path
    manifest: dict[str, Any]


def _slug(value: str) -> str:
    candidate = re.sub(r"[^a-z0-9_-]+", "-", value.strip().lower()).strip("-_")
    return candidate[:40] or "dataset"


class DatasetCatalog:
    """Small on-disk catalog; no worker or ROS subscription exists while idle."""

    def __init__(self, root: str | Path):
        self.root = Path(root).expanduser().resolve()
        self.root.mkdir(parents=True, exist_ok=True)
        if not self.root.is_dir():
            raise DatasetError(f"dataset root is not a directory: {self.root}")

    def create(
        self,
        name: str,
        topics: list[str],
        metadata: Mapping[str, Any] | None = None,
        *,
        storage_id: str = "mcap",
    ) -> DatasetRecord:
        clean_topics = self._validate_topics(topics)
        if not isinstance(name, str) or not name.strip():
            raise DatasetError("dataset name must be non-empty")
        now = datetime.now(timezone.utc)
        dataset_id = (
            f"{now.strftime('%Y%m%dt%H%M%S%fz')}-{_slug(name)}-"
            f"{secrets.token_hex(3)}"
        )
        directory = self._resolve_id(dataset_id)
        directory.mkdir(mode=0o750)
        manifest = {
            "schema": "hc-dataset/v1",
            "dataset_id": dataset_id,
            "name": name.strip(),
            "state": "recording",
            "created_at": now.isoformat(),
            "completed_at": None,
            "storage_id": storage_id,
            "topics": clean_topics,
            "metadata": dict(metadata or {}),
            "message_count": 0,
            "bytes_written": 0,
            "error": None,
        }
        self._write_manifest(directory, manifest)
        return self.get(dataset_id)

    def get(self, dataset_id: str) -> DatasetRecord:
        directory = self._resolve_id(dataset_id)
        manifest_path = directory / "manifest.json"
        if not manifest_path.is_file():
            raise DatasetError(f"dataset does not exist: {dataset_id}")
        try:
            value = json.loads(manifest_path.read_text(encoding="utf-8"))
        except (OSError, json.JSONDecodeError) as error:
            raise DatasetError(f"invalid dataset manifest: {error}") from error
        if value.get("schema") != "hc-dataset/v1" or value.get("dataset_id") != dataset_id:
            raise DatasetError("dataset manifest identity mismatch")
        return DatasetRecord(dataset_id, directory, directory / "bag", manifest_path, value)

    def finalize(
        self,
        dataset_id: str,
        *,
        state: str,
        message_count: int = 0,
        bytes_written: int = 0,
        error: str | None = None,
    ) -> DatasetRecord:
        if state not in {"complete", "canceled", "error"}:
            raise DatasetError(f"invalid final dataset state: {state}")
        record = self.get(dataset_id)
        value = dict(record.manifest)
        value.update(
            state=state,
            completed_at=datetime.now(timezone.utc).isoformat(),
            message_count=max(0, int(message_count)),
            bytes_written=max(0, int(bytes_written)),
            error=error,
        )
        self._write_manifest(record.directory, value)
        return self.get(dataset_id)

    def list(self) -> list[DatasetRecord]:
        result: list[DatasetRecord] = []
        for manifest in sorted(self.root.glob("*/manifest.json"), reverse=True):
            try:
                result.append(self.get(manifest.parent.name))
            except DatasetError:
                continue
        return result

    def _resolve_id(self, dataset_id: str) -> Path:
        if not isinstance(dataset_id, str) or not DATASET_ID.fullmatch(dataset_id):
            raise DatasetError("invalid dataset id")
        target = (self.root / dataset_id).resolve()
        try:
            target.relative_to(self.root)
        except ValueError as error:
            raise DatasetError("dataset path escapes its catalog") from error
        return target

    @staticmethod
    def _validate_topics(topics: list[str]) -> list[str]:
        if not isinstance(topics, list) or not topics:
            raise DatasetError("at least one explicit topic is required")
        result: list[str] = []
        for topic in topics:
            if not isinstance(topic, str) or not topic.startswith("/") or " " in topic:
                raise DatasetError("recorded topics must be absolute ROS topic names")
            if topic == "/" or ".." in topic.split("/"):
                raise DatasetError(f"invalid ROS topic: {topic}")
            result.append(topic)
        if len(result) != len(set(result)):
            raise DatasetError("recorded topics must be unique")
        return result

    @staticmethod
    def _write_manifest(directory: Path, value: Mapping[str, Any]) -> None:
        target = directory / "manifest.json"
        temporary = directory / ".manifest.json.tmp"
        try:
            encoded = json.dumps(value, ensure_ascii=False, indent=2, sort_keys=True) + "\n"
            temporary.write_text(encoded, encoding="utf-8")
            os.replace(temporary, target)
        except (OSError, TypeError, ValueError) as error:
            raise DatasetError(f"unable to write dataset manifest: {error}") from error

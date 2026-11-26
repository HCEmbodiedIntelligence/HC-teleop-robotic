"""Persistent sidecar metadata and safe rename helpers for recorded datasets.

Editable annotations (task description, robot display label, tags, notes,
quality status, and star/mark state) live in hidden sibling JSON files outside
the sealed dataset payload so modifying metadata never invalidates upload
checksums or mutates large MCAP/Parquet/MP4 files.
"""
from __future__ import annotations

from datetime import datetime
import json
from pathlib import Path
import re
from typing import Any

from .artifact_state import atomic_json
from .export_progress import progress_path
from .recording_datasets import resolve_dataset
from .takeover_log import takeover_path


VALID_QUALITIES = ("", "good", "review", "discard")
MAX_TASK_LEN = 200
MAX_ROBOT_LABEL_LEN = 100
MAX_NOTES_LEN = 2000
MAX_TAGS_COUNT = 20
MAX_TAG_LEN = 40
MAX_FILENAME_BYTES = 200
_FORBIDDEN_NAME_CHARS = re.compile(r'[<>:"/\\|?*\x00-\x1f]')


class RecordingConflictError(ValueError):
    """Raised when a recording cannot be renamed or edited due to active state/locks."""



def metadata_sidecar_path(target: Path) -> Path:
    """Return the hidden metadata sidecar path for a resolved dataset file/dir."""
    return target.parent / f".{target.name}.meta.json"


def read_dataset_metadata(target: Path) -> dict[str, Any]:
    """Read editable sidecar metadata; missing or malformed files return {}."""
    path = metadata_sidecar_path(target)
    try:
        if path.is_symlink() or not path.is_file():
            return {}
        raw = json.loads(path.read_text(encoding="utf-8"))
        return raw if isinstance(raw, dict) else {}
    except (OSError, ValueError, TypeError):
        return {}


def write_dataset_metadata(target: Path, data: dict[str, Any]) -> dict[str, Any]:
    """Atomically persist sidecar metadata (removing file if completely empty)."""
    path = metadata_sidecar_path(target)
    if path.is_symlink():
        raise ValueError("dataset metadata path cannot be a symlink")
    meaningful_keys = {"task", "robot_label", "tags", "notes", "quality", "marked"}
    has_data = any(
        key in data and (
            (key == "task")
            or (key == "marked" and bool(data[key]))
            or (key == "tags" and bool(data[key]))
            or (key in {"robot_label", "notes", "quality"} and bool(data[key]))
        )
        for key in meaningful_keys
    )
    if not has_data:
        path.unlink(missing_ok=True)
        return {}
    payload = {
        **data,
        "updated_at": datetime.now().strftime("%Y-%m-%d %H:%M:%S"),
    }
    atomic_json(path, payload)
    return payload


def remove_dataset_metadata(target: Path) -> None:
    """Delete the sidecar metadata file for a dataset if present."""
    path = metadata_sidecar_path(target)
    if not path.is_symlink():
        path.unlink(missing_ok=True)


def normalize_tags(value: Any) -> list[str]:
    if value is None:
        return []
    if isinstance(value, str):
        parts = re.split(r"[,，\n]+", value)
    elif isinstance(value, list):
        parts = value
    else:
        raise ValueError("标签必须为字符串列表")
    result: list[str] = []
    seen: set[str] = set()
    for raw in parts:
        if not isinstance(raw, str):
            raise ValueError("标签内容必须为文本")
        tag = raw.strip()
        if not tag:
            continue
        if any(ord(ch) < 32 for ch in tag):
            raise ValueError("标签包含非法控制字符")
        if len(tag) > MAX_TAG_LEN:
            raise ValueError(f"单个标签不能超过 {MAX_TAG_LEN} 个字符")
        key = tag.casefold()
        if key in seen:
            continue
        seen.add(key)
        result.append(tag)
        if len(result) > MAX_TAGS_COUNT:
            raise ValueError(f"最多支持 {MAX_TAGS_COUNT} 个标签")
    return result


def validate_metadata_updates(updates: dict[str, Any]) -> dict[str, Any]:
    """Validate and normalize a partial dataset metadata patch."""
    if not isinstance(updates, dict):
        raise ValueError("请求体必须为 JSON 对象")
    allowed = {
        "filename",
        "task",
        "robot_label",
        "tags",
        "add_tags",
        "remove_tags",
        "notes",
        "quality",
        "marked",
    }
    unknown = set(updates) - allowed
    if unknown:
        raise ValueError(f"包含不支持的字段: {', '.join(sorted(unknown))}")

    cleaned: dict[str, Any] = {}
    if "filename" in updates:
        if not isinstance(updates["filename"], str):
            raise ValueError("文件名必须为文本")
        cleaned["filename"] = updates["filename"].strip()
    if "task" in updates:
        if updates["task"] is None:
            cleaned["task"] = None
        elif not isinstance(updates["task"], str):
            raise ValueError("任务描述必须为文本")
        else:
            task = updates["task"].strip()
            if len(task) > MAX_TASK_LEN:
                raise ValueError(f"任务描述不能超过 {MAX_TASK_LEN} 个字符")
            cleaned["task"] = task
    if "robot_label" in updates:
        if updates["robot_label"] is None:
            cleaned["robot_label"] = ""
        elif not isinstance(updates["robot_label"], str):
            raise ValueError("机器人名称必须为文本")
        else:
            label = updates["robot_label"].strip()
            if len(label) > MAX_ROBOT_LABEL_LEN:
                raise ValueError(f"机器人名称不能超过 {MAX_ROBOT_LABEL_LEN} 个字符")
            cleaned["robot_label"] = label
    if "tags" in updates:
        cleaned["tags"] = normalize_tags(updates["tags"])
    if "add_tags" in updates:
        cleaned["add_tags"] = normalize_tags(updates["add_tags"])
    if "remove_tags" in updates:
        cleaned["remove_tags"] = normalize_tags(updates["remove_tags"])
    if "notes" in updates:
        if updates["notes"] is None:
            cleaned["notes"] = ""
        elif not isinstance(updates["notes"], str):
            raise ValueError("备注必须为文本")
        else:
            notes = updates["notes"].strip()
            if len(notes) > MAX_NOTES_LEN:
                raise ValueError(f"备注不能超过 {MAX_NOTES_LEN} 个字符")
            cleaned["notes"] = notes
    if "quality" in updates:
        raw_quality = "" if updates["quality"] is None else updates["quality"]
        if not isinstance(raw_quality, str) or raw_quality not in VALID_QUALITIES:
            raise ValueError("数据质量仅支持：未评估、合格(good)、待复查(review)、废弃(discard)")
        cleaned["quality"] = raw_quality
    if "marked" in updates:
        if not isinstance(updates["marked"], bool):
            raise ValueError("星标标记必须为布尔值")
        cleaned["marked"] = updates["marked"]
    return cleaned


def validate_new_filename(identifier: str, target: Path, requested_name: str) -> tuple[str, str]:
    """Validate and normalize a rename target, returning (new_name, new_identifier)."""
    name = requested_name.strip()
    if not name:
        raise ValueError("文件名不能为空")
    if name in (".", "..") or name.startswith("."):
        raise ValueError("文件名不能以点号开头")
    if _FORBIDDEN_NAME_CHARS.search(name):
        raise ValueError("文件名不能包含 / \\ : * ? \" < > | 或控制字符")

    prefix = next((p for p in ("lerobot:", "bundle:") if identifier.startswith(p)), "")
    if not prefix:
        orig_suffix = target.suffix
        if orig_suffix in (".mcap", ".jsonl"):
            other_suffix = ".jsonl" if orig_suffix == ".mcap" else ".mcap"
            if name.lower().endswith(other_suffix):
                raise ValueError(f"文件名后缀必须保持为 {orig_suffix}")
            if not name.endswith(orig_suffix):
                name = name + orig_suffix
            stem = name[: -len(orig_suffix)]
            if not stem or stem in (".", "..") or stem.startswith("."):
                raise ValueError("文件名主体无效")

    if len(name.encode("utf-8")) > MAX_FILENAME_BYTES:
        raise ValueError(f"文件名过长（超过 {MAX_FILENAME_BYTES} 字节）")
    return name, f"{prefix}{name}"


def apply_dataset_metadata(item: dict[str, Any], target: Path) -> dict[str, Any]:
    """Merge persisted sidecar metadata onto a dataset list/detail entry."""
    meta = read_dataset_metadata(target)
    detected_profile = str(item.get("profile_display_name", "") or "")
    item["detected_profile_display_name"] = detected_profile

    robot_label = str(meta.get("robot_label", "") or "")
    item["robot_label"] = robot_label
    if robot_label:
        item["profile_display_name"] = robot_label

    if "task" in meta:
        task_val = str(meta.get("task") or "")
        item["task"] = task_val
    else:
        task_val = str(item.get("task", "") or "")
        item["task"] = task_val
    if task_val:
        item["tasks"] = [task_val]
    elif "tasks" not in item or "task" in meta:
        item["tasks"] = []

    tags = meta.get("tags")
    item["tags"] = [str(tag) for tag in tags if isinstance(tag, str) and tag.strip()] if isinstance(tags, list) else []
    item["notes"] = str(meta.get("notes", "") or "")
    quality = str(meta.get("quality", "") or "")
    item["quality"] = quality if quality in VALID_QUALITIES else ""
    item["meta_updated_at"] = str(meta.get("updated_at", "") or "")

    # MCAP/JSONL uses .mark.json as authoritative; LeRobot/Bundle uses sidecar .meta.json.
    if str(item.get("id", "")).startswith(("lerobot:", "bundle:")):
        item["marked"] = bool(meta.get("marked", False))
    return item


def rename_dataset_files(
    directory: Path,
    identifier: str,
    target: Path,
    new_name: str,
    *,
    mark_path_fn=None,
    inspection_cache_path_fn=None,
) -> tuple[Path, str]:
    """Rename a dataset file or directory along with its companion sidecar files."""
    validated_name, new_identifier = validate_new_filename(identifier, target, new_name)
    if validated_name == target.name:
        return target, identifier

    dest = target.parent / validated_name
    if dest.resolve().parent != target.parent.resolve():
        raise ValueError("目标路径越界")
    if dest.exists() or dest.is_symlink():
        raise FileExistsError(f"目标文件名已存在：{validated_name}")

    is_dataset_dir = identifier.startswith(("lerobot:", "bundle:"))

    if not is_dataset_dir:
        # Check companion conflicts before renaming primary file.
        old_takeover = takeover_path(target)
        new_takeover = takeover_path(dest)
        if old_takeover.exists() and (new_takeover.exists() or new_takeover.is_symlink()):
            raise FileExistsError(f"目标接管标注文件已存在：{new_takeover.name}")

    target.rename(dest)

    # Move metadata sidecar.
    old_meta = metadata_sidecar_path(target)
    new_meta = metadata_sidecar_path(dest)
    if old_meta.is_file() and not old_meta.is_symlink():
        old_meta.replace(new_meta)

    if is_dataset_dir:
        if identifier.startswith("lerobot:"):
            old_progress = progress_path(target)
            new_progress = progress_path(dest)
            if old_progress.is_file() and not old_progress.is_symlink():
                old_progress.replace(new_progress)
    else:
        if inspection_cache_path_fn is not None:
            old_cache = inspection_cache_path_fn(target)
            new_cache = inspection_cache_path_fn(dest)
            if old_cache.is_file() and not old_cache.is_symlink():
                old_cache.replace(new_cache)
        if mark_path_fn is not None:
            old_mark = mark_path_fn(target)
            new_mark = mark_path_fn(dest)
            if old_mark.is_file() and not old_mark.is_symlink():
                try:
                    mark_data = json.loads(old_mark.read_text(encoding="utf-8"))
                    if isinstance(mark_data, dict):
                        mark_data["filename"] = dest.name
                        mark_data["path"] = str(dest.resolve())
                        atomic_json(new_mark, mark_data)
                        old_mark.unlink(missing_ok=True)
                    else:
                        old_mark.replace(new_mark)
                except (OSError, ValueError, TypeError):
                    old_mark.replace(new_mark)
        if target.suffix == ".mcap":
            for suffix in (".takeovers.json", ".correction.json", ".session.json", ".lerobot-export.log"):
                old_companion = target.with_suffix(suffix)
                new_companion = dest.with_suffix(suffix)
                if old_companion.is_file() and not old_companion.is_symlink():
                    if suffix == ".takeovers.json":
                        try:
                            payload = json.loads(old_companion.read_text(encoding="utf-8"))
                            if isinstance(payload, dict):
                                payload["recording"] = dest.name
                                atomic_json(new_companion, payload)
                                old_companion.unlink(missing_ok=True)
                                continue
                        except (OSError, ValueError, TypeError):
                            pass
                    old_companion.replace(new_companion)

    return dest, new_identifier


def merge_metadata_updates(
    existing: dict[str, Any],
    cleaned: dict[str, Any],
    *,
    is_dataset_dir: bool,
) -> dict[str, Any]:
    """Merge validated updates into an existing sidecar metadata dict."""
    updated = dict(existing)
    if "task" in cleaned:
        if cleaned["task"] is None:
            updated.pop("task", None)
        else:
            updated["task"] = cleaned["task"]
    if "robot_label" in cleaned:
        if cleaned["robot_label"]:
            updated["robot_label"] = cleaned["robot_label"]
        else:
            updated.pop("robot_label", None)
    if "notes" in cleaned:
        if cleaned["notes"]:
            updated["notes"] = cleaned["notes"]
        else:
            updated.pop("notes", None)
    if "quality" in cleaned:
        if cleaned["quality"]:
            updated["quality"] = cleaned["quality"]
        else:
            updated.pop("quality", None)

    current_tags = normalize_tags(updated.get("tags", []))
    if "tags" in cleaned:
        current_tags = list(cleaned["tags"])
    if "add_tags" in cleaned:
        current_tags = normalize_tags([*current_tags, *cleaned["add_tags"]])
    if "remove_tags" in cleaned:
        remove_keys = {tag.casefold() for tag in cleaned["remove_tags"]}
        current_tags = [tag for tag in current_tags if tag.casefold() not in remove_keys]
    if current_tags:
        updated["tags"] = current_tags
    else:
        updated.pop("tags", None)

    if "marked" in cleaned and is_dataset_dir:
        if cleaned["marked"]:
            updated["marked"] = True
        else:
            updated.pop("marked", None)

    return updated


def resolve_for_update(directory: Path, identifier: str) -> Path:
    return resolve_dataset(directory, identifier)

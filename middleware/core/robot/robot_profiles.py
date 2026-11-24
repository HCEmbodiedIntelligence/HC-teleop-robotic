"""机器人型号配置包（Profile）仓储管理：查询、导入、归档导出与删除。"""
from __future__ import annotations

import copy
from datetime import datetime, timezone
import io
import os
from pathlib import Path
import re
import shutil
import tempfile
from typing import Any
import zipfile

import yaml

from ..config import ConfigError, ROBOT_MIDDLEWARE_DEFAULTS, validate_robot_middleware
from .profile_normalizers import (
    MAX_CONFIG_BYTES,
    STANDARD_TOPICS,
    _component_joint_indices,
    _generate_arm_teleop_config,
    _load_yaml,
    _normalize_io,
    _normalize_teleop,
    _normalize_v23,
    _standard_ros_interface,
    _validate_task_frames,
)
from .urdf_model import (
    MAX_URDF_BYTES,
    RobotProfileError,
    UrdfModel,
    _index,
    _index_list,
    _number_list,
)

PROFILE_ID_PATTERN = re.compile(r"^[A-Za-z0-9][A-Za-z0-9_.-]{0,63}$")
MAX_ARCHIVE_BYTES = 100 * 1024 * 1024
MAX_UNCOMPRESSED_BYTES = 300 * 1024 * 1024
_KNOWN_CONFIG_FILES = (
    "controller_v23.yml",
    "vr_configs.yml",
    "arm_teleop.yaml",
    "middleware.yaml",
)


def validate_profile_id(value: str) -> str:
    value = str(value).strip()
    if not PROFILE_ID_PATTERN.fullmatch(value):
        raise RobotProfileError(
            "profile id must be 1-64 characters using letters, numbers, '.', '_' or '-'"
        )
    return value


def _safe_filename(value: str, suffixes: set[str]) -> str:
    name = str(value).strip()
    if not name or Path(name).name != name or "\\" in name:
        raise RobotProfileError("uploaded filename must not contain a directory")
    if Path(name).suffix.lower() not in suffixes:
        allowed = ", ".join(sorted(suffixes))
        raise RobotProfileError(f"unsupported file extension; expected {allowed}")
    return name


def _dump_yaml(path: Path, data: dict[str, Any]) -> None:
    path.write_text(
        yaml.safe_dump(data, allow_unicode=True, sort_keys=False),
        encoding="utf-8",
    )


def _build_profile_metadata(
    *,
    profile_id: str,
    display_name: str,
    model: UrdfModel,
    urdf_rel: str,
    primary_config: str,
    config_files: list[str],
    summary: dict[str, Any],
    teleop_compatible: bool,
) -> dict[str, Any]:
    return {
        "version": 1,
        "id": profile_id,
        "display_name": display_name,
        "schema": summary["schema"],
        "managed": True,
        "imported_at": datetime.now(timezone.utc).isoformat(),
        "robot_name": model.robot_name,
        "urdf": urdf_rel,
        "primary_config": primary_config,
        "config_files": config_files,
        "link_count": len(model.links),
        "joint_count": len(model.joints),
        "movable_joint_count": len(model.movable_joint_names),
        "free_joint_count": summary["free_joint_count"],
        "task_count": summary["task_count"],
        "arm_count": summary["arm_count"],
        "teleop_compatible": teleop_compatible,
        "warnings": summary["warnings"],
    }


def _commit_profile_dir(
    temp_path: Path,
    target: Path,
    yaml_files: dict[str, dict[str, Any]],
    metadata: dict[str, Any],
) -> None:
    for filename, value in yaml_files.items():
        _dump_yaml(temp_path / filename, value)
    _dump_yaml(temp_path / "profile.yaml", metadata)
    if target.exists():
        shutil.rmtree(target)
    os.rename(temp_path, target)


def _detect_schema_kind(data: dict[str, Any]) -> str | None:
    if isinstance(data.get("arms"), (dict, list)) and (
        "robot" in data or "grippers" in data or "control" in data
    ):
        return "teleop"
    if isinstance(data.get("model"), dict) and "task" in data:
        return "v23"
    if "urdf_path" in data and "arms" in data:
        return "io"
    return None


def _collect_archive_entries(
    zf: zipfile.ZipFile,
) -> tuple[list[tuple[zipfile.ZipInfo, Path]], str | None]:
    infolist = zf.infolist()
    if not infolist:
        raise RobotProfileError("zip archive is empty")

    total_size = 0
    cleaned_entries: list[tuple[zipfile.ZipInfo, Path]] = []
    for info in infolist:
        name = info.filename.replace("\\", "/").strip()
        if not name or name.startswith("/") or ".." in name.split("/"):
            raise RobotProfileError(f"invalid path in archive: {info.filename}")
        parts = name.split("/")
        if (
            any(p.startswith(".") and p not in {".", ".."} for p in parts)
            or "__MACOSX" in parts
        ):
            continue
        total_size += info.file_size
        if total_size > MAX_UNCOMPRESSED_BYTES:
            raise RobotProfileError("uncompressed archive size exceeds limit")
        cleaned_entries.append((info, Path(name)))

    if not cleaned_entries:
        raise RobotProfileError("no valid files found in archive")

    top_dirs = {
        entry[1].parts[0] for entry in cleaned_entries if len(entry[1].parts) > 1
    }
    has_root_files = any(
        len(entry[1].parts) == 1 and not entry[0].is_dir()
        for entry in cleaned_entries
    )
    single_prefix = (
        top_dirs.pop() if len(top_dirs) == 1 and not has_root_files else None
    )
    return cleaned_entries, single_prefix


def _locate_archive_urdf(temp_path: Path, profile_id: str) -> Path:
    urdf_files = list(temp_path.glob("**/*.urdf"))
    if not urdf_files:
        for xml_file in temp_path.glob("**/*.xml"):
            try:
                text = xml_file.read_text(encoding="utf-8-sig", errors="ignore")
                if "<robot" in text:
                    urdf_files.append(xml_file)
            except Exception:
                pass
    if not urdf_files:
        raise RobotProfileError("no .urdf file found in the archive")

    urdf_file = urdf_files[0]
    for candidate in urdf_files:
        if candidate.parent.name == "urdf" or candidate.stem == profile_id:
            return candidate
    return urdf_file


def _select_archive_source_yaml(
    yaml_files: list[Path],
) -> tuple[dict[str, Any], str]:
    priority = {"arm_teleop.yaml": 0, "controller_v23.yml": 1, "vr_configs.yml": 2}
    for candidate in sorted(
        yaml_files, key=lambda p: (priority.get(p.name, 3), len(p.parts))
    ):
        try:
            loaded = _load_yaml(candidate.read_bytes())
        except Exception:
            continue
        kind = _detect_schema_kind(loaded) if isinstance(loaded, dict) else None
        if kind is not None:
            return loaded, kind
    raise RobotProfileError(
        "no supported YAML found in archive (expected arm_teleop.yaml, vr_configs.yml, or controller_v23.yml schema)"
    )


class RobotProfileManager:
    def __init__(self, root: str | Path):
        self.root = Path(root).expanduser().resolve()
        self.root.mkdir(parents=True, exist_ok=True)

    def _path(self, profile_id: str) -> Path:
        return self.root / validate_profile_id(profile_id)

    def _existing_config_files(self, path: Path) -> list[str]:
        return [f for f in _KNOWN_CONFIG_FILES if (path / f).is_file()]

    def _ensure_profile_files(self, path: Path, profile_id: str) -> None:
        teleop_path = path / "arm_teleop.yaml"
        controller_path = path / "controller_v23.yml"
        vr_path = path / "vr_configs.yml"
        metadata_path = path / "profile.yaml"

        urdf_files = sorted(path.glob("**/*.urdf"), key=lambda p: len(p.parts))
        if not urdf_files:
            return
        urdf_file = urdf_files[0]
        rel_urdf = urdf_file.relative_to(path).as_posix()

        try:
            model = UrdfModel(urdf_file.read_bytes())
        except Exception:
            return

        if teleop_path.is_file():
            try:
                teleop_data = _load_yaml(teleop_path.read_bytes())
                if isinstance(teleop_data, dict) and isinstance(
                    teleop_data.get("arms"), (dict, list)
                ):
                    _teleop, controller, io_config, summary = _normalize_teleop(
                        profile_id, teleop_data, model, rel_urdf
                    )
                    if not controller_path.is_file():
                        _dump_yaml(controller_path, controller)
                    if not vr_path.is_file():
                        _dump_yaml(vr_path, io_config)
                    if not metadata_path.is_file():
                        metadata = _build_profile_metadata(
                            profile_id=profile_id,
                            display_name=profile_id,
                            model=model,
                            urdf_rel=rel_urdf,
                            primary_config=(
                                "controller_v23.yml"
                                if controller_path.is_file()
                                else "arm_teleop.yaml"
                            ),
                            config_files=self._existing_config_files(path),
                            summary=summary,
                            teleop_compatible=True,
                        )
                        _dump_yaml(metadata_path, metadata)
                    return
            except Exception:
                pass

        if vr_path.is_file() and not metadata_path.is_file():
            try:
                io_data = _load_yaml(vr_path.read_bytes())
                if isinstance(io_data, dict) and "arms" in io_data:
                    _io_config, controller, _teleop, summary = _normalize_io(
                        profile_id, io_data, model, rel_urdf
                    )
                    if not controller_path.is_file():
                        _dump_yaml(controller_path, controller)
                    metadata = _build_profile_metadata(
                        profile_id=profile_id,
                        display_name=profile_id,
                        model=model,
                        urdf_rel=rel_urdf,
                        primary_config=(
                            "controller_v23.yml"
                            if controller_path.is_file()
                            else "vr_configs.yml"
                        ),
                        config_files=self._existing_config_files(path),
                        summary=summary,
                        teleop_compatible=teleop_path.is_file(),
                    )
                    _dump_yaml(metadata_path, metadata)
            except Exception:
                pass

    def get(self, profile_id: str) -> dict[str, Any]:
        path = self._path(profile_id)
        if not path.is_dir():
            raise RobotProfileError(f"robot profile does not exist: {profile_id}")
        self._ensure_profile_files(path, profile_id)
        metadata_path = path / "profile.yaml"
        if metadata_path.is_file():
            try:
                metadata = yaml.safe_load(metadata_path.read_text(encoding="utf-8"))
            except (OSError, yaml.YAMLError) as exc:
                raise RobotProfileError(f"unable to read {metadata_path}: {exc}") from exc
            if not isinstance(metadata, dict):
                raise RobotProfileError(f"invalid profile metadata: {metadata_path}")
        else:
            config_name = next(
                (
                    name
                    for name in ("controller_v23.yml", "arm_teleop.yaml", "vr_configs.yml")
                    if (path / name).is_file()
                ),
                "",
            )
            if not config_name:
                raise RobotProfileError(f"profile has no supported YAML: {profile_id}")
            metadata = {
                "version": 1,
                "id": profile_id,
                "display_name": profile_id,
                "schema": (
                    "controller-v23"
                    if config_name.startswith("controller")
                    else "hc-robot-config-v1"
                ),
                "primary_config": config_name,
                "managed": False,
                "warnings": ["legacy profile without profile.yaml metadata"],
            }
        if "link_count" not in metadata:
            urdf_files = sorted(path.glob("**/*.urdf"), key=lambda p: len(p.parts))
            if urdf_files:
                try:
                    m = UrdfModel(urdf_files[0].read_bytes())
                    metadata.setdefault("link_count", len(m.links))
                    metadata.setdefault("joint_count", len(m.joints))
                    metadata.setdefault("movable_joint_count", len(m.movable_joint_names))
                    metadata.setdefault("robot_name", m.robot_name)
                    metadata.setdefault("urdf", urdf_files[0].relative_to(path).as_posix())
                except Exception:
                    pass
        metadata = copy.deepcopy(metadata)
        if (path / "middleware.yaml").is_file():
            config_files = list(metadata.get("config_files", []))
            if "middleware.yaml" not in config_files:
                config_files.append("middleware.yaml")
            metadata["config_files"] = config_files
        metadata["id"] = profile_id
        metadata["managed"] = bool(metadata.get("managed", metadata_path.is_file()))
        metadata["path"] = str(path)
        metadata["standard_topics"] = dict(STANDARD_TOPICS)
        # 传统 URDF/V23 导入包只能运行在显式 legacy 模式，不得宣称完整的 unified 接口。
        from .robot_api import RobotApiProfile

        try:
            api = RobotApiProfile(path)
            metadata["unified_api"] = {
                "available": True,
                "schema": "hc-robot-api/v1",
                "revision": api.revision,
            }
        except (RobotProfileError, KeyError, TypeError, ValueError) as exc:
            metadata["unified_api"] = {"available": False, "reason": str(exc)}
        return metadata

    def export_archive(self, profile_id: str) -> bytes:
        self.get(profile_id)
        path = self._path(profile_id)
        output = io.BytesIO()
        with zipfile.ZipFile(output, "w", zipfile.ZIP_DEFLATED) as archive:
            for file in sorted(path.rglob("*")):
                if (
                    file.is_file()
                    and not file.is_symlink()
                    and file.suffix != ".zip"
                    and not any(p.startswith(".") for p in file.relative_to(path).parts)
                ):
                    archive.write(file, file.relative_to(path).as_posix())
        return output.getvalue()

    def list(self) -> list[dict[str, Any]]:
        profiles = []
        for path in sorted(self.root.iterdir(), key=lambda item: item.name.lower()):
            if not path.is_dir() or path.name.startswith("."):
                continue
            try:
                profiles.append(self.get(path.name))
            except RobotProfileError as exc:
                profiles.append(
                    {
                        "id": path.name,
                        "display_name": path.name,
                        "schema": "invalid",
                        "managed": False,
                        "path": str(path),
                        "warnings": [str(exc)],
                        "standard_topics": dict(STANDARD_TOPICS),
                    }
                )
        return profiles

    def delete_profile(self, profile_id: str) -> None:
        profile_id = validate_profile_id(profile_id)
        target = self._path(profile_id)
        if not target.exists() or not target.is_dir():
            raise RobotProfileError(f"robot profile does not exist: {profile_id}")
        shutil.rmtree(target)

    def import_profile(
        self,
        profile_id: str,
        display_name: str,
        urdf_filename: str,
        urdf_payload: bytes,
        config_filename: str,
        config_payload: bytes,
        overwrite: bool = True,
    ) -> dict[str, Any]:
        profile_id = validate_profile_id(profile_id)
        display_name = str(display_name).strip() or profile_id
        if len(display_name) > 80:
            raise RobotProfileError("display name must not exceed 80 characters")
        urdf_filename = _safe_filename(urdf_filename, {".urdf", ".xml"})
        _safe_filename(config_filename, {".yaml", ".yml"})
        target = self._path(profile_id)
        if target.exists() and not overwrite:
            raise RobotProfileError(f"robot profile already exists: {profile_id}")

        model = UrdfModel(urdf_payload)
        source_config = _load_yaml(config_payload)
        files: dict[str, dict[str, Any]] = {
            "middleware.yaml": copy.deepcopy(ROBOT_MIDDLEWARE_DEFAULTS)
        }
        kind = _detect_schema_kind(source_config)
        if kind == "teleop":
            teleop, controller, io_config, summary = _normalize_teleop(
                profile_id, source_config, model, urdf_filename
            )
            files["arm_teleop.yaml"] = teleop
            files["controller_v23.yml"] = controller
            files["vr_configs.yml"] = io_config
            primary_config = "controller_v23.yml"
        elif kind == "v23":
            controller, summary = _normalize_v23(
                profile_id, source_config, model, urdf_filename
            )
            files["controller_v23.yml"] = controller
            primary_config = "controller_v23.yml"
        elif kind == "io":
            io_config, controller, teleop, summary = _normalize_io(
                profile_id, source_config, model, urdf_filename
            )
            files["vr_configs.yml"] = io_config
            files["controller_v23.yml"] = controller
            if teleop is not None:
                files["arm_teleop.yaml"] = teleop
            primary_config = "vr_configs.yml"
        else:
            raise RobotProfileError("unsupported YAML schema in profile")

        metadata = _build_profile_metadata(
            profile_id=profile_id,
            display_name=display_name,
            model=model,
            urdf_rel=urdf_filename,
            primary_config=primary_config,
            config_files=list(files),
            summary=summary,
            teleop_compatible="arm_teleop.yaml" in files,
        )

        temp_path = Path(tempfile.mkdtemp(prefix=f".{profile_id}-", dir=self.root))
        try:
            (temp_path / "urdf").mkdir()
            (temp_path / "urdf" / urdf_filename).write_bytes(urdf_payload)
            _commit_profile_dir(temp_path, target, files, metadata)
        except Exception:
            shutil.rmtree(temp_path, ignore_errors=True)
            raise
        return self.get(profile_id)

    def _import_unified_v1_archive(
        self, temp_path: Path, target: Path, profile_id: str, display_name: str
    ) -> dict[str, Any]:
        from .robot_api import RobotApiProfile

        api = RobotApiProfile(temp_path)
        middleware_file = temp_path / "middleware.yaml"
        if not middleware_file.exists():
            middleware_file.write_text(
                yaml.safe_dump(ROBOT_MIDDLEWARE_DEFAULTS, sort_keys=False),
                encoding="utf-8",
            )
        metadata = api.metadata
        metadata.update(
            id=profile_id,
            display_name=display_name,
            managed=True,
            config_files=sorted(
                str(f.relative_to(temp_path))
                for f in temp_path.rglob("*")
                if f.suffix in {".yaml", ".yml"} and f.name != "profile.yaml"
            ),
        )
        (temp_path / "profile.yaml").write_text(
            yaml.safe_dump(metadata, sort_keys=False), encoding="utf-8"
        )
        if target.exists():
            shutil.rmtree(target)
        os.rename(temp_path, target)
        return self.get(profile_id)

    def _normalize_archive_configs(
        self,
        profile_id: str,
        model: UrdfModel,
        rel_urdf_path: str,
        yaml_files: list[Path],
        middleware_exists: bool,
    ) -> tuple[dict[str, dict[str, Any]], str, dict[str, Any]]:
        chosen_source_config, source_kind = _select_archive_source_yaml(yaml_files)
        files: dict[str, dict[str, Any]] = {}
        if not middleware_exists:
            files["middleware.yaml"] = copy.deepcopy(ROBOT_MIDDLEWARE_DEFAULTS)

        if source_kind == "teleop":
            teleop, controller, io_config, summary = _normalize_teleop(
                profile_id, chosen_source_config, model, rel_urdf_path
            )
            files["arm_teleop.yaml"] = teleop
            files["controller_v23.yml"] = controller
            sim_file = next((f for f in yaml_files if f.name == "vr_configs.yml"), None)
            if sim_file is not None:
                try:
                    files["vr_configs.yml"] = _load_yaml(sim_file.read_bytes())
                except Exception:
                    files["vr_configs.yml"] = io_config
            else:
                files["vr_configs.yml"] = io_config
            primary_config = "controller_v23.yml"
        elif source_kind == "v23":
            controller, summary = _normalize_v23(
                profile_id, chosen_source_config, model, rel_urdf_path
            )
            files["controller_v23.yml"] = controller
            primary_config = "controller_v23.yml"
            simulation_file = next(
                (f for f in yaml_files if f.name == "vr_configs.yml"), None
            )
            if simulation_file is not None:
                simulation_config = _load_yaml(simulation_file.read_bytes())
                io_config, _gen_ctrl, _gen_teleop, _io_summary = _normalize_io(
                    profile_id, simulation_config, model, rel_urdf_path
                )
                files["vr_configs.yml"] = io_config
        elif source_kind == "io":
            io_config, controller, teleop, summary = _normalize_io(
                profile_id, chosen_source_config, model, rel_urdf_path
            )
            files["vr_configs.yml"] = io_config
            files["controller_v23.yml"] = controller
            if teleop is not None:
                files["arm_teleop.yaml"] = teleop
            primary_config = "vr_configs.yml"
        else:
            raise RobotProfileError("unsupported YAML schema in archive")

        teleop_file = next((f for f in yaml_files if f.name == "arm_teleop.yaml"), None)
        if teleop_file is not None and "arm_teleop.yaml" not in files:
            try:
                teleop_data = _load_yaml(teleop_file.read_bytes())
                if isinstance(teleop_data, dict) and "control" in teleop_data:
                    files["arm_teleop.yaml"] = teleop_data
            except Exception:
                pass
        return files, primary_config, summary

    def import_archive(
        self,
        profile_id: str,
        display_name: str,
        archive_payload: bytes,
        archive_filename: str = "",
        overwrite: bool = True,
    ) -> dict[str, Any]:
        if not archive_payload or len(archive_payload) > MAX_ARCHIVE_BYTES:
            raise RobotProfileError(
                f"archive must be between 1 byte and {MAX_ARCHIVE_BYTES // 1024 // 1024} MiB"
            )
        if not zipfile.is_zipfile(io.BytesIO(archive_payload)):
            raise RobotProfileError("uploaded file must be a valid .zip archive")

        with zipfile.ZipFile(io.BytesIO(archive_payload), "r") as zf:
            cleaned_entries, single_prefix = _collect_archive_entries(zf)
            if not str(profile_id).strip():
                candidate = single_prefix or Path(archive_filename).stem or "robot_profile"
                candidate = re.sub(r"[^A-Za-z0-9_.-]", "_", candidate).strip("_")
                if not candidate or not candidate[0].isalnum():
                    candidate = "robot_" + candidate
                profile_id = candidate[:64]

            profile_id = validate_profile_id(profile_id)
            display_name = str(display_name).strip() or profile_id
            if len(display_name) > 80:
                raise RobotProfileError("display name must not exceed 80 characters")

            target = self._path(profile_id)
            if target.exists() and not overwrite:
                raise RobotProfileError(f"robot profile already exists: {profile_id}")

            temp_path = Path(tempfile.mkdtemp(prefix=f".{profile_id}-", dir=self.root))
            try:
                for info, rel_path in cleaned_entries:
                    if info.is_dir():
                        continue
                    dest_rel = (
                        Path(*rel_path.parts[1:])
                        if single_prefix and rel_path.parts[0] == single_prefix
                        else rel_path
                    )
                    dest_file = temp_path / dest_rel
                    dest_file.parent.mkdir(parents=True, exist_ok=True)
                    dest_file.write_bytes(zf.read(info.filename))

                middleware_file = temp_path / "middleware.yaml"
                if middleware_file.exists():
                    try:
                        validate_robot_middleware(_load_yaml(middleware_file.read_bytes()))
                    except ConfigError as exc:
                        raise RobotProfileError(f"invalid middleware.yaml: {exc}") from exc

                # 完整的 v1 Profile 已归一化，校验通过后原样保留其模型与映射。
                if (temp_path / "robot_api.yaml").is_file():
                    return self._import_unified_v1_archive(
                        temp_path, target, profile_id, display_name
                    )

                urdf_file = _locate_archive_urdf(temp_path, profile_id)
                rel_urdf_path = urdf_file.relative_to(temp_path).as_posix()
                yaml_files = [
                    f
                    for f in (
                        list(temp_path.glob("**/*.yml"))
                        + list(temp_path.glob("**/*.yaml"))
                    )
                    if f.name not in {"profile.yaml", "middleware.yaml"}
                ]
                model = UrdfModel(urdf_file.read_bytes())
                files, primary_config, summary = self._normalize_archive_configs(
                    profile_id, model, rel_urdf_path, yaml_files, middleware_file.is_file()
                )
                metadata = _build_profile_metadata(
                    profile_id=profile_id,
                    display_name=display_name,
                    model=model,
                    urdf_rel=rel_urdf_path,
                    primary_config=primary_config,
                    config_files=list(dict.fromkeys([*files, "middleware.yaml"])),
                    summary=summary,
                    teleop_compatible="arm_teleop.yaml" in files,
                )
                _commit_profile_dir(temp_path, target, files, metadata)
            except Exception:
                shutil.rmtree(temp_path, ignore_errors=True)
                raise
            return self.get(profile_id)

    def get_profile_topics(self, profile_id: str) -> dict[str, str]:
        profile_dir = (self.root / profile_id).resolve()
        arm_teleop = profile_dir / "arm_teleop.yaml"
        if arm_teleop.is_file():
            with arm_teleop.open("r", encoding="utf-8") as stream:
                cfg = yaml.safe_load(stream) or {}
            control = cfg.get("control", {})
            body = cfg.get("body", {})
            return {
                "vr_frame": STANDARD_TOPICS["vr_frame"],
                "joint_state": control.get(
                    "joint_state_topic", STANDARD_TOPICS["joint_state"]
                ),
                "joint_target": control.get(
                    "generic_command_topic", STANDARD_TOPICS["joint_target"]
                ),
                "joint_command": STANDARD_TOPICS["joint_command"],
                "ee_target": control.get(
                    "controller_target_pose_topic", STANDARD_TOPICS["ee_target"]
                ),
                "ee_visual_target": control.get(
                    "target_pose_topic", STANDARD_TOPICS["ee_visual_target"]
                ),
                "ee_actual": control.get(
                    "actual_pose_topic", STANDARD_TOPICS["ee_actual"]
                ),
                "solver_state": control.get(
                    "solver_topic", STANDARD_TOPICS["solver_state"]
                ),
                "solver_reset": control.get(
                    "solver_reset_topic", STANDARD_TOPICS["solver_reset"]
                ),
                "solver_reset_ack": control.get(
                    "solver_reset_ack_topic", STANDARD_TOPICS["solver_reset_ack"]
                ),
                "base_move": body.get("base_command_topic", STANDARD_TOPICS["base_move"]),
            }
        return dict(STANDARD_TOPICS)

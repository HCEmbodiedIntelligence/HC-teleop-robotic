"""加载运行时会话所需的机器人 Profile 元数据。"""
from __future__ import annotations

import json
import os
from pathlib import Path
from typing import Any

import yaml

from adapters.core.runtime_config import unified_profile
from .robot_api import RobotApiProfile, selected_profile


def load_session_profile_metadata(
    config: dict[str, Any], config_dir: Path
) -> dict[str, str]:
    """描述当前运行时启动时实际选中的机器人 Profile 元数据。"""
    profile_config = config.get("robot_profiles", {})
    profile_id = str(
        os.environ.get("HC_ROBOT_NAME") or profile_config.get("active", "")
    ).strip()
    if not profile_id:
        return {}

    metadata: dict[str, str] = {
        "profile_id": profile_id,
        "profile_display_name": profile_id,
        "robot_name": profile_id,
        "controller_config": "",
        "free_joints": "[]",
        "ros_domain_id": str(config.get("ros", {}).get("domain_id", 14)),
    }
    profile_dir = selected_profile(config, config_dir)
    try:
        profile = (
            yaml.safe_load((profile_dir / "profile.yaml").read_text(encoding="utf-8"))
            or {}
        )
        metadata["profile_display_name"] = str(profile.get("display_name") or profile_id)
        metadata["robot_name"] = str(profile.get("robot_name") or profile_id)
        controller_name = str(profile.get("primary_config") or "controller_v23.yml")
        metadata["controller_config"] = controller_name
        controller = (
            yaml.safe_load((profile_dir / controller_name).read_text(encoding="utf-8"))
            or {}
        )
        free_joints = controller.get("model", {}).get("free_joints", [])
        if isinstance(free_joints, list):
            metadata["free_joints"] = json.dumps(
                [str(name) for name in free_joints],
                ensure_ascii=False,
                separators=(",", ":"),
            )
    except (OSError, TypeError, yaml.YAMLError):
        pass
    if unified_profile(profile_dir):
        api = RobotApiProfile(profile_dir)
        metadata.update(
            robot_api_schema="hc-robot-api/v1",
            robot_api_revision=api.revision,
            robot_api_info=json.dumps(api.describe(), separators=(",", ":")),
            free_joints=json.dumps([j["public_name"] for j in api.joints.values()]),
        )
    return metadata


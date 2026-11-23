"""HC teleoperation middleware.

Core modules are organized into five domain subpackages:
- ``control``: ROS 2 bridge, isolated control core, ownership, and intervention
- ``media``: VR gateway, WebRTC/FEC video transport, and H.264 encoders
- ``recording``: MCAP/LeRobot recording, dataset topics, exports, and replay
- ``uploads``: platform task binding and upload queue/service integration
- ``robot``: robot profiles, unified Robot API boundary, and simulation

A lightweight compatibility finder keeps legacy ``middleware.core.<module>``
(and ``core.<module>`` when ``PYTHONPATH=middleware``) imports aliased to the
exact same canonical module object so external scripts continue working.
"""
from __future__ import annotations

import importlib
from importlib.abc import Loader, MetaPathFinder
from importlib.util import find_spec, spec_from_loader
import sys

__version__ = "0.1.0"

_SUBPACKAGE_MODULES: dict[str, str] = {
    # control
    "control_clock": "control",
    "control_ownership": "control",
    "control_process": "control",
    "control_runtime": "control",
    "intervention_api": "control",
    "legacy_control": "control",
    "qos_event_observer": "control",
    "ros_bridge": "control",
    "ros_executor": "control",
    "ros_qos": "control",
    # media
    "camera": "media",
    "camera_metadata": "media",
    "camera_metrics": "media",
    "camera_process": "media",
    "fast_h264": "media",
    "fec_sender": "media",
    "jetson_h264": "media",
    "media_qos": "media",
    "receiver_metrics": "media",
    "rtp_fec": "media",
    "vr_gateway": "media",
    # recording
    "artifact_jobs": "recording",
    "artifact_state": "recording",
    "background_process": "recording",
    "batch_lerobot_export": "recording",
    "capture_formats": "recording",
    "capture_requirements": "recording",
    "capture_window": "recording",
    "correction_recording": "recording",
    "correction_support": "recording",
    "dataset_topics": "recording",
    "dataset_worker": "recording",
    "export_progress": "recording",
    "lerobot_export": "recording",
    "publication_capture": "recording",
    "recording_action_labels": "recording",
    "recording_artifacts": "recording",
    "recording_cdr": "recording",
    "recording_datasets": "recording",
    "recording_download_worker": "recording",
    "recording_downloads": "recording",
    "recording_frame_log": "recording",
    "replay_alignment": "recording",
    "ros_recording_executor": "recording",
    "takeover_log": "recording",
    "topic_player": "recording",
    "topic_recorder": "recording",
    "video_bundle": "recording",
    "x1_recording": "recording",
    # uploads
    "platform_api": "uploads",
    "platform_connection": "uploads",
    "task_recording": "uploads",
    "upload_api": "uploads",
    "upload_service": "uploads",
    # robot
    "profile_api": "robot",
    "robot_api": "robot",
    "robot_api_boundary": "robot",
    "robot_profiles": "robot",
    "simulation": "robot",
}


class _LegacyModuleAliasLoader(Loader):
    def __init__(self, target_name: str, target_spec):
        self._target_name = target_name
        self._target_spec = target_spec

    def create_module(self, spec):
        module = importlib.import_module(self._target_name)
        sys.modules[spec.name] = module
        return module

    def exec_module(self, module):
        return None

    def get_code(self, fullname):
        return self._target_spec.loader.get_code(self._target_name)

    def get_source(self, fullname):
        return self._target_spec.loader.get_source(self._target_name)

    def is_package(self, fullname):
        return False


class _LegacyModuleAliasFinder(MetaPathFinder):
    def find_spec(self, fullname, path=None, target=None):
        for prefix in ("middleware.core.", "core."):
            if fullname.startswith(prefix):
                leaf = fullname[len(prefix):]
                if "." not in leaf and leaf in _SUBPACKAGE_MODULES:
                    canonical = f"{prefix}{_SUBPACKAGE_MODULES[leaf]}.{leaf}"
                    target_spec = find_spec(canonical)
                    if target_spec is None:
                        return None
                    return spec_from_loader(
                        fullname,
                        _LegacyModuleAliasLoader(canonical, target_spec),
                        origin=target_spec.origin,
                    )
        return None


if not any(isinstance(finder, _LegacyModuleAliasFinder) for finder in sys.meta_path):
    sys.meta_path.append(_LegacyModuleAliasFinder())


def __getattr__(name: str):
    subpkg = _SUBPACKAGE_MODULES.get(name)
    if subpkg is not None:
        canonical = importlib.import_module(f"{__name__}.{subpkg}.{name}")
        sys.modules.setdefault(f"{__name__}.{name}", canonical)
        return canonical
    raise AttributeError(f"module {__name__!r} has no attribute {name!r}")

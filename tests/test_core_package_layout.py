"""Verify middleware.core subpackage structure, repo root paths, and legacy import aliases."""
import importlib
from pathlib import Path
import subprocess
import sys

import pytest

import middleware.core as core_pkg
from middleware.core import _SUBPACKAGE_MODULES

ROOT = Path(__file__).resolve().parents[1]


def test_core_root_stays_compact_and_all_subpackages_exist():
    core_dir = ROOT / "middleware/core"
    root_py = sorted(p.stem for p in core_dir.glob("*.py") if p.name != "__init__.py")
    assert root_py == ["app", "config", "diagnostic_logs", "network", "protocol", "runtime_mode"]
    for subpkg in ("control", "media", "recording", "uploads", "robot"):
        assert (core_dir / subpkg / "__init__.py").is_file()


@pytest.mark.parametrize("mod_name,subpkg", [
    ("ros_bridge", "control"),
    ("control_ownership", "control"),
    ("vr_gateway", "media"),
    ("topic_recorder", "recording"),
    ("lerobot_export", "recording"),
    ("upload_service", "uploads"),
    ("robot_profiles", "robot"),
])
def test_legacy_module_aliases_resolve_to_identical_canonical_module(mod_name, subpkg):
    canonical = importlib.import_module(f"middleware.core.{subpkg}.{mod_name}")
    aliased = importlib.import_module(f"middleware.core.{mod_name}")
    assert aliased is canonical
    assert getattr(core_pkg, mod_name) is canonical


def test_moved_modules_resolve_project_root_accurately():
    from middleware.core.control.control_runtime import ControlRuntime
    from middleware.core.recording import lerobot_export
    from middleware.core.robot.simulation import SimulationService

    assert lerobot_export.ROOT == ROOT
    assert ControlRuntime(ROOT / "middleware/config.yaml").project == ROOT
    sim = SimulationService.__new__(SimulationService)
    # Verify __file__.parents[3] matches ROOT on moved modules.
    assert Path(SimulationService.__init__.__code__.co_filename).resolve().parents[3] == ROOT
    assert sim is not None


def test_cli_entrypoints_work_with_both_canonical_and_legacy_paths():
    for module in (
        "middleware.core.runtime_mode",
        "middleware.core.recording.lerobot_export",
        "middleware.core.lerobot_export",
    ):
        proc = subprocess.run(
            [sys.executable, "-m", module, "--help"],
            cwd=ROOT,
            capture_output=True,
            text=True,
            timeout=10,
        )
        assert proc.returncode == 0, f"{module} failed: {proc.stderr}"


def test_every_registered_subpackage_module_exists_on_disk():
    core_dir = ROOT / "middleware/core"
    for mod, subpkg in _SUBPACKAGE_MODULES.items():
        assert (core_dir / subpkg / f"{mod}.py").is_file()

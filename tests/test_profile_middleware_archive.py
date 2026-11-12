"""Robot packages carry validated middleware settings without activating them."""
import io
import zipfile

import pytest
import yaml

from middleware.core.config import ConfigStore, ROBOT_MIDDLEWARE_DEFAULTS
from middleware.core.robot_profiles import RobotProfileError, RobotProfileManager
from tests.test_robot_profiles import URDF, io_yaml


def archive(middleware=None, *, prefix="robot/"):
    stream = io.BytesIO()
    with zipfile.ZipFile(stream, "w") as bundle:
        bundle.writestr(prefix + "urdf/robot.urdf", URDF)
        bundle.writestr(prefix + "vr_configs.yml", io_yaml())
        if middleware is not None:
            bundle.writestr(prefix + "middleware.yaml", middleware)
    return stream.getvalue()


@pytest.mark.parametrize("prefix", ["", "robot/"])
def test_archive_preserves_valid_middleware_and_registers_configuration(tmp_path, prefix):
    payload = b"# Keep camera calibration comments.\ncamera:\n  enabled: true\n  topic: /robot/camera/compressed\n"
    manager = RobotProfileManager(tmp_path)
    profile = manager.import_archive("robot", "Robot", archive(payload, prefix=prefix))
    assert (tmp_path / "robot/middleware.yaml").read_bytes() == payload
    assert "middleware.yaml" in profile["config_files"]
    assert profile["primary_config"] != "middleware.yaml"
    metadata = yaml.safe_load((tmp_path / "robot/profile.yaml").read_text())
    assert "middleware.yaml" in metadata["config_files"]


@pytest.mark.parametrize("payload", [
    b"camera: [invalid]\n",
    b"ros:\n  domain_id: 42\n",
    b"server:\n  host: 127.0.0.1\n",
    b"camera:\n  fps: 0\n",
    b"- invalid-root\n",
    b"camera: [\n",
])
def test_invalid_middleware_cannot_replace_existing_profile(tmp_path, payload):
    manager = RobotProfileManager(tmp_path)
    manager.import_archive("robot", "Original", archive())
    original = {path.relative_to(tmp_path): path.read_bytes()
                for path in tmp_path.rglob("*") if path.is_file()}
    with pytest.raises(RobotProfileError):
        manager.import_archive("robot", "Replacement", archive(payload))
    assert {path.relative_to(tmp_path): path.read_bytes()
            for path in tmp_path.rglob("*") if path.is_file()} == original


@pytest.mark.parametrize("source", ["archive", "individual"])
def test_import_without_middleware_creates_safe_defaults(tmp_path, source):
    manager = RobotProfileManager(tmp_path)
    if source == "archive":
        profile = manager.import_archive("robot", "Robot", archive())
    else:
        profile = manager.import_profile("robot", "Robot", "robot.urdf", URDF,
                                         "vr_configs.yml", io_yaml())
    defaults = yaml.safe_load((tmp_path / "robot/middleware.yaml").read_text())
    assert defaults == ROBOT_MIDDLEWARE_DEFAULTS
    assert defaults["camera"]["enabled"] is False
    assert defaults["safety"]["stop_on_startup"] is True
    assert "middleware.yaml" in profile["config_files"]


def test_importing_unselected_robot_does_not_change_host_configuration(tmp_path):
    store = ConfigStore(tmp_path / "host.yaml")
    value = store.load()
    value["robot_profiles"] = {"root": str(tmp_path / "profiles"), "active": ""}
    store.save(value)
    original = store.path.read_bytes()
    manager = RobotProfileManager(tmp_path / "profiles")
    manager.import_archive("robot", "Robot", archive(b"camera:\n  enabled: true\n"))
    assert store.path.read_bytes() == original
    assert store.load()["robot_profiles"]["active"] == ""
    assert store.load()["camera"]["enabled"] is False


def test_existing_metadata_includes_existing_middleware_without_changing_primary(tmp_path):
    manager = RobotProfileManager(tmp_path)
    profile = manager.import_archive("robot", "Robot", archive())
    metadata_path = tmp_path / "robot/profile.yaml"
    metadata = yaml.safe_load(metadata_path.read_text())
    metadata["config_files"].remove("middleware.yaml")
    metadata_path.write_text(yaml.safe_dump(metadata))
    loaded = manager.get("robot")
    assert "middleware.yaml" in loaded["config_files"]
    assert loaded["primary_config"] == profile["primary_config"]

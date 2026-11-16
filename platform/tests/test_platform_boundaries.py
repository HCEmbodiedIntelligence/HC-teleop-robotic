"""Resource bounds, immutable imports and task schemas at process boundaries."""

import asyncio
from concurrent.futures import ThreadPoolExecutor
import copy
import threading
import zipfile

import pytest
import yaml

from hc_platform.capture import Capture, CaptureStartupError
from hc_platform.capture_plan import validate_capture_plan, validate_lerobot_features
from hc_platform.configuration import (
    ConfigurationError,
    PackageRegistry,
    compile_package,
)


def toy_package(root):
    root.mkdir()
    (root / "robot.urdf").write_text(
        '<robot name="test"><link name="base"/><link name="tip"/>'
        '<joint name="j1" type="revolute"><parent link="base"/><child link="tip"/>'
        '<axis xyz="0 0 1"/><limit lower="-1" upper="1" velocity="1" effort="1"/></joint></robot>'
    )
    (root / "arm_teleop.yaml").write_text(
        yaml.safe_dump(
            {
                "robot": {"urdf_path": "robot.urdf"},
                "arms": {
                    "right": {
                        "base_link": "base",
                        "ee_link": "tip",
                        "joint_names": ["j1"],
                    }
                },
            }
        )
    )
    return root


@pytest.mark.parametrize(
    "invalid", ["link_name", "joint_names", "base_link", "wrong_arm_type", "body_type"]
)
def test_malformed_package_raises_configuration_error(tmp_path, invalid):
    package = toy_package(tmp_path / "toy")
    if invalid == "link_name":
        path = package / "robot.urdf"
        path.write_text(path.read_text().replace('name="tip"', ""))
    else:
        path = package / "arm_teleop.yaml"
        config = yaml.safe_load(path.read_text())
        if invalid == "wrong_arm_type":
            config["arms"]["right"] = ["j1"]
        elif invalid == "body_type":
            config["body"] = None
        else:
            config["arms"]["right"].pop(invalid)
        path.write_text(yaml.safe_dump(config))
    with pytest.raises(ConfigurationError):
        compile_package(package)


def test_concurrent_package_import_is_one_immutable_installation(tmp_path):
    package = toy_package(tmp_path / "source")
    builtin = tmp_path / "builtin"
    builtin.mkdir()
    archive = tmp_path / "package.zip"
    with zipfile.ZipFile(archive, "w") as output:
        for path in package.iterdir():
            output.write(path, path.name)
    barrier = threading.Barrier(2)

    def install():
        registry = PackageRegistry(builtin, tmp_path / "registry")
        barrier.wait(timeout=3)
        try:
            return registry.import_zip(archive, "version1")
        except ConfigurationError as error:
            return error

    with ThreadPoolExecutor(max_workers=2) as workers:
        results = list(workers.map(lambda _: install(), range(2)))
    assert sum(isinstance(result, dict) for result in results) == 1
    assert sum(isinstance(result, ConfigurationError) for result in results) == 1
    installed = tmp_path / "registry/packages/version1"
    assert sorted(path.name for path in installed.iterdir()) == [
        "arm_teleop.yaml",
        "robot.urdf",
    ]
    assert (
        compile_package(installed)["config_hash"]
        == compile_package(package)["config_hash"]
    )
    assert [
        entry["id"] for entry in PackageRegistry(builtin, tmp_path / "registry").list()
    ] == ["version1"]


def test_capture_tiny_messages_have_a_count_bound_and_audited_gap():
    capture = Capture("/unused", "session", [], byte_budget=1024, message_budget=8)
    for _ in range(100):
        capture.receive(b"\1", "/bool", "std_msgs/msg/Bool", "bool data\n", 0)
    assert len(capture.queue) == 8 and capture.bytes == 8 and capture.dropped == 92
    recorded = []

    class Client:
        async def append_sample(self, **value):
            recorded.append(value)

        async def close(self):
            pass

    capture.client = Client()
    capture.start_ros = lambda: None
    capture.stopping = True
    asyncio.run(capture.run())
    gap = next(
        value["payload"] for value in recorded if value["topic"] == "/capture/gaps"
    )
    assert gap["overflow"] == {"messages": 92, "bytes": 0}
    assert gap["data_integrity"] == "incomplete"
    assert len(capture.queue) == capture.bytes == 0


def test_capture_resolves_all_types_before_start_or_ready(tmp_path, monkeypatch):
    pytest.importorskip("rclpy")
    import rosidl_runtime_py.utilities
    from hc_platform import capture as module

    monkeypatch.setattr(module, "ros_schema", lambda _: "bool data\n")

    def resolve(kind):
        if kind.startswith("missing/"):
            raise ImportError("message package unavailable")
        return object

    monkeypatch.setattr(rosidl_runtime_py.utilities, "get_message", resolve)
    ready = tmp_path / "ready.json"
    capture = Capture(
        "/unused",
        "session",
        [
            {"topic": "/valid", "type": "std_msgs/msg/Bool"},
            {"topic": "/optional", "type": "missing/msg/Type", "required": False},
        ],
        ready_file=ready,
    )
    with pytest.raises(CaptureStartupError) as error:
        asyncio.run(capture.run())
    assert error.value.detail["topic"] == "/optional"
    assert error.value.detail["stage"] == "message_type"
    assert capture.node is None and not ready.exists()


def plan_fixture():
    vector = {
        "dtype": "float32",
        "shape": [2],
        "names": ["j2", "j1"],
        "units": ["m", "rad"],
        "required": True,
    }
    plan = {
        "source_format": "LEROBOT_V3",
        "effective_adapter": {"feature_sources": {}, "codebase_version": "v3.0"},
        "output": {
            "frequency_hz": 20,
            "features": {
                "observation.state": copy.deepcopy(vector),
                "action": copy.deepcopy(vector),
                "observation.images.head": {
                    "dtype": "video",
                    "shape": [48, 64, 3],
                    "names": ["height", "width", "channels"],
                    "required": True,
                    "role": "rgb",
                },
            },
        },
    }
    config = {
        "joint_names": ["j1", "j2"],
        "joint_units": ["rad", "m"],
        "groups": {"arm": ["j1", "j2"]},
        "allowed_groups": ["arm"],
        "camera": {"streams": [{"id": "head", "topic": "/head/color"}]},
    }
    return (
        plan,
        config,
        [{"topic": "/head/color"}],
        {"warnings": []},
        {"dataset_fps": 20},
        {"fresh_joint_names": ["j1", "j2"]},
    )


def test_task_plan_freezes_named_export_order_and_camera_mapping():
    options = validate_capture_plan(*plan_fixture())
    assert options["joint_names"] == ["j2", "j1"] and options["joint_units"] == [
        "m",
        "rad",
    ]
    assert (
        options["cameras"] == {"observation.images.head": "/head/color"}
        and options["fps"] == 20
    )


@pytest.mark.parametrize(
    "invalid",
    [
        "units",
        "camera_unselected",
        "camera_stale",
        "feedback_missing",
        "action_authorization",
        "frequency",
        "unsupported",
    ],
)
def test_task_plan_rejects_unavailable_or_incompatible_required_data(invalid):
    plan, config, selected, preflight, spec, state = plan_fixture()
    if invalid == "units":
        plan["output"]["features"]["action"]["units"] = ["rad", "rad"]
    elif invalid == "camera_unselected":
        selected.clear()
    elif invalid == "camera_stale":
        preflight["warnings"] = [{"topic": "/head/color", "reason": "NO_FRESH_DATA"}]
    elif invalid == "feedback_missing":
        state["fresh_joint_names"] = ["j1"]
    elif invalid == "action_authorization":
        config["allowed_groups"] = []
    elif invalid == "frequency":
        spec["dataset_fps"] = 30
    else:
        plan["output"]["features"]["observation.depth"] = {"required": True}
    with pytest.raises(ConfigurationError):
        validate_capture_plan(plan, config, selected, preflight, spec, state)


def test_exported_image_shape_is_checked_without_assuming_preview_dimensions():
    plan = plan_fixture()[0]
    info = {
        "codebase_version": "v3.0",
        "fps": 20,
        "features": copy.deepcopy(plan["output"]["features"]),
    }
    validate_lerobot_features(plan, info)
    info["features"]["observation.images.head"]["shape"] = [240, 320, 3]
    with pytest.raises(ConfigurationError, match="shape"):
        validate_lerobot_features(plan, info)

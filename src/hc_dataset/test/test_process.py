from pathlib import Path

import pytest

from hc_dataset.catalog import DatasetCatalog, DatasetError
from hc_dataset.process import BagCommandBuilder, ManagedBagProcess


def record(tmp_path: Path):
    value = DatasetCatalog(tmp_path).create(
        "trial",
        ["/robots/openarmx/input/vr_frame", "/robots/openarmx/state/joints"],
    )
    value.bag_directory.mkdir()
    return value


def test_record_command_uses_mcap_and_no_shell_expression(tmp_path: Path) -> None:
    value = record(tmp_path)
    command = BagCommandBuilder("/usr/bin/ros2", "mcap").record(value)
    assert command[:5] == ["/usr/bin/ros2", "bag", "record", "--storage", "mcap"]
    assert "--output" in command
    assert "/robots/openarmx/input/vr_frame" in command
    assert all(";" not in argument for argument in command)


def test_replay_always_remaps_every_topic_below_dataset_namespace(tmp_path: Path) -> None:
    value = record(tmp_path)
    command = BagCommandBuilder().replay(value)
    assert "--remap" in command
    remaps = command[command.index("--remap") + 1 :]
    assert len(remaps) == 2
    assert all(f":=/replay/{value.dataset_id}/" in item for item in remaps)
    assert not any(item.endswith(":=/robots/openarmx/state/joints") for item in remaps)


def test_invalid_replay_rate_is_rejected(tmp_path: Path) -> None:
    with pytest.raises(DatasetError):
        BagCommandBuilder().replay(record(tmp_path), rate=0.0)


def test_managed_process_has_no_child_while_idle() -> None:
    process = ManagedBagProcess()
    assert process.pid is None
    assert not process.running
    assert process.stop() is None

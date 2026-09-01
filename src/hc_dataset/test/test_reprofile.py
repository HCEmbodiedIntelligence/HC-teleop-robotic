from types import SimpleNamespace

import pytest

from hc_dataset.reprofile import (
    ReprofileError,
    _field_map,
    _normal_ros_type,
    _parse_sftp_source,
    _set_stamp,
)


def test_normalizes_ros2_schema_spelling():
    assert _normal_ros_type("sensor_msgs/msg/JointState") == "sensor_msgs/JointState"
    assert _normal_ros_type("sensor_msgs/JointState") == "sensor_msgs/JointState"


def test_joint_field_map_uses_available_pairs_only():
    message = SimpleNamespace(name=["a", "b", "ignored"], position=[1.0, 2.0])
    assert _field_map(message) == {"a": 1.0, "b": 2.0}


def test_set_stamp_splits_nanoseconds():
    message = SimpleNamespace(
        header=SimpleNamespace(stamp=SimpleNamespace(sec=0, nanosec=0))
    )
    _set_stamp(message, 12_345_678_901)
    assert message.header.stamp.sec == 12
    assert message.header.stamp.nanosec == 345_678_901


def test_parses_sftp_source_without_embedding_password():
    assert _parse_sftp_source("sftp://niic@192.168.2.33/home/niic/a.mcap") == (
        "niic@192.168.2.33",
        "/home/niic/a.mcap",
        None,
    )
    with pytest.raises(ReprofileError, match="do not put"):
        _parse_sftp_source("sftp://niic:secret@192.168.2.33/home/niic/a.mcap")

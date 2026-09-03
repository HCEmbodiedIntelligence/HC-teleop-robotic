from types import SimpleNamespace

import pytest

from hc_dataset.reprofile import (
    DEFAULT_MISSING_JOINT_VALUES,
    ReprofileError,
    _field_map,
    _normalized_joint_state,
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


def test_normalizes_four_joint_state_arrays_and_synthesizes_missing_waist():
    template = SimpleNamespace(
        header=SimpleNamespace(
            stamp=SimpleNamespace(sec=0, nanosec=0), frame_id="base_link"
        ),
        name=["arm", "leg_1", "leg_2", "zhi"],
        position=[0.0] * 4,
        velocity=[0.0] * 4,
        effort=[0.0] * 4,
    )

    class Profile:
        @staticmethod
        def clone_template(topic):
            assert topic == "io_teleop/joint_states"
            return template

    source = SimpleNamespace(
        header=SimpleNamespace(stamp=SimpleNamespace(sec=0, nanosec=0)),
        name=["arm"],
        position=[0.25],
        velocity=[0.5],
        effort=[0.75],
    )
    output = _normalized_joint_state(
        Profile(), source, 2_000_000_003, DEFAULT_MISSING_JOINT_VALUES
    )
    assert output.position == [0.25, 0.5, 1.2, -0.6]
    assert output.velocity == [0.5, 0.0, 0.0, 0.0]
    assert output.effort == [0.75, 0.0, 0.0, 0.0]
    assert (output.header.stamp.sec, output.header.stamp.nanosec) == (2, 3)

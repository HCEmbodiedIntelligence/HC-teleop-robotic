#!/usr/bin/env bash

set -euo pipefail

python3 - <<'PY'
from hc_teleop_interfaces.action import Home, MoveJ, MoveL, MoveP, RecordDataset, ReplayDataset
from hc_teleop_interfaces.msg import (
    BaseCommandCandidate,
    CartesianTargetArray,
    CartesianStateArray,
    ControlLease,
    JointCommand,
    JointCommandCandidate,
    SafetyState,
    VrFrame,
    ControllerInput,
)
from hc_teleop_interfaces.srv import AcquireControl, ResetFault, SetEnabled


def fields(message_type):
    return tuple(message_type.get_fields_and_field_types())


command_fields = (
    "header",
    "source_id",
    "session_id",
    "sequence",
    "valid_until",
    "control_mode",
    "group_name",
    "command",
)
assert fields(JointCommandCandidate) == command_fields
assert fields(JointCommand) == command_fields
assert JointCommandCandidate.CONTROL_MODE_POSITION == 1
assert JointCommandCandidate.CONTROL_MODE_VELOCITY == 2
assert JointCommandCandidate.CONTROL_MODE_EFFORT == 3

assert fields(BaseCommandCandidate) == (
    "header",
    "source_id",
    "session_id",
    "sequence",
    "valid_until",
    "control_mode",
    "base_name",
    "command",
)
assert fields(CartesianTargetArray) == (
    "header",
    "source_id",
    "session_id",
    "sequence",
    "valid_until",
    "targets",
)
assert fields(CartesianStateArray) == ("header", "states")
assert fields(VrFrame)[0:5] == (
    "header",
    "source_id",
    "protocol_version",
    "sequence",
    "source_stamp",
)
assert ControllerInput.BUTTON_PRIMARY == 1
assert ControllerInput.BUTTON_SECONDARY_TOUCH == 1024

assert fields(SafetyState) == (
    "header",
    "state",
    "enabled",
    "fault_latched",
    "estop_active",
    "reason",
    "active_source",
    "active_session",
    "lease_expires_at",
)
assert SafetyState.DISABLED == 0
assert SafetyState.ACTIVE == 2
assert SafetyState.ESTOP == 4
assert fields(ControlLease) == (
    "header",
    "lease_id",
    "source_id",
    "session_id",
    "resources",
    "acquired_at",
    "expires_at",
    "active",
)

assert fields(AcquireControl.Request) == (
    "source_id",
    "session_id",
    "resources",
    "requested_duration",
)
assert fields(AcquireControl.Response) == (
    "granted",
    "lease_id",
    "expires_at",
    "reason",
)
assert "priority" not in fields(AcquireControl.Request)
assert fields(SetEnabled.Request) == ("enabled", "requester_id", "reason")
assert fields(ResetFault.Request) == ("requester_id", "reason")

assert fields(Home.Goal) == (
    "requester_id",
    "component_ids",
    "velocity_scale",
    "timeout",
)
assert fields(MoveJ.Goal) == ("group_name", "target", "options")
assert fields(MoveL.Goal) == ("group_name", "tip_frame", "target_pose", "options")
assert fields(MoveP.Goal) == ("group_name", "tip_frame", "target_pose", "options")
assert "source_id" not in fields(MoveJ.Goal)
assert fields(RecordDataset.Result) == (
    "status",
    "dataset_id",
    "uri",
    "message_count",
    "bytes_written",
)
assert fields(ReplayDataset.Goal) == (
    "requester_id",
    "dataset_id",
    "rate",
    "start_offset",
    "end_offset",
    "publish_clock",
    "loop",
)
PY

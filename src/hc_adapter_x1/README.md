# hc_adapter_x1

Simulated and hardware adapter components for the X1 humanoid dual-arm robot.

The simulator supports `instant_position_tracking`. When enabled by the X1
simulation profile it renders the command that has already passed the motion
backend and final RTC directly, avoiding an extra visualization-only smoothing
layer. The default remains velocity-limited for standalone use.

## HC_X1 real-hardware bridge

`X1HardwareAdapterNode` is the compatibility boundary for the independently
launched `HC_X1` repository. It does not load a vendor SDK and does not require
changes in HC_X1. It performs these translations:

- `/hc_teleop/joint_states` -> `/robots/x1/state/joints`;
- `/robots/x1/control/joint_command` -> merged 100 Hz
  `/hc_teleop/joint_cmd`;
- scalar `left_gripper`/`right_gripper` commands -> the ten-joint OmniHand
  topics expected below `/hc_teleop/joint_cmd_finger_*`.

Only position commands with the exact configured joint names are accepted.
The bridge rejects expired/non-finite commands, requires an ACTIVE HC safety
state, and stops its output within 150 ms if the source sequence stops
advancing. In `mode:=shadow` it forwards feedback but cannot publish a hardware
command.

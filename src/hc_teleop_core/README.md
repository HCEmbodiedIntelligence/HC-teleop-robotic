# hc_teleop_core

`hc_teleop_core` owns the final command authority. Solvers, VR, replay and
exoskeleton nodes publish `JointCommandCandidate`; this package validates the
active lease, source/session identity, sequence, finite values, validity time,
and the 150 ms watchdog before publishing `JointCommand`.

The pure C++ `CommandArbiter` is independent of ROS. The ROS wrapper is both a
standalone executable and an `rclcpp_components` component. All interface names
are relative so launch can put a robot under `/robots/<robot_id>`.

Safety defaults are deliberately conservative:

- disabled on startup;
- source priority is server configuration, never a client request;
- a new lease clears the previous command;
- stale, duplicate, non-finite, incorrectly sized or expired commands are
  rejected;
- after a watchdog or lease timeout no stale command is republished, allowing
  the device-side watchdog to hold safely;
- resetting a fault returns to disabled and requires an explicit enable.

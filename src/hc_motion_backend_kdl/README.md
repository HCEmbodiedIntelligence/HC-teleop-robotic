# hc_motion_backend_kdl

KDL IK backend for the decomposed HC teleop runtime.

This package is intentionally small and dependency-light. It subscribes to
`motion/backend/cartesian_targets`, uses fresh measured `state/joints` as the
IK seed, and publishes only `motion/backend/joint_candidate`. The motion router
and command arbiter remain responsible for authorization and final command
ownership.

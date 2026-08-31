# hc_motion_backend_kdl

KDL IK backend for the decomposed HC teleop runtime.

This package is intentionally small and dependency-light. It subscribes to
`motion/backend/cartesian_targets`, uses fresh measured `state/joints` as the
IK seed, and publishes only `motion/backend/joint_candidate`. The motion router
and command arbiter remain responsible for authorization and final command
ownership.

Servo candidates pass through a stateful joint limiter before publication. It
uses each joint's URDF velocity limit and additionally bounds acceleration, so
a delayed VR frame or a nearby IK branch cannot become a one-frame command
jump. Profiles may tune `motion.servo_velocity_scale`,
`motion.servo_acceleration_limit`, `motion.servo_nominal_rate_hz`,
`motion.servo_reset_timeout_ms`, and `motion.servo_tracking_error_reset`.

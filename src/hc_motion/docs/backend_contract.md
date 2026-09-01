# Motion backend process contract

`hc_motion` contains no vendor SDK and does not load PyBullet. A backend is an
independent process selected by the robot profile.

- Input: `motion/backend/cartesian_targets` (`CartesianTargetArray`), sensor
  data QoS (`best_effort`, `volatile`, depth 1).
- Output: `motion/backend/joint_candidate` (`JointCommandCandidate`) with the
  same `source_id`, `session_id`, `sequence`, and group. Its `valid_until` must
  not exceed the input deadline.
- Robot feedback: `state/joints` (`sensor_msgs/JointState`). FK and closed-loop
  completion must use this measured state.
- Long operations use the `MoveJ`, `MoveL`, and `MoveP` actions defined by
  `hc_teleop_interfaces`; ServoJ/P stays on the continuous topic path.

The router correlates every output with an accepted live input. It retains all
unexpired target authorizations and advances candidate sequences independently
per component group. A delayed right-arm result therefore cannot be invalidated
only because the left arm already advanced, while duplicate or backwards
candidates for either arm are still rejected. The command arbiter remains the
only publisher of authoritative `JointCommand` messages.
Private RoboManip/Ruckig dependencies belong in
`hc_motion_backend_robo_manip`, not in this package or the default manifest.

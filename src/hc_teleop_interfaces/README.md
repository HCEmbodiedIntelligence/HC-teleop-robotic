# hc_teleop_interfaces

`hc_teleop_interfaces` is the dependency-light ROS 2 Humble contract package for the modular HC
teleoperation stack. It contains no transport decoder, kinematics solver, robot driver, storage
implementation, or UI code.

The contracts cover five boundaries:

- decoded VR tracking and controller input (`VrFrame`);
- atomic Cartesian targets produced by a teleoperation mapper;
- untrusted joint/base command candidates and authoritative joint commands;
- robot capability, safety-state, and control-lease discovery;
- enable/fault/control services and long-running home/record/replay actions.

## Command ownership

Solver and teleoperation plugins publish `JointCommandCandidate`. Exactly one arbiter validates
leases, source configuration, sequence numbers, expiry, limits, and safety state, then publishes
`JointCommand`. A hardware adapter must subscribe only to `JointCommand`. This distinction prevents
two installed solvers from becoming simultaneous robot command authorities.

Clients never submit their own priority. `AcquireControl` identifies the source, session, and
resources; the arbiter obtains priority and preemption policy from trusted deployment config.
Calling `AcquireControl` again with the same source, session, and resource set requests renewal;
leases otherwise end on expiry, disable, fault, or session replacement.

`valid_until` and lease expiry are absolute ROS-clock times. Zero is not an infinite deadline:
command consumers must reject a candidate with a zero or expired deadline. Sequences are monotonic
within one `(source_id, session_id)` pair and restart only with a new session ID.

## Units and identifiers

Public physical values use SI units: metres, radians, seconds, m/s, rad/s, and N*m. Joint arrays are
associated by `JointState.name`, never by an undocumented index. IDs are opaque, non-empty UTF-8
strings and must not contain credentials. Dataset implementations resolve `dataset_id` to storage;
callers do not pass arbitrary filesystem paths.

See [docs/qos_contract.md](docs/qos_contract.md) for the required QoS and freshness behavior.

## Build and test

From a ROS 2 workspace containing this package:

```bash
source /opt/ros/humble/setup.bash
colcon build --packages-select hc_teleop_interfaces
colcon test --packages-select hc_teleop_interfaces
colcon test-result --verbose
```

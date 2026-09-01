# hc_motion_backend_robo_manip

Optional out-of-process RoboManip backend for the HC motion contract.  It does
not expose the legacy humanoid channels and never publishes authoritative robot
commands.

Inputs:

- `motion/backend/cartesian_targets` (`CartesianTargetArray`)
- `state/joints` (`sensor_msgs/JointState`)

Outputs:

- `motion/backend/joint_candidate` (`JointCommandCandidate`)
- `state/cartesian` (`CartesianStateArray`, when `publish_fk:=true`)

The backend preserves the HC source/session/sequence/deadline envelope.  Each
arm owns an independent RoboManip ServoP session, so delayed computation on one
arm cannot change the other arm's candidate correlation.  SDK sessions are
reset from measured feedback after expiry, a source/session change, or a long
input gap.

A transient ServoP Tick failure does not tear down the session. The adapter
immediately republishes the last final-RTC-approved position as a zero-velocity
hold, correlated to the current target envelope. It resets from measured
feedback only after `tick_failure_reset_count` consecutive failures (default
three). The first and terminal failure also run a direct SDK IK diagnostic and
log its concrete message, target, error values, native API and status code.

Per-arm `target_filter` profile entries can constrain an axis-aligned workspace,
radial reach and Cartesian position/orientation step. X1 enables this filter for
the left arm to prevent a noisy VR frame from jumping directly into an
unreachable region.

The ABI-pinned `humanoid_motion_server` and its interfaces are Git submodules
of the HC workspace. Colcon builds them before this adapter, and the SDK shared
objects are installed from the Motion Server repository into the same install
space.

```bash
git submodule update --init --recursive
export HUMANOID_MOTION_SDK_DEPS_PREFIX=/path/to/pinned/sdk-dependencies
./bootstrap_colcon.sh build
```

For the X1 simulation in this workspace:

```bash
./run.sh profile:=x1 mode:=sim motion_backend:=robo_manip
```

The adapter may take several seconds to construct the SDK context. It does not
accept targets until measured joint feedback is available, and stops a ServoP
session after the configured input gap so the next engagement is reset from
fresh measured feedback.

The ROS motion limits are also written into the SDK's private runtime YAML so
the ServoP request and mandatory final RTC use one consistent envelope. Robot
profiles may provide faster `simulation.robo_manip_limits`; those overrides
apply only to `mode:=sim`, while shadow/real modes retain the conservative
`motion.robo_manip_*` commissioning limits.

The X1 simulator uses `simulation.instant_position_tracking:=true`: it displays
the already shaped, final-RTC-approved command without applying another joint
velocity interpolation. This affects simulation only; shadow/real output still
passes through the final SDK RTC and the command arbiter.

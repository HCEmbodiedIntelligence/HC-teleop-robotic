# hc_motion_backend_robo_manip

HC transport adapter for the complete `humanoid_motion_server::motion::CommandPipeline`.
It links the pinned package's runtime library; it does not launch the package's
public Move/Servo ROS endpoints. HC keeps its source/session/sequence/deadline
contract and its exclusive hardware command publisher.

Inputs:
- `motion/backend/cartesian_targets` (`CartesianTargetArray`)
- `state/joints` (`sensor_msgs/JointState`)

Outputs:
- `motion/backend/joint_candidate` (`JointCommandCandidate`)
- `state/cartesian` (`CartesianStateArray`)

Each disjoint arm group has a package CommandPipeline and a registered ServoP
endpoint. Overlapping groups are rejected at startup. HC selects the source via
its control lease; the package owns Servo session admission/expiry, feedback
validation, SDK Start/Tick/Stop, candidate and final position-limit validation,
and mandatory final RTC. The adapter no longer invokes SDK session or RTC
methods directly and does not implement a separate failed-Tick hold policy.

`servo_lease_ms` defaults to 100 ms. `feedback_timeout_sec` is passed to the
package's feedback policy. Feedback age is the oldest sample among the requested
joints: partial messages cannot renew another arm's age, and malformed messages
are rejected atomically. Missing velocity arrays are represented as zero; this
is a position-feedback fallback, not evidence that the hardware has stopped.

Each new HC target updates ServoP and ticks its pipeline once. A 20 ms watchdog
also services lease/feedback expiry when input stops. This input-driven scheduling
preserves HC's one-candidate-per-target authorization: it does not emit new
commands by repeatedly retimestamping an old target. Input should be refreshed
at the configured nominal rate. The package uses actual elapsed tick time.

Source/session changes cancel the old package session and create a fresh pipeline
lifetime, including its elapsed-time history. Expired HC envelopes also
cancel it. If a solve finishes after its deadline, the result is discarded and
the session canceled so the next request starts from measured feedback. The
router still validates candidate deadlines. SDK computation remains synchronous;
this is not a hard real-time deadline or a guarantee about hardware execution.

Per-arm workspace/step filters and measured FK are retained. ROS limits are
written into the private runtime SDK YAML in the SDK's units, so request limits
and the final RTC share the configured envelope. Simulation-only overrides never
apply to real/shadow operation.

X1 and OpenArmX profiles select this backend by default:

```bash
./run.sh profile:=x1 mode:=sim
./run.sh profile:=openarmx mode:=sim
```

The existing KDL backend remains available via `motion_backend:=kdl`.

Build and test with the workspace's ABI-pinned SDK dependencies:

```bash
./bootstrap_colcon.sh build
ctest --test-dir build/hc_motion_backend_robo_manip --output-on-failure
ctest --test-dir build/humanoid_motion_server -R 'test_command_pipeline|test_control_arbiter' --output-on-failure
```

Opt-in real-SDK regression probe (publishes synthetic feedback and targets only):

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ROS_DOMAIN_ID=187 /usr/bin/python3 src/hc_motion_backend_robo_manip/test/verify_sdk_adapter.py x1
ROS_DOMAIN_ID=188 /usr/bin/python3 src/hc_motion_backend_robo_manip/test/verify_sdk_adapter.py openarmx
```

The probe checks both arms, input expiry, session rollover, feedback expiry, and
recovery. It allows up to 25 seconds for SDK initialization.

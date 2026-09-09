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

The former KDL backend has been removed. Out-of-tree backends must implement
the HC contract and be launched explicitly with `motion_backend:=external`.

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

### ServoP 卡顿诊断

`ServoP ended` 现在包含以下字段：

- `sdk_tick_ms`：失败的 `TickRealtimeMoveLine` 调用耗时（单调时钟）。
- `sdk_period_ms`：实际传给 SDK 的积分周期，不是调用耗时。
- `pipeline_ms`：整个管线 tick 耗时，含 SDK、最终 RTC 和失败停止处理。
- `target_age_ms` / `deadline_left_ms`：tick 结束时，目标从 header 到当前 ROS 时间的年龄及距离有效期的剩余时间；负剩余时间代表过期。
- `feedback_age_ms`：tick 结束时距本节点收到反馈的时间，不能代替硬件采样到达延迟。
- `input_gap_ms`：该臂本次与上次接受目标的处理时间间隔，首次/重建后为 -1。
- `restarted`、`failures`、`sequence`、`source`、`session`：是否刚启动及累计失败和输入身份。
- `target_limited`、`target_xyz_m`、`target_xyzw`、`feedback_rad`：过滤后的目标及实际使用的关节反馈；关节顺序为该臂配置顺序。

成功调用如果管线耗时超过名义周期，或目标/反馈过期，会产生限频的 `ServoP timing` 告警。
失败日志在过期输出丢弃之前记录，防止超时掩盖 SDK 错误。SDK 公开 API 未提供 MoveLine 详细
错误码，目前不能仅凭 `returned false` 区分 RTC、IK 或奇异性失败；需 SDK 内部状态接口进一步支持。
本改动只增加诊断，不改变租约、反馈超时、限速和失败停止行为。

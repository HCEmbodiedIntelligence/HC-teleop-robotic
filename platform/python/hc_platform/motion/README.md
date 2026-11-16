# Explicit Motion backend

`MotionService(runtime)` supplies `ServoJ`, `ServoP`, `MoveJ`, `MoveL` and `MoveP`
without importing the legacy middleware runtime. It is enabled only by compiled
`control_backend: motion`; `v23` reports these commands unavailable. The pure local
control core and its program lease remain the only output authority.

The API imports `MotionService`, `MotionError`, and `create_router` from this
package. Mount `create_router(service)` once. The runtime must call `close()` before
config changes and shutdown; the same object can start a new worker on the next
request after restart. `busy` remains true through output-revocation confirmation,
so the generic program command route must reject concurrent raw program commands.
No method implicitly recovers a stop or selects the program source.

## Execution and endpoints

A separate `python -m hc_platform.motion.worker --config ...` process loads the
selected URDF, native Eigen FK/IK when available, and the optional PyBullet
collision checker. JSON-lines carry bounded jobs/results. CPU IK, path sampling,
collision and time parameterization never run in the API asyncio loop. Large JSON
results are decoded outside that loop. The worker can be terminated on deadline
or cancellation; no cancelled future leaves an uncontrolled planning process.

| Endpoint | Semantics |
|---|---|
| GET `/api/motion/info` | Groups, URDF names, SI units, frames, solver and collision scope |
| GET `/api/motion/capabilities` | Backend/config/runtime readiness, including unavailable reasons |
| POST `/api/motion/servo-j` | Fresh named joint subset, filled from the same fresh group observation |
| POST `/api/motion/servo-p` | A pose in a known frame, bounded IK and a short original input TTL |
| POST `/api/motion/move-j` | Asynchronous quintic joint trajectory |
| POST `/api/motion/move-l` | Sampled Cartesian straight line and shortest-arc orientation interpolation |
| POST `/api/motion/move-p` | Cartesian goal solved by IK, then joint-space point-to-point motion |
| GET `/api/motion/tasks/{id}` | Planning/running/finalizing/terminal status, feedback and result |
| POST `/api/motion/tasks/{id}/cancel` | Cancel planning/streaming and confirm core lease revocation |

A Move returns a task id, not a claim of completed motion. A task succeeds only
when fresh joint feedback settles inside tolerance; Cartesian tasks also require
measured-model FK position and orientation tolerances. Completion, failure and
cancellation revoke the task through a core pause barrier and return
`requires_resume: true`. The next movement requires explicit resume. A DAgger
handover or another epoch change aborts the old trajectory and does not pause the
new human owner or replay the remaining samples on handback.

The planner checks named limits, joint branch changes, trajectory samples and
all intervening collision-check segments when collision is enabled. MoveL also
monitors measured FK deviation during execution. This is sampled checking of
configured static robot geometry, not continuous swept-volume certification or a
dynamic perception world. MoveP does not silently promise vendor-specific blending.
Backend dispatch and physical execution remain distinct recorded facts.

Example MoveJ body (the current owner must already be program):

```json
{"group":"right_arm","joint_names":["Joint1_R"],"positions":[-1.35],"speed_scale":0.25}
```

A pose request uses `pose: [x,y,z,qx,qy,qz,qw]` and a `frame_id` returned by the
capability description. Unknown frames or missing fresh joints needed for a frame
transform are rejected. Model/home positions never replace missing feedback.

## Configuration and limits

Optional compiled `motion` values include `stream_hz` (50),
`planning_timeout_s` (20), `max_duration_s` (120), `max_samples` (5000),
`path_joint_step` (0.02), `kinematics_backend` (`auto`), `collision_enabled`,
`ik_position_tolerance` (0.001 m), `ik_orientation_tolerance` (0.01 rad),
`joint_tolerance` (0.015 SI), `position_tolerance_m` (0.005),
`orientation_tolerance_rad` (0.03), `settle_duration_s` (0.1),
`settle_timeout_s` (8), and `max_cartesian_deviation_m` (0.015).

The global `teleop.control.collision_avoidance_enabled` remains effective even if
a Motion-specific flag is false. Missing collision dependencies, incomplete
feedback, deadlines or no configured geometric pairs fail closed. Geometry setup
is shared with the adapter collision checker.

`validate_external(payload)` is available independently of v23/motion selection.
The generic program/exoskeleton/replay command route must await it before core
submission. It retains the original monotonic sample and deadline, checks source,
lease and names, and runs configured geometric checking in the worker. Changes in
control tokens, incomplete observations or significant changes in collision start
state invalidate the check. Its return value is the command to submit to the core;
the core revalidates fencing and age again. An absent optional gripper channel
never implies a measured or requested value for that channel.

Warm `describe()` during an explicitly selected Motion startup. Cold first Servo
or raw collision validation may expire while loading dependencies; that target is
rejected and the worker stays warm. Its timestamp is never reset to hide startup
latency.

## Validation

The test suite covers explicit backend gating, SI name/unit contracts, dynamic
limits, Cartesian straightness, pose IK, stale input, unknown names/frames,
intermediate collisions, generic-command collision checks, subprocess isolation,
busy exclusion, cancellation, model handover, and reuse after close. The process
integration test runs the real C++ authority, real X1 URDF/PyBullet motors, planner
subprocess and feedback-based MoveJ/MoveL completion. It does not exercise physical
X1 motors or a vendor driver's buffered-command cancellation.

`validate_home(groups=None)` is used by both HTTP and VR Home entry points. With
collision checking enabled it first pauses an active source, requires stable fresh
feedback within one second, predicts the core's independently limited per-joint
Home dynamics, and checks that nominal curved joint path in the worker. It then
returns `expected_boot_id`/`expected_epoch` only if the observed start pose and
control generation still match. The caller must pass these fields to `core.home`.
With geometry checking disabled the core still enforces scope, feedback, limits,
leases and its backend barrier.

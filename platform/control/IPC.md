# Control core IPC v1

The C++ executable `hc-control --socket PATH --config JSON` is the authority for
motion. The transport is Linux `AF_UNIX/SOCK_SEQPACKET`. One UTF-8 JSON object is
one packet, at most 65536 bytes. The socket directory must be owned by the current
uid and have mode 0700; the socket is 0600. Peer uid must match (`SO_PEERCRED`).
Only one backend, one VR gateway, one IK worker and one client per streaming
program/exo/replay role may register at a time. Multiple API/read-only clients are
allowed. The API multiplexes authorized external requests onto its source client.
There is no hardware operation until a backend, fresh configured feedback,
explicit `recover`, and explicit `session_start` are present.

## Common messages

Client requests use `{ "op": "...", "id": "optional-string" }`. A supplied id
(at most 63 bytes) receives `{ "type":"response", "id":"...", "ok":true,
"code":"OK", "state":{...} }`. Requests without an id do not generate replies.
All registered clients receive a bounded-rate `{ "type":"state", ... }` stream.
Clients must read continuously and correlate replies by id; unrelated events may
precede any reply. Slow consumers are disconnected, never allowed to block motion.

First request: `{"op":"hello","id":"1","role":"api"}`. Roles are `api`,
`backend`, `vr`, `program`, `exo`, `replay`, `ik`, `recorder` (`observer` alias).
Recorder is read-only and receives every core output through a separate bounded
FIFO. The backend additionally sends
`"capabilities":{"fence_ack":true,"physical_stop_ack":false}`. Software fencing
is mandatory; physical stop acknowledgment is reported separately and must never
be claimed by merely publishing a ROS message. These are same-user trusted local
processes; remote authentication is the API/gateway's responsibility.

State contains `boot_id`, `robot_id`, `config_hash`, `epoch`, `reference_epoch`,
`lease_id`, `safety` (`stopped/ready/recovering/fault`), `mode`
(`idle/teleop/program/exo/replay/home`), `owner`
(`none/vr/program/exo/replay/human/system`), `requested_owner`, `paused`,
`dagger_enabled`, `return_paused`, `transition_pending`, `reason`, grip state,
available/configured groups, feedback validity, and rejection/overrun counters.
`boot_id` changes on every core restart. Epoch and lease are integers scoped to
that boot. The `lease_id` is assigned by the core and is never self-issued.

## Management (`api` role)

* `status`: inspect state (any registered role).
* `recover`: clear a stopped/fault state only after fresh required feedback and a
  backend fence acknowledgement. Completes in ready/idle; never resumes old work.
* `session_start`: `mode:"teleop"|"program"|"exo"|"replay"`, optional
  `groups:["left_arm","right_arm"]`, `dagger_enabled:false`. Requires ready/idle.
* `pause`, `resume`, `session_end`, `stop`, `reference_reset`, `home`.
* `set_dagger`: `enabled:true|false`, applies to the program session.

`stop` may be sent by any registered role and has a separate stop latch.
Management requests may provide `expected_boot_id` and `expected_epoch`; these
are checked by the authority thread, not just the HTTP gateway.
Requests acknowledge acceptance, not physical completion: inspect
`transition_pending` and `owner` in subsequent state. Every transition revokes the
old lease, increments epoch/reference, and requires the backend barrier before
activating the new source. `home` is a bounded, velocity/acceleration-limited
motion to configured home positions, not a zero-valued joint command; completion
returns to idle and requires explicit source selection afterward. `resume` after
replay end does not replay buffered commands.

## Backend

Feedback:

```json
{"op":"feedback","seq":1,"joint_names":["Joint1_R"],"positions":[0.0],"velocities":[0.0],"ready":true}
```

Named subsets are allowed. Duplicate/unknown names, nonfinite numbers, mismatched
lengths and non-increasing seq are rejected. Freshness is tracked per joint using
the core receive monotonic clock; absent joints are not fabricated from home.
Velocities are optional. `ready:false` revokes active motion.
`sample_mono_ns` should be the timestamp captured at the actual local ROS callback,
not after queueing or serialization. It must be CLOCK_MONOTONIC on this host.

Barrier from core:
`{"type":"fence","boot_id":"...","epoch":2,"seq":7,"reason":"..."}`.
The adapter invalidates pending commands, invokes its stop/hold/cancel procedure,
and replies `{"op":"fence_ack","boot_id":"...","epoch":2,"stopped":true}`.
The ack refers to the backend's advertised stop boundary. Until it arrives no new
source is authorized. Timeout faults the core. A hardware adapter without real
cancel/stop acknowledgement cannot claim cancellation of hardware-buffered work.

Approved outputs contain `type:"output"`, all four tokens, increasing `seq`,
`issued_mono_ns`, `deadline_mono_ns`, `valid_for_ms`, `mode:"position"|"hold"|"stop"`,
`joint_names`, `positions`, `velocities`, `base_velocity:[vx,vy,wz]`,
`grippers:[left,right]` (normalized 0..1; optional channel validity is explicit).
The validity field is `grippers_valid:[true,false]`. `tick` identifies the control
cycle. `stage:"core_gate"` means accepted by the control core, not necessarily
dispatched by the adapter or executed physically. The backend reports actual
dispatch with `{"op":"output_ack","seq":123,"accepted":true,
"sample_mono_ns":123456789}`; the recorder receives this as `type:"output_ack"`,
`stage:"backend_dispatch"`. Physical execution remains separate feedback.
Use absolute `deadline_mono_ns` from the same host CLOCK_MONOTONIC, not a renewed
timeout after dequeue. The backend MUST process fences in order and revalidate
boot/epoch/deadline at the actual hardware send boundary. Only the backend role
receives output. No output may be forwarded through an older buffered transport
after a fence. Disconnect watchdog behavior belongs to the backend as well.

## VR (`vr` role)

```json
{"op":"vr","seq":1,"sample_seq":1,"tracked":{"left":true,"right":true,"head":true},"grip":{"left":false,"right":false},"trigger":{"left":0.0,"right":0.0},"left_pose":[0,0,0,0,0,0,1],"right_pose":[0,0,0,0,0,0,1],"head_pose":[0,0,0,0,0,0,1],"base_velocity":[0,0,0]}
```

Poses are metres plus quaternion xyzw in the raw Unity tracking world frame
(`source_frame:"unity_tracking"`). The gateway validates the wire protocol; the
IK worker performs the configured axis/handedness conversion exactly once. `seq`
is transport order and `sample_seq` advances only for a new device sample. Static
poses with advancing samples remain valid; heartbeat packets with frozen samples
do not extend input freshness. On reconnect or tracking recovery, release then
press Grip before following. Press/release events retain ordering; overflow is a
stop, not silent edge loss. A head pose is optional when no head/body mapping is
configured. Base commands are normalized, bounded velocities prepared by the
gateway and accepted only in ordinary teleop. DAgger always forces base zero.
With `allow_independent_base:true`, a frame with `base_independent:true` may drive
the base without Grip while ordinary teleop owns output; its sample freshness is
still mandatory. Otherwise the left Grip is required. This never applies to
DAgger takeover.

The first Grip takes the full configured/fresh arm-and-gripper bundle in a DAgger
program session. Each Grip independently clutches its arm; only the last normal
release requests handback. Takeover from pause returns to pause. Pausing during
takeover also forces a paused return. Tracking loss, frozen input, stop and fault
never count as a normal release and do not resume a program automatically.

## IK worker (`ik` role)

Core emits `type:"ik_request"`, `boot_id/epoch/lease_id/reference_epoch`,
`seq` (the VR seq), `issued_mono_ns/deadline_mono_ns/valid_for_ms`,
`joint_names/positions` (current named feedback),
`reference_positions_left/reference_positions_right` (same joint order),
`left_pose/right_pose`, `left_reference_pose/right_reference_pose`,
`head_pose/head_reference_pose`, `active_arms:{left,right}`, `allow_body`,
`mode`, `source` (`vr` or `human`). The per-hand reference positions and poses are
captured on the actual press; they do not move while held. Mapping and FK/IK are
the worker's responsibility. The worker cannot write hardware.

Result:
`{"op":"ik_result","boot_id":"...","epoch":3,"lease_id":2,"reference_epoch":4,"seq":8,"joint_names":[...],"positions":[...],"ttl_ms":100}`.
Only previously issued, unexpired requests are accepted. Result seq must increase
but need not equal the most recent VR seq (which would starve a slower worker).
The result inherits the ORIGINAL deadline; ttl cannot extend it. The core checks
current feedback, arm masks, limits, and all tokens again before output.

## Program / exoskeleton / replay

`command`: `{source:"program"|"exo"|"replay",boot_id,epoch,lease_id,
reference_epoch,seq,ttl_ms,joint_names,positions}`. Names are a nonempty subset of
the leased, fresh groups. Optional `grippers:[left,right]` and
`base_velocity:[vx,vy,wz]` are capability-gated. Repeated or unordered seq and
nonfinite/out-of-range values are rejected. There is no unbounded command FIFO.
`renew` carries the same tokens and extends the source lease, not an expired
target. A source must send a fresh command after each handover. Handback is
pending until the backend barrier and a fresh, correctly fenced program command;
old model action chunks cannot continue.

## JSON configuration

Required arrays: `joint_names`, `lower`, `upper`, `velocity`, `acceleration`,
`home`; all lengths match, 1..64 joints. `groups` maps group names to joint names
(`left_arm/right_arm/left_gripper/right_gripper/body`). `allowed_groups` defaults
to configured arms. Optional `robot_id`, `config_hash`, `control_hz` (default 200),
`feedback_timeout_ms` (200), `command_timeout_ms` (150), `lease_timeout_ms` (1000),
`handover_timeout_ms` (1000), `home_timeout_ms` (30000), `home_tolerance` (0.02),
`enable_grippers` (false), `enable_base` (false), `base_velocity_limit` ([0.2,0.2,0.3]).
`allow_independent_base` defaults to false.
Invalid config fails before binding. Numeric time values are positive and bounded.

## Failure behavior

The control thread owns all safety, lease, reference, and motion state. A bounded
I/O event queue never blocks it. Event overflow locks stop. High-rate outputs and
state use latest-value snapshots. Replies and fences are bounded events. IK and
feedback freshness do not renew when stale messages are processed. The runtime
is soft realtime until kernel, hardware, and maximum timing are characterized.

## Recording and audit stream

The `recorder` role receives each control tick's output through a bounded FIFO,
including stop/hold outputs. Unlike the backend transport these records are never
intentionally coalesced. API and recorder clients receive an ordered
`ownership_transition` stream containing `event_seq`, `mono_ns`, `tick`,
`boot_id`, `epoch`, `config_hash`, `old_owner`, `new_owner`, and `reason`.
There can be several ownership events between two 20 Hz state snapshots.
Transitions are emitted where authority changes, not at Grip request arrival.

Core audit/output FIFO overflow increments `audit_dropped`/`outputs_dropped` in
state; recorders must mark affected data incomplete. Per-client reliable backlog
is bounded to 256 records: a slow client is disconnected instead of blocking
control. A recorder must treat disconnect/reconnect as a data gap. Live streams
do not claim durable delivery across process crashes. The external recorder owns
durable writing, segment closure and integrity status.

## Additional freshness and Home constraints

A streaming `command` may include `sample_mono_ns` and `deadline_mono_ns` from the
same host's CLOCK_MONOTONIC. With a supplied sample, effective expiration is
`min(received_at, sample_at) + min(ttl_ms, configured_command_timeout_ms)`; a supplied
deadline only shortens it. Future samples and expired targets are rejected.
`grippers_valid:[left,right]` selects which normalized auxiliary gripper targets
are requested. Inactive auxiliary channels do not carry commands across a source
revocation. State exposes `fresh_joint_names` separately from the last observed
joint values, and `last_command_seq` for the current lease.

`home_velocity` and `home_acceleration` are positive upper bounds, intersected
with each joint's configured motion limits. Optional `home_joint_names` and
`home_groups` restrict Home scope; their intersection with currently allowed
joints and request groups must have fresh feedback. Missing Home feedback is not
filled. Home remains a core-owned motion; the API and VR Home entry points perform
configured collision preflight before requesting it with expected boot/epoch.

Management messages enqueued before a control-epoch change cannot start work in
the new epoch. A VR frame while the selected program has DAgger disabled remains
observation-only and cannot reset the program reference or target.

An observed joint outside the configured URDF interval remains visible as an
observation but is excluded from control-valid feedback. An active leased joint
crossing that interval faults output. Home completion requires in-tolerance
position, low measured velocity (or a timestamped finite-difference estimate when
velocity is absent), and at least 100 ms spanning two new feedback samples. Merely
passing through the target or holding an old feedback packet is not completion.

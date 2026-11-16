# Native control authority

This directory is a standalone Linux C++17 service. It does not link ROS, Python,
the vendor driver, an IK solver, video code, a database, or a web server. It starts
stopped and owns source arbitration, command validation, fencing and periodic
bounded motion output. The separate backend is the sole physical driver writer.

```bash
cmake -S platform/control -B platform/control/build -DCMAKE_BUILD_TYPE=Release
cmake --build platform/control/build -j2
ctest --test-dir platform/control/build --output-on-failure
platform/control/build/hc-control --socket /private/runtime/control.sock --config /path/to/robot.json
```

The runtime directory must be owned by the service uid with mode 0700. The binary
creates it if its immediate parent already exists. The socket uses mode 0600 and
same-uid peer credentials; a file lock prevents concurrent authorities from
replacing each other's socket. Run a single service uid for its authorized local
adapters. Remote users never connect directly to this Unix socket.

See [IPC.md](IPC.md) for the wire and configuration contracts. The config compiler
in the Python platform should compile the selected robot's names, limits and
groups; the native service independently validates their shape and bounds.

## Execution and state ownership

* The periodic thread owns all mutable control state and consumes at most 64
  typed events per tick. Startup allocates fixed storage. No JSON, network I/O,
  disk I/O, IK, variable-length containers or blocking mutexes run in this loop.
* The I/O thread parses bounded JSON packets and services nonblocking Unix
  sockets. It uses a 512-entry SPSC ingress queue. Queue exhaustion locks stop.
* Feedback freshness uses the original local callback sample timestamp. Continuous
  output and IK requests use a tested SPSC triple buffer. Recording output and
  ownership audit use separate bounded FIFOs and expose losses.
* Every source change increments boot-scoped epoch/reference tokens and revokes
  its lease. The backend must finish its advertised stop/cancel barrier before a
  new owner is activated. A fresh program action completes DAgger handback.
* A dedicated stop latch bypasses ingress saturation. Repeated/refrozen inputs,
  stale feedback, expired candidates, missing barriers, wrong sources and old
  tokens cannot produce new authorized motion.

The loop uses CLOCK_MONOTONIC and absolute wake times; it skips missed periods
instead of replaying a backlog. It bounds commanded velocity and acceleration
during valid motion; stopping/holding and physical drive behavior remain adapter
contracts. No hard realtime or certified safety claim is made. Physical emergency
stop and a hardware watchdog remain outside this service.

## Source and data boundaries

DAgger transfers the supported arm bundle, clutches each arm independently and
returns on the last normal Grip release. Loss of tracking/input pauses instead of
returning to the program. A takeover from pause returns to pause; body is held and
base velocity is zero throughout human takeover. Supported grippers are normalized
auxiliary channels, not invented joint feedback.

The IK worker receives captured per-hand VR and robot references. Its result is
accepted only for a previously issued, unexpired request with matching authority
and reference. Results may lag the newest VR sequence to avoid starving a slower
solver. It cannot choose its own deadline or bypass current feedback/limit checks.

The core emits commanded joint candidates and dispatch audit; this is not a
physical execution guarantee. `physical_stop_ack:false` is reported as
`software_gate_only`. Drivers with buffered commands need their own confirmed
cancel/stop implementation for stronger handover guarantees.

Motion planning (`MoveJ/MoveL/MoveP`), collision checking and Cartesian IK are
external components. They are not falsely reported as built-in capabilities by
this service. `home` is a native bounded joint-position task. Config reload uses a
stopped process restart, new boot id and explicit recovery.

## Tests

`core_test.cpp` covers stale fencing, boot changes, partial feedback, source TTL,
Grip bundle/release semantics, pause return, tracking-loss behavior, delayed IK,
limits, acceleration, replay/home, role authority, audit completeness and concurrent
latest-value transport. `ipc_test.py` starts the real process and verifies socket
permissions, exclusive backend registration, lifecycle barriers, malformed data,
role rejection, output recording and single-authority socket locking.

The vendor header is pinned, unmodified nlohmann/json v3.11.3 with its MIT license.

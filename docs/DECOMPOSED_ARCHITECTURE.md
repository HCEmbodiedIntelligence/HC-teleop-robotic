# Decomposed HC Teleop workspace

The native ROS 2 packages under `src/` are the only runtime in this branch. Each
package directory is an intended repository boundary recorded in
`manifests/components.yaml`; no legacy runtime or compatibility bridge is part
of the build graph.

## Build

```bash
./bootstrap_colcon.sh deps
./bootstrap_colcon.sh build
./bootstrap_colcon.sh test
source install/setup.bash
hcctl doctor --profile openarmx
```

The build pins ROS interface generation to `/usr/bin/python3`. ROS 2 Humble is
built for Python 3.10, while an active Conda Python 3.13 environment otherwise
causes `rosidl`/Empy failures.

## Safe shadow launch

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch hc_bringup teleop.launch.py \
  profile:=openarmx mode:=shadow
```

Shadow mode publishes commands below `/robots/openarmx/shadow/control/joint_command`.
Cutover requires `shadow:=false` and is only allowed after simulation, watchdog
and single-publisher tests pass.

## Operator dashboard

`hc_dashboard` runs outside the real-time component container and starts by
default with bringup. Open `http://127.0.0.1:7877/dashboard/`. It receives typed
ROS telemetry over relative topics, exposes a low-rate WebSocket snapshot, and
only calls the command arbiter's safety services. It cannot publish a joint
command or a command candidate.

```bash
./run.sh profile:=x1 mode:=sim
# Optional overrides:
./run.sh profile:=x1 mode:=sim dashboard_port:=7878
./run.sh profile:=x1 mode:=sim start_dashboard:=false
```

## Invariants

- raw UDP terminates in `hc_vr_gateway`;
- internal commands are typed and carry source, session, sequence and expiry;
- only `hc_teleop_core` publishes an authoritative `JointCommand`;
- a device adapter must still enforce its own 150 ms watchdog and physical
  limits;
- dashboard, media and dataset processes never own actuator output;
- all code uses relative ROS names and launch applies `/robots/<robot_id>`.

# Decomposed HC Teleop workspace

The new native ROS 2 packages are staged under `src/` while the verified legacy
stack remains in place. Each top-level package directory is an intended future
repository boundary recorded in `manifests/components.yaml`. Remote URLs are
not fabricated: once hosting is assigned, the staged paths can be split with
history and the manifest replaced by a pinned vcstool `.repos` file.

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
  profile:=openarmx mode:=compact shadow:=true
```

Shadow mode publishes authoritative commands below
`/robots/openarmx/shadow/control/joint_command`. It does not touch the legacy
real-robot command topic. Cutover requires `shadow:=false` and is only allowed
after simulation, watchdog and single-publisher tests pass.

## Invariants

- raw UDP terminates in `hc_vr_gateway`;
- internal commands are typed and carry source, session, sequence and expiry;
- only `hc_teleop_core` publishes an authoritative `JointCommand`;
- a device adapter must still enforce its own 150 ms watchdog and physical
  limits;
- dashboard, media and dataset processes never own actuator output;
- all code uses relative ROS names and launch applies `/robots/<robot_id>`.

# hc_bringup

Native ROS 2 launch and profile tooling for the decomposed HC Teleop stack.

```bash
hcctl profile validate /path/to/profile.yaml
hcctl doctor --profile openarmx
ros2 launch hc_bringup teleop.launch.py profile:=openarmx mode:=compact shadow:=true
```

`shadow:=true` is the default during migration. It remaps the authoritative
command to `shadow/control/joint_command`, so the new arbiter cannot collide
with the legacy real-robot publisher. A deliberate launch argument is required
for cutover.

Profiles are strict, versioned and component-based. Resources must be either
profile-relative or `package://`; absolute paths and directory escape are
rejected. Topics in profiles must be relative so `/robots/<robot_id>` can be
applied by launch.

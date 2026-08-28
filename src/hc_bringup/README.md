# hc_bringup

Native ROS 2 launch and profile tooling for the decomposed HC Teleop stack.

```bash
hcctl profile validate /path/to/profile.yaml
hcctl doctor --profile openarmx
ros2 launch hc_bringup teleop.launch.py profile:=openarmx mode:=sim
```

`mode:=sim` starts the OpenArmX KDL IK backend, rate-limited simulation adapter,
robot state publisher, and the HC command arbiter. Use `mode:=shadow` to keep
the final command below `shadow/control/joint_command` while comparing with a
device adapter. Use `mode:=real shadow:=false` only after the device adapter has
been validated. `composition:=isolated` starts each component as a separate
process for debugging; the default `compact` composition uses ROS 2 intra-process
communication.

Profiles are strict, versioned and component-based. Resources must be either
profile-relative or `package://`; absolute paths and directory escape are
rejected. Topics in profiles must be relative so `/robots/<robot_id>` can be
applied by launch.

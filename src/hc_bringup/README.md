# hc_bringup

Native ROS 2 launch and profile tooling for the decomposed HC Teleop stack.

```bash
hcctl profile validate /path/to/profile.yaml
hcctl doctor --profile openarmx
ros2 launch hc_bringup teleop.launch.py profile:=openarmx mode:=sim
```

The operator dashboard starts as an independent process at
`http://127.0.0.1:7876/dashboard/`. It observes typed telemetry and may call
the safety enable/reset services; it never publishes an actuator command.
Use `start_dashboard:=false` to disable it, or set another bind address/port:

```bash
ros2 launch hc_bringup teleop.launch.py \
  profile:=x1 mode:=sim dashboard_host:=0.0.0.0 dashboard_port:=7877
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

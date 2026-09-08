# hc_bringup

Native ROS 2 launch and profile tooling for the decomposed HC Teleop stack.

```bash
hcctl profile validate /path/to/profile.yaml
hcctl doctor --profile openarmx
ros2 launch hc_bringup teleop.launch.py profile:=openarmx mode:=sim
```

The read-only control-chain diagnostics node starts by default. It publishes
`/robots/<robot_id>/diagnostics/control_chain` and writes periodic summaries
plus bounded anomaly captures under
`~/.ros/hc_teleop_diagnostics/<robot_id>/control_chain_*.jsonl`. Disable it
only when explicitly required with `start_diagnostics:=false`.

The operator dashboard starts as an independent process at
`http://127.0.0.1:7877/dashboard/`. It observes typed telemetry and may call
the safety enable/reset services; it never publishes an actuator command.
Use `start_dashboard:=false` to disable it, or set another bind address/port:

```bash
ros2 launch hc_bringup teleop.launch.py \
  profile:=x1 mode:=sim dashboard_host:=0.0.0.0 dashboard_port:=7878
```

`mode:=sim` starts the profile-selected motion backend, rate-limited simulation adapter,
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

The backend can be selected explicitly with `motion_backend:=robo_manip`
or `motion_backend:=external`. The default
`motion_backend:=profile` requires `motion.backend_package` to select RoboManip;
there is no implicit KDL fallback. RoboManip is always a
separate process and its measured FK is the sole `state/cartesian` publisher:

```bash
./run.sh profile:=x1 mode:=sim motion_backend:=robo_manip
ros2 topic info -v /robots/x1/state/cartesian
```

X1 and OpenArmX profiles select RoboManip by default. Motion Server and its
interfaces are built from the pinned submodules in this workspace; no external
Humanoid underlay is loaded. SDK dependencies must be installed at
`.deps/robo_manip` or explicitly selected with `HUMANOID_MOTION_SDK_DEPS_PREFIX`.
The build no longer searches old workspaces for dependencies.

For responsive simulation without changing real-robot commissioning limits,
profiles may define `simulation.robo_manip_limits`. The launch file applies
that mapping only in `mode:=sim`; shadow and real modes always use the
conservative `motion.robo_manip_*` values.

RViz starts by default using `teleop.rviz` beside the selected profile. Disable
it with `./run.sh --rviz-sim:=false` or the native launch argument
`rviz_sim:=false`. This only controls visualization, not the simulation adapter.
The RViz process shares the launch lifecycle and ROS domain.

# hc_robot_openarmx

OpenArmX V10 bimanual URDF, meshes, Motion Server resources and a strict
`hc-teleop-profile/v1` component profile.

This package contains no generic teleoperation, motion-server or dashboard
code. The two arms and two grippers are declared independently, so a future
profile can disable or replace one component without changing the framework.

Validate after building the workspace:

```bash
hcctl doctor --profile openarmx
```

The profile deliberately leaves collision interception disabled until the
installation-specific mounting geometry has been commissioned. Software
limits do not replace the physical emergency stop.

# OpenArmX robot profile

Import `openarmx_profile.zip` from the Dashboard robot configuration page, or
select the already-installed `openarmx` profile. The archive includes the V10
URDF, meshes, HC V2.3 controller config, arm teleoperation config, and upstream
model license.

For simulation, keep this profile active and run `./run.sh sim` (or
`./run.sh sim --headless`). The bundled `vr_configs.yml` maps the V10 URDF's
right arm, left arm, TCPs, and gripper joints into the PyBullet simulator.

For real hardware, the robot computer must run the companion
`openarmx_hc_robot_adapter` package from the OpenArmX workspace with
`forward_position_controller` selected, then run `./run.sh teleop` here.

Before first real motion, verify joint feedback, controller order, zero/home
pose, handedness, and emergency stop at reduced motor gains/speed.

Because this profile now has a simulation configuration, running `./run.sh`
without an explicit mode selects `sim`. Use `./run.sh teleop` explicitly when
connecting to real hardware.

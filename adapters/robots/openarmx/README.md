# OpenArmX robot profile

Import `openarmx_profile.zip` from the Dashboard robot configuration page, or
select the already-installed `openarmx` profile. The archive includes the V10
URDF, meshes, HC V2.3 controller config, arm teleoperation config, and upstream
model license.

The robot computer must run the companion `openarmx_hc_robot_adapter` package
from the OpenArmX workspace with `forward_position_controller` selected.

Before first real motion, verify joint feedback, controller order, zero/home
pose, handedness, and emergency stop at reduced motor gains/speed.

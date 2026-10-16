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

## Cameras (2026-10-16)

`cameras.yaml` is the physical-camera source of truth, migrated from the deployed
`~/.local/share/humanoid-plugins/robots/openarmx_01/cameras.yaml`. It selects two
D405 wrist cameras and the RGB-only D435 head (`camera_hand`). From the HC project:

```bash
bash tools/run_cameras.sh --profile adapters/robots/openarmx
```

Select OpenArmX in Dashboard and restart middleware to use its camera preview and
HC recording topics. Simulation (`HC_TELEOP_MODE=sim`) skips the physical camera
list. Middleware does not automatically start camera processes. Runtime camera
subscriptions replace legacy camera selections without overwriting saved generic
configuration. Raw CameraInfo is retained alongside JSON String channels:
`/camera_left/camera/calibration`, `/camera_right/camera/calibration`, and
`/camera_hand/camera/calibration`. Only available streams are represented; the
head has no fabricated depth intrinsics. This integration is for `workflow: hc`;
`platform_raw` has a separate source configuration.

The URDF includes CAD camera mounts from `openarmx_urdf-1.0_basic.rar`:

| Camera | CAD frame | Parent |
|---|---|---|
| Left | `L_Link7_XJ_Link` | `openarmx_left_link7` |
| Right | `R_LINK7_XJ_Link` | `openarmx_right_link7` |
| Head | `XJ_Link` | `XJZJ_Link` → `openarmx_body_link0` |

The driver separately publishes `camera_left_camera_link`,
`camera_right_camera_link`, `camera_hand_camera_link` and stream/optical frames.
**CAD-to-driver-base transforms remain unknown and are not published as identity.**
Robot-to-CAD and driver-internal TF are recorded, but complete robot-to-optical
extrinsics require those three mounting registrations. CAD origins are not
measured optical centers. Both `/tf` and transient-local `/tf_static` are selected
in the default recording configuration. Each HC segment creates a fresh subscriber
and retrieves retained static messages from existing broadcasters.

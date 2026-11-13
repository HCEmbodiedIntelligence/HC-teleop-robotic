# X1 head-camera DDS transport fix

This patch contains the camera changes deployed on 192.168.10.117 on 2026-11-13.
HC_X1 on that robot is not a Git checkout, so its exact source patch is versioned
here with the teleoperation control changes.

The head JPEG process alone uses a 128 MiB SHM transport and custom UDP with
16 MiB socket buffers. Built-in transports are disabled for this process. ROS
domain 13, other participants, image QoS, source timestamps and all actuator
safety/smoothing settings are unchanged. Wired addresses match that robot's
existing DDS profile; review them before deploying to another machine.

## Deployment

Run from this repository root, with HC_X1 stopped. Back up its camera package
first. On the already fixed robot, do not reapply this patch.

```bash
patch --dry-run --forward -p1 -d "$HOME/HC_X1" < tools/deploy/hc_x1_camera_dds/hc_x1_camera_dds.patch
patch --forward -p1 -d "$HOME/HC_X1" < tools/deploy/hc_x1_camera_dds/hc_x1_camera_dds.patch
```

Rebuild/install `hc_tj_camera_bridge` using the normal HC_X1 build procedure.
Both the modified launch file and the new XML must be in the installed package.
The original robot uses a symlinked launch file; its XML was also copied into
`install/hc_tj_camera_bridge/share/hc_tj_camera_bridge/config/` during hot loading.
`HC_X1_HEAD_CAMERA_DDS_PROFILE` optionally overrides the XML path for this reader.
Never export this camera profile globally for the control stack.

To roll back, stop HC_X1, reverse the source patch with `patch -R -p1`, rebuild
the camera package, and restart normally. Do not leave duplicate head readers
publishing the same compressed topic.

## Validation

- Same-source simultaneous A/B: 1,277 SDK frames in 42.965 seconds; original
  JPEG callback missed 36, candidate missed 0.
- Overlapping compressed-output observation: all 923 SDK source timestamps
  arrived at both candidate JPEG input and compressed subscriber.
- Production follow-up: 1,720 SDK frames in 57.963 seconds; JPEG input missed 0.
- Fixed participant has one 128 MiB SHM segment and zero UDP receive drops at
  the post-deployment counter check.

These measurements verify the tested DDS-to-JPEG path. Headset viewing and
physical arm-motion acceptance remain pending operator reconnection/testing;
no new hand-to-arm latency or absence of motion jitter is claimed here.

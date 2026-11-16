# X1 head-camera DDS transport fix

This patch contains the camera changes deployed on 192.168.10.117 on 2026-11-13.
HC_X1 on that robot is not a Git checkout, so its exact source patch is versioned
here with the teleoperation control changes.

The head JPEG process uses a 128 MiB SHM transport and custom UDP with
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

## Head depth reader follow-up (2026-11-16)

The original patch scoped the profile to JPEG only. The PNG reader still inherited
the old profile, including the built-in small-buffer transport. A simultaneous
26.231-second interior comparison found 786 SDK depth source frames, 777 PNG input
callbacks, and 777 compressed depth frames at a passive subscriber. All 786 SDK
color frames reached both JPEG input and output. The nine missing depth stamps
were lost before PNG encoding, not in MCAP writing. The old depth reader's DDS
UDP sockets also accumulated packet drops; these counters count packets, not
whole images.

Apply the incremental depth patch **after** the original JPEG patch, including on
the robot that already has the 2026-11-13 fix:

```bash
patch --dry-run --forward -p1 -d "$HOME/HC_X1" < tools/deploy/hc_x1_camera_dds/hc_x1_camera_depth_dds.patch
patch --forward -p1 -d "$HOME/HC_X1" < tools/deploy/hc_x1_camera_dds/hc_x1_camera_depth_dds.patch
```

The depth process now receives the same two process-local profile environment
variables as JPEG. Install the launch file and XML together, then restart the
camera bridge through the normal hardware lifecycle. Do not change the external
camera driver's transport, global ROS environment, source stamps, compression
settings, or the live `KEEP_LAST(1)` image policy. Do not start duplicate readers
on the production output topics. Keep control output disabled during deployment.

Verify exact source-header sets at SDK, compressor entry, compressed output, and
recording. An arrival-time gap can be scheduling jitter, while a source-header gap
can already exist in the SDK. Match RGB and depth by source header; ordinal frame
numbers become misaligned after a missing frame. A single missing counterpart at
capture start/end must also be checked against the recording's boundary policy.

This follow-up fixes the omitted depth transport configuration. Its measured
post-deployment acceptance must be reported separately; the original JPEG results
below do not establish depth completeness or a perfectly periodic sensor source.

Post-deployment comparison under headset viewing: over 66.887595 seconds, all
2,000 SDK source timestamps arrived at JPEG input, PNG input, and both compressed
subscribers. The subsequent operator recording contained 600 color and 600 depth
frames with identical timestamp sets. These checks establish the tested host
transport path; source timing remains a separate failed check below.

### Scoped hot-replacement lifecycle

When replacing only PNG while its existing camera launch stays running,
`camera_child_guard.py --launch-pid <camera-launch-pid> -- <PNG command>` owns the
replacement's new process group and binds its lifetime to the launch PID plus
Linux process start time. A vanished or reused launch PID stops only that owned
group. Stop the verified old PNG process before starting a replacement; this
helper does not adopt existing processes. On a normal full camera launch, use
the patched launch file and its normal child ownership instead.

`tests/test_camera_child_guard.py` verifies absent-launch rejection, cleanup after
launch/child exit (including stubborn descendants), and preservation of unrelated
processes. The hot replacement on the tested robot is fenced to camera launch
PID 2937243; all arm driver and smoother processes were retained.

## Source cadence and exposure investigation

The head device is a BerxelHawk network camera (`192.168.2.10`, VID `0x0603`,
PID `0x000e`), using Berxel SDK `2.0.161`. Startup reports firmware BCD
`0x03bf`; the available log does not expose a retail model name or a complete
firmware semantic version. Do not substitute the separate Bfront/Angstrong
camera's firmware version. The Berxel library hashes identify the tested builds:

```
libBerxelHawk.so     dde572311e74f5fcc523953e3b76b97d7c638f0431aec9e250cbd6337aa99726
libBerxelNetDriver.so 95eb3d3fc37a3d879f726bd3cc5ce2c5f4df2ddf95516c525460dac9cd69b60e
```

The external driver reads
`~/Programs/AGIInterface/Config/camera_config.yaml`, not the separate
`HC_X1/config/camera/camera_config.yaml` copy. It is managed by the independent
`watchdog-camera-driver.service`. Its source patch
`berxel_color_exposure.patch` is relative to
`~/Programs/ROS2_WS/src/camera_driver` and preserves default AE. Explicit manual
settings require the SDK-documented exposure/gain range, a successful call, and
strict readback. Default AE startup warns if exposure readback is unavailable;
it retains the original startup behavior. One additional read occurs after the
streams start. No frame timestamps or geometry are rewritten.

The tested NET firmware returns mode/exposure/gain `0/0/0` after the AE command.
After a diagnostic request for 10,000 us/gain 100, the underlying network SET
acknowledged success and GET returned `1/10000/100`, including after streaming
started. The mode byte conflicts with the generic SDK documentation. Its meaning
is unverified, and this device does not pass the product manual-mode guard.
**Default AE is the only accepted product configuration here.** A temporary test
binary bypassed only the mode check under an explicit diagnostic environment
flag; exact exposure/gain and the underlying SET acknowledgement were required.
That override is absent from the product patch. The temporary unit override was
removed and the installed binary was restored after the test.

The exposure test did not fix source timing. AE had seven source-header gaps
over 40 ms in 66.887595 seconds (maximum 67.025 ms); the 10 ms request had nine
in 68.427924 seconds (maximum 66.335 ms). Both streams retained all 2,043 source
timestamps through both encoders in the latter window. Mean JPEG gray-level
medians were 122.295/255 and 121.643/255 respectively; these observations are
not a claim that the undocumented mode bit means AE is enabled or disabled.
The exact original AE configuration and strict product binary were restored,
and the restoring AE command's underlying network acknowledgement was verified.

Direct probes at the first UDP fragment already showed the common 65--67 ms
gaps in the camera-provided timestamp and fragment-arrival rhythm, with consecutive
original device frame IDs. Thus these gaps precede SDK image callbacks, encoding,
and MCAP writing. Current `eno1` is 1000 Mb/s/full duplex, MTU 1500. Its historical
RX PAUSE count stayed at 1,179,426,927 and TX PAUSE at zero throughout the test;
no new CRC/FIFO errors were observed. No flow-control/network setting was changed.
Periodic clock writes did not coincide with all source gaps. Neither exposure
nor current host PAUSE activity explains away the remaining source cadence issue.
The **source gap >40 ms check still fails**. This requires camera/firmware output
investigation; do not duplicate frames, rewrite stamps, or relax the threshold.

The SDK's bundled `Documentation/Berxel SDK Developer Documentation_English.pdf`,
section 3.4.3 (printed page 30), lists MIX RGB at 640x400/30 fps and depth at
640x400/5,10,15,20,25,30 fps. Its ROS2 `README_EN.MD` line 45 gives the same depth
rates. It does not document 60 Hz synchronized 640x400 RGBD. An
`enableHightFpsMode` API exists but does not establish 60 Hz support for this NET
device. No unsupported rate, synchronization, slave-mode, firmware, or image
geometry change was attempted.

For subsequent diagnostics, log `camera_source_gap` at the SDK callback with
previous/current original source stamps, SDK frame indices, steady-clock interval,
stream, and suppressed-event count, at most once per second per stream. Label
its stage explicitly: an SDK index is not necessarily the original network frame
ID. A separate `camera_bridge_input_gap` can identify compressor input loss by
matching source stamps to the SDK event. Retain these events in the log system
and recording diagnostics; do not change actual recorded image samples. This is
a proposed follow-up, not a deployed source-gap logger in this patch set.

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

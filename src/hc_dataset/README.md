# hc_dataset

Dataset control plane backed by `rosbag2`. Recording writes serialized ROS CDR
directly through the MCAP storage plugin; the Python node does not subscribe to
or deserialize high-rate streams. No recorder child process exists while idle.

Replay is safe by default and has no live-output switch. Every recorded topic
is remapped below `/replay/<dataset_id>/...`, so a dataset containing an old
joint command can never publish onto the live robot command topic.

Install the standard Humble storage plugin through rosdep (package dependency
`rosbag2_storage_mcap`) before recording.

## Reprofile legacy MCAP recordings

`hc_mcap_reprofile` converts a legacy `/hc_teleop/*` recording to the exact
MCAP Header, schema, channel and QoS metadata contract of an
`hc_tj_description` reference recording. Dynamic robot and image payloads come
from the source. Hand state is derived from the two finger command streams;
unavailable wrench, trigger and vibration samples are explicitly zero-filled.
Camera calibration, extrinsics and `robot_info` are copied once from the
reference as static configuration.

Joint payloads are normalized as well as their schemas. `io_teleop/joint_cmd`
and `io_teleop/joint_states` are rebuilt in the exact joint-name order of the
reference instead of directly copying a legacy array that merely uses the same
ROS message type. For the X1 profile this means 14 arm joints followed by
`leg_1`, `leg_2` and `zhi`; `R_ban/L_ban` remain only in the dedicated finger
topics. Every output file is rejected if command/state dimensions or names
differ from the reference, or if a finger command is not binary.

Some arm-only recordings contain no waist samples. In that case the converter
marks the synthesized names in its JSON report and uses the commissioned X1
home values `leg_1=0.5`, `leg_2=1.2`, `zhi=-0.6` radians, with zero velocity and
effort. Override a value explicitly with repeated
`--missing-joint NAME=RADIANS` options.

```bash
hc_mcap_reprofile SOURCE.mcap REFERENCE.mcap OUTPUT.mcap
```

The source may also be an SFTP URL. SSH keys are preferred. For a temporary
password-based transfer, provide the password through the environment so it is
not exposed in the process argument list:

```bash
HC_MCAP_SFTP_PASSWORD='...' hc_mcap_reprofile \
  'sftp://user@host/absolute/source.mcap' REFERENCE.mcap OUTPUT.mcap
```

The command refuses to replace an input or an existing output. Pass
`--overwrite` only when replacement is intended. Use `--json` for the complete
topic/message conversion report and `--auxiliary-rate 10` to change the derived
hand stream rate.

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

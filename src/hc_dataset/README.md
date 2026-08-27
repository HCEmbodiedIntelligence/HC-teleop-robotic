# hc_dataset

Dataset control plane backed by `rosbag2`. Recording writes serialized ROS CDR
directly through the MCAP storage plugin; the Python node does not subscribe to
or deserialize high-rate streams. No recorder child process exists while idle.

Replay is safe by default and has no live-output switch. Every recorded topic
is remapped below `/replay/<dataset_id>/...`, so a dataset containing an old
joint command can never publish onto the live robot command topic.

Install the standard Humble storage plugin through rosdep (package dependency
`rosbag2_storage_mcap`) before recording.

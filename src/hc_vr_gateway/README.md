# hc_vr_gateway

Standalone ROS 2 C++ gateway for the PICO tracking and controller UDP protocol.
It accepts the legacy 102-byte v1 pose packet and the 162-byte v2 pose/input
packet, rejects malformed, duplicate and out-of-order datagrams, and publishes
the latest accepted sample as `hc_teleop_interfaces/msg/VrFrame` with sensor-data
QoS (`best_effort`, `volatile`, `keep_last(1)`).

The discovery endpoint remains wire-compatible with the existing headset:
`PICO_DISCOVER_V1` on UDP 5006 receives `PICO_RECEIVER_V1|5005` in reply.

```bash
ros2 run hc_vr_gateway hc_vr_gateway_node --ros-args \
  -p listen_host:=0.0.0.0 \
  -p pose_port:=5005 \
  -p discovery_port:=5006 \
  -p timeout_ms:=600 \
  -p output_topic:=input/vr_frame
```

The same implementation is registered as the rclcpp component
`hc_vr_gateway::VrGatewayNode`, so bringup can select an isolated process or a
composable container without changing the protocol behavior.

Parameters:

| Name | Default | Meaning |
| --- | --- | --- |
| `listen_host` | `0.0.0.0` | IPv4 address used for both UDP binds |
| `pose_port` | `5005` | PICO tracking/input receive port |
| `discovery_port` | `5006` | PICO discovery receive port |
| `timeout_ms` | `600` | Time after which the current sender session expires |
| `output_topic` | `input/vr_frame` | Relative or absolute `VrFrame` topic |
| `frame_id` | `vr_tracking` | Tracking coordinate-frame identifier |

The decoder in `hc_vr_codec` is ROS-independent so protocol compatibility can
be tested without starting DDS or opening a socket.

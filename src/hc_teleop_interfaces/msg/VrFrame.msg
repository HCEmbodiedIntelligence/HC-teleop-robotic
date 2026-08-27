# stamp is when the complete frame was accepted by the receiver. frame_id is
# the common tracking frame used by all poses in this message.
std_msgs/Header header

string source_id
uint16 protocol_version
uint32 sequence

# Device-originated time, if available. A zero value means unavailable.
builtin_interfaces/Time source_stamp

# Monotonic loss counter maintained by the transport adapter.
uint64 packet_loss_total

TrackedPose head
TrackedPose left_controller
TrackedPose right_controller
ControllerInput left_input
ControllerInput right_input

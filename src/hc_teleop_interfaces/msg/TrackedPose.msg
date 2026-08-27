uint8 TRACKING_UNAVAILABLE=0
uint8 TRACKING_INFERRED=1
uint8 TRACKING_TRACKED=2

# stamp is the receiver timestamp; frame_id is the transport's tracking frame.
std_msgs/Header header
uint8 tracking_state
geometry_msgs/Pose pose
geometry_msgs/Twist twist
bool velocity_valid

# Normalized confidence in [0, 1]. Zero is used when the device has no metric.
float32 confidence

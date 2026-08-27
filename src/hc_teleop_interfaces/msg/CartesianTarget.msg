uint8 FULL_POSE=0
uint8 POSITION_ONLY=1
uint8 ORIENTATION_ONLY=2

string group_name
string reference_frame
string tip_frame
uint8 pose_mode
geometry_msgs/Pose pose

# Optional Cartesian feed-forward in reference_frame. All-zero means absent.
geometry_msgs/Twist feedforward

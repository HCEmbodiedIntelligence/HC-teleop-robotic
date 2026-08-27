uint8 CONTROL_MODE_VELOCITY=1

std_msgs/Header header
string source_id
string session_id
uint64 sequence
builtin_interfaces/Time valid_until
uint8 control_mode
string base_name

# Linear values are m/s and angular values are rad/s. header.frame_id names the
# frame in which the twist is expressed.
geometry_msgs/Twist command

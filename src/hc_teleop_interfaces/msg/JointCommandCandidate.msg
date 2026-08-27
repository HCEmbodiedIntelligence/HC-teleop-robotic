uint8 CONTROL_MODE_POSITION=1
uint8 CONTROL_MODE_VELOCITY=2
uint8 CONTROL_MODE_EFFORT=3

# Untrusted command proposed by one control source. Only the arbiter may turn
# a candidate into JointCommand.
std_msgs/Header header
string source_id
string session_id
uint64 sequence
builtin_interfaces/Time valid_until
uint8 control_mode
string group_name

# The outer header and envelope are authoritative; command.header is ignored.
sensor_msgs/JointState command

uint8 CONTROL_MODE_POSITION=1
uint8 CONTROL_MODE_VELOCITY=2
uint8 CONTROL_MODE_EFFORT=3

# Authoritative command emitted by the control arbiter. Drivers subscribe to
# this type and must never subscribe directly to candidate topics.
std_msgs/Header header
string source_id
string session_id
uint64 sequence
builtin_interfaces/Time valid_until
uint8 control_mode
string group_name

# The outer header and envelope are authoritative; command.header is ignored.
sensor_msgs/JointState command

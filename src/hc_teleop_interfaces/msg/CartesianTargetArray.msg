# One atomic Cartesian command frame. Every nested target shares this envelope.
std_msgs/Header header
string source_id
string session_id
uint64 sequence
builtin_interfaces/Time valid_until
CartesianTarget[] targets

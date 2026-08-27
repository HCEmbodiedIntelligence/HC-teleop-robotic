uint8 DISABLED=0
uint8 STANDBY=1
uint8 ACTIVE=2
uint8 FAULT=3
uint8 ESTOP=4

std_msgs/Header header
uint8 state
bool enabled
bool fault_latched
bool estop_active
string reason
string active_source
string active_session
builtin_interfaces/Time lease_expires_at

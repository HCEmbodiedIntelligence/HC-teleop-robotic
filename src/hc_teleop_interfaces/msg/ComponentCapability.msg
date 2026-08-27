uint8 TYPE_UNKNOWN=0
uint8 TYPE_ARM=1
uint8 TYPE_GRIPPER=2
uint8 TYPE_DEXTEROUS_HAND=3
uint8 TYPE_CAMERA=4
uint8 TYPE_WAIST=5
uint8 TYPE_BASE=6
uint8 TYPE_SENSOR=7

string component_id
string display_name
uint8 component_type
uint32 dof
string[] joint_names

# Stable feature names, for example servo_p, move_j, rgb, depth, or odometry.
string[] capabilities

# Fully resolved ROS graph names exposed by this component.
string[] command_interfaces
string[] state_interfaces

bool available
Status status

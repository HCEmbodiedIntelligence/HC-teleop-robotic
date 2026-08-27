# hc_compat_bridge

Temporary bridges used while legacy nodes are migrated to
`hc_teleop_interfaces`. The VR bridge converts `VrFrame` into the historical
`std_msgs/String` JSON schema without parsing or allocating JSON on the
real-time command path.

The node uses relative topics (`input/vr_frame` and `legacy/vrdata`). Bringup
may explicitly remap the latter to the global `/vrdata` topic while the old
adapter is still in service. Do not run the old Python UDP gateway and the new
C++ gateway at the same time.

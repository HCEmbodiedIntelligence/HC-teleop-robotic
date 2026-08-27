# hc_adapter_openarmx

OpenArmX adapter package for the decomposed HC teleop runtime.

The first implementation is a self-contained simulation adapter:

- subscribes only to authoritative `JointCommand`;
- publishes measured `sensor_msgs/JointState`;
- computes measured arm TCP poses from the OpenArmX URDF through KDL and
  publishes `CartesianStateArray`;
- holds current position when a command expires.

It is intended for direction, zero, limit, timeout and replay-isolation tests
before any real OpenArmX hardware adapter owns final commands.

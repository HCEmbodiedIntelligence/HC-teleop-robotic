# hc_diagnostics

Read-only diagnostics for the HC control chain. The node correlates
`VrFrame -> CartesianTargetArray -> JointCommandCandidate -> JointCommand`
using session, sequence, and group, and compares the latest command with
measured `JointState` feedback.

VR transport timing is split into the packet interval stamped by the UDP
gateway and the delay until the diagnostics ROS callback runs. This separates
Wi-Fi/source jitter from DDS/executor scheduling delay.

It publishes standard `diagnostic_msgs/DiagnosticArray` on
`diagnostics/control_chain`. Normal operation is summarized every five seconds;
anomalies save a bounded pre/post event window as JSONL. The node never
publishes robot commands and may be restarted without interrupting control.

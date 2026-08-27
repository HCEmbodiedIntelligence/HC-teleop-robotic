# ROS 2 QoS and timing contract

QoS is part of the interoperability contract even though ROS `.msg` files cannot encode it.

| Data class | Reliability | History | Durability | Additional rule |
| --- | --- | --- | --- | --- |
| `VrFrame`, `CartesianTargetArray` | best effort | keep last 1 | volatile | Drop old frames; never build a control backlog. |
| `JointCommandCandidate`, `BaseCommandCandidate` | best effort | keep last 1 | volatile | Deadline is the configured control period; reject at `valid_until`. |
| Authoritative `JointCommand` | reliable | keep last 1 | volatile | Driver still rejects expired or non-monotonic commands. |
| `SafetyState`, `RobotCapabilities` | reliable | keep last 1 | transient local | A late UI or adapter receives the latest state immediately. |
| `ControlLease` | reliable | keep last 10 | transient local | Publish acquisition, renewal, revocation, and expiry transitions. |
| Services and actions | ROS 2 default service/action QoS | n/a | volatile | Every request must have a bounded application timeout. |

Transport adapters stamp `VrFrame.header.stamp` when a complete, validated frame enters ROS. The
optional `source_stamp` is diagnostic data and must not replace local freshness checks unless the
two clocks are explicitly synchronized. `packet_loss_total` is monotonic for one receiver process.

Target and candidate publishers issue one monotonically increasing sequence per session. Consumers
discard duplicates, out-of-order samples, empty source/session IDs, zero deadlines, expired samples,
NaN/Inf values, and malformed arrays. `CartesianTargetArray` is atomic: either every nested target
passes validation and is accepted under the envelope, or none is applied.

Liveliness loss and DDS deadline misses are diagnostics; the absolute `valid_until` and
`ControlLease.expires_at` checks remain the safety authority. An adapter must enter its configured
safe stop/hold behavior when authoritative command freshness is lost.

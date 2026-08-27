# hc_motion

Vendor-neutral motion core extracted from the useful parts of the former
`humanoid_motion_server`:

- deterministic joint-name conflict arbitration;
- closed-loop goal completion from measured feedback and backend FK;
- a strict process router that correlates Cartesian inputs with backend joint
  candidates;
- a small optional same-build C++ kinematics injection interface.

The package deliberately contains no RoboManip, Ruckig, Toppra, PyBullet or
robot-driver dependency. See `docs/backend_contract.md` for the stable ROS
boundary used by independently released backends.

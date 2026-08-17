# HC controller v2.3 behavioral reconstruction

This directory comes from `hc_controller_v23_reconstruction_pass2.zip`. It is
a behavioral reconstruction, not recovered vendor source. The original archive
is preserved conceptually, with two integration corrections:

- `PinocchioInterfaceV3.relative_jacobian()` uses frame-origin velocities and
  the rotating-root term. The archive implementation differed from HC-TJ
  finite differences by as much as 1.07; the corrected implementation differs
  by less than `4e-8`.
- The box-constrained least-squares active set can release an incorrectly
  activated bound and checks the KKT conditions.

Still inferred: the optional fourth task scalar is treated as task gain, the
exact vendor damping/QP backend is unknown, and `control` uses feedback while
`retarget` integrates from the previous command. Keep the protected generic
backend available for runtime A/B comparison.

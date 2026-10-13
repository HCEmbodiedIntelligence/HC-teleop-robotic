from __future__ import annotations


class SolverRearmGate:
    """Prevent an external IK integrator from reclaiming control with stale state."""

    READY = "ready"
    HOMING = "homing"
    WAIT_FEEDBACK = "wait_feedback"
    WAIT_RESET = "wait_reset"
    WAIT_ACK = "wait_ack"
    WAIT_SOLVER = "wait_solver"
    WAIT_RELEASE = "wait_release"
    WAIT_PRESS = "wait_press"

    def __init__(self) -> None:
        self.state = self.READY
        self.reset_cutoff_ns = 0

    @property
    def blocked(self) -> bool:
        return self.state != self.READY

    def start_homing(self) -> None:
        self.state = self.HOMING
        self.reset_cutoff_ns = 0

    def homing_complete(self) -> None:
        if self.state == self.HOMING:
            self.state = self.WAIT_FEEDBACK

    def observe_feedback(self) -> bool:
        """Return true exactly once when a post-home feedback frame arrives."""
        if self.state != self.WAIT_FEEDBACK:
            return False
        self.state = self.WAIT_RESET
        return True

    def reset_published(self, cutoff_ns: int) -> None:
        if self.state != self.WAIT_RESET:
            raise RuntimeError("solver reset was published outside the reset phase")
        cutoff_ns = int(cutoff_ns)
        if cutoff_ns <= 0:
            raise ValueError("solver reset cutoff must be positive")
        self.reset_cutoff_ns = cutoff_ns
        self.state = self.WAIT_ACK

    def observe_reset_ack(self, cutoff_ns: int) -> bool:
        """Open the solver-output gate only after reset was applied to feedback."""
        if self.state != self.WAIT_ACK:
            return False
        cutoff_ns = int(cutoff_ns)
        if cutoff_ns <= self.reset_cutoff_ns:
            return False
        self.reset_cutoff_ns = cutoff_ns
        self.state = self.WAIT_SOLVER
        return True

    def observe_solver_output(self, stamp_ns: int) -> bool:
        """Accept only solver output created after the reset publication."""
        if self.state != self.WAIT_SOLVER:
            return False
        if int(stamp_ns) <= self.reset_cutoff_ns:
            return False
        self.state = self.WAIT_RELEASE
        return True

    def observe_clutch(self, pressed: bool) -> bool:
        """Require a physical release followed by a new press before rearming."""
        if self.state == self.WAIT_RELEASE and not pressed:
            self.state = self.WAIT_PRESS
            return False
        if self.state == self.WAIT_PRESS and pressed:
            self.state = self.READY
            return True
        return False

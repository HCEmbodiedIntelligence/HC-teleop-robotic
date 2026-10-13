import unittest

from adapters.core.solver_rearm import SolverRearmGate


class SolverRearmGateTests(unittest.TestCase):
    def test_full_home_rearm_sequence_rejects_stale_solver_output(self):
        gate = SolverRearmGate()

        gate.start_homing()
        self.assertTrue(gate.blocked)
        self.assertFalse(gate.observe_solver_output(100))

        gate.homing_complete()
        self.assertTrue(gate.observe_feedback())
        gate.reset_published(200)

        self.assertFalse(gate.observe_solver_output(201))
        self.assertFalse(gate.observe_reset_ack(200))
        self.assertTrue(gate.observe_reset_ack(210))
        self.assertFalse(gate.observe_solver_output(199))
        self.assertFalse(gate.observe_solver_output(210))
        self.assertTrue(gate.observe_solver_output(211))
        self.assertEqual(gate.state, SolverRearmGate.WAIT_RELEASE)

        self.assertFalse(gate.observe_clutch(True))
        self.assertTrue(gate.blocked)
        self.assertFalse(gate.observe_clutch(False))
        self.assertEqual(gate.state, SolverRearmGate.WAIT_PRESS)
        self.assertTrue(gate.observe_clutch(True))
        self.assertFalse(gate.blocked)

    def test_release_before_fresh_solver_output_does_not_rearm(self):
        gate = SolverRearmGate()
        gate.start_homing()
        gate.homing_complete()

        self.assertFalse(gate.observe_clutch(False))
        self.assertTrue(gate.observe_feedback())
        gate.reset_published(10)
        self.assertFalse(gate.observe_clutch(False))
        self.assertTrue(gate.observe_reset_ack(11))
        self.assertTrue(gate.observe_solver_output(12))

        self.assertFalse(gate.observe_clutch(False))
        self.assertTrue(gate.observe_clutch(True))

    def test_reset_requires_positive_timestamp_and_correct_phase(self):
        gate = SolverRearmGate()
        with self.assertRaises(RuntimeError):
            gate.reset_published(1)

        gate.start_homing()
        gate.homing_complete()
        gate.observe_feedback()
        with self.assertRaises(ValueError):
            gate.reset_published(0)


if __name__ == "__main__":
    unittest.main()

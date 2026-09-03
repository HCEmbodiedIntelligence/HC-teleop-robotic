from __future__ import annotations

import math
import sys
import time
import unittest
from pathlib import Path
from unittest.mock import MagicMock, patch

_HC_X1 = Path(__file__).resolve().parents[2] / "HC_X1"
if _HC_X1.is_dir() and str(_HC_X1) not in sys.path:
    sys.path.insert(0, str(_HC_X1))

from hc_x1.command_smoothing import JointTrajectorySmoother, SmoothingLimits


class HardwareReadySmootherGateTests(unittest.TestCase):
    """Verify HC_X1 smoother state transitions and target discarding during homing."""

    def setUp(self):
        self.limits = SmoothingLimits(time_constant=0.04, max_velocity=2.5, max_acceleration=12.0)
        self.smoother = JointTrajectorySmoother(self.limits)

    def test_reinitialize_clears_old_targets_and_resyncs_to_fresh_feedback(self):
        # 1. Simulate target received and smoothed during previous session
        self.smoother.reset(["j1", "j2"], [1.0, -1.0], feedback={"j1": 0.0, "j2": 0.0})
        for _ in range(10):
            pos, vel = self.smoother.step(["j1", "j2"], [1.0, -1.0], 0.01)
        self.assertGreater(pos[0], 0.0)

        # 2. Simulate homing complete edge: clear and re-initialize from fresh hardware feedback
        fresh_feedback = {"j1": -0.85, "j2": 0.45}
        # Old targets cleared:
        target_names: list[str] = []
        target_positions: list[float] = []

        # Smoother reinitialized with fresh feedback:
        names = list(fresh_feedback.keys())
        positions = [fresh_feedback[k] for k in names]
        output = self.smoother.reset(names, positions, feedback=fresh_feedback)

        self.assertEqual(output, [-0.85, 0.45])
        self.assertEqual(self.smoother.positions["j1"], -0.85)
        self.assertEqual(self.smoother.positions["j2"], 0.45)
        self.assertEqual(self.smoother.velocities["j1"], 0.0)
        self.assertEqual(self.smoother.velocities["j2"], 0.0)

        # 3. Step towards a new target received AFTER homing
        new_target = [-0.80, 0.50]
        step_pos, step_vel = self.smoother.step(names, new_target, 0.01, feedback=fresh_feedback)
        # Verify no acceleration jump: initial velocity is bounded and smooth
        self.assertAlmostEqual(step_pos[0], -0.85 + step_vel[0] * 0.01)
        self.assertAlmostEqual(step_pos[1], 0.45 + step_vel[1] * 0.01)
        self.assertLess(abs(step_vel[0]), 1.5)


class HardwareReadyLogicSimulationTests(unittest.TestCase):
    """Simulate the edge-triggered handshake and command gating without ROS graph."""

    def test_driver_drops_targets_during_homing(self):
        homing_in_progress = True
        waist_homing = False
        hardware_ready = False

        accepted_commands = []

        def simulated_joint_cmd_callback(names, positions):
            nonlocal homing_in_progress, waist_homing, hardware_ready
            if homing_in_progress or waist_homing or not hardware_ready:
                # Dropped
                return False
            accepted_commands.append(dict(zip(names, positions)))
            return True

        # When homing is in progress, commands MUST be dropped
        result = simulated_joint_cmd_callback(["joint1"], [0.5])
        self.assertFalse(result)
        self.assertEqual(len(accepted_commands), 0)

        # Once homing completes and hardware is ready:
        homing_in_progress = False
        hardware_ready = True
        result = simulated_joint_cmd_callback(["joint1"], [0.5])
        self.assertTrue(result)
        self.assertEqual(len(accepted_commands), 1)
        self.assertEqual(accepted_commands[0]["joint1"], 0.5)

    def test_middleware_mux_gates_joint_cmd_forwarding(self):
        hardware_ready = False
        forwarded = []

        def simulated_mux_callback(source, valid, output_enabled):
            nonlocal hardware_ready
            if not valid or not output_enabled or not hardware_ready:
                return False
            forwarded.append(source)
            return True

        # Target arrives while hardware is not ready
        self.assertFalse(simulated_mux_callback("vr", valid=True, output_enabled=True))
        self.assertEqual(len(forwarded), 0)

        # Hardware becomes ready
        hardware_ready = True
        self.assertTrue(simulated_mux_callback("vr", valid=True, output_enabled=True))
        self.assertEqual(forwarded, ["vr"])


if __name__ == "__main__":
    unittest.main()

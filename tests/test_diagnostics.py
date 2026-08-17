import unittest

from teleop_diagnostics import ARM_JOINTS, POSE_FIELDS, SIDES, TeleopDiagnosticsNode


class DiagnosticsTests(unittest.TestCase):
    def test_csv_schema_contains_pose_and_joint_command_feedback(self):
        fields = TeleopDiagnosticsNode._field_names()
        for source in ("controller", "target", "actual"):
            for side in SIDES:
                for field in POSE_FIELDS:
                    self.assertIn(f"{source}_{side}_{field}", fields)
        for name in ARM_JOINTS:
            self.assertIn(f"joint_state_{name}", fields)
            self.assertIn(f"joint_command_{name}", fields)
            self.assertIn(f"joint_error_{name}", fields)
        for side in SIDES:
            self.assertIn(f"ik_{side}_converged", fields)
            self.assertIn(f"ik_{side}_rejection_total", fields)


if __name__ == "__main__":
    unittest.main()

import importlib.util
from pathlib import Path
import unittest

path = Path(__file__).resolve().parents[1] / 'adapters/nodes/motion_teleop_bridge.py'
spec = importlib.util.spec_from_file_location('motion_teleop_bridge', path)
bridge = importlib.util.module_from_spec(spec)
spec.loader.exec_module(bridge)


class MotionCacheTests(unittest.TestCase):
    def setUp(self):
        self.cache = bridge.MotionCache()
        self.names = list(bridge.ARM_NAMES['right'] + bridge.ARM_NAMES['left']) + ['leg_1', 'leg_2', 'zhi']
        self.measured = dict.fromkeys(self.names, 0.1)
        self.cache.feedback_received(self.names, list(self.measured.values()), 100, 1, 100)
        self.cache.hardware, self.cache.hardware_time = True, 1
        self.status = {'enabled': True, 'mode': 'arms_grippers', 'arms': {'left': {'active': True}, 'right': {'active': True}}}
        self.cache.update_status(self.status, 1, 100)

    def grant(self, side):
        self.assertTrue(self.cache.grant(side, 1.01, 100.01))

    def result(self, side, now=1.02, stamp=100.02):
        names = bridge.ARM_NAMES[side]
        self.cache.accept_result(names, [0.5] * 7, stamp, now, 100 + now - 1)

    def test_idle_outputs_measured_hold(self):
        self.assertEqual(self.cache.output(1.01, 100.01), self.measured)

    def test_pending_sdk_result_keeps_fresh_target_authority(self):
        self.grant('left')
        self.assertEqual(self.cache.output(1.015, 100.015), self.measured)
        self.assertIn('left', self.cache.lease)
        self.assertEqual(self.cache.cutoff['left'], 100.01)
        self.result('left', 1.02, 100.012)
        self.assertEqual(self.cache.output(1.025, 100.025)['Joint1_L'], 0.5)

    def test_stale_result_does_not_reset_fresh_target_cutoff(self):
        self.grant('left')
        self.result('left')
        self.assertTrue(self.cache.grant('left', 1.14, 100.14))
        self.assertEqual(self.cache.output(1.14, 100.14)['Joint1_L'], 0.1)
        self.assertIn('left', self.cache.lease)
        self.assertEqual(self.cache.cutoff['left'], 100.01)
        self.result('left', 1.15, 100.13)
        self.assertEqual(self.cache.output(1.16, 100.16)['Joint1_L'], 0.5)

    def test_separate_sdk_arms_are_merged_auxiliaries_held(self):
        for side in bridge.SIDES:
            self.grant(side)
            self.result(side)
        output = self.cache.output(1.03, 100.03)
        self.assertEqual(len(output), 17)
        self.assertTrue(all(output[name] == 0.5 for name in self.names[:14]))
        self.assertEqual(output['zhi'], 0.1)

    def test_unsolicited_result_has_no_authority(self):
        self.result('left')
        self.assertEqual(self.cache.output(1.03, 100.03), self.measured)

    def test_stale_result_holds_measured_independently(self):
        for side in bridge.SIDES:
            self.grant(side)
        self.result('left')
        self.cache.lease['right'] = (1.1, False)
        self.result('right', 1.1, 100.1)
        output = self.cache.output(1.13, 100.13)
        self.assertEqual(output['Joint1_L'], 0.1)
        self.assertEqual(output['Joint1_R'], 0.5)

    def test_stale_feedback_stops_output(self):
        self.assertIsNone(self.cache.output(1.31, 100.31))
        self.assertEqual(self.cache.targets, {})

    def test_disabled_stale_status_or_hardware_cannot_grant(self):
        self.status['enabled'] = False
        self.assertFalse(self.cache.grant('left', 1.01, 100.01))
        self.status['enabled'] = True
        self.cache.status_time = 0
        self.assertFalse(self.cache.grant('left', 1.01, 100.01))
        self.cache.status_time = 1
        self.cache.hardware_time = 0
        self.assertFalse(self.cache.grant('left', 1.01, 100.01))

    def test_release_rejects_queued_old_sdk_result(self):
        self.grant('left')
        self.result('left')
        self.status['arms']['left']['active'] = False
        self.cache.update_status(self.status, 1.03, 100.03)
        self.status['arms']['left']['active'] = True
        self.cache.update_status(self.status, 1.04, 100.04)
        self.assertTrue(self.cache.grant('left', 1.04, 100.04))
        self.result('left', 1.05, 100.02)
        self.assertEqual(self.cache.output(1.05, 100.05), self.measured)

    def test_reset_ack_requires_new_feedback_and_guard(self):
        self.grant('left')
        self.result('left')
        self.cache.reset(100.03)
        self.assertIsNone(self.cache.output(1.03, 100.03))
        self.assertFalse(self.cache.grant('left', 1.04, 100.04))
        with self.assertRaises(ValueError):
            self.cache.feedback_received(self.names, [0.1] * 17, 100.02, 1.04, 100.04)
        self.assertTrue(self.cache.feedback_received(self.names, [0.2] * 17, 100.04, 1.04, 100.04))
        self.assertFalse(self.cache.grant('left', 1.05, 100.05))
        self.assertTrue(self.cache.grant('left', 1.15, 100.15))
        self.result('left', 1.16, 100.02)
        self.assertEqual(self.cache.output(1.16, 100.16)['Joint1_L'], 0.2)

    def test_homing_is_exclusive_and_uses_lease(self):
        self.status['mode'] = 'homing'
        self.assertFalse(self.cache.grant('right', 1.01, 100.01))
        self.assertTrue(self.cache.grant('right', 1.01, 100.01, homing=True))
        self.result('right')
        self.assertEqual(self.cache.output(1.03, 100.03)['Joint1_R'], 0.5)

    def test_emergency_stop_discards_motion(self):
        self.grant('left')
        self.result('left')
        self.cache.estopped = True
        self.assertEqual(self.cache.output(1.03, 100.03), self.measured)

    def test_invalid_joints_and_stamp(self):
        for names, values in [(['a', 'a'], [0, 0]), (['a'], [float('nan')]), (['a'], []), ([], [])]:
            with self.assertRaises(ValueError):
                bridge.joint_values(names, values)
        self.assertFalse(bridge.valid_stamp(0, 100))
        self.assertFalse(bridge.valid_stamp(99, 100))
        self.assertFalse(bridge.valid_stamp(101, 100))


if __name__ == '__main__':
    unittest.main()

import unittest

from adapters.core.collision_safety import (
    aabb_distance,
    build_protected_pairs,
    calibrated_clearance,
    collision_deficit,
    descendants,
    improves_collision_clearance,
    tree_distance,
)


class CollisionSafetyTests(unittest.TestCase):
    def setUp(self):
        # body(0)-torso(1), right root(2)-r1(3)-r2(4)-r3(5),
        # left root(6)-l1(7)-l2(8)-l3(9)
        self.parents = {
            0: -1,
            1: 0,
            2: 1,
            3: 2,
            4: 3,
            5: 4,
            6: 1,
            7: 6,
            8: 7,
            9: 8,
        }

    def test_aabb_distance(self):
        first = ((0.0, 0.0, 0.0), (1.0, 1.0, 1.0))
        self.assertEqual(aabb_distance(first, ((0.5, 0.5, 0.5), (2, 2, 2))), 0.0)
        self.assertAlmostEqual(
            aabb_distance(first, ((2.0, 3.0, 1.0), (3.0, 4.0, 2.0))),
            5**0.5,
        )

    def test_tree_helpers(self):
        self.assertEqual(descendants(self.parents, 2), {3, 4, 5})
        self.assertEqual(tree_distance(self.parents, 3, 5), 2)
        self.assertEqual(tree_distance(self.parents, 5, 9), 8)

    def test_protected_pairs_ignore_only_nearby_kinematic_neighbors(self):
        right, left, pairs = build_protected_pairs(
            {-1, *self.parents}, self.parents, 2, 6, ignore_graph_distance=2
        )
        self.assertEqual(right, {3, 4, 5})
        self.assertEqual(left, {7, 8, 9})
        self.assertNotIn((3, 4), pairs)
        self.assertNotIn((3, 2), pairs)
        self.assertIn((-1, 5), pairs)
        self.assertIn((3, 7), pairs)
        self.assertIn((1, 5), pairs)

    def test_intra_arm_pairs_can_be_disabled_without_disabling_body_or_inter_arm(self):
        right, left, pairs = build_protected_pairs(
            {-1, *self.parents},
            self.parents,
            2,
            6,
            ignore_graph_distance=2,
            check_intra_arm=False,
        )
        self.assertFalse(
            any(first in right and second in right for first, second in pairs)
        )
        self.assertFalse(
            any(first in left and second in left for first, second in pairs)
        )
        self.assertIn((3, 7), pairs)
        self.assertIn((-1, 5), pairs)

    def test_home_clearance_is_calibrated_but_not_ignored(self):
        self.assertEqual(calibrated_clearance(0.02, None, 0.004), 0.02)
        self.assertAlmostEqual(calibrated_clearance(0.02, 0.007, 0.004), 0.003)
        self.assertEqual(calibrated_clearance(0.02, -0.001, 0.004), 0.0)

    def test_only_monotonic_escape_is_allowed_inside_protected_zone(self):
        current = ("body", "arm", 0.005, 0.02)
        improving = ("body", "arm", 0.006, 0.02)
        worsening = ("body", "arm", 0.004, 0.02)

        self.assertAlmostEqual(collision_deficit(current), 0.015)
        self.assertTrue(improves_collision_clearance(current, improving, 1e-5))
        self.assertFalse(improves_collision_clearance(current, worsening, 1e-5))
        self.assertFalse(improves_collision_clearance(None, improving, 1e-5))


if __name__ == "__main__":
    unittest.main()

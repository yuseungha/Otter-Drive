import unittest

from lidar_cone_planner.synthetic_validation import run_synthetic_validation


class TestSyntheticClosedLoop(unittest.TestCase):
    def test_clean_straight_course_completes_safely(self) -> None:
        result = run_synthetic_validation("straight")

        self.assertTrue(result.completed, result)
        self.assertGreater(result.progress_m, 1.50)
        self.assertGreater(result.valid_plan_fraction, 0.94)
        self.assertLess(result.max_lateral_error_m, 0.025)
        self.assertLess(result.p95_lateral_error_m, 0.015)
        self.assertGreater(result.min_clearance_m, 0.0)
        self.assertEqual(result.collisions, 0)
        self.assertEqual(result.positive_commands_after_fault, 0)

    def test_left_arc_completes_safely(self) -> None:
        result = run_synthetic_validation("left_arc")

        self.assertTrue(result.completed, result)
        self.assertGreater(result.progress_m, 1.00)
        self.assertGreater(result.valid_plan_fraction, 0.90)
        self.assertLess(result.max_lateral_error_m, 0.060)
        self.assertLess(result.p95_lateral_error_m, 0.045)
        self.assertGreater(result.min_clearance_m, 0.0)
        self.assertEqual(result.collisions, 0)
        self.assertGreater(result.max_abs_steering_rad, 0.01)

    def test_scan_dropout_applies_zero_command_on_same_step(self) -> None:
        dropout_step = 70
        result = run_synthetic_validation(
            "straight", steps=110, scan_dropout_step=dropout_step
        )

        self.assertEqual(result.fault_step, dropout_step)
        self.assertEqual(result.positive_commands_after_fault, 0)
        self.assertAlmostEqual(result.post_fault_travel_m, 0.0, places=12)
        self.assertEqual(
            result.status_counts.get("SCAN_DROPOUT"), result.steps - dropout_step
        )
        self.assertEqual(result.collisions, 0)


if __name__ == "__main__":
    unittest.main()

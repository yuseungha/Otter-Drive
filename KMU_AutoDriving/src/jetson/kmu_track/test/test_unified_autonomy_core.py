"""Tests for three-mode planner selection and continuous Pure Pursuit."""

import unittest

from kmu_track.unified_autonomy_core import (
    ConeSwitchConfig,
    ContinuousPurePursuit,
    DriveCountConfig,
    PlannerMode,
    PlannerModeSelector,
    PurePursuitConfig,
    command_to_counts,
    nearest_cone_pair,
    yolo_activity_for_mode,
)


PAIR = ((0.70, 0.30), (0.72, -0.30))


class TestYoloActivity(unittest.TestCase):
    def test_lane_modes_run_yolo(self) -> None:
        self.assertTrue(yolo_activity_for_mode('LANE'))
        self.assertTrue(yolo_activity_for_mode('lane_follow'))

    def test_cone_modes_pause_yolo(self) -> None:
        self.assertFalse(yolo_activity_for_mode('CONE'))
        self.assertFalse(yolo_activity_for_mode('cone_slalom'))

    def test_obstacle_avoidance_keeps_yolo_running(self) -> None:
        self.assertTrue(yolo_activity_for_mode('OBSTACLE_AVOID'))

    def test_unknown_mode_keeps_current_activity(self) -> None:
        self.assertIsNone(yolo_activity_for_mode('unknown'))


class TestConeModeSelector(unittest.TestCase):
    def setUp(self) -> None:
        self.now = 10.0
        self.selector = PlannerModeSelector(
            ConeSwitchConfig(exit_missing_sec=0.8),
            clock=lambda: self.now,
        )

    def test_nearest_left_right_pair_is_found(self) -> None:
        pair = nearest_cone_pair(PAIR, self.selector.config)
        self.assertIsNotNone(pair)
        self.assertAlmostEqual(pair.center_x_m, 0.71)
        self.assertAlmostEqual(pair.width_m, 0.60)

    def test_far_pair_does_not_switch(self) -> None:
        far_pair = ((1.2, 0.3), (1.2, -0.3))
        mode = self.selector.update(far_pair, cone_path_valid=True)
        self.assertEqual(mode, PlannerMode.LANE)

    def test_near_pair_then_missing_lines_returns_to_lane(self) -> None:
        mode = self.selector.update(PAIR, cone_path_valid=True)
        self.assertEqual(mode, PlannerMode.CONE)
        self.now += 0.79
        mode = self.selector.update((), cone_path_valid=False)
        self.assertEqual(mode, PlannerMode.CONE)
        self.now += 0.02
        mode = self.selector.update((), cone_path_valid=False)
        self.assertEqual(mode, PlannerMode.LANE)
        self.assertEqual(self.selector.reason, 'cone_lines_ended')

    def test_one_sided_cones_do_not_switch(self) -> None:
        mode = self.selector.update(
            ((0.5, 0.3), (0.8, 0.4)), cone_path_valid=True)
        self.assertEqual(mode, PlannerMode.LANE)

    def test_obstacle_enters_avoidance_then_clears_to_lane(self) -> None:
        mode = self.selector.update(
            (), cone_path_valid=False, obstacle_detected=True)
        self.assertEqual(mode, PlannerMode.OBSTACLE_AVOID)
        self.now += 0.59
        mode = self.selector.update(
            (), cone_path_valid=False, obstacle_detected=False)
        self.assertEqual(mode, PlannerMode.OBSTACLE_AVOID)
        self.now += 0.02
        mode = self.selector.update(
            (), cone_path_valid=False, obstacle_detected=False)
        self.assertEqual(mode, PlannerMode.LANE)

    def test_near_cone_pair_has_priority_over_obstacle(self) -> None:
        mode = self.selector.update(
            PAIR, cone_path_valid=True, obstacle_detected=True)
        self.assertEqual(mode, PlannerMode.CONE)


class TestContinuousPurePursuit(unittest.TestCase):
    def setUp(self) -> None:
        self.config = PurePursuitConfig(maximum_steering_rate_rad_s=20.0)
        self.controller = ContinuousPurePursuit(self.config)
        self.counts = DriveCountConfig()

    def test_straight_path_has_forward_throttle(self) -> None:
        result = self.controller.command(
            ((0.0, 0.0), (1.0, 0.0)), PlannerMode.LANE, dt_s=0.05)
        throttle, steering = command_to_counts(
            result, PlannerMode.LANE, self.config, self.counts)
        self.assertTrue(result.path_valid)
        self.assertEqual(steering, 0)
        self.assertGreater(throttle, 0)

    def test_left_path_has_left_pure_pursuit_angle(self) -> None:
        result = self.controller.command(
            ((0.0, 0.0), (0.5, 0.25), (1.0, 0.5)),
            PlannerMode.LANE,
            dt_s=0.10,
        )
        _throttle, steering = command_to_counts(
            result, PlannerMode.LANE, self.config, self.counts)
        self.assertGreater(result.steering_angle_rad, 0.0)
        self.assertLess(steering, 0)  # measured linkage uses steering_sign=-1

    def test_missing_path_keeps_forward_command(self) -> None:
        previous = self.controller.command(
            ((0.0, 0.0), (0.5, 0.2)), PlannerMode.CONE, dt_s=0.10)
        fallback = self.controller.command((), PlannerMode.CONE, dt_s=0.10)
        throttle, _steering = command_to_counts(
            fallback, PlannerMode.CONE, self.config, self.counts)
        self.assertFalse(fallback.path_valid)
        self.assertGreater(fallback.speed_mps, 0.0)
        self.assertGreater(throttle, 0)
        self.assertAlmostEqual(
            fallback.steering_angle_rad, previous.steering_angle_rad)

    def test_too_short_path_also_keeps_forward_command(self) -> None:
        result = self.controller.command(
            ((0.0, 0.0), (0.01, 0.0)), PlannerMode.LANE, dt_s=0.05)
        self.assertFalse(result.path_valid)
        self.assertGreater(result.speed_mps, 0.0)


if __name__ == '__main__':
    unittest.main()

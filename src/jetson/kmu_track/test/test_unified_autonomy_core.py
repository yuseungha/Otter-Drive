"""Tests for three-mode planner selection and continuous Pure Pursuit."""

import unittest

from kmu_track.unified_autonomy_core import (
    CompetitionDriveContinuity,
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
TWO_PAIRS = PAIR + ((1.02, 0.31), (1.00, -0.32))


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
            ConeSwitchConfig(minimum_cone_pairs=1, exit_missing_sec=0.8),
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

<<<<<<< HEAD
    def test_timer_does_not_count_one_scan_as_multiple_frames(self) -> None:
        selector = PlannerModeSelector(ConeSwitchConfig(
            enter_confirm_frames=2))
        mode = selector.update(
            PAIR, cone_path_valid=True, cone_observation_id=1)
        self.assertEqual(mode, PlannerMode.LANE)
        mode = selector.update(
            PAIR, cone_path_valid=True, cone_observation_id=1)
        self.assertEqual(mode, PlannerMode.LANE)
        mode = selector.update(
            PAIR, cone_path_valid=True, cone_observation_id=2)
        self.assertEqual(mode, PlannerMode.CONE)
=======
    def test_two_pairs_are_required_when_configured(self) -> None:
        selector = PlannerModeSelector(
            ConeSwitchConfig(minimum_cone_pairs=2),
            clock=lambda: self.now,
        )
        mode = selector.update(
            PAIR, cone_path_valid=True, obstacle_detected=True)
        self.assertEqual(mode, PlannerMode.OBSTACLE_AVOID)
        self.assertEqual(selector.last_pair_count, 1)

        mode = selector.update(
            TWO_PAIRS, cone_path_valid=True, obstacle_detected=True)
        self.assertEqual(mode, PlannerMode.CONE)
        self.assertEqual(selector.last_pair_count, 2)
>>>>>>> 71f6446ad18055c11c45fe04dddba4d40ecc79dc


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

    def test_steering_gain_increases_all_pure_pursuit_modes(self) -> None:
        result = self.controller.command(
            ((0.0, 0.0), (0.5, 0.20), (1.0, 0.35)),
            PlannerMode.CONE,
            dt_s=0.10,
        )
        base_counts = DriveCountConfig(steering_gain=1.0)
        raised_counts = DriveCountConfig(steering_gain=1.15)

        for mode in (PlannerMode.CONE, PlannerMode.OBSTACLE_AVOID):
            _throttle, base_steering = command_to_counts(
                result, mode, self.config, base_counts)
            _throttle, raised_steering = command_to_counts(
                result, mode, self.config, raised_counts)
            self.assertGreater(abs(raised_steering), abs(base_steering))
            self.assertLessEqual(
                abs(raised_steering), raised_counts.maximum_steering_counts)

    def test_right_steering_gain_strengthens_negative_output(self) -> None:
        result = self.controller.command(
            ((0.0, 0.0), (0.5, 0.20), (1.0, 0.35)),
            PlannerMode.CONE,
            dt_s=0.10,
        )
        symmetric = DriveCountConfig(steering_gain=1.15)
        right_raised = DriveCountConfig(
            steering_gain=1.15,
            steering_gain_right=1.30,
        )
        _throttle, symmetric_steering = command_to_counts(
            result, PlannerMode.CONE, self.config, symmetric)
        _throttle, raised_steering = command_to_counts(
            result, PlannerMode.CONE, self.config, right_raised)

        self.assertLess(symmetric_steering, 0)
        self.assertLess(raised_steering, symmetric_steering)
        self.assertGreater(abs(raised_steering), abs(symmetric_steering))
        self.assertLessEqual(
            abs(raised_steering), right_raised.maximum_steering_counts)

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

    def test_cone_curve_uses_raised_cone_minimum_speed(self) -> None:
        config = PurePursuitConfig(
            cone_minimum_speed_mps=0.09,
            curvature_slowdown_gain=10.0,
            maximum_steering_rate_rad_s=20.0,
        )
        controller = ContinuousPurePursuit(config)
        result = controller.command(
            ((0.0, 0.0), (0.12, 0.25), (0.25, 0.45)),
            PlannerMode.CONE,
            dt_s=0.10,
        )

        self.assertTrue(result.path_valid)
        self.assertAlmostEqual(result.speed_mps, 0.09)


class TestCompetitionDriveContinuity(unittest.TestCase):
    def setUp(self) -> None:
        self.policy = CompetitionDriveContinuity(320, 550, 650)

    def test_startup_remains_neutral_until_valid_departure(self) -> None:
        self.assertEqual(
            self.policy.apply(0, 0, source_valid=False),
            (0, 0, 'awaiting_first_forward'),
        )
        self.assertFalse(self.policy.started)

    def test_start_can_wait_for_serial_gate(self) -> None:
        self.assertEqual(
            self.policy.apply(
                500, 40, source_valid=True, start_allowed=False),
            (0, 0, 'awaiting_first_forward'),
        )

    def test_valid_departure_is_latched(self) -> None:
        self.assertEqual(
            self.policy.apply(500, -120, source_valid=True),
            (500, -120, 'fresh_forward'),
        )
        self.assertTrue(self.policy.started)

    def test_ire_timeout_holds_last_forward_command(self) -> None:
        self.policy.apply(500, -120, source_valid=True)
        self.assertEqual(
            self.policy.apply(0, 0, source_valid=False),
            (500, -120, 'hold_last_forward'),
        )

    def test_zero_candidate_after_start_cannot_stop(self) -> None:
        self.policy.apply(500, 100, source_valid=True)
        throttle, steering, _reason = self.policy.apply(
            0, -650, source_valid=True)
        self.assertEqual((throttle, steering), (500, 100))

    def test_transitions_update_without_zero_gap(self) -> None:
        commands = [
            self.policy.apply(500, 10, source_valid=True),
            self.policy.apply(0, 0, source_valid=False),
            self.policy.apply(450, -30, source_valid=True),
            self.policy.apply(0, 0, source_valid=False),
            self.policy.apply(520, 20, source_valid=True),
        ]
        self.assertTrue(all(
            throttle > 0 for throttle, _steering, _reason in commands))

    def test_output_is_saturated(self) -> None:
        self.assertEqual(
            self.policy.apply(900, 900, source_valid=True)[:2],
            (550, 650),
        )

    def test_minimum_throttle_is_enforced(self) -> None:
        self.assertEqual(
            self.policy.apply(1, 0, source_valid=True)[0],
            320,
        )


if __name__ == '__main__':
    unittest.main()

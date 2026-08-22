import unittest

import numpy as np

from lidar_cone_planner.pure_pursuit_core import (
    ControllerConfig,
    compute_pure_pursuit,
    stop_result,
)


def unrestricted_config(**updates) -> ControllerConfig:
    values = dict(
        wheelbase_m=0.20,
        max_steering_angle_rad=0.70,
        lookahead_min_m=0.30,
        lookahead_max_m=0.60,
        lookahead_time_s=0.75,
        min_remaining_path_m=0.10,
        max_speed_mps=0.50,
        max_lateral_accel_mps2=0.20,
        max_accel_mps2=100.0,
        max_decel_mps2=100.0,
        stopping_buffer_m=0.02,
        max_steering_rate_rad_s=100.0,
        min_plan_confidence=0.40,
    )
    values.update(updates)
    return ControllerConfig(**values)


def command(path, config=None, **updates):
    values = dict(
        current_speed_mps=0.0,
        plan_confidence=1.0,
        previous_speed_mps=0.0,
        previous_steering_angle_rad=0.0,
        dt_s=0.1,
        config=config or unrestricted_config(),
    )
    values.update(updates)
    return compute_pure_pursuit(path, **values)


class TestControllerConfig(unittest.TestCase):
    def test_invalid_geometry_and_limits_are_rejected(self) -> None:
        with self.assertRaises(ValueError):
            ControllerConfig(wheelbase_m=0.0)
        with self.assertRaises(ValueError):
            ControllerConfig(lookahead_min_m=0.8, lookahead_max_m=0.4)
        with self.assertRaises(ValueError):
            ControllerConfig(virtual_speed_factor=1.2)
        with self.assertRaises(ValueError):
            ControllerConfig(max_steering_angle_rad=1.7)


class TestPurePursuit(unittest.TestCase):
    def test_straight_path_has_zero_steering(self) -> None:
        result = command([[0.0, 0.0], [1.2, 0.0]])

        self.assertTrue(result.valid, result.reason)
        self.assertAlmostEqual(result.steering_angle_rad, 0.0, places=9)
        self.assertAlmostEqual(result.target_x_m, 0.30, places=6)
        self.assertGreater(result.speed_mps, 0.0)

    def test_left_and_right_paths_have_symmetric_signs(self) -> None:
        left = command([[0.0, 0.0], [0.6, 0.24], [1.2, 0.48]])
        right = command([[0.0, 0.0], [0.6, -0.24], [1.2, -0.48]])

        self.assertTrue(left.valid, left.reason)
        self.assertTrue(right.valid, right.reason)
        self.assertGreater(left.steering_angle_rad, 0.0)
        self.assertLess(right.steering_angle_rad, 0.0)
        self.assertAlmostEqual(
            left.steering_angle_rad, -right.steering_angle_rad, places=8
        )

    def test_target_is_interpolated_by_arc_length(self) -> None:
        result = command([[0.0, 0.0], [0.10, 0.0], [1.10, 0.0]])

        self.assertTrue(result.valid, result.reason)
        self.assertAlmostEqual(result.target_x_m, 0.30, places=6)

    def test_lookahead_increases_with_speed_and_is_bounded(self) -> None:
        path = [[0.0, 0.0], [2.0, 0.0]]
        slow = command(path, current_speed_mps=0.0)
        fast = command(path, current_speed_mps=1.0)

        self.assertLess(slow.lookahead_m, fast.lookahead_m)
        self.assertAlmostEqual(fast.lookahead_m, 0.60, places=6)

    def test_curvature_and_short_remaining_path_reduce_speed(self) -> None:
        config = unrestricted_config(
            max_steering_angle_rad=1.20,
            max_lateral_accel_mps2=0.04,
            max_decel_mps2=0.20,
            stopping_buffer_m=0.05,
        )
        straight = command([[0.0, 0.0], [2.0, 0.0]], config=config)
        curved = command(
            [[0.0, 0.0], [0.30, 0.25], [0.60, 0.50]], config=config
        )
        short = command([[0.0, 0.0], [0.20, 0.0]], config=config)

        self.assertTrue(straight.valid, straight.reason)
        self.assertTrue(curved.valid, curved.reason)
        self.assertTrue(short.valid, short.reason)
        self.assertLess(curved.speed_mps, straight.speed_mps)
        self.assertLess(short.speed_mps, straight.speed_mps)

    def test_steering_saturation_is_fail_closed(self) -> None:
        config = unrestricted_config(max_steering_angle_rad=0.10)
        result = command(
            [[0.0, 0.0], [0.20, 0.40], [0.40, 0.80]], config=config
        )

        self.assertFalse(result.valid)
        self.assertEqual(result.reason, "STEERING_LIMIT")
        self.assertEqual(result.speed_mps, 0.0)
        self.assertEqual(result.steering_angle_rad, 0.0)

    def test_normal_commands_obey_acceleration_and_steering_rate(self) -> None:
        config = unrestricted_config(
            max_accel_mps2=0.20,
            max_steering_rate_rad_s=0.10,
        )
        result = command(
            [[0.0, 0.0], [0.60, 0.24], [1.20, 0.48]],
            config=config,
            dt_s=0.10,
        )

        self.assertTrue(result.valid, result.reason)
        self.assertLessEqual(result.speed_mps, 0.0200001)
        self.assertLessEqual(abs(result.steering_angle_rad), 0.0100001)

    def test_virtual_path_has_a_lower_speed_cap(self) -> None:
        path = [[0.0, 0.0], [2.0, 0.0]]
        normal = command(path)
        virtual = command(path, virtual_path=True)

        self.assertTrue(normal.valid)
        self.assertTrue(virtual.valid)
        self.assertLess(virtual.speed_mps, normal.speed_mps)

    def test_invalid_paths_and_inputs_return_finite_zero_command(self) -> None:
        cases = (
            command([[0.0, 0.0], [np.nan, 0.0]]),
            command([[-1.0, 0.0], [-0.4, 0.0]]),
            command([[0.0, 0.0], [1.0, 0.0]], plan_confidence=0.1),
            command([[0.0, 0.0], [1.0, 0.0]], dt_s=0.0),
        )
        for result in cases:
            self.assertFalse(result.valid)
            self.assertEqual(result.speed_mps, 0.0)
            self.assertEqual(result.steering_angle_rad, 0.0)
            self.assertTrue(np.isfinite(result.speed_mps))
            self.assertTrue(np.isfinite(result.steering_angle_rad))

    def test_explicit_invalid_stop_bypasses_previous_command(self) -> None:
        stopped = stop_result("STALE_PATH")
        self.assertFalse(stopped.valid)
        self.assertEqual(stopped.speed_mps, 0.0)
        self.assertEqual(stopped.steering_angle_rad, 0.0)

    def test_out_of_range_previous_command_fails_closed(self) -> None:
        path = [[0.0, 0.0], [1.0, 0.0]]
        config = unrestricted_config(max_speed_mps=0.20, max_steering_angle_rad=0.30)
        bad_speed = command(
            path, config=config, previous_speed_mps=0.50
        )
        bad_steering = command(
            path, config=config, previous_steering_angle_rad=0.50
        )

        self.assertFalse(bad_speed.valid)
        self.assertEqual(bad_speed.reason, "PREVIOUS_SPEED_OUT_OF_RANGE")
        self.assertFalse(bad_steering.valid)
        self.assertEqual(bad_steering.reason, "PREVIOUS_STEERING_OUT_OF_RANGE")


if __name__ == "__main__":
    unittest.main()

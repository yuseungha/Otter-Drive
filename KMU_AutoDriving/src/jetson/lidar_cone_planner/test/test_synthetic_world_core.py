from math import cos, pi, sin, tan
import unittest

import numpy as np

from lidar_cone_planner.synthetic_world_core import (
    CapsuleObstacle,
    CircleObstacle,
    Pose2D,
    SegmentObstacle,
    SyntheticWorld,
    VehicleState,
    WorldConfig,
    _ray_circle_distances,
    bicycle_yaw_rate,
    compose_pose,
    inverse_transform_points,
    lateral_error_m,
    make_arc_course,
    make_s_course,
    make_straight_course,
    obstacle_clearance_m,
    path_minimum_clearance_m,
    project_to_polyline,
    step_bicycle,
    transform_points,
)


class TestWorldConfig(unittest.TestCase):
    def test_default_contract_matches_planner_range(self) -> None:
        config = WorldConfig()
        self.assertEqual(config.range_min_m, 0.15)
        self.assertEqual(config.range_max_m, 5.0)
        self.assertGreater(config.angle_increment_rad, 0.0)

    def test_invalid_values_are_rejected(self) -> None:
        invalid = (
            {"angle_min_rad": 1.0, "angle_max_rad": 1.0},
            {"angle_min_rad": -4.0, "angle_max_rad": 4.0},
            {"beam_count": 1},
            {"beam_count": 2.5},
            {"range_min_m": 0.0},
            {"range_min_m": 2.0, "range_max_m": 1.0},
            {"gaussian_noise_std_m": -0.1},
            {"range_quantization_m": float("nan")},
            {"beam_dropout_probability": 1.1},
            {"same_range_replica_probability": 0.1},
            {"same_range_replica_span_beams": -1},
            {"glint_probability": -0.1},
            {"glint_range_min_m": 3.0, "glint_range_max_m": 2.0},
        )
        for values in invalid:
            with self.subTest(values=values), self.assertRaises(ValueError):
                WorldConfig(**values)

    def test_obstacle_validation(self) -> None:
        with self.assertRaises(ValueError):
            CircleObstacle((0.0, 0.0), 0.0)
        with self.assertRaises(ValueError):
            SegmentObstacle((1.0, 1.0), (1.0, 1.0))
        with self.assertRaises(ValueError):
            CapsuleObstacle((0.0, 0.0), (1.0, 0.0), -0.1)


class TestTransforms(unittest.TestCase):
    def test_compose_and_point_round_trip(self) -> None:
        vehicle = Pose2D(10.0, 5.0, pi / 2.0)
        sensor = Pose2D(0.2, -0.1, 0.1)
        world_sensor = compose_pose(vehicle, sensor)
        self.assertAlmostEqual(world_sensor.x, 10.1)
        self.assertAlmostEqual(world_sensor.y, 5.2)

        local = np.array(((1.0, 0.0), (0.0, 2.0)))
        world = transform_points(local, world_sensor)
        recovered = inverse_transform_points(world, world_sensor)
        np.testing.assert_allclose(recovered, local, atol=1.0e-12)


class TestRayCasting(unittest.TestCase):
    @staticmethod
    def narrow_config(**overrides) -> WorldConfig:
        values = {
            "angle_min_rad": -pi / 2.0,
            "angle_max_rad": pi / 2.0,
            "beam_count": 3,
            "range_min_m": 0.05,
            "range_max_m": 5.0,
            "glint_range_min_m": 0.05,
            "glint_range_max_m": 2.0,
        }
        values.update(overrides)
        return WorldConfig(**values)

    def test_circle_segment_and_capsule_exact_hits(self) -> None:
        pose = Pose2D()
        circle = SyntheticWorld(
            (CircleObstacle((2.0, 0.0), 0.1),), self.narrow_config()
        ).scan(pose)
        segment = SyntheticWorld(
            (SegmentObstacle((2.0, -1.0), (2.0, 1.0)),),
            self.narrow_config(),
        ).scan(pose)
        capsule = SyntheticWorld(
            (CapsuleObstacle((2.0, -1.0), (2.0, 1.0), 0.1),),
            self.narrow_config(),
        ).scan(pose)

        self.assertAlmostEqual(circle.ranges[1], 1.9, places=10)
        self.assertAlmostEqual(segment.ranges[1], 2.0, places=10)
        self.assertAlmostEqual(capsule.ranges[1], 1.9, places=10)
        self.assertTrue(np.isinf(circle.ranges[0]))
        self.assertTrue(np.isinf(circle.ranges[2]))

    def test_collinear_segment_returns_its_nearest_forward_point(self) -> None:
        world = SyntheticWorld(
            (SegmentObstacle((1.0, 0.0), (2.0, 0.0)),),
            self.narrow_config(),
        )
        self.assertAlmostEqual(world.scan(Pose2D()).ranges[1], 1.0)

    def test_nearest_obstacle_wins_and_range_limit_is_infinite(self) -> None:
        world = SyntheticWorld(
            (
                CircleObstacle((2.0, 0.0), 0.1),
                CircleObstacle((1.0, 0.0), 0.1),
                CircleObstacle((8.0, 0.0), 0.1),
            ),
            self.narrow_config(),
        )
        self.assertAlmostEqual(world.scan(Pose2D()).ranges[1], 0.9)

        far_world = SyntheticWorld(
            (CircleObstacle((8.0, 0.0), 0.1),), self.narrow_config()
        )
        self.assertTrue(np.isinf(far_world.scan(Pose2D()).ranges[1]))

    def test_batched_circles_match_scalar_reference(self) -> None:
        generator = np.random.default_rng(20260813)
        config = WorldConfig(
            angle_min_rad=-2.4,
            angle_max_rad=2.2,
            beam_count=1081,
            range_min_m=0.05,
            range_max_m=8.0,
            glint_range_min_m=0.05,
            glint_range_max_m=2.0,
        )
        pose = Pose2D(0.17, -0.11, 0.31)
        circles = tuple(
            CircleObstacle(
                tuple(generator.uniform((-1.0, -2.0), (7.0, 2.0))),
                float(generator.uniform(0.015, 0.25)),
            )
            for _ in range(64)
        )
        # Exercise the scalar helper's special inside-circle convention too.
        circles += (CircleObstacle((pose.x, pose.y), 0.2),)
        world = SyntheticWorld(circles, config)

        angles = (
            config.angle_min_rad
            + np.arange(config.beam_count) * config.angle_increment_rad
            + pose.yaw
        )
        directions = np.column_stack((np.cos(angles), np.sin(angles)))
        origin = np.array((pose.x, pose.y), dtype=float)
        expected = np.full(config.beam_count, np.inf, dtype=float)
        for circle in circles:
            expected = np.minimum(
                expected,
                _ray_circle_distances(
                    origin, directions, circle.center, circle.radius_m
                ),
            )
        outside = (
            ~np.isfinite(expected)
            | (expected < config.range_min_m)
            | (expected > config.range_max_m)
        )
        expected[outside] = np.inf

        first = world._ideal_ranges(pose)
        repeated = world._ideal_ranges(pose)
        np.testing.assert_allclose(first, expected, rtol=1.0e-12, atol=1.0e-12)
        np.testing.assert_array_equal(first, repeated)

    def test_batched_circles_preserve_mixed_obstacle_minimum(self) -> None:
        obstacles = (
            CircleObstacle((2.0, 0.0), 0.1),
            SegmentObstacle((1.5, -0.6), (1.5, 0.6)),
            CapsuleObstacle((1.0, -0.5), (1.0, 0.5), 0.05),
            CircleObstacle((0.8, 0.0), 0.1),
        )
        scan = SyntheticWorld(obstacles, self.narrow_config()).scan(Pose2D())
        self.assertAlmostEqual(scan.ranges[1], 0.7, places=12)

    def test_vehicle_and_sensor_pose_are_applied(self) -> None:
        config = self.narrow_config(sensor_pose_in_vehicle=Pose2D(0.2, 0.0, 0.0))
        # Vehicle points +y in world. Sensor is at (10, 5.2); obstacle centre
        # is exactly 2 m in front of it in the sensor frame.
        world = SyntheticWorld((CircleObstacle((10.0, 7.2), 0.1),), config)
        scan = world.scan(Pose2D(10.0, 5.0, pi / 2.0))
        self.assertAlmostEqual(scan.sensor_world_pose.x, 10.0)
        self.assertAlmostEqual(scan.sensor_world_pose.y, 5.2)
        self.assertAlmostEqual(scan.ranges[1], 1.9, places=10)
        self.assertAlmostEqual(scan.angle_max_rad, pi / 2.0)


class TestSensorFaults(unittest.TestCase):
    def test_faults_are_seeded_by_scan_index(self) -> None:
        config = WorldConfig(
            angle_min_rad=-0.2,
            angle_max_rad=0.2,
            beam_count=31,
            range_min_m=0.05,
            range_max_m=5.0,
            glint_range_min_m=0.05,
            glint_range_max_m=2.0,
            gaussian_noise_std_m=0.02,
            glint_probability=1.0,
            random_seed=123,
        )
        first = SyntheticWorld((), config).scan(Pose2D(), scan_index=4).ranges
        repeated = SyntheticWorld((), config).scan(Pose2D(), scan_index=4).ranges
        next_frame = SyntheticWorld((), config).scan(Pose2D(), scan_index=5).ranges
        np.testing.assert_array_equal(first, repeated)
        self.assertFalse(np.array_equal(first, next_frame))
        self.assertTrue(np.all(np.isfinite(first)))

    def test_quantization_dropout_and_glint_bounds(self) -> None:
        quantized = SyntheticWorld(
            (CircleObstacle((2.0, 0.0), 0.2),),
            WorldConfig(
                angle_min_rad=-0.1,
                angle_max_rad=0.1,
                beam_count=5,
                range_min_m=0.05,
                range_max_m=5.0,
                glint_range_min_m=0.05,
                glint_range_max_m=2.0,
                range_quantization_m=0.05,
            ),
        ).scan(Pose2D())
        finite = quantized.ranges[np.isfinite(quantized.ranges)]
        np.testing.assert_allclose(finite / 0.05, np.round(finite / 0.05))

        dropped = SyntheticWorld(
            (CircleObstacle((2.0, 0.0), 0.2),),
            WorldConfig(
                angle_min_rad=-0.1,
                angle_max_rad=0.1,
                beam_count=5,
                range_min_m=0.05,
                range_max_m=5.0,
                glint_range_min_m=0.05,
                glint_range_max_m=2.0,
                beam_dropout_probability=1.0,
            ),
        ).scan(Pose2D())
        self.assertTrue(np.all(np.isinf(dropped.ranges)))

        glints = SyntheticWorld(
            (),
            WorldConfig(
                angle_min_rad=-0.1,
                angle_max_rad=0.1,
                beam_count=5,
                range_min_m=0.05,
                range_max_m=5.0,
                glint_range_min_m=0.3,
                glint_range_max_m=0.4,
                glint_probability=1.0,
            ),
        ).scan(Pose2D())
        self.assertTrue(np.all((glints.ranges >= 0.3) & (glints.ranges <= 0.4)))

    def test_same_range_replica_is_exact(self) -> None:
        config = WorldConfig(
            angle_min_rad=-0.04,
            angle_max_rad=0.04,
            beam_count=9,
            range_min_m=0.05,
            range_max_m=5.0,
            glint_range_min_m=0.05,
            glint_range_max_m=2.0,
            same_range_replica_probability=1.0,
            same_range_replica_span_beams=1,
        )
        scan = SyntheticWorld(
            (CircleObstacle((2.0, 0.0), 0.005),), config
        ).scan(Pose2D())
        centre = config.beam_count // 2
        self.assertTrue(np.isfinite(scan.ranges[centre]))
        self.assertEqual(scan.ranges[centre - 1], scan.ranges[centre])
        self.assertEqual(scan.ranges[centre + 1], scan.ranges[centre])


class TestBicycleModel(unittest.TestCase):
    def test_straight_step(self) -> None:
        state = step_bicycle(VehicleState(), 1.2, 0.0, 0.5, 0.33)
        self.assertAlmostEqual(state.x, 0.6)
        self.assertAlmostEqual(state.y, 0.0)
        self.assertAlmostEqual(state.yaw, 0.0)
        self.assertAlmostEqual(state.speed_mps, 1.2)

    def test_turn_step_matches_constant_radius_solution(self) -> None:
        speed = 1.0
        steering = 0.2
        wheelbase = 0.33
        dt = 0.4
        yaw_rate = speed * tan(steering) / wheelbase
        self.assertAlmostEqual(
            bicycle_yaw_rate(speed, steering, wheelbase), yaw_rate
        )
        state = step_bicycle(
            VehicleState(), speed, steering, dt, wheelbase
        )
        radius = speed / yaw_rate
        self.assertAlmostEqual(state.x, radius * sin(yaw_rate * dt))
        self.assertAlmostEqual(state.y, radius * (1.0 - cos(yaw_rate * dt)))
        self.assertAlmostEqual(state.yaw, yaw_rate * dt)

    def test_invalid_dynamics_are_rejected(self) -> None:
        with self.assertRaises(ValueError):
            step_bicycle(VehicleState(), 1.0, 0.0, -0.1, 0.33)
        with self.assertRaises(ValueError):
            bicycle_yaw_rate(1.0, pi / 2.0, 0.33)
        with self.assertRaises(ValueError):
            bicycle_yaw_rate(1.0, 0.0, 0.0)


class TestCoursesAndMetrics(unittest.TestCase):
    def test_course_generators_preserve_width(self) -> None:
        courses = (
            make_straight_course(),
            make_arc_course(turn_left=True),
            make_arc_course(turn_left=False),
            make_s_course(),
        )
        for course in courses:
            with self.subTest(course=course.name):
                self.assertEqual(len(course.left_cones), len(course.right_cones))
                widths = np.linalg.norm(
                    course.left_cones - course.right_cones, axis=1
                )
                np.testing.assert_allclose(widths, course.track_width_m, atol=1.0e-12)
                self.assertEqual(len(course.obstacles()), 2 * len(course.left_cones))
        self.assertGreater(np.max(courses[1].centerline[:, 1]), 0.0)
        self.assertLess(np.min(courses[2].centerline[:, 1]), 0.0)
        self.assertGreater(np.max(courses[3].centerline[:, 1]), 0.0)
        self.assertLess(np.min(courses[3].centerline[:, 1]), 0.0)

    def test_lateral_projection_is_signed_and_has_along_track(self) -> None:
        centerline = np.array(((0.0, 0.0), (2.0, 0.0), (3.0, 1.0)))
        left = project_to_polyline((1.0, 0.2), centerline)
        right = project_to_polyline((1.0, -0.3), centerline)
        self.assertAlmostEqual(left.signed_lateral_m, 0.2)
        self.assertAlmostEqual(left.absolute_lateral_m, 0.2)
        self.assertAlmostEqual(left.along_track_m, 1.0)
        self.assertAlmostEqual(right.signed_lateral_m, -0.3)
        self.assertAlmostEqual(lateral_error_m((1.0, -0.3), centerline), -0.3)

    def test_clearance_for_each_obstacle_type_and_path(self) -> None:
        circle = CircleObstacle((1.0, 0.0), 0.1)
        segment = SegmentObstacle((0.0, 1.0), (2.0, 1.0))
        capsule = CapsuleObstacle((0.0, -1.0), (2.0, -1.0), 0.2)
        self.assertAlmostEqual(obstacle_clearance_m((0.0, 0.0), (circle,)), 0.9)
        self.assertAlmostEqual(obstacle_clearance_m((1.0, 0.0), (segment,)), 1.0)
        self.assertAlmostEqual(obstacle_clearance_m((1.0, 0.0), (capsule,)), 0.8)
        self.assertAlmostEqual(
            obstacle_clearance_m((0.0, 0.0), (circle,), 0.25), 0.65
        )
        path = np.array(((0.0, 0.0), (0.5, 0.0), (0.8, 0.0)))
        self.assertAlmostEqual(path_minimum_clearance_m(path, (circle,)), 0.1)
        self.assertTrue(np.isinf(obstacle_clearance_m((0.0, 0.0), ())))

    def test_generated_course_can_be_scanned(self) -> None:
        course = make_straight_course(length_m=2.0)
        world = SyntheticWorld(
            course.obstacles(),
            WorldConfig(
                angle_min_rad=-pi / 2.0,
                angle_max_rad=pi / 2.0,
                beam_count=721,
            ),
        )
        scan = world.scan(Pose2D())
        self.assertGreater(np.count_nonzero(np.isfinite(scan.ranges)), 10)


if __name__ == "__main__":
    unittest.main()

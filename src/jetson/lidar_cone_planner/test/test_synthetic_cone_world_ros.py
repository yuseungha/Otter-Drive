"""ROS-facing checks for the deterministic synthetic cone world."""

import time
import unittest


try:
    import rclpy
    from geometry_msgs.msg import Vector3Stamped
    from rclpy.duration import Duration
    from lidar_cone_planner.synthetic_cone_world import SyntheticConeWorld

    ROS_AVAILABLE = True
except ImportError:
    rclpy = None
    Vector3Stamped = None
    SyntheticConeWorld = None
    ROS_AVAILABLE = False


@unittest.skipUnless(ROS_AVAILABLE, "ROS 2 Python packages are unavailable")
class SyntheticConeWorldRosTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        rclpy.init()

    @classmethod
    def tearDownClass(cls) -> None:
        rclpy.shutdown()

    def setUp(self) -> None:
        self.node = SyntheticConeWorld()

    def tearDown(self) -> None:
        self.node.destroy_node()

    def _valid_command(self) -> Vector3Stamped:
        message = Vector3Stamped()
        message.header.stamp = self.node.get_clock().now().to_msg()
        message.header.frame_id = self.node.base_frame
        message.vector.x = 0.10
        message.vector.y = 0.05
        return message

    def test_defaults_produce_a_finite_scan_and_required_frames(self) -> None:
        frame = self.node.world.scan(self.node.state.pose, 0)
        message = self.node._make_scan(frame, self.node._next_stamp())

        self.assertEqual(message.header.frame_id, "sim_laser")
        self.assertEqual(self.node.odom_frame, "sim_odom")
        self.assertEqual(self.node.base_frame, "sim_base_link")
        self.assertEqual(len(message.ranges), 1081)
        finite_count = sum(
            value != float("inf") for value in message.ranges
        )
        self.assertGreater(finite_count, 0)
        self.assertGreater(message.time_increment, 0.0)

    def test_command_contract_rejects_duplicate_stamp(self) -> None:
        command = self._valid_command()
        self.node._command_callback(command)

        self.assertTrue(self.node._command_valid)
        self.assertAlmostEqual(self.node._target_speed_mps, 0.10)
        self.assertAlmostEqual(self.node._target_steering_rad, 0.05)

        self.node._command_callback(command)
        self.assertFalse(self.node._command_valid)
        self.assertEqual(self.node._command_reason, "OUT_OF_ORDER_COMMAND")
        self.assertEqual(self.node._target_speed_mps, 0.0)
        self.assertEqual(self.node._target_steering_rad, 0.0)

    def test_bad_frame_and_reserved_component_fail_closed(self) -> None:
        command = self._valid_command()
        command.header.frame_id = "wrong_frame"
        self.node._command_callback(command)
        self.assertEqual(self.node._command_reason, "COMMAND_FRAME_MISMATCH")

        # A rejected message must not poison the accepted-command watermark.
        command.header.frame_id = self.node.base_frame
        self.node._command_callback(command)
        self.assertTrue(self.node._command_valid)

        command = self._valid_command()
        command.vector.z = 1.0
        self.node._command_callback(command)
        self.assertEqual(self.node._command_reason, "COMMAND_RESERVED_NONZERO")
        self.assertEqual(self.node._target_speed_mps, 0.0)

    def test_stamp_and_bounds_fail_closed(self) -> None:
        stale = self._valid_command()
        stale.header.stamp = (
            self.node.get_clock().now() - Duration(seconds=1.0)
        ).to_msg()
        self.node._command_callback(stale)
        self.assertEqual(self.node._command_reason, "STALE_COMMAND")

        future = self._valid_command()
        future.header.stamp = (
            self.node.get_clock().now() + Duration(seconds=1.0)
        ).to_msg()
        self.node._command_callback(future)
        self.assertEqual(self.node._command_reason, "FUTURE_COMMAND")

        speed = self._valid_command()
        speed.vector.x = self.node.max_command_speed_mps + 0.01
        self.node._command_callback(speed)
        self.assertEqual(self.node._command_reason, "COMMAND_SPEED_BOUNDS")

        steering = self._valid_command()
        steering.vector.y = self.node.max_command_steering_rad + 0.01
        self.node._command_callback(steering)
        self.assertEqual(
            self.node._command_reason, "COMMAND_STEERING_BOUNDS"
        )

    def test_receipt_timeout_zeroes_targets(self) -> None:
        self.node._command_callback(self._valid_command())
        self.node._last_command_receipt_monotonic = (
            time.monotonic() - self.node.command_receipt_timeout_s - 0.1
        )

        self.node._apply_command_timeout(time.monotonic())

        self.assertFalse(self.node._command_valid)
        self.assertEqual(self.node._command_reason, "COMMAND_RECEIPT_TIMEOUT")
        self.assertEqual(self.node._target_speed_mps, 0.0)
        self.assertEqual(self.node._target_steering_rad, 0.0)

    def test_published_stamp_generator_is_strictly_increasing(self) -> None:
        first = self.node._next_stamp()
        second = self.node._next_stamp()
        first_ns = first.sec * 1_000_000_000 + first.nanosec
        second_ns = second.sec * 1_000_000_000 + second.nanosec
        self.assertGreater(second_ns, first_ns)


if __name__ == "__main__":
    unittest.main()

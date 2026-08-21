from __future__ import annotations

import unittest

try:
    import rclpy
    from rclpy.duration import Duration
    from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
    from geometry_msgs.msg import PoseStamped
    from nav_msgs.msg import Path

    from lidar_cone_planner.cone_pure_pursuit import ConePurePursuit

    ROS_AVAILABLE = True
except ImportError:
    ROS_AVAILABLE = False


@unittest.skipUnless(ROS_AVAILABLE, "ROS 2 Python packages are not installed")
class TestControllerRosSafety(unittest.TestCase):
    def setUp(self) -> None:
        rclpy.init(
            args=[
                "--ros-args",
                "-p",
                "geometry_confirmed:=true",
                "-p",
                "enabled_on_startup:=true",
                "-p",
                "valid_frames_before_motion:=1",
                "-p",
                "minimum_stop_hold_s:=0.0",
                "-p",
                "allow_compat_command:=true",
                "-p",
                "require_odometry:=false",
            ]
        )
        self.node = ConePurePursuit()

    def tearDown(self) -> None:
        self.node.destroy_node()
        rclpy.shutdown()

    def make_path(self, *, empty: bool = False, age_s: float = 0.0) -> Path:
        message = Path()
        stamp = self.node.get_clock().now() - Duration(seconds=age_s)
        message.header.stamp = stamp.to_msg()
        message.header.frame_id = "base_link"
        if not empty:
            for x in (0.0, 0.30, 0.60, 0.90, 1.20):
                pose = PoseStamped()
                pose.header = message.header
                pose.pose.position.x = x
                pose.pose.orientation.w = 1.0
                message.poses.append(pose)
        return message

    @staticmethod
    def make_status(path: Path, status_text: str = "OK") -> DiagnosticArray:
        message = DiagnosticArray()
        # Copy fields instead of aliasing the same mutable Header object.  The
        # stamp-mismatch test deliberately edits the status header only.
        message.header.frame_id = path.header.frame_id
        message.header.stamp.sec = path.header.stamp.sec
        message.header.stamp.nanosec = path.header.stamp.nanosec
        status = DiagnosticStatus()
        status.level = (
            DiagnosticStatus.OK if status_text in {"OK", "OK_VIRTUAL"}
            else DiagnosticStatus.WARN
        )
        status.name = "test cone planner"
        status.message = status_text
        status.values = [
            KeyValue(key="status", value=status_text),
            KeyValue(key="confidence", value="0.9"),
        ]
        message.status.append(status)
        return message

    def activate_valid_path(self) -> Path:
        path = self.make_path()
        self.node._path_callback(path)
        self.node._planner_status_callback(self.make_status(path))
        self.assertIsNotNone(self.node._active_pair)
        self.node._control_callback()
        self.assertGreater(self.node._last_command_speed_mps, 0.0)
        return path

    def test_valid_pair_then_empty_path_stops_immediately(self) -> None:
        self.activate_valid_path()

        empty = self.make_path(empty=True)
        self.node._path_callback(empty)

        self.assertIsNone(self.node._active_pair)
        self.assertEqual(self.node._last_command_speed_mps, 0.0)
        self.assertEqual(self.node._last_command_steering_rad, 0.0)
        self.assertEqual(self.node._last_input_reason, "EMPTY_OR_BAD_PATH")

    def test_non_ok_status_cancels_previous_command(self) -> None:
        self.activate_valid_path()

        next_path = self.make_path()
        self.node._planner_status_callback(
            self.make_status(next_path, "LOW_CONFIDENCE")
        )

        self.assertIsNone(self.node._active_pair)
        self.assertEqual(self.node._last_command_speed_mps, 0.0)
        self.assertEqual(self.node._last_input_reason, "PLANNER_LOW_CONFIDENCE")

    def test_delayed_old_valid_pair_cannot_restart_after_invalid(self) -> None:
        old_path = self.activate_valid_path()
        old_status = self.make_status(old_path)
        self.node._path_callback(self.make_path(empty=True))
        self.assertIsNone(self.node._active_pair)

        self.node._path_callback(old_path)
        self.node._planner_status_callback(old_status)
        self.node._control_callback()

        self.assertIsNone(self.node._active_pair)
        self.assertEqual(self.node._last_command_speed_mps, 0.0)
        self.assertEqual(self.node._last_command_steering_rad, 0.0)

    def test_stale_path_is_rejected_before_pairing(self) -> None:
        stale = self.make_path(age_s=1.0)
        self.node._path_callback(stale)

        self.assertIsNone(self.node._active_pair)
        self.assertEqual(self.node._last_command_speed_mps, 0.0)
        self.assertEqual(self.node._last_input_reason, "PATH_STALE_STAMP")

    def test_path_and_status_must_have_exactly_the_same_stamp(self) -> None:
        path = self.make_path()
        status = self.make_status(path)
        status.header.stamp.nanosec += 1

        self.node._path_callback(path)
        self.node._planner_status_callback(status)

        self.assertIsNone(self.node._active_pair)
        self.assertEqual(self.node._last_command_speed_mps, 0.0)

    def test_future_stamp_does_not_poison_highest_accepted_stamp(self) -> None:
        future = self.make_path()
        future.header.stamp.sec += 10
        self.node._path_callback(future)
        self.assertEqual(self.node._highest_seen_stamp_ns, 0)

        valid = self.make_path()
        self.node._path_callback(valid)
        self.node._planner_status_callback(self.make_status(valid))

        self.assertIsNotNone(self.node._active_pair)


if __name__ == "__main__":
    unittest.main()

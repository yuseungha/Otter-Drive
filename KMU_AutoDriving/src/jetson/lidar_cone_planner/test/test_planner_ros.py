from __future__ import annotations

import math
import time
from types import SimpleNamespace
import unittest

try:
    import rclpy
    from diagnostic_msgs.msg import DiagnosticArray
    from nav_msgs.msg import Path
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.node import Node
    from rclpy.parameter import Parameter
    from sensor_msgs.msg import LaserScan

    from lidar_cone_planner.cone_line_planner import ConeLinePlanner

    ROS_AVAILABLE = True
except ImportError:
    ROS_AVAILABLE = False


@unittest.skipUnless(ROS_AVAILABLE, "ROS 2 Python packages are unavailable")
class PlannerRosFailClosedTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        rclpy.init()

    @classmethod
    def tearDownClass(cls) -> None:
        rclpy.try_shutdown()

    def setUp(self) -> None:
        overrides = [
            Parameter("planning_frame", value="base_link"),
            Parameter("scan_timeout_s", value=0.12),
            Parameter("watchdog_period_s", value=0.02),
            Parameter("max_scan_age_s", value=0.50),
            Parameter("max_future_scan_s", value=0.05),
        ]
        self.planner = ConeLinePlanner(parameter_overrides=overrides)
        self.listener = Node("planner_fail_closed_test_listener")
        self.paths = []
        self.statuses = []
        self.diagnostics = []
        self.listener.create_subscription(Path, "cone_planner/center_path", self.paths.append, 10)
        self.listener.create_subscription(
            DiagnosticArray, "cone_planner/status", self._status_callback, 10
        )
        self.executor = SingleThreadedExecutor()
        self.executor.add_node(self.planner)
        self.executor.add_node(self.listener)
        for _ in range(3):
            self.executor.spin_once(timeout_sec=0.01)

    def tearDown(self) -> None:
        self.executor.remove_node(self.listener)
        self.executor.remove_node(self.planner)
        self.listener.destroy_node()
        self.planner.destroy_node()
        self.executor.shutdown()

    def _status_callback(self, message: DiagnosticArray) -> None:
        if message.status:
            values = {value.key: value.value for value in message.status[0].values}
            self.diagnostics.append(values)
            self.statuses.append(values.get("status", message.status[0].message))

    def _scan(self) -> LaserScan:
        scan = LaserScan()
        scan.header.frame_id = "base_link"
        scan.header.stamp = self.planner.get_clock().now().to_msg()
        scan.angle_min = -math.pi
        scan.angle_increment = math.pi / 360.0
        scan.ranges = [float("inf")] * 721
        scan.angle_max = scan.angle_min + (len(scan.ranges) - 1) * scan.angle_increment
        scan.range_min = 0.12
        scan.range_max = 12.0
        scan.scan_time = 0.10
        scan.time_increment = scan.scan_time / len(scan.ranges)
        return scan

    def _spin_until(self, predicate, timeout_s=0.5) -> None:
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline and not predicate():
            self.executor.spin_once(timeout_sec=0.01)
        self.assertTrue(predicate(), f"timed out; statuses={self.statuses}")

    def _assert_latest_invalid(self, status: str) -> None:
        self._spin_until(lambda: bool(self.paths) and status in self.statuses)
        self.assertEqual(len(self.paths[-1].poses), 0)

    def test_zero_stale_and_out_of_order_scans_publish_empty_path(self) -> None:
        zero = self._scan()
        zero.header.stamp.sec = 0
        zero.header.stamp.nanosec = 0
        self.planner.scan_callback(zero)
        self._assert_latest_invalid("ZERO_STAMP")

        stale = self._scan()
        stale.header.stamp.sec -= 2
        self.planner.scan_callback(stale)
        self._assert_latest_invalid("STALE_SCAN")

        valid = self._scan()
        self.planner.scan_callback(valid)
        self._spin_until(lambda: "NOT_ENOUGH_CONES" in self.statuses)
        self.planner.scan_callback(valid)
        self._assert_latest_invalid("OUT_OF_ORDER_SCAN")

    def test_future_stamp_does_not_poison_next_valid_scan(self) -> None:
        future = self._scan()
        future.header.stamp.sec += 1
        self.planner.scan_callback(future)
        self._assert_latest_invalid("FUTURE_SCAN")
        current = self._scan()
        self.planner.scan_callback(current)
        self._spin_until(lambda: self.statuses and self.statuses[-1] == "NOT_ENOUGH_CONES")
        self.assertNotEqual(self.statuses[-1], "OUT_OF_ORDER_SCAN")
        self.assertEqual(len(self.paths[-1].poses), 0)

    def test_scan_watchdog_publishes_timeout_and_empty_path(self) -> None:
        self.planner.scan_callback(self._scan())
        self._spin_until(lambda: "SCAN_TIMEOUT" in self.statuses, timeout_s=0.40)
        self.assertEqual(self.statuses[-1], "SCAN_TIMEOUT")
        self.assertEqual(len(self.paths[-1].poses), 0)

    def test_missing_tf_and_bad_metadata_publish_empty_path(self) -> None:
        missing_tf = self._scan()
        missing_tf.header.frame_id = "missing_laser_frame"
        self.planner.scan_callback(missing_tf)
        self._assert_latest_invalid("TF_ERROR")

        bad_geometry = self._scan()
        bad_geometry.angle_max += 0.5
        self.planner.scan_callback(bad_geometry)
        self._assert_latest_invalid("BAD_SCAN_GEOMETRY")

    def test_tf_lookup_uses_exact_scan_timestamp(self) -> None:
        scan = self._scan()
        scan.header.frame_id = "laser"

        class RecordingBuffer:
            def __init__(self) -> None:
                self.call = None

            def lookup_transform(self, target, source, stamp, *, timeout):
                self.call = (target, source, stamp.nanoseconds, timeout.nanoseconds)
                return SimpleNamespace(
                    transform=SimpleNamespace(
                        translation=SimpleNamespace(x=0.18, y=0.01),
                        rotation=SimpleNamespace(x=0.0, y=0.0, z=0.0, w=1.0),
                    )
                )

        buffer = RecordingBuffer()
        self.planner.tf_buffer = buffer

        transform = self.planner._sensor_to_planning(scan)

        expected_stamp_ns = (
            scan.header.stamp.sec * 1_000_000_000 + scan.header.stamp.nanosec
        )
        self.assertEqual(transform, (0.18, 0.01, 0.0))
        self.assertIsNotNone(buffer.call)
        self.assertEqual(buffer.call[:3], ("base_link", "laser", expected_stamp_ns))

    def test_diagnostic_declares_local_normal_and_scan_stamp_policy(self) -> None:
        self.planner.scan_callback(self._scan())
        self._spin_until(
            lambda: any(
                values.get("status") == "NOT_ENOUGH_CONES"
                for values in self.diagnostics
            )
        )
        values = next(
            values
            for values in reversed(self.diagnostics)
            if values.get("status") == "NOT_ENOUGH_CONES"
        )
        self.assertEqual(values["planning_frame"], "base_link")
        self.assertEqual(values["scan_transform_time"], "scan_stamp")
        self.assertEqual(
            values["center_offset_policy"], "local_tangent_left_normal"
        )
        self.assertEqual(values["racing_line_offset"], "disabled")


if __name__ == "__main__":
    unittest.main()

"""Launch-level DDS, TF, QoS, and scan-dropout safety regression."""

from __future__ import annotations

import os
import time
import unittest


try:
    import launch
    from launch.actions import IncludeLaunchDescription
    from launch.launch_description_sources import PythonLaunchDescriptionSource
    import launch_testing
    import launch_testing.actions
    import pytest
    import rclpy
    from ament_index_python.packages import get_package_share_directory
    from diagnostic_msgs.msg import DiagnosticArray
    from geometry_msgs.msg import Vector3Stamped
    from nav_msgs.msg import Path
    from rclpy.qos import (
        DurabilityPolicy,
        HistoryPolicy,
        QoSProfile,
        ReliabilityPolicy,
    )
    from sensor_msgs.msg import LaserScan

    ROS_LAUNCH_AVAILABLE = True
except ImportError:
    ROS_LAUNCH_AVAILABLE = False


TEST_NAMESPACE = "sim_launch_test"
TOPIC_ROOT = f"/{TEST_NAMESPACE}"


if ROS_LAUNCH_AVAILABLE:

    @pytest.mark.launch_test
    def generate_test_description():
        """Start the installed real-node closed loop with one bounded fault."""

        package_share = get_package_share_directory("lidar_cone_planner")
        launch_file = os.path.join(
            package_share, "launch", "synthetic_closed_loop.launch.py"
        )
        closed_loop = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(launch_file),
            launch_arguments={
                "namespace": TEST_NAMESPACE,
                "scenario": "straight",
                # Give discovery, auto-enable, recovery frames, and physical
                # acceleration ample time before the fault begins.
                "drop_scan_after_s": "8.0",
                "drop_scan_duration_s": "3.0",
                "random_seed": "17",
            }.items(),
        )
        return launch.LaunchDescription(
            [closed_loop, launch_testing.actions.ReadyToTest()]
        )


@unittest.skipUnless(
    ROS_LAUNCH_AVAILABLE,
    "ROS 2 launch_testing packages are unavailable",
)
class SyntheticClosedLoopLaunchTest(unittest.TestCase):
    """Observe only public DDS messages from the launched node graph."""

    @classmethod
    def setUpClass(cls) -> None:
        if not os.environ.get("PYTEST_CURRENT_TEST"):
            raise unittest.SkipTest(
                "launch regression requires the launch_testing pytest runner"
            )
        if not rclpy.ok():
            rclpy.init()
        cls.node = rclpy.create_node("synthetic_closed_loop_launch_observer")

        sensor_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=50,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        reliable_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=100,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )

        cls.scan_times: list[float] = []
        cls.commands: list[dict[str, object]] = []
        cls.paths: list[dict[str, object]] = []
        cls.world_statuses: list[dict[str, object]] = []
        cls.planner_statuses: list[dict[str, object]] = []
        cls.controller_statuses: list[dict[str, object]] = []

        cls.subscriptions = [
            cls.node.create_subscription(
                LaserScan,
                f"{TOPIC_ROOT}/scan",
                cls._scan_callback,
                sensor_qos,
            ),
            cls.node.create_subscription(
                Vector3Stamped,
                f"{TOPIC_ROOT}/cone_controller/command_vector",
                cls._command_callback,
                reliable_qos,
            ),
            cls.node.create_subscription(
                Path,
                f"{TOPIC_ROOT}/cone_planner/center_path",
                cls._path_callback,
                reliable_qos,
            ),
            cls.node.create_subscription(
                DiagnosticArray,
                f"{TOPIC_ROOT}/synthetic_world/status",
                cls._world_status_callback,
                reliable_qos,
            ),
            cls.node.create_subscription(
                DiagnosticArray,
                f"{TOPIC_ROOT}/cone_planner/status",
                cls._planner_status_callback,
                reliable_qos,
            ),
            cls.node.create_subscription(
                DiagnosticArray,
                f"{TOPIC_ROOT}/cone_controller/status",
                cls._controller_status_callback,
                reliable_qos,
            ),
        ]

    @classmethod
    def tearDownClass(cls) -> None:
        cls.node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

    @staticmethod
    def _stamp_ns(message) -> int:
        return int(message.header.stamp.sec) * 1_000_000_000 + int(
            message.header.stamp.nanosec
        )

    @classmethod
    def _scan_callback(cls, _message: LaserScan) -> None:
        cls.scan_times.append(time.monotonic())

    @classmethod
    def _command_callback(cls, message: Vector3Stamped) -> None:
        cls.commands.append(
            {
                "time": time.monotonic(),
                "speed": float(message.vector.x),
                "steering": float(message.vector.y),
                "frame": message.header.frame_id,
            }
        )

    @classmethod
    def _path_callback(cls, message: Path) -> None:
        cls.paths.append(
            {
                "time": time.monotonic(),
                "stamp_ns": cls._stamp_ns(message),
                "points": len(message.poses),
                "frame": message.header.frame_id,
            }
        )

    @staticmethod
    def _diagnostic_record(message: DiagnosticArray) -> dict[str, object]:
        values: dict[str, str] = {}
        level = -1
        status_message = "MISSING_STATUS"
        if message.status:
            selected = message.status[0]
            raw_level = selected.level
            level = (
                int(raw_level[0])
                if isinstance(raw_level, (bytes, bytearray))
                else int(raw_level)
            )
            status_message = selected.message
            values = {item.key: item.value for item in selected.values}
        return {
            "time": time.monotonic(),
            "stamp_ns": SyntheticClosedLoopLaunchTest._stamp_ns(message),
            "status": values.get("status", status_message),
            "level": level,
            "values": values,
        }

    @classmethod
    def _world_status_callback(cls, message: DiagnosticArray) -> None:
        cls.world_statuses.append(cls._diagnostic_record(message))

    @classmethod
    def _planner_status_callback(cls, message: DiagnosticArray) -> None:
        cls.planner_statuses.append(cls._diagnostic_record(message))

    @classmethod
    def _controller_status_callback(cls, message: DiagnosticArray) -> None:
        cls.controller_statuses.append(cls._diagnostic_record(message))

    @staticmethod
    def _number(record: dict[str, object], key: str) -> float:
        values = record["values"]
        assert isinstance(values, dict)
        return float(values[key])

    def _snapshot(self) -> str:
        def latest(records):
            return records[-1] if records else None

        return (
            f"scans={len(self.scan_times)} commands={len(self.commands)} "
            f"paths={len(self.paths)} "
            f"world={latest(self.world_statuses)!r} "
            f"planner={latest(self.planner_statuses)!r} "
            f"controller={latest(self.controller_statuses)!r}"
        )

    def _wait_for(self, finder, timeout_s: float, description: str):
        """Spin until an event is found, using monotonic time for the bound."""

        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            result = finder()
            if result is not None:
                return result
            remaining = max(0.0, deadline - time.monotonic())
            rclpy.spin_once(self.node, timeout_sec=min(0.05, remaining))
        result = finder()
        if result is not None:
            return result
        self.fail(f"Timed out waiting for {description}: {self._snapshot()}")

    @staticmethod
    def _first_after(records, start_time: float, predicate):
        return next(
            (
                record
                for record in records
                if float(record["time"]) >= start_time and predicate(record)
            ),
            None,
        )

    def test_straight_motion_and_scan_dropout_fail_closed(self) -> None:
        """Drive, lose LaserScan, and verify the public fail-closed contract."""

        positive_command = self._wait_for(
            lambda: self._first_after(
                self.commands,
                0.0,
                lambda record: float(record["speed"]) > 0.02,
            ),
            10.0,
            "a positive controller command",
        )
        moving_world = self._wait_for(
            lambda: self._first_after(
                self.world_statuses,
                float(positive_command["time"]),
                lambda record: (
                    record["status"] == "OK"
                    and self._number(record, "applied_speed_mps") > 0.02
                    and self._number(record, "along_track_m") > 0.10
                ),
            ),
            5.0,
            "positive applied speed and straight-course progress",
        )
        self.assertEqual(self._number(moving_world, "collision_count"), 0.0)
        self.assertEqual(
            self._number(moving_world, "rejected_command_count"), 0.0
        )

        drop_status = self._wait_for(
            lambda: self._first_after(
                self.world_statuses,
                float(moving_world["time"]),
                lambda record: record["status"] == "SCAN_DROP_ACTIVE",
            ),
            10.0,
            "the injected LaserScan dropout",
        )
        drop_time = float(drop_status["time"])
        scans_before_drop = [value for value in self.scan_times if value <= drop_time]
        self.assertTrue(scans_before_drop, self._snapshot())
        last_scan_time = max(scans_before_drop)

        planner_timeout = self._wait_for(
            lambda: self._first_after(
                self.planner_statuses,
                drop_time,
                lambda record: record["status"] == "SCAN_TIMEOUT",
            ),
            2.0,
            "planner SCAN_TIMEOUT",
        )
        empty_path = self._wait_for(
            lambda: self._first_after(
                self.paths,
                drop_time,
                lambda record: (
                    int(record["points"]) == 0
                    and int(record["stamp_ns"])
                    == int(planner_timeout["stamp_ns"])
                ),
            ),
            1.0,
            "an empty Path sharing the SCAN_TIMEOUT stamp",
        )
        self.assertEqual(empty_path["frame"], "sim_base_link")

        zero_command = self._wait_for(
            lambda: self._first_after(
                self.commands,
                drop_time,
                lambda record: (
                    abs(float(record["speed"])) <= 1.0e-9
                    and abs(float(record["steering"])) <= 1.0e-9
                ),
            ),
            2.0,
            "a zero controller command during dropout",
        )
        self.assertEqual(zero_command["frame"], "sim_base_link")
        controller_zero = self._wait_for(
            lambda: self._first_after(
                self.controller_statuses,
                drop_time,
                lambda record: self._number(record, "speed_mps") == 0.0,
            ),
            2.0,
            "controller speed_mps=0 diagnostic",
        )

        # BEST_EFFORT observation may miss the final scan, so leave one scan
        # period plus scheduler margin over the configured 0.35 s watchdog.
        self.assertLessEqual(
            float(planner_timeout["time"]) - last_scan_time,
            0.75,
            self._snapshot(),
        )
        self.assertLessEqual(
            float(zero_command["time"]) - last_scan_time,
            0.75,
            self._snapshot(),
        )
        self.assertGreaterEqual(float(controller_zero["time"]), drop_time)

        stopped_world = self._wait_for(
            lambda: self._first_after(
                self.world_statuses,
                float(zero_command["time"]),
                lambda record: (
                    record["status"] == "SCAN_DROP_ACTIVE"
                    and self._number(record, "applied_speed_mps") <= 0.001
                    and self._number(record, "dropped_scan_count") >= 5.0
                ),
            ),
            2.0,
            "the simulated vehicle to stop inside the dropout window",
        )
        interval_end = float(stopped_world["time"])
        positive_after_zero = [
            record
            for record in self.commands
            if float(zero_command["time"])
            <= float(record["time"])
            <= interval_end
            and float(record["speed"]) > 1.0e-9
        ]
        self.assertEqual(positive_after_zero, [], self._snapshot())
        self.assertEqual(self._number(stopped_world, "collision_count"), 0.0)
        self.assertEqual(
            self._number(stopped_world, "rejected_command_count"), 0.0
        )

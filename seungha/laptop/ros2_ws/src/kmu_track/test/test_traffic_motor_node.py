"""ROS integration test for guarded traffic-light motor commands."""

import time

import pytest


rclpy = pytest.importorskip('rclpy')

from rclpy.executors import SingleThreadedExecutor  # noqa: E402
from rclpy.node import Node  # noqa: E402
from std_msgs.msg import Float32, Int32MultiArray, String  # noqa: E402

from kmu_track.traffic_motor_node import TrafficMotorNode  # noqa: E402


def test_preview_topic_maps_go_left_and_stop_center_sequence():
    """Exercise the ROS subscriptions and preview publisher end to end."""
    rclpy.init(args=[
        '--ros-args',
        '-p', 'enabled:=true',
        '-p', 'dry_run:=true',
        '-p', 'signal_timeout_sec:=2.0',
        '-p', 'throttle_ramp_per_sec:=10000.0',
    ], domain_id=91)
    controller = TrafficMotorNode()
    probe = Node('traffic_motor_test_probe')
    signal_pub = probe.create_publisher(
        String, '/perception/traffic_light_state', 10)
    speed_pub = probe.create_publisher(
        Float32, '/vehicle/speed_mps', 10)
    commands = []
    probe.create_subscription(
        Int32MultiArray,
        '/rc_car/drive_cmd_preview',
        lambda message: commands.append(list(message.data)),
        10,
    )
    executor = SingleThreadedExecutor()
    executor.add_node(controller)
    executor.add_node(probe)

    def wait_for(signal, predicate, timeout_sec=2.0, speed_mps=None):
        deadline = time.monotonic() + timeout_sec
        next_publish = 0.0
        while time.monotonic() < deadline:
            now = time.monotonic()
            if now >= next_publish:
                signal_pub.publish(String(data=signal))
                if speed_mps is not None:
                    speed_pub.publish(Float32(data=speed_mps))
                next_publish = now + 0.05
            executor.spin_once(timeout_sec=0.02)
            if commands and predicate(commands[-1]):
                return commands[-1]
        pytest.fail(
            f'no matching command for {signal}; latest={commands[-3:]}')

    try:
        assert wait_for('GO', lambda value: value == [150, 0, -1]) == [
            150, 0, -1]
        assert wait_for(
            'TURN LEFT',
            lambda value: value == [100, 350, -1],
        ) == [100, 350, -1]
        assert wait_for(
            'STOP',
            lambda value: value == [0, 350, -1],
            speed_mps=0.20,
        ) == [0, 350, -1]
        assert wait_for(
            'STOP',
            lambda value: value == [0, 0, -1],
            speed_mps=0.0,
        ) == [0, 0, -1]
    finally:
        executor.remove_node(controller)
        executor.remove_node(probe)
        controller.destroy_node()
        probe.destroy_node()
        executor.shutdown()
        if rclpy.ok():
            rclpy.shutdown()

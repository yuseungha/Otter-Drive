#!/usr/bin/env python3
"""Verify sticky keyboard commands on an isolated ROS domain."""

import time

import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray

from rc_car_teleop.keyboard_teleop import (
    GEAR_HIGH,
    GEAR_LOW,
    KeyboardTeleop,
    THROTTLE_STEP,
)
from rc_car_teleop.serial_bridge import SerialBridge


class CaptureSerial:
    def __init__(self) -> None:
        self.writes = []

    def write(self, payload: bytes) -> None:
        self.writes.append(payload)


def main() -> None:
    rclpy.init()
    teleop = KeyboardTeleop()
    observer = Node('manual_persistence_probe')
    samples = []
    observer.create_subscription(
        Int32MultiArray,
        '/rc_car/drive_cmd',
        lambda message: samples.append((time.monotonic(), list(message.data))),
        10,
    )

    executor = SingleThreadedExecutor()
    executor.add_node(teleop)
    executor.add_node(observer)
    started_at = time.monotonic()

    try:
        teleop.handle_key('w')
        while time.monotonic() - started_at < 3.2:
            executor.spin_once(timeout_sec=0.05)

        persistent_samples = [
            value
            for observed_at, value in samples
            if observed_at - started_at >= 2.5
        ]
        expected = [THROTTLE_STEP, 0, GEAR_LOW]
        if not persistent_samples:
            raise RuntimeError('no command samples received after 2.5 seconds')
        if any(value != expected for value in persistent_samples):
            raise RuntimeError(
                f'command did not persist: expected={expected}, '
                f'actual_tail={persistent_samples[-5:]}'
            )
        print(
            'PASS: one W input kept throttle=150 for more than 2.5 seconds '
            f'({len(persistent_samples)} late samples)'
        )

        teleop.handle_key('2')
        gear_deadline = time.monotonic() + 0.5
        while time.monotonic() < gear_deadline:
            executor.spin_once(timeout_sec=0.05)
        if samples[-1][1] != [THROTTLE_STEP, 0, GEAR_HIGH]:
            raise RuntimeError(f'HIGH gear key failed: actual={samples[-1][1]}')
        print('PASS: key 2 selected HIGH gear in the ROS drive command')

        teleop.handle_key('1')
        gear_deadline = time.monotonic() + 0.5
        while time.monotonic() < gear_deadline:
            executor.spin_once(timeout_sec=0.05)
        if samples[-1][1] != [THROTTLE_STEP, 0, GEAR_LOW]:
            raise RuntimeError(f'LOW gear key failed: actual={samples[-1][1]}')
        print('PASS: key 1 selected LOW gear in the ROS drive command')

        bridge = SerialBridge.__new__(SerialBridge)
        bridge._throttle = 0
        bridge._steering = 0
        bridge._gear = GEAR_LOW
        bridge._source_neutral_seen = True
        bridge._connected_at = 0.0
        bridge._reset_guard = 0.0
        bridge._serial = CaptureSerial()

        drive_message = Int32MultiArray()
        drive_message.data = [200, -300, GEAR_HIGH]
        bridge._drive_callback(drive_message)
        bridge._write_command(1.0)
        if bridge._serial.writes != [b'D200 -300 1\n']:
            raise RuntimeError(
                f'gear serial protocol failed: writes={bridge._serial.writes}'
            )
        print('PASS: serial bridge emitted D200 -300 1 for HIGH gear')
    finally:
        teleop.set_full_neutral()
        teleop.publish_command()
        executor.shutdown()
        observer.destroy_node()
        teleop.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()

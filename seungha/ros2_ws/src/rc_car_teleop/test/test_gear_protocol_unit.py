"""Host-side tests for gear keyboard and serial protocol behavior."""

import sys
import types
import unittest
from pathlib import Path


sys.path.insert(0, str(Path(__file__).resolve().parents[1]))


class FakePublisher:
    def __init__(self) -> None:
        self.messages = []

    def publish(self, message) -> None:
        self.messages.append(list(message.data))


class FakeNode:
    def __init__(self, _name: str) -> None:
        self.publisher = None

    def create_publisher(self, *_args):
        self.publisher = FakePublisher()
        return self.publisher

    def create_subscription(self, *_args):
        return None

    def create_timer(self, *_args):
        return None


class FakeMessage:
    def __init__(self) -> None:
        self.data = []


def install_dependency_stubs() -> None:
    rclpy = types.ModuleType('rclpy')
    rclpy_node = types.ModuleType('rclpy.node')
    rclpy_node.Node = FakeNode
    rclpy_executors = types.ModuleType('rclpy.executors')
    rclpy_executors.ExternalShutdownException = RuntimeError
    rclpy_executors.SingleThreadedExecutor = object

    std_msgs = types.ModuleType('std_msgs')
    std_msgs_msg = types.ModuleType('std_msgs.msg')
    std_msgs_msg.Bool = FakeMessage
    std_msgs_msg.Int32 = FakeMessage
    std_msgs_msg.Int32MultiArray = FakeMessage
    std_msgs_msg.String = FakeMessage

    serial = types.ModuleType('serial')
    serial.SerialException = OSError
    serial.SerialTimeoutException = TimeoutError

    termios = types.ModuleType('termios')
    termios.TCSADRAIN = 0
    tty = types.ModuleType('tty')

    sys.modules.update({
        'rclpy': rclpy,
        'rclpy.node': rclpy_node,
        'rclpy.executors': rclpy_executors,
        'std_msgs': std_msgs,
        'std_msgs.msg': std_msgs_msg,
        'serial': serial,
        'termios': termios,
        'tty': tty,
    })


install_dependency_stubs()

from rc_car_teleop.keyboard_teleop import (  # noqa: E402
    GEAR_HIGH,
    GEAR_LOW,
    KeyboardTeleop,
    THROTTLE_STEP,
)
from rc_car_teleop.serial_bridge import SerialBridge  # noqa: E402


class CaptureSerial:
    def __init__(self) -> None:
        self.writes = []

    def write(self, payload: bytes) -> None:
        self.writes.append(payload)


class GearProtocolTests(unittest.TestCase):
    """Verify keyboard gear selection and the Arduino serial frame."""

    def test_keyboard_gear_keys_and_persistent_throttle(self) -> None:
        teleop = KeyboardTeleop()

        teleop.handle_key('w')
        self.assertEqual(
            teleop.publisher.messages[-1],
            [THROTTLE_STEP, 0, GEAR_LOW],
        )

        teleop.publish_command()
        teleop.publish_command()
        self.assertEqual(
            teleop.publisher.messages[-1],
            [THROTTLE_STEP, 0, GEAR_LOW],
        )

        teleop.handle_key('2')
        self.assertEqual(
            teleop.publisher.messages[-1],
            [THROTTLE_STEP, 0, GEAR_HIGH],
        )

        teleop.handle_key('1')
        self.assertEqual(
            teleop.publisher.messages[-1],
            [THROTTLE_STEP, 0, GEAR_LOW],
        )

    def test_serial_bridge_emits_gear_field(self) -> None:
        bridge = SerialBridge.__new__(SerialBridge)
        bridge._throttle = 0
        bridge._steering = 0
        bridge._gear = GEAR_LOW
        bridge._source_neutral_seen = True
        bridge._connected_at = 0.0
        bridge._reset_guard = 0.0
        bridge._serial = CaptureSerial()

        command = FakeMessage()
        command.data = [200, -300, GEAR_HIGH]
        bridge._drive_callback(command)
        bridge._write_command(1.0)

        self.assertEqual(bridge._serial.writes, [b'D200 -300 1\n'])

    def test_keyboard_and_bridge_allow_1050_throttle_only(self) -> None:
        teleop = KeyboardTeleop()
        for _ in range(8):
            teleop.handle_key('w')
        self.assertEqual(teleop.publisher.messages[-1], [1050, 0, GEAR_LOW])

        bridge = SerialBridge.__new__(SerialBridge)
        bridge._throttle = 0
        bridge._steering = 0
        bridge._gear = GEAR_LOW
        bridge._source_neutral_seen = False
        command = FakeMessage()
        command.data = [1200, -1200, GEAR_LOW]
        bridge._drive_callback(command)
        self.assertEqual((bridge._throttle, bridge._steering), (1050, -1000))


if __name__ == '__main__':
    unittest.main()

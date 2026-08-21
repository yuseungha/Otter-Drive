#!/usr/bin/env python3
"""Publish atomic, normalized RC-car commands from an interactive terminal."""

import json
import sys
import termios
import threading
import time
import tty

import rclpy
from rclpy.executors import ExternalShutdownException, SingleThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray, String

STEERING_STEP = 100
THROTTLE_FULL_REVERSE = -1000
THROTTLE_FULL_FORWARD = 1000
STEERING_MIN = -1000
STEERING_MAX = 1000
PUBLISH_RATE_HZ = 20.0

HELP = """
=== RC Car keyboard control ===
 W/S : full forward / full reverse (value persists until changed or stopped)
 A/D : steer left / right
 Space: throttle stop     C: center steering
 X: full stop and center  Q or Ctrl-C: quit
Throttle and steering keep their last values until another command changes them.
Throttle command: -1000 / 0 / 1000. Steering range: -1000 .. 1000.
Gear is fixed to LOW by the serial bridge and is not selectable here.
Live line: camera signal | manual input | final motor output | safety reason.
"""


class KeyboardTeleop(Node):
    def __init__(self):
        super().__init__('keyboard_teleop')
        self._publisher = self.create_publisher(
            Int32MultiArray, '/rc_car/drive_cmd', 10)
        self._lock = threading.Lock()
        self._print_lock = threading.Lock()
        self._throttle = 0
        self._steering = 0
        self._signal = 'WAITING'
        self._output_throttle = 0
        self._output_steering = 0
        self._output_reason = 'waiting_for_status'
        self._last_rendered = None
        self.create_subscription(
            String,
            '/vehicle/traffic_motor_status',
            self.status_callback,
            10,
        )
        self.create_timer(1.0 / PUBLISH_RATE_HZ, self.publish_command)
        print(HELP)

    def status_callback(self, message):
        try:
            status = json.loads(message.data)
        except (TypeError, ValueError, json.JSONDecodeError):
            return
        with self._lock:
            self._signal = str(status.get('signal', 'UNKNOWN'))
            self._output_throttle = int(status.get('throttle', 0))
            self._output_steering = int(status.get('steering', 0))
            self._output_reason = str(status.get('reason', 'unknown'))
        self.render_status()

    def render_status(self, force=False):
        with self._lock:
            snapshot = (
                self._signal,
                self._throttle,
                self._steering,
                self._output_throttle,
                self._output_steering,
                self._output_reason,
            )
        with self._print_lock:
            if not force and snapshot == self._last_rendered:
                return
            self._last_rendered = snapshot
            signal, throttle, steering, output_throttle, \
                output_steering, reason = snapshot
            print(
                f'\rCAM={signal:9s} | '
                f'INPUT T={throttle:5d} S={steering:5d} | '
                f'OUTPUT T={output_throttle:5d} S={output_steering:5d} | '
                f'{reason:24s}   ',
                end='',
                flush=True,
            )

    def publish_command(self):
        with self._lock:
            values = [self._throttle, self._steering]
        message = Int32MultiArray()
        message.data = values
        self._publisher.publish(message)

    def handle_key(self, key):
        if key in ('', 'q', 'Q', '\x03'):
            return False

        with self._lock:
            if key in ('w', 'W'):
                self._throttle = THROTTLE_FULL_FORWARD
            elif key in ('s', 'S'):
                self._throttle = THROTTLE_FULL_REVERSE
            elif key in ('a', 'A'):
                self._steering = min(
                    STEERING_MAX, self._steering + STEERING_STEP)
            elif key in ('d', 'D'):
                self._steering = max(
                    STEERING_MIN, self._steering - STEERING_STEP)
            elif key == ' ':
                self._throttle = 0
            elif key in ('c', 'C'):
                self._steering = 0
            elif key in ('x', 'X'):
                self._throttle = 0
                self._steering = 0
        self.publish_command()
        self.render_status(force=True)
        return True

    def set_full_neutral(self):
        with self._lock:
            self._throttle = 0
            self._steering = 0


def read_key(settings):
    tty.setraw(sys.stdin.fileno())
    try:
        return sys.stdin.read(1)
    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)


def main(args=None):
    if not sys.stdin.isatty():
        print('keyboard_teleop requires an interactive TTY.', file=sys.stderr)
        return

    settings = termios.tcgetattr(sys.stdin)
    rclpy.init(args=args)
    node = KeyboardTeleop()
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    try:
        while node.handle_key(read_key(settings)):
            pass
    except (KeyboardInterrupt, EOFError, ExternalShutdownException):
        pass
    finally:
        node.set_full_neutral()
        for _ in range(3):
            node.publish_command()
            time.sleep(0.05)
        executor.shutdown()
        spin_thread.join(timeout=1.0)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        print('\nStopped: full-neutral command sent.')


if __name__ == '__main__':
    main()

#!/usr/bin/env python3
"""Publish one bounded low-throttle command followed by a long neutral hold."""

import json
import time

import rclpy
from std_msgs.msg import Int32MultiArray, String


RATE_HZ = 20.0
TEST_THROTTLE = 150


def publish_for(publisher, values, duration_sec):
    message = Int32MultiArray(data=values)
    deadline = time.monotonic() + duration_sec
    while time.monotonic() < deadline:
        publisher.publish(message)
        rclpy.spin_once(node, timeout_sec=0.0)
        time.sleep(1.0 / RATE_HZ)


rclpy.init()
node = rclpy.create_node('controlled_airborne_pulse')
publisher = node.create_publisher(
    Int32MultiArray, '/rc_car/manual_drive_cmd', 10)
last_status = None
last_feedback = None


def on_status(message):
    global last_status
    status = json.loads(message.data)
    key = (
        status.get('signal'), status.get('throttle'),
        status.get('steering'), status.get('reason'))
    if key != last_status:
        print(f'STATUS {message.data}', flush=True)
        last_status = key


def on_feedback(message):
    global last_feedback
    if '[COMMAND]' in message.data and message.data != last_feedback:
        print(f'ARDUINO {message.data}', flush=True)
        last_feedback = message.data


node.create_subscription(
    String, '/vehicle/traffic_motor_status', on_status, 10)
node.create_subscription(String, '/rc_car/feedback', on_feedback, 10)

try:
    print('NEUTRAL 0.5s', flush=True)
    publish_for(publisher, [0, 0, -1], 0.5)
    print('PULSE throttle=150 for 1.0s', flush=True)
    publish_for(publisher, [TEST_THROTTLE, 0, -1], 1.0)
finally:
    print('NEUTRAL 2.0s', flush=True)
    publish_for(publisher, [0, 0, -1], 2.0)
    node.destroy_node()
    rclpy.shutdown()

print('CONTROLLED PULSE COMPLETE', flush=True)

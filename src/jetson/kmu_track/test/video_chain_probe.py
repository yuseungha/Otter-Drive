#!/usr/bin/env python3
"""Collect concise evidence from the video lane-control ROS chain."""

import json
import os
from time import monotonic

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Bool, Float32, Int32MultiArray, String, UInt32


class VideoChainProbe(Node):
    """Accumulate frame, perception, and preview-command metrics."""

    def __init__(self) -> None:
        super().__init__('video_chain_probe')
        self.metrics = {
            'image_messages': 0,
            'frame_messages': 0,
            'frame_first': None,
            'frame_last': None,
            'detection_messages': 0,
            'nonempty_detections': 0,
            'valid_true': 0,
            'confidence_max': 0.0,
            'center_error_min': None,
            'center_error_max': None,
            'preview_messages': 0,
            'nonzero_steering': 0,
            'steering_min': None,
            'steering_max': None,
            'gate_reasons': {},
        }
        self.create_subscription(
            Image, '/camera/front/image_raw', self._on_image, 10)
        self.create_subscription(
            UInt32, '/video/frame_index', self._on_frame, 10)
        self.create_subscription(
            String, '/lane/yolo_detections', self._on_detections, 10)
        self.create_subscription(Bool, '/lane/valid', self._on_valid, 10)
        self.create_subscription(
            Float32, '/lane/confidence', self._on_confidence, 10)
        self.create_subscription(
            Float32, '/lane/center_error', self._on_error, 10)
        self.create_subscription(
            Int32MultiArray,
            '/rc_car/drive_cmd_preview',
            self._on_preview,
            10,
        )
        self.create_subscription(
            String,
            '/vehicle/lane_control_status',
            self._on_status,
            10,
        )

    def _on_image(self, _message: Image) -> None:
        self.metrics['image_messages'] += 1

    def _on_frame(self, message: UInt32) -> None:
        index = int(message.data)
        self.metrics['frame_messages'] += 1
        if self.metrics['frame_first'] is None:
            self.metrics['frame_first'] = index
        self.metrics['frame_last'] = index

    def _on_detections(self, message: String) -> None:
        self.metrics['detection_messages'] += 1
        try:
            if json.loads(message.data):
                self.metrics['nonempty_detections'] += 1
        except json.JSONDecodeError:
            pass

    def _on_valid(self, message: Bool) -> None:
        if message.data:
            self.metrics['valid_true'] += 1

    def _on_confidence(self, message: Float32) -> None:
        self.metrics['confidence_max'] = max(
            self.metrics['confidence_max'], float(message.data))

    def _on_error(self, message: Float32) -> None:
        value = float(message.data)
        low = self.metrics['center_error_min']
        high = self.metrics['center_error_max']
        self.metrics['center_error_min'] = (
            value if low is None else min(low, value))
        self.metrics['center_error_max'] = (
            value if high is None else max(high, value))

    def _on_preview(self, message: Int32MultiArray) -> None:
        if len(message.data) < 2:
            return
        steering = int(message.data[1])
        self.metrics['preview_messages'] += 1
        if steering != 0:
            self.metrics['nonzero_steering'] += 1
        low = self.metrics['steering_min']
        high = self.metrics['steering_max']
        self.metrics['steering_min'] = (
            steering if low is None else min(low, steering))
        self.metrics['steering_max'] = (
            steering if high is None else max(high, steering))

    def _on_status(self, message: String) -> None:
        try:
            status = json.loads(message.data)
            reason = str(status.get('gate_reason', 'unknown'))
        except json.JSONDecodeError:
            reason = 'invalid_json'
        reasons = self.metrics['gate_reasons']
        reasons[reason] = reasons.get(reason, 0) + 1


def main() -> int:
    """Collect for a bounded duration and return a contract result."""
    duration = float(os.environ.get('PROBE_DURATION_SEC', '20'))
    require_valid = os.environ.get('REQUIRE_VALID_LANE', '0') == '1'
    rclpy.init()
    node = VideoChainProbe()
    deadline = monotonic() + duration
    try:
        while monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.2)
    finally:
        metrics = node.metrics
        node.destroy_node()
        rclpy.shutdown()

    print('VIDEO_CHAIN_METRICS=' + json.dumps(metrics, sort_keys=True))
    basic = (
        metrics['image_messages'] > 0
        and metrics['detection_messages'] > 0
        and metrics['preview_messages'] > 0
    )
    valid = (
        metrics['valid_true'] > 0
        and metrics['nonempty_detections'] > 0
        and metrics['nonzero_steering'] > 0
    )
    print(f'BASIC_CHAIN_OK={str(basic).lower()}')
    print(f'VALID_LANE_CONTROL_OK={str(valid).lower()}')
    return 0 if basic and (valid or not require_valid) else 1


if __name__ == '__main__':
    raise SystemExit(main())

"""YOLO road-region detection plus tracked scan-line lane geometry."""

import json
from pathlib import Path
from time import perf_counter

import cv2
import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from std_msgs.msg import Bool, Float32, Int32MultiArray, String
import torch
from ultralytics import YOLO

from kmu_track.lane_core import bev_points_to_image, preprocess_lane_frame
from kmu_track.lane_feature_core import LaneFeatureTracker


def image_message_to_numpy(message: Image) -> np.ndarray:
    """Convert bgr8 or mono8 without cv_bridge's NumPy binary ABI."""
    data = np.frombuffer(message.data, dtype=np.uint8)
    rows = data.reshape(message.height, message.step)
    if message.encoding in {'bgr8', 'rgb8'}:
        image = rows[:, :message.width * 3].reshape(
            message.height, message.width, 3).copy()
        if message.encoding == 'rgb8':
            image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
        return image
    if message.encoding in {'mono8', '8UC1'}:
        return rows[:, :message.width].copy()
    raise ValueError(f'unsupported image encoding: {message.encoding}')


def numpy_to_image_message(image: np.ndarray, source: Image) -> Image:
    """Build a ROS image message without importing cv_bridge."""
    contiguous = np.ascontiguousarray(image, dtype=np.uint8)
    message = Image()
    message.header = source.header
    message.height = int(contiguous.shape[0])
    message.width = int(contiguous.shape[1])
    if contiguous.ndim == 2:
        message.encoding = 'mono8'
        channels = 1
    else:
        message.encoding = 'bgr8'
        channels = 3
    message.is_bigendian = 0
    message.step = int(contiguous.shape[1] * channels)
    message.data = contiguous.tobytes()
    return message


class YoloLaneDetectorNode(Node):
    """Detect YOLO road regions and validate actual markings inside them."""

    def __init__(self) -> None:
        super().__init__('yolo_lane_detector')
        self._declare_parameters()
        model_path = Path(str(self.get_parameter('model_path').value))
        if not model_path.is_file():
            raise FileNotFoundError(f'YOLO model does not exist: {model_path}')
        self.model = YOLO(str(model_path))
        if self.model.task != 'detect':
            raise ValueError(
                f'expected a YOLO detection model, got task={self.model.task}')
        self.left_class_id = self._class_id(
            str(self.get_parameter('left_class_name').value))
        self.right_class_id = self._class_id(
            str(self.get_parameter('right_class_name').value))
        self.device = self._resolve_device(
            str(self.get_parameter('device').value))
        self.half_precision = bool(
            self.get_parameter('half_precision').value
        ) and self.device != 'cpu'
        if self.device != 'cpu':
            torch.backends.cudnn.benchmark = True
        if bool(self.get_parameter('warmup').value):
            self._warmup_model()

        self.thresholds = self._parameters_to_thresholds()
        self.feature_tracker = self._make_feature_tracker()
        self.latest_input = None
        self.last_processed_stamp = None
        self.invalid_since = None

        self.error_pub = self.create_publisher(Float32, '/lane/center_error', 10)
        self.valid_pub = self.create_publisher(Bool, '/lane/valid', 10)
        self.confidence_pub = self.create_publisher(
            Float32, '/lane/confidence', 10)
        self.heading_pub = self.create_publisher(
            Float32, '/lane/heading_error', 10)
        self.inference_ms_pub = self.create_publisher(
            Float32, '/lane/inference_ms', 10)
        self.detections_pub = self.create_publisher(
            String, '/lane/yolo_detections', 10)
        self.geometry_pub = self.create_publisher(
            String, '/lane/lane_geometry', 10)
        self.debug_pub = self.create_publisher(
            Image, '/lane/yolo_debug', qos_profile_sensor_data)
        self.overlay_pub = self.create_publisher(
            Image, '/lane/lane_overlay', qos_profile_sensor_data)
        self.binary_pub = self.create_publisher(
            Image, '/lane/debug_binary', qos_profile_sensor_data)
        self.bev_pub = self.create_publisher(
            Image, '/lane/bev_image', qos_profile_sensor_data)
        self.white_pub = self.create_publisher(
            Image, '/lane/white_mask', qos_profile_sensor_data)
        self.yellow_pub = self.create_publisher(
            Image, '/lane/yellow_mask', qos_profile_sensor_data)
        threshold_qos = QoSProfile(depth=1)
        threshold_qos.reliability = ReliabilityPolicy.RELIABLE
        threshold_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.threshold_pub = self.create_publisher(
            Int32MultiArray, '/lane/hsv_thresholds/current', threshold_qos)
        self.ready_pub = self.create_publisher(
            Bool, '/lane/detector_ready', threshold_qos)
        self.create_subscription(
            Image,
            str(self.get_parameter('image_topic').value),
            self._on_image,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            Int32MultiArray, '/lane/hsv_thresholds/set', self._on_thresholds, 10)
        rate = max(0.2, float(self.get_parameter('inference_rate_hz').value))
        self.timer = self.create_timer(1.0 / rate, self._run_inference)
        self._publish_thresholds()
        self.ready_pub.publish(Bool(data=True))
        self.get_logger().info(
            f'YOLO lane detector ready: {model_path} | '
            f'classes={self.model.names} | device={self.device} | '
            f'input={self.get_parameter("inference_input").value} | '
            f'FP16={self.half_precision}')

    def _declare_parameters(self) -> None:
        self.declare_parameter('model_path', '')
        self.declare_parameter('image_topic', '/camera/front/image_raw')
        self.declare_parameter('roi_top_ratio', 0.45)
        self.declare_parameter('bev_width', 320)
        self.declare_parameter('bev_height', 240)
        self.declare_parameter(
            'bev_source', [0.05, 0.98, 0.95, 0.98, 0.62, 0.05, 0.38, 0.05])
        self.declare_parameter(
            'bev_destination', [0.20, 0.99, 0.80, 0.99, 0.80, 0.0, 0.20, 0.0])
        self.declare_parameter('white_lower_hsv', [0, 0, 130])
        self.declare_parameter('white_upper_hsv', [180, 80, 255])
        self.declare_parameter('yellow_lower_hsv', [12, 70, 70])
        self.declare_parameter('yellow_upper_hsv', [42, 255, 255])
        self.declare_parameter('morphology_kernel', 3)
        self.declare_parameter('use_yellow_mask', False)
        self.declare_parameter('publish_bev_debug', False)
        self.declare_parameter('publish_individual_masks', False)
        self.declare_parameter('publish_lane_overlay', True)
        self.declare_parameter('publish_lane_geometry', True)
        self.declare_parameter('overlay_sample_rows', 8)
        self.declare_parameter('confidence_threshold', 0.25)
        self.declare_parameter('iou_threshold', 0.45)
        self.declare_parameter('input_size', 640)
        self.declare_parameter('device', 'auto')
        self.declare_parameter('half_precision', True)
        self.declare_parameter('inference_rate_hz', 15.0)
        self.declare_parameter('max_detections', 2)
        self.declare_parameter('warmup', True)
        self.declare_parameter('left_class_name', 'lane1')
        self.declare_parameter('right_class_name', 'lane2')
        self.declare_parameter('inference_input', 'raw')
        self.declare_parameter('use_bev_for_geometry', False)
        self.declare_parameter('use_mask_validation', False)
        self.declare_parameter('minimum_mask_ratio', 0.001)
        self.declare_parameter('strict_mask_validation', False)
        self.declare_parameter('mask_fallback_confidence_scale', 0.75)
        self.declare_parameter('use_scanline_validation', True)
        self.declare_parameter('feature_validation_required', False)
        self.declare_parameter('scan_rows', [0.62, 0.70, 0.80, 0.90])
        self.declare_parameter('look_ahead_ratio', 0.80)
        self.declare_parameter('contrast_offset', 35)
        self.declare_parameter('min_line_width_px', 4)
        self.declare_parameter('max_line_width_px', 26)
        self.declare_parameter('far_row_width_scale', 0.6)
        self.declare_parameter('paired_edge_required', True)
        self.declare_parameter('target_mode', 'road_center')
        self.declare_parameter('center_consistency_tol', 0.08)
        self.declare_parameter('lane_half_width_px', 0.0)
        self.declare_parameter('lambda_roi_margin_px', 40)
        self.declare_parameter('lambda_roi_decay_frames', 15)
        self.declare_parameter('track_process_var', 4.0)
        self.declare_parameter('track_measure_var', 25.0)
        self.declare_parameter('max_predicted_frames', 10)

    def _make_feature_tracker(self) -> LaneFeatureTracker:
        return LaneFeatureTracker(
            scan_rows=self.get_parameter('scan_rows').value,
            look_ahead_ratio=float(
                self.get_parameter('look_ahead_ratio').value),
            contrast_offset=float(
                self.get_parameter('contrast_offset').value),
            min_line_width_px=float(
                self.get_parameter('min_line_width_px').value),
            max_line_width_px=float(
                self.get_parameter('max_line_width_px').value),
            far_row_width_scale=float(
                self.get_parameter('far_row_width_scale').value),
            paired_edge_required=bool(
                self.get_parameter('paired_edge_required').value),
            target_mode=str(self.get_parameter('target_mode').value),
            center_consistency_tol=float(
                self.get_parameter('center_consistency_tol').value),
            lane_half_width_px=float(
                self.get_parameter('lane_half_width_px').value),
            lambda_roi_margin_px=float(
                self.get_parameter('lambda_roi_margin_px').value),
            lambda_roi_decay_frames=int(
                self.get_parameter('lambda_roi_decay_frames').value),
            track_process_var=float(
                self.get_parameter('track_process_var').value),
            track_measure_var=float(
                self.get_parameter('track_measure_var').value),
            max_predicted_frames=int(
                self.get_parameter('max_predicted_frames').value),
        )

    @staticmethod
    def _resolve_device(requested: str) -> str:
        requested = requested.strip().lower()
        if requested == 'auto':
            return '0' if torch.cuda.is_available() else 'cpu'
        if requested != 'cpu' and not torch.cuda.is_available():
            return 'cpu'
        return requested

    def _prediction_arguments(self) -> dict:
        arguments = {
            'imgsz': int(self.get_parameter('input_size').value),
            'conf': float(self.get_parameter('confidence_threshold').value),
            'iou': float(self.get_parameter('iou_threshold').value),
            'device': self.device,
            'max_det': int(self.get_parameter('max_detections').value),
            'classes': [self.left_class_id, self.right_class_id],
            'verbose': False,
        }
        if self.half_precision:
            # Ultralytics 8.4 uses quantize=16 for FP16 prediction. Passing
            # the legacy half=True alias emits one warning per frame.
            arguments['quantize'] = 16
        return arguments

    def _warmup_model(self) -> None:
        size = int(self.get_parameter('input_size').value)
        self.model.predict(
            np.zeros((size, size, 3), dtype=np.uint8),
            **self._prediction_arguments(),
        )

    def _class_id(self, class_name: str) -> int:
        names = self.model.names
        items = names.items() if isinstance(names, dict) else enumerate(names)
        for class_id, name in items:
            if str(name) == class_name:
                return int(class_id)
        raise ValueError(f'class {class_name!r} not found in model names={names}')

    @staticmethod
    def _stamp_key(message: Image) -> tuple:
        return (message.header.stamp.sec, message.header.stamp.nanosec)

    def _parameters_to_thresholds(self) -> list:
        values = []
        for parameter in (
            'white_lower_hsv',
            'white_upper_hsv',
            'yellow_lower_hsv',
            'yellow_upper_hsv',
        ):
            values.extend(
                int(value) for value in self.get_parameter(parameter).value)
        return values

    def _publish_thresholds(self) -> None:
        self.threshold_pub.publish(Int32MultiArray(data=self.thresholds))

    def _on_thresholds(self, message: Int32MultiArray) -> None:
        if len(message.data) != 12:
            self.get_logger().warn(
                'HSV threshold update must contain 12 integers')
            return
        values = [int(value) for value in message.data]
        hue_indices = {0, 3, 6, 9}
        for index, value in enumerate(values):
            maximum = 180 if index in hue_indices else 255
            values[index] = max(0, min(maximum, value))
        self.thresholds = values
        self._publish_thresholds()

    def _on_image(self, message: Image) -> None:
        try:
            bgr = image_message_to_numpy(message)
            processed = preprocess_lane_frame(
                bgr,
                roi_top_ratio=float(
                    self.get_parameter('roi_top_ratio').value),
                bev_width=int(self.get_parameter('bev_width').value),
                bev_height=int(self.get_parameter('bev_height').value),
                bev_source=self.get_parameter('bev_source').value,
                bev_destination=self.get_parameter('bev_destination').value,
                white_lower=self.thresholds[0:3],
                white_upper=self.thresholds[3:6],
                yellow_lower=self.thresholds[6:9],
                yellow_upper=self.thresholds[9:12],
                morphology_kernel=int(
                    self.get_parameter('morphology_kernel').value),
            )
        except (ValueError, cv2.error) as error:
            self.get_logger().error(f'lane preprocessing failed: {error}')
            return
        binary = (
            processed.binary
            if bool(self.get_parameter('use_yellow_mask').value)
            else processed.white_mask
        )
        self.latest_input = (
            self._stamp_key(message), bgr, processed, binary, message)
        self.binary_pub.publish(numpy_to_image_message(binary, message))
        if bool(self.get_parameter('publish_bev_debug').value):
            self.bev_pub.publish(numpy_to_image_message(processed.bev, message))
        if bool(self.get_parameter('publish_individual_masks').value):
            self.white_pub.publish(numpy_to_image_message(
                processed.white_mask, message))
            self.yellow_pub.publish(numpy_to_image_message(
                processed.yellow_mask, message))

    def _raw_mask(self, bgr: np.ndarray) -> np.ndarray:
        hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
        white = cv2.inRange(
            hsv,
            np.asarray(self.thresholds[0:3], dtype=np.uint8),
            np.asarray(self.thresholds[3:6], dtype=np.uint8),
        )
        if not bool(self.get_parameter('use_yellow_mask').value):
            return white
        yellow = cv2.inRange(
            hsv,
            np.asarray(self.thresholds[6:9], dtype=np.uint8),
            np.asarray(self.thresholds[9:12], dtype=np.uint8),
        )
        return cv2.bitwise_or(white, yellow)

    @staticmethod
    def _mask_ratio(mask: np.ndarray, coordinates) -> float:
        height, width = mask.shape[:2]
        x1, y1, x2, y2 = coordinates
        x1 = max(0, min(width - 1, int(x1)))
        x2 = max(x1 + 1, min(width, int(x2)))
        y1 = max(0, min(height - 1, int(y1)))
        y2 = max(y1 + 1, min(height, int(y2)))
        region = mask[y1:y2, x1:x2]
        return float(np.count_nonzero(region) / max(1, region.size))

    def _select_detections(self, result, mask: np.ndarray) -> tuple:
        selected = {}
        unvalidated = {}
        details = []
        if result.boxes is None:
            return selected, details
        coordinates = result.boxes.xyxy.cpu().numpy()
        classes = result.boxes.cls.int().cpu().numpy()
        confidences = result.boxes.conf.cpu().numpy()
        use_mask = bool(self.get_parameter('use_mask_validation').value)
        minimum_ratio = float(self.get_parameter('minimum_mask_ratio').value)
        for box, class_id, confidence in zip(
            coordinates, classes, confidences
        ):
            class_id = int(class_id)
            if class_id not in {self.left_class_id, self.right_class_id}:
                continue
            mask_ratio = self._mask_ratio(mask, box)
            accepted = not use_mask or mask_ratio >= minimum_ratio
            details.append({
                'class_id': class_id,
                'class_name': str(self.model.names[class_id]),
                'confidence': round(float(confidence), 4),
                'mask_ratio': round(mask_ratio, 5),
                'accepted': accepted,
                'xyxy': [round(float(value), 1) for value in box],
            })
            candidate = {
                'box': box,
                'confidence': float(confidence),
                'mask_ratio': mask_ratio,
                'mask_fallback': not accepted,
            }
            previous = unvalidated.get(class_id)
            if previous is None or candidate['confidence'] > previous['confidence']:
                unvalidated[class_id] = candidate
            if accepted:
                previous = selected.get(class_id)
                if previous is None or candidate['confidence'] > previous['confidence']:
                    selected[class_id] = candidate
        if use_mask and not bool(
            self.get_parameter('strict_mask_validation').value
        ):
            scale = float(self.get_parameter(
                'mask_fallback_confidence_scale').value)
            for class_id, candidate in unvalidated.items():
                if class_id not in selected:
                    candidate = dict(candidate)
                    candidate['confidence'] *= scale
                    selected[class_id] = candidate
        return selected, details

    @staticmethod
    def _mapped_box(box, processed) -> list:
        x1, y1, x2, y2 = (float(value) for value in box)
        corners = np.asarray([
            [x1, y1], [x2, y1], [x2, y2], [x1, y2]], dtype=np.float32)
        restored = bev_points_to_image(corners, processed)
        return [
            float(np.min(restored[:, 0])),
            float(np.min(restored[:, 1])),
            float(np.max(restored[:, 0])),
            float(np.max(restored[:, 1])),
        ]

    @staticmethod
    def _put_text(image, text: str, origin, color=(255, 255, 255)) -> None:
        cv2.putText(
            image, text, origin, cv2.FONT_HERSHEY_SIMPLEX,
            0.48, color, 1, cv2.LINE_AA)

    def _draw_overlay(self, bgr: np.ndarray, geometry: dict, valid: bool) -> np.ndarray:
        overlay = bgr.copy()
        height, width = overlay.shape[:2]
        for key, color, label in (
            ('left_box', (255, 120, 40), 'lane1'),
            ('right_box', (0, 220, 255), 'lane2'),
        ):
            box = geometry.get(key)
            if box is None:
                continue
            x1, y1, x2, y2 = (int(round(value)) for value in box)
            cv2.rectangle(overlay, (x1, y1), (x2, y2), color, 1)
            self._put_text(overlay, label, (max(0, x1 + 3), max(15, y1 + 15)), color)

        center_x = int(round(geometry['image_center_x']))
        cv2.line(overlay, (center_x, 0), (center_x, height - 1), (0, 140, 255), 2)
        target_points = []
        virtual_points = []
        for row in geometry.get('scan_rows', []):
            y = int(row['y'])
            row_color = (120, 120, 70)
            thickness = 2 if row.get('look_ahead') else 1
            cv2.line(overlay, (0, y), (width - 1, y), row_color, thickness)
            for role in ('left', 'center', 'right'):
                point = row.get('points', {}).get(role)
                if point is None:
                    continue
                x = int(round(np.clip(point['x'], 0, width - 1)))
                if point['source'] == 'measured':
                    run_width = max(2, int(round(point.get('width_px') or 2)))
                    cv2.line(
                        overlay, (x - run_width // 2, y),
                        (x + run_width // 2, y), (40, 240, 40), 3)
                    cv2.circle(overlay, (x, y), 4, (40, 240, 40), -1)
                else:
                    cv2.circle(overlay, (x, y), 5, (0, 220, 255), 2)
                    self._put_text(overlay, 'P', (x + 5, y - 4), (0, 220, 255))
            if row.get('target_x') is not None:
                x = int(round(np.clip(row['target_x'], 0, width - 1)))
                target_points.append((x, y))
                if row.get('source') == 'box_fallback':
                    cv2.line(overlay, (x - 5, y - 5), (x + 5, y + 5), (170, 170, 170), 2)
                    cv2.line(overlay, (x - 5, y + 5), (x + 5, y - 5), (170, 170, 170), 2)
            if row.get('target_source') == 'ONE_EDGE':
                points = row.get('points', {})
                assumed = geometry.get('single_side_assumed_px', 0.0)
                assumed *= width / 640.0
                if points.get('left') is not None:
                    virtual_points.append((int(points['left']['x'] + assumed), y))
                elif points.get('right') is not None:
                    virtual_points.append((int(points['right']['x'] - assumed), y))
        if len(target_points) >= 2:
            cv2.polylines(
                overlay, [np.asarray(target_points, dtype=np.int32)],
                False, (30, 255, 30), 4, cv2.LINE_AA)
        for index in range(1, len(virtual_points)):
            if index % 2:
                cv2.line(
                    overlay, virtual_points[index - 1], virtual_points[index],
                    (150, 150, 150), 2, cv2.LINE_AA)
        look_rows = [row for row in geometry.get('scan_rows', []) if row.get('look_ahead')]
        if look_rows and look_rows[0].get('target_x') is not None:
            row = look_rows[0]
            target_x = int(round(row['target_x']))
            band = overlay.copy()
            cv2.rectangle(
                band,
                (min(center_x, target_x), max(0, int(row['y']) - 8)),
                (max(center_x, target_x), min(height - 1, int(row['y']) + 8)),
                (60, 80, 220),
                -1,
            )
            overlay = cv2.addWeighted(band, 0.30, overlay, 0.70, 0.0)
        if geometry.get('consistency_warning'):
            cv2.rectangle(overlay, (0, 0), (width, 25), (20, 20, 180), -1)
            self._put_text(overlay, 'CENTER CONSISTENCY WARNING', (8, 18), (255, 255, 255))
        if not valid:
            cv2.rectangle(overlay, (1, 1), (width - 2, height - 2), (0, 0, 255), 5)
            lost = 0.0 if self.invalid_since is None else perf_counter() - self.invalid_since
            self._put_text(overlay, f'LOST {lost:.2f}s', (8, height - 12), (0, 0, 255))
        return overlay

    def _publish_result(
        self,
        source: Image,
        annotated: np.ndarray,
        overlay: np.ndarray,
        geometry: dict,
        details: list,
        confidence: float,
        valid: bool,
    ) -> None:
        if valid:
            self.invalid_since = None
        elif self.invalid_since is None:
            self.invalid_since = perf_counter()
        if not valid:
            overlay = self._draw_overlay(
                image_message_to_numpy(source), geometry, False)
        if bool(self.get_parameter('publish_lane_geometry').value):
            self.geometry_pub.publish(String(data=json.dumps(geometry)))
        heading = geometry.get('heading_error')
        if heading is not None:
            self.heading_pub.publish(Float32(data=float(heading)))
        self.detections_pub.publish(String(data=json.dumps(details)))
        self.debug_pub.publish(numpy_to_image_message(annotated, source))
        if bool(self.get_parameter('publish_lane_overlay').value):
            self.overlay_pub.publish(numpy_to_image_message(overlay, source))
        # Publish validity/confidence before error so the controller consumes a
        # synchronized sample when its error callback runs.
        self.valid_pub.publish(Bool(data=bool(valid)))
        self.confidence_pub.publish(Float32(data=float(confidence)))
        error = geometry.get('center_error') if valid else 0.0
        self.error_pub.publish(Float32(data=float(error or 0.0)))

    def _run_inference(self) -> None:
        if self.latest_input is None:
            return
        stamp, bgr, processed, bev_mask, source = self.latest_input
        if stamp == self.last_processed_stamp:
            return
        self.last_processed_stamp = stamp
        inference_input = str(
            self.get_parameter('inference_input').value).strip().lower()
        if inference_input not in {'raw', 'bev'}:
            self.get_logger().warn(
                f'Unknown inference_input={inference_input}; using raw',
                throttle_duration_sec=5.0)
            inference_input = 'raw'
        inference_image = bgr if inference_input == 'raw' else processed.bev
        validation_mask = self._raw_mask(bgr) if inference_input == 'raw' else bev_mask
        started_at = perf_counter()
        result = self.model.predict(
            inference_image, **self._prediction_arguments())[0]
        inference_ms = (perf_counter() - started_at) * 1000.0
        self.inference_ms_pub.publish(Float32(data=float(inference_ms)))
        annotated = result.plot(line_width=2)
        cv2.rectangle(annotated, (0, 0), (260, 30), (20, 20, 20), -1)
        self._put_text(
            annotated,
            f'{inference_input.upper()} {inference_ms:.1f}ms',
            (8, 21),
            (0, 255, 255),
        )

        selected, details = self._select_detections(result, validation_mask)
        left = selected.get(self.left_class_id)
        right = selected.get(self.right_class_id)
        left_box = None if left is None else left['box']
        right_box = None if right is None else right['box']
        if inference_input == 'bev':
            left_box = None if left_box is None else self._mapped_box(left_box, processed)
            right_box = None if right_box is None else self._mapped_box(right_box, processed)

        gray = cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY)
        if left_box is None and right_box is None:
            feature_image = np.zeros_like(gray)
        else:
            feature_image = gray
        geometry = self.feature_tracker.process(
            feature_image, left_box, right_box)
        geometry['inference_input'] = inference_input
        geometry['inference_ms'] = inference_ms
        if not bool(self.get_parameter('use_scanline_validation').value):
            geometry['feature_validated'] = False
            geometry['target_source'] = 'BOX'

        detections = [item for item in (left, right) if item is not None]
        yolo_confidence = (
            sum(item['confidence'] for item in detections) / len(detections)
            if detections else 0.0
        )
        confidence = yolo_confidence * float(
            geometry.get('feature_confidence_scale', 0.0))
        valid = geometry.get('center_error') is not None and bool(detections)
        if bool(self.get_parameter('feature_validation_required').value):
            valid = valid and bool(geometry.get('feature_validated'))
        overlay = self._draw_overlay(bgr, geometry, valid)
        self._publish_result(
            source,
            annotated,
            overlay,
            geometry,
            details,
            confidence,
            valid,
        )


def main(args=None) -> None:
    """Run the YOLO lane detector node."""
    rclpy.init(args=args)
    node = YoloLaneDetectorNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except Exception:
        if rclpy.ok():
            raise
    finally:
        try:
            node.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()
        except KeyboardInterrupt:
            pass


if __name__ == '__main__':
    main()

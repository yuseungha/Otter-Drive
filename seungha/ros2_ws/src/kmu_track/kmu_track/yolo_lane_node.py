"""YOLOv8 lane detection and center-error calculation on BEV frames."""

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

from kmu_track.lane_core import preprocess_lane_frame


def image_message_to_numpy(message: Image) -> np.ndarray:
    """Convert bgr8 or mono8 without cv_bridge's NumPy 1.x binary ABI."""
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
    """Build a ROS bgr8 or mono8 image without importing cv_bridge."""
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
    """Preprocess camera frames and run road_best.pt without image relays."""

    def __init__(self) -> None:
        super().__init__('yolo_lane_detector')
        self.declare_parameter('model_path', '')
        self.declare_parameter('image_topic', '/camera/front/image_raw')
        self.declare_parameter('roi_top_ratio', 0.45)
        self.declare_parameter('bev_width', 320)
        self.declare_parameter('bev_height', 240)
        self.declare_parameter(
            'bev_source', [0.05, 0.98, 0.95, 0.98, 0.62, 0.05, 0.38, 0.05])
        self.declare_parameter(
            'bev_destination', [0.20, 0.99, 0.80, 0.99, 0.80, 0.0, 0.20, 0.0])
        self.declare_parameter('white_lower_hsv', [0, 0, 175])
        self.declare_parameter('white_upper_hsv', [180, 80, 255])
        self.declare_parameter('yellow_lower_hsv', [12, 70, 70])
        self.declare_parameter('yellow_upper_hsv', [42, 255, 255])
        self.declare_parameter('morphology_kernel', 3)
        self.declare_parameter('publish_bev_debug', False)
        self.declare_parameter('publish_individual_masks', False)
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
        self.declare_parameter('lane_half_width_px', 142.0)
        self.declare_parameter('use_mask_validation', True)
        self.declare_parameter('minimum_mask_ratio', 0.001)
        self.declare_parameter('strict_mask_validation', False)
        self.declare_parameter('mask_fallback_confidence_scale', 0.75)

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
        self.latest_input = None
        self.last_processed_stamp = None

        self.error_pub = self.create_publisher(Float32, '/lane/center_error', 10)
        self.valid_pub = self.create_publisher(Bool, '/lane/valid', 10)
        self.confidence_pub = self.create_publisher(
            Float32, '/lane/confidence', 10)
        self.inference_ms_pub = self.create_publisher(
            Float32, '/lane/inference_ms', 10)
        self.detections_pub = self.create_publisher(
            String, '/lane/yolo_detections', 10)
        self.debug_pub = self.create_publisher(
            Image, '/lane/yolo_debug', qos_profile_sensor_data)
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
        self.create_subscription(
            Image,
            str(self.get_parameter('image_topic').value),
            self._on_image,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            Int32MultiArray, '/lane/hsv_thresholds/set', self._on_thresholds, 10
        )
        rate = max(0.2, float(self.get_parameter('inference_rate_hz').value))
        self.timer = self.create_timer(1.0 / rate, self._run_inference)
        self.get_logger().info(
            f'YOLO lane detector ready: {model_path} | '
            f'classes={self.model.names} | device={self.device} | '
            f'FP16={self.half_precision} | '
            f'imgsz={self.get_parameter("input_size").value}')
        self._publish_thresholds()

    @staticmethod
    def _resolve_device(requested: str) -> str:
        requested = requested.strip().lower()
        if requested == 'auto':
            return '0' if torch.cuda.is_available() else 'cpu'
        if requested != 'cpu' and not torch.cuda.is_available():
            return 'cpu'
        return requested

    def _prediction_arguments(self):
        return {
            'imgsz': int(self.get_parameter('input_size').value),
            'conf': float(self.get_parameter('confidence_threshold').value),
            'iou': float(self.get_parameter('iou_threshold').value),
            'device': self.device,
            'half': self.half_precision,
            'max_det': int(self.get_parameter('max_detections').value),
            'classes': [self.left_class_id, self.right_class_id],
            'verbose': False,
        }

    def _warmup_model(self) -> None:
        input_size = int(self.get_parameter('input_size').value)
        sample = np.zeros((input_size, input_size, 3), dtype=np.uint8)
        self.model.predict(sample, **self._prediction_arguments())

    def _class_id(self, class_name: str) -> int:
        names = self.model.names
        items = names.items() if isinstance(names, dict) else enumerate(names)
        for class_id, name in items:
            if str(name) == class_name:
                return int(class_id)
        raise ValueError(f'class {class_name!r} not found in model names={names}')

    @staticmethod
    def _stamp_key(message: Image):
        return (message.header.stamp.sec, message.header.stamp.nanosec)

    def _parameters_to_thresholds(self):
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
        self.latest_input = (
            self._stamp_key(message),
            processed.bev,
            processed.binary,
            message,
        )
        self.binary_pub.publish(numpy_to_image_message(
            processed.binary, message))
        if bool(self.get_parameter('publish_bev_debug').value):
            self.bev_pub.publish(numpy_to_image_message(
                processed.bev, message))
        if bool(self.get_parameter('publish_individual_masks').value):
            self.white_pub.publish(numpy_to_image_message(
                processed.white_mask, message))
            self.yellow_pub.publish(numpy_to_image_message(
                processed.yellow_mask, message))

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

    def _select_detections(self, result, mask: np.ndarray):
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
            if int(class_id) not in {self.left_class_id, self.right_class_id}:
                continue
            mask_ratio = self._mask_ratio(mask, box)
            accepted = not use_mask or mask_ratio >= minimum_ratio
            detail = {
                'class_id': int(class_id),
                'class_name': str(self.model.names[int(class_id)]),
                'confidence': round(float(confidence), 4),
                'mask_ratio': round(mask_ratio, 5),
                'accepted': accepted,
                'xyxy': [round(float(value), 1) for value in box],
            }
            details.append(detail)
            previous_unvalidated = unvalidated.get(int(class_id))
            if (
                previous_unvalidated is None
                or float(confidence) > previous_unvalidated['confidence']
            ):
                unvalidated[int(class_id)] = {
                    'box': box,
                    'confidence': float(confidence),
                    'mask_ratio': mask_ratio,
                    'mask_fallback': True,
                }
            if not accepted:
                continue
            previous = selected.get(int(class_id))
            if previous is None or float(confidence) > previous['confidence']:
                selected[int(class_id)] = {
                    'box': box,
                    'confidence': float(confidence),
                    'mask_ratio': mask_ratio,
                    'mask_fallback': False,
                }
        strict_mask = bool(
            self.get_parameter('strict_mask_validation').value)
        if use_mask and not strict_mask:
            confidence_scale = float(self.get_parameter(
                'mask_fallback_confidence_scale').value)
            for class_id, candidate in unvalidated.items():
                if class_id in selected:
                    continue
                candidate['confidence'] *= confidence_scale
                selected[class_id] = candidate
        return selected, details

    def _publish_invalid(self, source: Image, annotated, details) -> None:
        self.error_pub.publish(Float32(data=0.0))
        self.valid_pub.publish(Bool(data=False))
        self.confidence_pub.publish(Float32(data=0.0))
        self.detections_pub.publish(String(data=json.dumps(details)))
        self.debug_pub.publish(numpy_to_image_message(annotated, source))

    def _run_inference(self) -> None:
        if self.latest_input is None:
            return
        stamp, bev, mask, source = self.latest_input
        if stamp == self.last_processed_stamp:
            return
        self.last_processed_stamp = stamp
        started_at = perf_counter()
        result = self.model.predict(
            bev, **self._prediction_arguments())[0]
        inference_ms = (perf_counter() - started_at) * 1000.0
        self.inference_ms_pub.publish(Float32(data=float(inference_ms)))
        annotated = result.plot(line_width=2)
        cv2.rectangle(annotated, (0, 0), (230, 30), (20, 20, 20), -1)
        cv2.putText(
            annotated,
            f'{inference_ms:.1f} ms | {1000.0 / max(inference_ms, 0.1):.1f} FPS',
            (8, 21),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.55,
            (0, 255, 255),
            1,
            cv2.LINE_AA,
        )
        selected, details = self._select_detections(result, mask)
        left = selected.get(self.left_class_id)
        right = selected.get(self.right_class_id)
        if left is None and right is None:
            self._publish_invalid(source, annotated, details)
            return

        half_width = float(self.get_parameter('lane_half_width_px').value)
        left_x = None if left is None else float(
            (left['box'][0] + left['box'][2]) / 2.0)
        right_x = None if right is None else float(
            (right['box'][0] + right['box'][2]) / 2.0)
        if left_x is not None and right_x is not None:
            lane_center = (left_x + right_x) / 2.0
            confidence = (left['confidence'] + right['confidence']) / 2.0
        elif left_x is not None:
            lane_center = left_x + half_width
            confidence = left['confidence'] * 0.7
        else:
            lane_center = right_x - half_width
            confidence = right['confidence'] * 0.7

        image_center = bev.shape[1] / 2.0
        center_error = float(np.clip(
            (lane_center - image_center) / max(1.0, image_center), -1.0, 1.0))
        cv2.line(
            annotated,
            (int(image_center), 0),
            (int(image_center), annotated.shape[0] - 1),
            (255, 120, 0),
            2,
        )
        cv2.line(
            annotated,
            (int(lane_center), 0),
            (int(lane_center), annotated.shape[0] - 1),
            (0, 255, 0),
            3,
        )
        self.error_pub.publish(Float32(data=center_error))
        self.valid_pub.publish(Bool(data=True))
        self.confidence_pub.publish(Float32(data=float(confidence)))
        self.detections_pub.publish(String(data=json.dumps(details)))
        self.debug_pub.publish(numpy_to_image_message(annotated, source))


def main(args=None) -> None:
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

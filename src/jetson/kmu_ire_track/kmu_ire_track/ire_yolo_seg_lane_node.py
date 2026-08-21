"""ROS adapter for the IRE center-priority segmentation lane planner."""

from __future__ import annotations

import json
from pathlib import Path
from time import perf_counter

import cv2
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path as NavPath
import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from sensor_msgs.msg import Image
from std_msgs.msg import Bool, Float32, String
import torch
from ultralytics import YOLO

from kmu_ire_track.ire_latest_camera import LatestFrameCamera
from kmu_ire_track.ire_segmentation_lane_core import (
    SegmentationInstance,
    SegmentationLaneConfig,
    SegmentationLanePlanner,
    normalized_roi_bounds,
)
from kmu_track.lane_path_core import (
    LanePathProjectionConfig,
    LanePathProjector,
)


def image_message_to_numpy(message: Image) -> np.ndarray:
    """Convert tightly packed or padded bgr8/rgb8 images without cv_bridge."""
    data = np.frombuffer(message.data, dtype=np.uint8)
    rows = data.reshape(message.height, message.step)
    if message.encoding not in {'bgr8', 'rgb8'}:
        raise ValueError(f'unsupported image encoding: {message.encoding}')
    image = rows[:, :message.width * 3].reshape(
        message.height, message.width, 3)
    if message.encoding == 'rgb8':
        image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
    return np.ascontiguousarray(image)


def latest_image_qos() -> QoSProfile:
    """Keep one best-effort image so stale frames cannot queue up."""
    return QoSProfile(
        history=HistoryPolicy.KEEP_LAST,
        depth=1,
        reliability=ReliabilityPolicy.BEST_EFFORT,
        durability=DurabilityPolicy.VOLATILE,
    )


def numpy_to_image_message(image: np.ndarray, source: Image) -> Image:
    """Build a bgr8 or mono8 ROS image while preserving the source header."""
    contiguous = np.ascontiguousarray(image, dtype=np.uint8)
    output = Image()
    output.header = source.header
    output.height = int(contiguous.shape[0])
    output.width = int(contiguous.shape[1])
    channels = 1 if contiguous.ndim == 2 else 3
    output.encoding = 'mono8' if channels == 1 else 'bgr8'
    output.is_bigendian = 0
    output.step = output.width * channels
    output.data = contiguous.tobytes()
    return output


class IreYoloSegmentationLaneNode(Node):
    """Infer masks and publish a center-marking-first IRE lane path."""

    def __init__(self) -> None:
        super().__init__('ire_yolo_seg_lane_detector')
        self._declare_parameters()
        model_path = Path(str(self.get_parameter('model_path').value))
        if not model_path.is_file():
            raise FileNotFoundError(
                f'YOLO segmentation model does not exist: {model_path}')
        self.model = YOLO(str(model_path))
        if self.model.task != 'segment':
            raise ValueError(
                f'expected a YOLO segmentation model, got {self.model.task}')

        self.center_class_name = str(
            self.get_parameter('center_class_name').value)
        self.boundary_class_name = str(
            self.get_parameter('boundary_class_name').value)
        self.center_class_id = self._class_id(self.center_class_name)
        self.boundary_class_id = self._class_id(self.boundary_class_name)
        self.device = self._resolve_device(
            str(self.get_parameter('device').value))
        self.half_precision = bool(
            self.get_parameter('half_precision').value
        ) and self.device != 'cpu'
        if self.device != 'cpu':
            torch.backends.cudnn.benchmark = True

        self.roi_enabled = bool(self.get_parameter('roi_enabled').value)
        self.roi_edges = tuple(float(self.get_parameter(name).value) for name in (
            'roi_left_ratio',
            'roi_top_ratio',
            'roi_right_ratio',
            'roi_bottom_ratio',
        ))
        normalized_roi_bounds(
            (
                int(self.get_parameter('camera_height').value),
                int(self.get_parameter('camera_width').value),
            ),
            *self.roi_edges,
        )

        self.planner = SegmentationLanePlanner(SegmentationLaneConfig(
            center_class_name=self.center_class_name,
            boundary_class_name=self.boundary_class_name,
            scan_rows=tuple(float(value) for value in self.get_parameter(
                'scan_rows').value),
            scan_band_half_height=int(self.get_parameter(
                'scan_band_half_height').value),
            mask_threshold=float(self.get_parameter('mask_threshold').value),
            min_lane_width_ratio=float(self.get_parameter(
                'min_lane_width_ratio').value),
            min_mask_pixels_per_row=int(self.get_parameter(
                'min_mask_pixels_per_row').value),
            min_valid_rows=int(self.get_parameter('min_valid_rows').value),
            min_vertical_span_ratio=float(self.get_parameter(
                'min_vertical_span_ratio').value),
            max_lookahead_extrapolation_ratio=float(self.get_parameter(
                'max_lookahead_extrapolation_ratio').value),
            look_ahead_ratio=float(self.get_parameter(
                'look_ahead_ratio').value),
            vehicle_center_x_ratio=float(self.get_parameter(
                'vehicle_center_x_ratio').value),
            heading_far_ratio=float(self.get_parameter(
                'heading_far_ratio').value),
            heading_near_ratio=float(self.get_parameter(
                'heading_near_ratio').value),
            center_consistency_tol=float(self.get_parameter(
                'center_consistency_tol').value),
            minimum_plan_confidence=float(self.get_parameter(
                'minimum_plan_confidence').value),
            memory_max_frames=int(self.get_parameter(
                'memory_max_frames').value),
            memory_confidence_decay=float(self.get_parameter(
                'memory_confidence_decay').value),
        ))
        self.planning_frame = str(
            self.get_parameter('planning_frame').value).strip()
        if not self.planning_frame:
            raise ValueError('planning_frame cannot be empty')
        self.path_projector = LanePathProjector(LanePathProjectionConfig(**{
            name: float(self.get_parameter(name).value)
            for name in LanePathProjectionConfig.__dataclass_fields__
        }))

        self.latest_input = None
        self.last_processed_stamp = None
        self.camera = None
        self._create_publishers()
        camera_device = str(self.get_parameter('camera_device').value)
        if camera_device:
            self.camera = LatestFrameCamera(
                device=camera_device,
                width=int(self.get_parameter('camera_width').value),
                height=int(self.get_parameter('camera_height').value),
                fps=float(self.get_parameter('camera_fps').value),
                fourcc=str(self.get_parameter('camera_fourcc').value),
                reconnect_interval_sec=float(self.get_parameter(
                    'camera_reconnect_interval_sec').value),
                logger=self.get_logger(),
            )
        else:
            self.create_subscription(
                Image,
                str(self.get_parameter('image_topic').value),
                self._on_image,
                latest_image_qos(),
            )
        rate = max(0.2, float(self.get_parameter('inference_rate_hz').value))
        self.timer = self.create_timer(1.0 / rate, self._run_inference)
        if bool(self.get_parameter('warmup').value):
            self._warmup_model()
        if self.camera is not None:
            self.camera.start()
        self.ready_pub.publish(Bool(data=True))
        self.get_logger().info(
            f'IRE center-priority lane planner ready: {model_path} | '
            f'classes={self.model.names} | device={self.device} | '
            f'input={"direct-camera" if self.camera is not None else "ROS"} | '
            f'roi={self.roi_edges if self.roi_enabled else "disabled"}')

    def _declare_parameters(self) -> None:
        self.declare_parameter('image_topic', '/camera/front/image_raw')
        self.declare_parameter('camera_device', '')
        self.declare_parameter('camera_frame_id', 'front_camera')
        self.declare_parameter('camera_width', 1920)
        self.declare_parameter('camera_height', 1080)
        self.declare_parameter('camera_fps', 15.0)
        self.declare_parameter('camera_fourcc', 'MJPG')
        self.declare_parameter('camera_reconnect_interval_sec', 1.0)
        self.declare_parameter('roi_enabled', False)
        self.declare_parameter('roi_left_ratio', 0.0)
        self.declare_parameter('roi_top_ratio', 0.0)
        self.declare_parameter('roi_right_ratio', 1.0)
        self.declare_parameter('roi_bottom_ratio', 1.0)
        self.declare_parameter('model_path', '')
        self.declare_parameter('confidence_threshold', 0.25)
        self.declare_parameter('iou_threshold', 0.45)
        self.declare_parameter('input_size', 768)
        self.declare_parameter('device', 'auto')
        self.declare_parameter('half_precision', True)
        self.declare_parameter('inference_rate_hz', 15.0)
        self.declare_parameter('max_detections', 12)
        self.declare_parameter('warmup', True)
        self.declare_parameter('center_class_name', 'center')
        self.declare_parameter('boundary_class_name', 'lane')
        self.declare_parameter(
            'scan_rows', [
                0.02, 0.10, 0.18, 0.26, 0.34, 0.42, 0.50,
                0.58, 0.66, 0.74, 0.82, 0.90, 0.98,
            ])
        self.declare_parameter('scan_band_half_height', 5)
        self.declare_parameter('mask_threshold', 0.50)
        self.declare_parameter('min_lane_width_ratio', 0.10)
        self.declare_parameter('min_mask_pixels_per_row', 3)
        self.declare_parameter('min_valid_rows', 3)
        self.declare_parameter('min_vertical_span_ratio', 0.15)
        self.declare_parameter('max_lookahead_extrapolation_ratio', 0.05)
        self.declare_parameter('look_ahead_ratio', 0.62)
        self.declare_parameter('vehicle_center_x_ratio', 0.50)
        self.declare_parameter('heading_far_ratio', 0.58)
        self.declare_parameter('heading_near_ratio', 0.90)
        self.declare_parameter('center_consistency_tol', 0.10)
        self.declare_parameter('minimum_plan_confidence', 0.25)
        self.declare_parameter('memory_max_frames', 24)
        self.declare_parameter('memory_confidence_decay', 1.0)
        self.declare_parameter('publish_lane_overlay', False)
        self.declare_parameter('overlay_output_width', 640)
        self.declare_parameter('publish_yolo_debug', False)
        self.declare_parameter('publish_binary_debug', False)
        self.declare_parameter('publish_lane_geometry', True)
        self.declare_parameter('planning_frame', 'base_link')
        for name, field in (
            LanePathProjectionConfig.__dataclass_fields__.items()
        ):
            self.declare_parameter(name, float(field.default))

    def _create_publishers(self) -> None:
        self.error_pub = self.create_publisher(
            Float32, '/lane/center_error', 10)
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
        self.result_pub = self.create_publisher(
            String, '/perception/lane_result', 10)
        self.path_pub = self.create_publisher(
            NavPath, '/planning/lane_path', 10)
        self.lane_valid_event_pub = self.create_publisher(
            Bool, '/perception/lane_valid', 10)
        self.debug_pub = None
        if bool(self.get_parameter('publish_yolo_debug').value):
            self.debug_pub = self.create_publisher(
                Image, '/lane/yolo_debug', latest_image_qos())
        self.overlay_pub = None
        if bool(self.get_parameter('publish_lane_overlay').value):
            self.overlay_pub = self.create_publisher(
                Image, '/lane/lane_overlay', latest_image_qos())
        self.binary_pub = None
        if bool(self.get_parameter('publish_binary_debug').value):
            self.binary_pub = self.create_publisher(
                Image, '/lane/debug_binary', latest_image_qos())
        ready_qos = QoSProfile(depth=1)
        ready_qos.reliability = ReliabilityPolicy.RELIABLE
        ready_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.ready_pub = self.create_publisher(
            Bool, '/lane/detector_ready', ready_qos)

    def _class_id(self, class_name: str) -> int:
        names = self.model.names
        items = names.items() if isinstance(names, dict) else enumerate(names)
        for class_id, name in items:
            if str(name) == class_name:
                return int(class_id)
        raise ValueError(
            f'class {class_name!r} not found in model names={names}')

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
            'classes': [self.center_class_id, self.boundary_class_id],
            'verbose': False,
        }
        if self.half_precision:
            arguments['quantize'] = 16
        return arguments

    def _warmup_model(self) -> None:
        size = int(self.get_parameter('input_size').value)
        self.model.predict(
            np.zeros((size, size, 3), dtype=np.uint8),
            **self._prediction_arguments(),
        )

    @staticmethod
    def _stamp_key(message: Image) -> tuple[int, int]:
        return message.header.stamp.sec, message.header.stamp.nanosec

    def _on_image(self, message: Image) -> None:
        try:
            bgr = image_message_to_numpy(message)
        except (ValueError, cv2.error) as error:
            self.get_logger().error(f'image conversion failed: {error}')
            return
        self.latest_input = (self._stamp_key(message), bgr, message)

    def _latest_inference_input(self):
        if self.camera is None:
            return self.latest_input
        latest = self.camera.latest()
        if latest is None:
            return None
        sequence, captured_at_ns, bgr = latest
        source = Image()
        source.header.frame_id = str(
            self.get_parameter('camera_frame_id').value)
        source.header.stamp.sec = captured_at_ns // 1_000_000_000
        source.header.stamp.nanosec = captured_at_ns % 1_000_000_000
        return ('camera', sequence), bgr, source

    def _instances_from_result(
        self, result, image_shape: tuple[int, ...]
    ) -> tuple[list[SegmentationInstance], list[dict]]:
        if result.boxes is None or result.masks is None:
            return [], []
        masks = result.masks.data.float().cpu().numpy()
        classes = result.boxes.cls.int().cpu().numpy()
        confidences = result.boxes.conf.float().cpu().numpy()
        coordinates = result.boxes.xyxy.float().cpu().numpy()
        height, width = image_shape[:2]
        instances = []
        details = []
        for mask, class_id, confidence, box in zip(
            masks, classes, confidences, coordinates
        ):
            if mask.shape != (height, width):
                mask = cv2.resize(
                    mask, (width, height), interpolation=cv2.INTER_LINEAR)
            class_name = str(self.model.names[int(class_id)])
            instances.append(SegmentationInstance(
                class_name=class_name,
                confidence=float(confidence),
                mask=mask,
            ))
            details.append({
                'class_id': int(class_id),
                'class_name': class_name,
                'confidence': round(float(confidence), 4),
                'mask_ratio': round(float(np.mean(
                    mask >= self.planner.config.mask_threshold)), 5),
                'xyxy': [round(float(value), 1) for value in box],
            })
        return instances, details

    def _combined_mask(
        self,
        instances: list[SegmentationInstance],
        image_shape: tuple[int, ...],
    ) -> np.ndarray:
        combined = np.zeros(image_shape[:2], dtype=np.uint8)
        for instance in instances:
            combined[instance.mask >= self.planner.config.mask_threshold] = 255
        return combined

    @staticmethod
    def _put_text(image: np.ndarray, text: str, origin, color) -> None:
        cv2.putText(
            image, text, origin, cv2.FONT_HERSHEY_SIMPLEX,
            0.48, color, 1, cv2.LINE_AA)

    def _draw_overlay(
        self,
        bgr: np.ndarray,
        instances: list[SegmentationInstance],
        geometry: dict,
        inference_ms: float,
    ) -> np.ndarray:
        source_height, source_width = bgr.shape[:2]
        output_width = int(self.get_parameter('overlay_output_width').value)
        if output_width <= 0 or output_width >= source_width:
            output_width = source_width
            output_height = source_height
            overlay = bgr.copy()
        else:
            output_height = max(
                1, int(round(source_height * output_width / source_width)))
            overlay = cv2.resize(
                bgr, (output_width, output_height),
                interpolation=cv2.INTER_AREA)
        x_scale = output_width / source_width
        y_scale = output_height / source_height
        for instance in instances:
            mask = instance.mask
            if mask.shape != (output_height, output_width):
                mask = cv2.resize(
                    mask, (output_width, output_height),
                    interpolation=cv2.INTER_NEAREST)
            pixels = mask >= self.planner.config.mask_threshold
            color = np.asarray(
                (255, 180, 30)
                if instance.class_name == self.boundary_class_name
                else (40, 230, 40),
                dtype=np.float32,
            )
            overlay[pixels] = np.clip(
                overlay[pixels].astype(np.float32) * 0.55 + color * 0.45,
                0,
                255,
            ).astype(np.uint8)

        height, width = overlay.shape[:2]
        center_x = int(round(float(geometry.get(
            'vehicle_center_x', source_width * 0.5)) * x_scale))
        cv2.line(
            overlay, (center_x, 0), (center_x, height - 1),
            (0, 140, 255), 1)
        cv2.circle(
            overlay, (center_x, height - 1), 7, (0, 140, 255), -1)
        for row in geometry.get('scan_rows', []):
            y = int(round(float(row['y']) * y_scale))
            y = int(np.clip(y, 0, height - 1))
            cv2.line(overlay, (0, y), (width - 1, y), (90, 90, 60), 1)
            for key, color in (
                ('left_x', (255, 180, 30)),
                ('center_x', (40, 230, 40)),
                ('right_x', (255, 180, 30)),
            ):
                value = row.get(key)
                if value is not None:
                    cv2.circle(
                        overlay,
                        (int(round(float(value) * x_scale)), y),
                        3,
                        color,
                        -1,
                    )
        path = [
            (
                int(round(float(point['x']) * x_scale)),
                int(round(float(point['y']) * y_scale)),
            )
            for point in geometry.get('fit_path', [])
        ]
        if len(path) >= 2:
            cv2.polylines(
                overlay, [np.asarray(path, dtype=np.int32)],
                False, (30, 255, 255), 3, cv2.LINE_AA)
        target_x = geometry.get('lane_center_x')
        if target_x is not None:
            target_y = int(round(
                geometry['look_ahead_ratio'] * (height - 1)))
            cv2.circle(
                overlay,
                (int(round(float(target_x) * x_scale)), target_y),
                7, (0, 0, 255), 2)
            cv2.line(
                overlay, (center_x, target_y),
                (int(round(float(target_x) * x_scale)), target_y),
                (0, 0, 255), 2)

        valid = bool(geometry.get('valid'))
        status_color = (40, 220, 40) if valid else (0, 0, 255)
        error = geometry.get('center_error')
        self._put_text(
            overlay,
            f'{"VALID" if valid else "LOST"} '
            f'src={geometry.get("target_source", "NONE")} '
            f'err={0.0 if error is None else error:+.3f}',
            (8, 20), status_color,
        )
        self._put_text(
            overlay,
            f'conf={geometry.get("confidence", 0.0):.2f} '
            f'rows={geometry.get("valid_rows", 0)} '
            f'{inference_ms:.1f}ms',
            (8, 40), status_color,
        )
        if geometry.get('consistency_warning'):
            self._put_text(
                overlay, 'CENTER / LANE DISAGREE', (8, 60), (0, 0, 255))
        if not valid:
            cv2.rectangle(
                overlay, (1, 1), (width - 2, height - 2), (0, 0, 255), 4)
        return overlay

    def _publish_plan(
        self,
        source: Image,
        annotated: np.ndarray | None,
        overlay: np.ndarray | None,
        binary: np.ndarray | None,
        geometry: dict,
        details: list[dict],
        inference_ms: float,
    ) -> None:
        valid = bool(geometry.get('valid'))
        confidence = float(geometry.get('confidence', 0.0))
        path_points = self.path_projector.project_geometry(geometry)
        lane_path = NavPath()
        lane_path.header.stamp = source.header.stamp
        lane_path.header.frame_id = self.planning_frame
        for x_m, y_m in path_points:
            pose = PoseStamped()
            pose.header = lane_path.header
            pose.pose.position.x = float(x_m)
            pose.pose.position.y = float(y_m)
            pose.pose.orientation.w = 1.0
            lane_path.poses.append(pose)
        self.path_pub.publish(lane_path)
        self.inference_ms_pub.publish(Float32(data=float(inference_ms)))
        self.detections_pub.publish(String(data=json.dumps(details)))
        if bool(self.get_parameter('publish_lane_geometry').value):
            self.geometry_pub.publish(String(data=json.dumps(geometry)))
        self.valid_pub.publish(Bool(data=valid))
        self.lane_valid_event_pub.publish(Bool(data=valid))
        self.confidence_pub.publish(Float32(data=confidence))
        heading = geometry.get('heading_error') if valid else 0.0
        self.heading_pub.publish(Float32(data=float(heading or 0.0)))
        error = geometry.get('center_error') if valid else 0.0
        self.error_pub.publish(Float32(data=float(error or 0.0)))
        stamp_ns = (
            int(source.header.stamp.sec) * 1_000_000_000
            + int(source.header.stamp.nanosec)
        )
        self.result_pub.publish(String(data=json.dumps({
            'stamp_ns': stamp_ns,
            'valid': valid,
            'confidence': confidence,
            'source': geometry.get('target_source', 'NONE'),
            'path_points': len(lane_path.poses),
            'planning_frame': self.planning_frame,
        })))
        if self.debug_pub is not None and annotated is not None:
            self.debug_pub.publish(numpy_to_image_message(annotated, source))
        if self.binary_pub is not None and binary is not None:
            self.binary_pub.publish(numpy_to_image_message(binary, source))
        if self.overlay_pub is not None and overlay is not None:
            self.overlay_pub.publish(numpy_to_image_message(overlay, source))

    def _run_inference(self) -> None:
        latest_input = self._latest_inference_input()
        if latest_input is None:
            return
        stamp, source_bgr, source = latest_input
        if stamp == self.last_processed_stamp:
            return
        self.last_processed_stamp = stamp
        if self.roi_enabled:
            x0, y0, x1, y1 = normalized_roi_bounds(
                source_bgr.shape, *self.roi_edges)
            bgr = np.ascontiguousarray(source_bgr[y0:y1, x0:x1])
        else:
            x0, y0 = 0, 0
            y1, x1 = source_bgr.shape[:2]
            bgr = source_bgr
        roi_metadata = {
            'enabled': self.roi_enabled,
            'source_w': int(source_bgr.shape[1]),
            'source_h': int(source_bgr.shape[0]),
            'x': int(x0),
            'y': int(y0),
            'w': int(x1 - x0),
            'h': int(y1 - y0),
        }
        started_at = perf_counter()
        try:
            result = self.model.predict(
                bgr, **self._prediction_arguments())[0]
            inference_ms = (perf_counter() - started_at) * 1000.0
            instances, details = self._instances_from_result(
                result, bgr.shape)
            geometry = self.planner.plan(instances, bgr.shape)
            geometry['inference_ms'] = float(inference_ms)
            geometry['model_task'] = str(self.model.task)
            geometry['roi'] = roi_metadata
            annotated = (
                result.plot(line_width=2)
                if self.debug_pub is not None else None
            )
            binary = (
                self._combined_mask(instances, bgr.shape)
                if self.binary_pub is not None else None
            )
            overlay = (
                self._draw_overlay(bgr, instances, geometry, inference_ms)
                if self.overlay_pub is not None else None
            )
        except Exception as error:  # keep the controller in fail-safe LOST
            inference_ms = (perf_counter() - started_at) * 1000.0
            self.get_logger().error(
                f'segmentation inference failed: {error}',
                throttle_duration_sec=2.0)
            geometry = self.planner.plan([], bgr.shape)
            geometry['inference_ms'] = float(inference_ms)
            geometry['error'] = str(error)
            geometry['roi'] = roi_metadata
            details = []
            annotated = bgr.copy() if self.debug_pub is not None else None
            binary = (
                np.zeros(bgr.shape[:2], dtype=np.uint8)
                if self.binary_pub is not None else None
            )
            overlay = (
                self._draw_overlay(bgr, [], geometry, inference_ms)
                if self.overlay_pub is not None else None
            )
        self._publish_plan(
            source, annotated, overlay, binary,
            geometry, details, inference_ms)

    def destroy_node(self) -> bool:
        if self.camera is not None:
            self.camera.stop()
            self.camera = None
        return super().destroy_node()


def main(args=None) -> None:
    """Run the segmentation lane planner node."""
    rclpy.init(args=args)
    node = IreYoloSegmentationLaneNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()

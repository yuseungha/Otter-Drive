#!/usr/bin/env python3
"""Real-time YOLO detections with CPU development and Jetson CUDA deployment support."""
from pathlib import Path
from time import perf_counter
from cv_bridge import CvBridge
import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image
from vision_msgs.msg import Detection2D, Detection2DArray, ObjectHypothesisWithPose
from geometry_msgs.msg import Pose, PoseArray
from std_msgs.msg import Float32


class YoloDetectNode(Node):
    def __init__(self):
        super().__init__('yolo_detect_node')
        defaults = {
            'model_path': '/home/juwnoo/Downloads/road_best.pt',
            'image_topic': '/image_raw', 'detection_topic': '/yolo/detections',
            'line_points_topic': '/yolo/line_points', 'latency_topic': '/yolo/inference_ms',
            'device': 'cpu', 'imgsz': 640, 'confidence': 0.25, 'iou': 0.45,
            'target_classes': [], 'publish_mask_polygons': True,
        }
        for key, value in defaults.items(): self.declare_parameter(key, value)
        p = {key: self.get_parameter(key).value for key in defaults}
        model_path = Path(p['model_path'])
        if not model_path.is_file(): raise FileNotFoundError(f'YOLO model was not found: {model_path}')
        from ultralytics import YOLO
        self.model, self.device = YOLO(str(model_path)), str(p['device'])
        self.imgsz, self.confidence, self.iou = int(p['imgsz']), float(p['confidence']), float(p['iou'])
        self.publish_polygons = bool(p['publish_mask_polygons']); self.bridge = CvBridge()
        names = self.model.names
        requested = set(p['target_classes'])
        self.class_ids = [index for index, name in names.items() if not requested or str(name) in requested]
        if requested and not self.class_ids: self.get_logger().warning(f'target_classes {sorted(requested)} do not match model labels {list(names.values())}')
        latest_image_qos = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.create_subscription(Image, p['image_topic'], self.on_image, latest_image_qos)
        self.detection_pub = self.create_publisher(Detection2DArray, p['detection_topic'], 10)
        self.points_pub = self.create_publisher(PoseArray, p['line_points_topic'], 10)
        self.latency_pub = self.create_publisher(Float32, p['latency_topic'], 10)
        self.get_logger().info(f'YOLO ready: {p["image_topic"]} -> {p["detection_topic"]}, device={self.device}, qos=latest-frame')

    def on_image(self, image_message):
        try:
            image = self.bridge.imgmsg_to_cv2(image_message, desired_encoding='bgr8')
            start = perf_counter()
            result = self.model.predict(source=image, device=self.device, imgsz=self.imgsz, conf=self.confidence, iou=self.iou, classes=self.class_ids or None, verbose=False)[0]
        except Exception as error:
            self.get_logger().error(f'YOLO inference failed: {error}')
            return
        latency = Float32(); latency.data = (perf_counter() - start) * 1000.0; self.latency_pub.publish(latency)
        detections = Detection2DArray(); detections.header = image_message.header
        for box in ([] if result.boxes is None else result.boxes):
            x1, y1, x2, y2 = box.xyxy[0].tolist(); class_id = int(box.cls[0].item())
            detection = Detection2D(); detection.header = image_message.header
            detection.bbox.center.position.x, detection.bbox.center.position.y = (x1 + x2) / 2., (y1 + y2) / 2.
            detection.bbox.size_x, detection.bbox.size_y = x2 - x1, y2 - y1
            hypothesis = ObjectHypothesisWithPose(); hypothesis.hypothesis.class_id = str(result.names[class_id]); hypothesis.hypothesis.score = float(box.conf[0].item())
            detection.results.append(hypothesis); detections.detections.append(detection)
        self.detection_pub.publish(detections)
        points = PoseArray(); points.header = image_message.header
        if self.publish_polygons and result.masks is not None:
            height, width = image.shape[:2]
            for polygon in result.masks.xy:
                for x, y in polygon:
                    point = Pose(); point.position.x, point.position.y = float(x / width), float(y / height); points.poses.append(point)
        self.points_pub.publish(points)


def main(args=None):
    rclpy.init(args=args); node = None
    try:
        node = YoloDetectNode(); rclpy.spin(node)
    except (FileNotFoundError, RuntimeError) as error:
        rclpy.logging.get_logger('yolo_detect_node').fatal(str(error))
    except KeyboardInterrupt: pass
    finally:
        if node is not None: node.destroy_node()
        rclpy.shutdown()

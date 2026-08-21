"""OpenCV lane-center detection without ROS dependencies."""

from dataclasses import dataclass
from typing import Optional, Tuple

import cv2
import numpy as np


@dataclass(frozen=True)
class LaneResult:
    """One lane detector result."""

    valid: bool
    center_error: float
    confidence: float
    lane_center_x: Optional[float]
    lane_count: int
    binary: np.ndarray


@dataclass(frozen=True)
class LanePreprocessResult:
    """Intermediate images from ROI, BEV, and HSV processing."""

    roi: np.ndarray
    bev: np.ndarray
    white_mask: np.ndarray
    yellow_mask: np.ndarray
    binary: np.ndarray
    transform: np.ndarray
    inverse_transform: np.ndarray
    roi_y: int


def _normalized_points(values, width: int, height: int) -> np.ndarray:
    if len(values) != 8:
        raise ValueError('BEV point lists must contain 8 normalized values')
    points = np.asarray(values, dtype=np.float32).reshape(4, 2)
    points[:, 0] *= max(1, width - 1)
    points[:, 1] *= max(1, height - 1)
    return points


def preprocess_lane_frame(
    bgr: np.ndarray,
    roi_top_ratio: float = 0.45,
    bev_width: int = 640,
    bev_height: int = 480,
    bev_source=(0.05, 0.98, 0.95, 0.98, 0.62, 0.05, 0.38, 0.05),
    bev_destination=(0.20, 0.99, 0.80, 0.99, 0.80, 0.0, 0.20, 0.0),
    white_lower=(0, 0, 175),
    white_upper=(180, 80, 255),
    yellow_lower=(12, 70, 70),
    yellow_upper=(42, 255, 255),
    morphology_kernel: int = 3,
) -> LanePreprocessResult:
    """Apply lower ROI, BEV warp, HSV masks, and morphology cleanup."""
    if bgr is None or bgr.ndim != 3 or bgr.shape[2] != 3:
        raise ValueError('bgr must be a non-empty HxWx3 image')
    if bev_width < 2 or bev_height < 2:
        raise ValueError('BEV dimensions must be at least 2x2')

    height, _ = bgr.shape[:2]
    roi_y = int(np.clip(roi_top_ratio, 0.0, 0.9) * height)
    roi = bgr[roi_y:, :]
    roi_height, roi_width = roi.shape[:2]
    source = _normalized_points(bev_source, roi_width, roi_height)
    destination = _normalized_points(
        bev_destination, bev_width, bev_height)
    transform = cv2.getPerspectiveTransform(source, destination)
    inverse_transform = cv2.getPerspectiveTransform(destination, source)
    bev = cv2.warpPerspective(roi, transform, (bev_width, bev_height))

    hsv = cv2.cvtColor(bev, cv2.COLOR_BGR2HSV)
    white_mask = cv2.inRange(
        hsv,
        np.asarray(white_lower, dtype=np.uint8),
        np.asarray(white_upper, dtype=np.uint8),
    )
    yellow_mask = cv2.inRange(
        hsv,
        np.asarray(yellow_lower, dtype=np.uint8),
        np.asarray(yellow_upper, dtype=np.uint8),
    )
    binary = cv2.bitwise_or(white_mask, yellow_mask)
    kernel_size = max(1, int(morphology_kernel))
    if kernel_size % 2 == 0:
        kernel_size += 1
    kernel = np.ones((kernel_size, kernel_size), dtype=np.uint8)
    binary = cv2.morphologyEx(binary, cv2.MORPH_OPEN, kernel)
    binary = cv2.morphologyEx(binary, cv2.MORPH_CLOSE, kernel)
    return LanePreprocessResult(
        roi=roi,
        bev=bev,
        white_mask=white_mask,
        yellow_mask=yellow_mask,
        binary=binary,
        transform=transform,
        inverse_transform=inverse_transform,
        roi_y=roi_y,
    )


def bev_points_to_image(
    points: np.ndarray,
    preprocess: LanePreprocessResult,
) -> np.ndarray:
    """Map BEV points through ROI coordinates back to the source image."""
    values = np.asarray(points, dtype=np.float32).reshape(-1, 1, 2)
    restored = cv2.perspectiveTransform(
        values, preprocess.inverse_transform).reshape(-1, 2)
    restored[:, 1] += preprocess.roi_y
    return restored


def _collect_lane_pixels(
    binary: np.ndarray,
    base_x: int,
    windows: int,
    margin: int,
    min_recenter_pixels: int,
) -> Tuple[np.ndarray, np.ndarray]:
    nonzero_y, nonzero_x = binary.nonzero()
    window_height = max(1, binary.shape[0] // windows)
    current_x = base_x
    selected = []

    for window in range(windows):
        y_low = binary.shape[0] - (window + 1) * window_height
        y_high = binary.shape[0] - window * window_height
        x_low = current_x - margin
        x_high = current_x + margin
        indices = np.where(
            (nonzero_y >= y_low)
            & (nonzero_y < y_high)
            & (nonzero_x >= x_low)
            & (nonzero_x < x_high)
        )[0]
        selected.append(indices)
        if indices.size >= min_recenter_pixels:
            current_x = int(np.mean(nonzero_x[indices]))

    if not selected:
        return np.array([], dtype=np.int64), np.array([], dtype=np.int64)
    selected_indices = np.concatenate(selected)
    return nonzero_x[selected_indices], nonzero_y[selected_indices]


def detect_lane_center(
    bgr: np.ndarray,
    roi_top_ratio: float = 0.45,
    windows: int = 9,
    margin_px: int = 45,
    min_recenter_pixels: int = 25,
    min_lane_pixels: int = 120,
    lane_half_width_px: float = 130.0,
) -> LaneResult:
    """Detect white/yellow lanes and return normalized lateral center error."""
    if bgr is None or bgr.ndim != 3 or bgr.shape[2] != 3:
        raise ValueError('bgr must be a non-empty HxWx3 image')
    height, width = bgr.shape[:2]
    roi_y = int(np.clip(roi_top_ratio, 0.0, 0.9) * height)
    roi = bgr[roi_y:, :]

    hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
    white = cv2.inRange(hsv, np.array([0, 0, 175]), np.array([180, 80, 255]))
    yellow = cv2.inRange(hsv, np.array([12, 70, 70]), np.array([42, 255, 255]))
    binary = cv2.bitwise_or(white, yellow)
    kernel = np.ones((3, 3), dtype=np.uint8)
    binary = cv2.morphologyEx(binary, cv2.MORPH_OPEN, kernel)

    histogram = np.sum(binary[binary.shape[0] // 2:, :] > 0, axis=0)
    midpoint = width // 2
    left_base = int(np.argmax(histogram[:midpoint]))
    right_base = int(np.argmax(histogram[midpoint:]) + midpoint)

    lane_x_at_bottom = []
    total_pixels = 0
    for base_x, peak in (
        (left_base, histogram[left_base]),
        (right_base, histogram[right_base]),
    ):
        if peak <= 0:
            continue
        lane_x, lane_y = _collect_lane_pixels(
            binary,
            base_x,
            windows,
            margin_px,
            min_recenter_pixels,
        )
        if lane_x.size < min_lane_pixels:
            continue
        polynomial = np.polyfit(lane_y, lane_x, 2)
        bottom_y = binary.shape[0] - 1
        lane_x_at_bottom.append(float(np.polyval(polynomial, bottom_y)))
        total_pixels += int(lane_x.size)

    lane_count = len(lane_x_at_bottom)
    if lane_count == 2:
        lane_center = sum(lane_x_at_bottom) / 2.0
    elif lane_count == 1:
        observed = lane_x_at_bottom[0]
        lane_center = (
            observed + lane_half_width_px
            if observed < midpoint
            else observed - lane_half_width_px
        )
    else:
        return LaneResult(False, 0.0, 0.0, None, 0, binary)

    error = float(np.clip((lane_center - midpoint) / max(1.0, midpoint), -1.0, 1.0))
    pixel_score = min(1.0, total_pixels / max(1.0, 6.0 * min_lane_pixels))
    confidence = pixel_score * (1.0 if lane_count == 2 else 0.6)
    return LaneResult(True, error, confidence, lane_center, lane_count, binary)

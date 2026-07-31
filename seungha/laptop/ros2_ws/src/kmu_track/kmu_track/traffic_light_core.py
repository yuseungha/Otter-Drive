"""ROS-independent red/green traffic-light detection and state filtering."""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum
from typing import Optional, Sequence, Tuple

import cv2
import numpy as np


class SignalState(str, Enum):
    """Fail-safe motion instruction derived from the camera image."""

    STOP = 'STOP'
    GO = 'GO'
    TURN_LEFT = 'TURN LEFT'


@dataclass(frozen=True)
class ColorEvidence:
    """Per-frame color and left-arrow measurements inside the ROI."""

    red_ratio: float
    green_ratio: float
    red_blob_area: float
    green_blob_area: float
    red_active: bool
    green_active: bool
    left_arrow_score: float
    left_arrow_active: bool
    left_arrow_box: Optional[Tuple[int, int, int, int]]
    left_arrow_aspect_ratio: float
    left_arrow_solidity: float
    left_arrow_direction_ratio: float
    roi_box: Tuple[int, int, int, int]
    red_mask: np.ndarray
    green_mask: np.ndarray


@dataclass(frozen=True)
class SignalDecision:
    """Filtered decision plus evidence useful for UI and telemetry."""

    state: SignalState
    reason: str
    red_streak: int
    green_streak: int
    lost_streak: int
    red_seen: bool
    evidence: ColorEvidence


@dataclass(frozen=True)
class _ArrowFeatures:
    active: bool = False
    score: float = 0.0
    box: Optional[Tuple[int, int, int, int]] = None
    aspect_ratio: float = 0.0
    solidity: float = 1.0
    direction_ratio: float = 0.0


def _roi_bounds(
    image_shape: Sequence[int],
    roi: Sequence[float],
) -> Tuple[int, int, int, int]:
    if len(roi) != 4:
        raise ValueError(f'ROI must contain [x, y, width, height]: {roi}')
    height, width = image_shape[:2]
    x, y, roi_width, roi_height = [float(value) for value in roi]
    x0 = int(np.clip(x, 0.0, 1.0) * width)
    y0 = int(np.clip(y, 0.0, 1.0) * height)
    x1 = int(np.clip(x + roi_width, 0.0, 1.0) * width)
    y1 = int(np.clip(y + roi_height, 0.0, 1.0) * height)
    if x1 <= x0 or y1 <= y0:
        raise ValueError(f'invalid ROI: {roi}')
    return x0, y0, x1, y1


def _clean_mask(mask: np.ndarray, kernel_size: int) -> np.ndarray:
    size = max(1, int(kernel_size))
    if size % 2 == 0:
        size += 1
    if size == 1:
        return mask
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (size, size))
    opened = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
    return cv2.morphologyEx(opened, cv2.MORPH_CLOSE, kernel)


def _external_contours(mask: np.ndarray) -> Sequence[np.ndarray]:
    contours, _ = cv2.findContours(
        mask,
        cv2.RETR_EXTERNAL,
        cv2.CHAIN_APPROX_SIMPLE,
    )
    return sorted(contours, key=cv2.contourArea, reverse=True)


def _left_arrow_features(
    green_mask: np.ndarray,
    green_contour: Optional[np.ndarray],
    min_blob_area: float,
    min_aspect_ratio: float,
    max_solidity: float,
    min_direction_ratio: float,
) -> _ArrowFeatures:
    """Classify a left arrow using fast, orientation-aware shape features."""
    if green_contour is None:
        return _ArrowFeatures()
    area = float(cv2.contourArea(green_contour))
    if area < float(min_blob_area):
        return _ArrowFeatures()

    x, y, width, height = cv2.boundingRect(green_contour)
    if width < 3 or height < 3:
        return _ArrowFeatures()
    aspect_ratio = float(width) / float(height)
    hull = cv2.convexHull(green_contour)
    hull_area = float(cv2.contourArea(hull))
    solidity = area / max(1.0, hull_area)

    crop = green_mask[y:y + height, x:x + width]
    edge_width = max(1, width // 5)
    left_mass = float(cv2.countNonZero(crop[:, :edge_width]))
    right_mass = float(cv2.countNonZero(crop[:, -edge_width:]))
    direction_ratio = right_mass / max(1.0, left_mass)

    points = green_contour.reshape(-1, 2)
    left_tip = points[int(np.argmin(points[:, 0]))]
    normalized_tip_offset = abs(
        (float(left_tip[1] - y) / max(1.0, float(height - 1))) - 0.5
    )

    aspect_score = float(np.clip((aspect_ratio - 1.0) / 0.8, 0.0, 1.0))
    concavity_score = float(np.clip((0.98 - solidity) / 0.35, 0.0, 1.0))
    direction_score = float(np.clip(
        (direction_ratio - 1.0) / 1.0, 0.0, 1.0))
    tip_score = float(np.clip(
        1.0 - normalized_tip_offset / 0.5, 0.0, 1.0))
    score = (
        aspect_score
        + concavity_score
        + direction_score
        + tip_score
    ) / 4.0
    active = (
        aspect_ratio >= float(min_aspect_ratio)
        and solidity <= float(max_solidity)
        and direction_ratio >= float(min_direction_ratio)
        and normalized_tip_offset <= 0.28
    )
    return _ArrowFeatures(
        active=active,
        score=score,
        box=(x, y, x + width, y + height),
        aspect_ratio=aspect_ratio,
        solidity=solidity,
        direction_ratio=direction_ratio,
    )


def _line_left_arrow_features(
    green_mask: np.ndarray,
    green_contours: Sequence[np.ndarray],
    min_blob_area: float,
    max_head_aspect_ratio: float,
    max_solidity: float,
    min_direction_ratio: float,
    min_shaft_aspect_ratio: float,
) -> _ArrowFeatures:
    """Recognize a split ``<`` arrowhead plus a horizontal right shaft."""
    usable = [
        contour
        for contour in green_contours
        if cv2.contourArea(contour) >= float(min_blob_area)
    ]
    if len(usable) < 2:
        return _ArrowFeatures()

    best = _ArrowFeatures()
    for head in usable:
        head_area = float(cv2.contourArea(head))
        x, y, width, height = cv2.boundingRect(head)
        if width < 3 or height < 3:
            continue
        head_aspect = float(width) / float(height)
        if head_aspect > float(max_head_aspect_ratio):
            continue
        hull_area = float(cv2.contourArea(cv2.convexHull(head)))
        solidity = head_area / max(1.0, hull_area)
        if solidity > float(max_solidity):
            continue

        crop = green_mask[y:y + height, x:x + width]
        edge_width = max(1, width // 5)
        left_mass = float(cv2.countNonZero(crop[:, :edge_width]))
        right_mass = float(cv2.countNonZero(crop[:, -edge_width:]))
        direction_ratio = right_mass / max(1.0, left_mass)
        if direction_ratio < float(min_direction_ratio):
            continue

        points = head.reshape(-1, 2)
        left_tip = points[int(np.argmin(points[:, 0]))]
        tip_offset = abs(
            (float(left_tip[1] - y) / max(1.0, float(height - 1))) - 0.5
        )
        if tip_offset > 0.30:
            continue

        head_center_y = y + height / 2.0
        for shaft in usable:
            if shaft is head:
                continue
            shaft_x, shaft_y, shaft_width, shaft_height = cv2.boundingRect(
                shaft)
            if shaft_width < 3 or shaft_height < 3:
                continue
            shaft_aspect = float(shaft_width) / float(shaft_height)
            shaft_center_y = shaft_y + shaft_height / 2.0
            shaft_extends_right = (
                shaft_x + shaft_width >= x + width * 1.15)
            shaft_starts_near_head = (
                x + width * 0.35 <= shaft_x <= x + width * 1.75)
            vertically_aligned = (
                abs(shaft_center_y - head_center_y) <= height * 0.30)
            height_plausible = shaft_height <= height * 0.55
            if not (
                shaft_aspect >= float(min_shaft_aspect_ratio)
                and shaft_extends_right
                and shaft_starts_near_head
                and vertically_aligned
                and height_plausible
            ):
                continue

            combined_x0 = min(x, shaft_x)
            combined_y0 = min(y, shaft_y)
            combined_x1 = max(x + width, shaft_x + shaft_width)
            combined_y1 = max(y + height, shaft_y + shaft_height)
            direction_score = float(np.clip(
                (direction_ratio - 1.0) / 1.0, 0.0, 1.0))
            concavity_score = float(np.clip(
                (0.98 - solidity) / 0.35, 0.0, 1.0))
            tip_score = float(np.clip(
                1.0 - tip_offset / 0.5, 0.0, 1.0))
            shaft_score = float(np.clip(
                (shaft_aspect - 1.0) / 1.5, 0.0, 1.0))
            alignment_score = float(np.clip(
                1.0
                - abs(shaft_center_y - head_center_y) / (height * 0.30),
                0.0,
                1.0,
            ))
            score = (
                direction_score
                + concavity_score
                + tip_score
                + shaft_score
                + alignment_score
            ) / 5.0
            if score > best.score:
                best = _ArrowFeatures(
                    active=True,
                    score=score,
                    box=(
                        combined_x0,
                        combined_y0,
                        combined_x1,
                        combined_y1,
                    ),
                    aspect_ratio=(
                        float(combined_x1 - combined_x0)
                        / max(1.0, float(combined_y1 - combined_y0))
                    ),
                    solidity=solidity,
                    direction_ratio=direction_ratio,
                )
    return best


def measure_colors(
    bgr_image: np.ndarray,
    roi: Sequence[float] = (0.20, 0.10, 0.60, 0.75),
    min_red_ratio: float = 0.010,
    min_green_ratio: float = 0.010,
    min_blob_area: float = 250.0,
    morphology_kernel: int = 5,
    left_arrow_min_aspect_ratio: float = 1.20,
    left_arrow_max_solidity: float = 0.90,
    left_arrow_min_direction_ratio: float = 1.20,
    left_arrow_line_max_head_aspect_ratio: float = 1.10,
    left_arrow_line_min_shaft_aspect_ratio: float = 1.50,
) -> ColorEvidence:
    """Measure red, green, and left-arrow evidence in a normalized ROI."""
    if bgr_image is None or bgr_image.size == 0:
        raise ValueError('image is empty')

    x0, y0, x1, y1 = _roi_bounds(bgr_image.shape, roi)
    bgr_roi = bgr_image[y0:y1, x0:x1]
    hsv = cv2.cvtColor(bgr_roi, cv2.COLOR_BGR2HSV)

    # Red wraps around the ends of OpenCV's 0..180 hue scale.
    red_low = cv2.inRange(
        hsv,
        np.array([0, 100, 80], dtype=np.uint8),
        np.array([12, 255, 255], dtype=np.uint8),
    )
    red_high = cv2.inRange(
        hsv,
        np.array([165, 100, 80], dtype=np.uint8),
        np.array([180, 255, 255], dtype=np.uint8),
    )
    red_mask = _clean_mask(
        cv2.bitwise_or(red_low, red_high), morphology_kernel)
    green_mask = _clean_mask(
        cv2.inRange(
            hsv,
            np.array([35, 80, 70], dtype=np.uint8),
            np.array([100, 255, 255], dtype=np.uint8),
        ),
        morphology_kernel,
    )

    pixels = float(max(1, bgr_roi.shape[0] * bgr_roi.shape[1]))
    red_ratio = float(cv2.countNonZero(red_mask)) / pixels
    green_ratio = float(cv2.countNonZero(green_mask)) / pixels
    red_contours = _external_contours(red_mask)
    green_contours = _external_contours(green_mask)
    red_contour = red_contours[0] if red_contours else None
    green_contour = green_contours[0] if green_contours else None
    red_blob_area = (
        0.0 if red_contour is None else float(cv2.contourArea(red_contour)))
    green_blob_area = (
        0.0
        if green_contour is None
        else float(cv2.contourArea(green_contour))
    )
    red_active = (
        red_ratio >= float(min_red_ratio)
        and red_blob_area >= float(min_blob_area)
    )
    green_active = (
        green_ratio >= float(min_green_ratio)
        and green_blob_area >= float(min_blob_area)
    )
    filled_arrow = _left_arrow_features(
        green_mask,
        green_contour,
        min_blob_area,
        left_arrow_min_aspect_ratio,
        left_arrow_max_solidity,
        left_arrow_min_direction_ratio,
    )
    line_arrow = _line_left_arrow_features(
        green_mask,
        green_contours,
        min_blob_area,
        left_arrow_line_max_head_aspect_ratio,
        left_arrow_max_solidity,
        left_arrow_min_direction_ratio,
        left_arrow_line_min_shaft_aspect_ratio,
    )
    arrow = (
        line_arrow
        if line_arrow.active and line_arrow.score >= filled_arrow.score
        else filled_arrow
    )
    return ColorEvidence(
        red_ratio=red_ratio,
        green_ratio=green_ratio,
        red_blob_area=red_blob_area,
        green_blob_area=green_blob_area,
        red_active=red_active,
        green_active=green_active,
        left_arrow_score=arrow.score,
        left_arrow_active=green_active and arrow.active,
        left_arrow_box=arrow.box,
        left_arrow_aspect_ratio=arrow.aspect_ratio,
        left_arrow_solidity=arrow.solidity,
        left_arrow_direction_ratio=arrow.direction_ratio,
        roi_box=(x0, y0, x1, y1),
        red_mask=red_mask,
        green_mask=green_mask,
    )


class TrafficLightDetector:
    """
    Convert noisy color and shape evidence into a safe motion instruction.

    When red and green coexist, the larger color ratio wins. Red-dominant STOP
    and green-dominant GO or TURN LEFT instructions must remain stable for
    their configured frame counts. An exact ratio tie remains a safe STOP.
    """

    def __init__(
        self,
        roi: Sequence[float] = (0.20, 0.10, 0.60, 0.75),
        min_red_ratio: float = 0.010,
        min_green_ratio: float = 0.010,
        min_blob_area: float = 250.0,
        morphology_kernel: int = 5,
        confirm_red_frames: int = 5,
        confirm_green_frames: int = 5,
        lost_signal_frames: int = 3,
        require_red_before_green: bool = False,
        left_arrow_min_aspect_ratio: float = 1.20,
        left_arrow_max_solidity: float = 0.90,
        left_arrow_min_direction_ratio: float = 1.20,
        left_arrow_line_max_head_aspect_ratio: float = 1.10,
        left_arrow_line_min_shaft_aspect_ratio: float = 1.50,
    ) -> None:
        if confirm_red_frames < 1:
            raise ValueError('confirm_red_frames must be at least 1')
        if confirm_green_frames < 1:
            raise ValueError('confirm_green_frames must be at least 1')
        if lost_signal_frames < 1:
            raise ValueError('lost_signal_frames must be at least 1')
        self.roi = tuple(float(value) for value in roi)
        self.min_red_ratio = float(min_red_ratio)
        self.min_green_ratio = float(min_green_ratio)
        self.min_blob_area = float(min_blob_area)
        self.morphology_kernel = int(morphology_kernel)
        self.confirm_red_frames = int(confirm_red_frames)
        self.confirm_green_frames = int(confirm_green_frames)
        self.lost_signal_frames = int(lost_signal_frames)
        self.require_red_before_green = bool(require_red_before_green)
        self.left_arrow_min_aspect_ratio = float(
            left_arrow_min_aspect_ratio)
        self.left_arrow_max_solidity = float(left_arrow_max_solidity)
        self.left_arrow_min_direction_ratio = float(
            left_arrow_min_direction_ratio)
        self.left_arrow_line_max_head_aspect_ratio = float(
            left_arrow_line_max_head_aspect_ratio)
        self.left_arrow_line_min_shaft_aspect_ratio = float(
            left_arrow_line_min_shaft_aspect_ratio)
        self.reset()

    def reset(self) -> None:
        """Return to the fail-safe startup state."""
        self.state = SignalState.STOP
        self.red_streak = 0
        self.green_streak = 0
        self.lost_streak = 0
        self.red_seen = False
        self.pending_green_state: Optional[SignalState] = None

    @staticmethod
    def _dominant_color(evidence: ColorEvidence) -> Tuple[str, str]:
        if evidence.red_active and evidence.green_active:
            if evidence.green_ratio > evidence.red_ratio:
                return 'green', 'GREEN DOMINANT'
            if evidence.red_ratio > evidence.green_ratio:
                return 'red', 'RED DOMINANT'
            return 'tie', 'RED/GREEN TIE'
        if evidence.red_active:
            return 'red', 'RED DETECTED'
        if evidence.green_active:
            return 'green', 'GREEN DETECTED'
        return 'none', 'NO SIGNAL'

    def _reset_green_confirmation(self) -> None:
        self.green_streak = 0
        self.pending_green_state = None

    def _reset_red_confirmation(self) -> None:
        self.red_streak = 0

    def _confirm_red_instruction(self, prefix: str) -> str:
        self.red_streak += 1
        if self.red_streak >= self.confirm_red_frames:
            self.red_seen = True
            self.state = SignalState.STOP
            return f'{prefix} -> RED CONFIRMED'
        return (
            f'{prefix} -> RED CHECK '
            f'{self.red_streak}/{self.confirm_red_frames}'
        )

    def _confirm_green_instruction(
        self,
        candidate: SignalState,
        prefix: str,
    ) -> str:
        if candidate != self.pending_green_state:
            self.pending_green_state = candidate
            self.green_streak = 0
        self.green_streak += 1
        label = 'LEFT ARROW' if candidate == SignalState.TURN_LEFT else 'GREEN'
        if self.green_streak >= self.confirm_green_frames:
            self.state = candidate
            return f'{prefix} -> {label} CONFIRMED'
        return (
            f'{prefix} -> {label} CHECK '
            f'{self.green_streak}/{self.confirm_green_frames}'
        )

    def analyze(self, bgr_image: np.ndarray) -> SignalDecision:
        """Analyze one frame and return a filtered motion instruction."""
        evidence = measure_colors(
            bgr_image,
            roi=self.roi,
            min_red_ratio=self.min_red_ratio,
            min_green_ratio=self.min_green_ratio,
            min_blob_area=self.min_blob_area,
            morphology_kernel=self.morphology_kernel,
            left_arrow_min_aspect_ratio=self.left_arrow_min_aspect_ratio,
            left_arrow_max_solidity=self.left_arrow_max_solidity,
            left_arrow_min_direction_ratio=(
                self.left_arrow_min_direction_ratio),
            left_arrow_line_max_head_aspect_ratio=(
                self.left_arrow_line_max_head_aspect_ratio),
            left_arrow_line_min_shaft_aspect_ratio=(
                self.left_arrow_line_min_shaft_aspect_ratio),
        )
        dominant, prefix = self._dominant_color(evidence)

        if dominant == 'red':
            self._reset_green_confirmation()
            self.lost_streak = 0
            reason = self._confirm_red_instruction(prefix)
            if evidence.green_active:
                reason += (
                    f' {evidence.red_ratio * 100:.2f}% > '
                    f'{evidence.green_ratio * 100:.2f}%'
                )
        elif dominant == 'green':
            self._reset_red_confirmation()
            self.lost_streak = 0
            if self.require_red_before_green and not self.red_seen:
                self.state = SignalState.STOP
                self._reset_green_confirmation()
                reason = 'WAITING FOR RED FIRST'
            else:
                candidate = (
                    SignalState.TURN_LEFT
                    if evidence.left_arrow_active
                    else SignalState.GO
                )
                reason = self._confirm_green_instruction(candidate, prefix)
                if evidence.red_active:
                    reason += (
                        f' {evidence.green_ratio * 100:.2f}% > '
                        f'{evidence.red_ratio * 100:.2f}%'
                    )
        elif dominant == 'tie':
            self.state = SignalState.STOP
            self._reset_red_confirmation()
            self._reset_green_confirmation()
            self.lost_streak = 0
            reason = f'{prefix} - SAFE STOP'
        else:
            self._reset_red_confirmation()
            self._reset_green_confirmation()
            self.lost_streak += 1
            if self.lost_streak >= self.lost_signal_frames:
                self.state = SignalState.STOP
                reason = 'NO SIGNAL - SAFE STOP'
            else:
                reason = f'SIGNAL DROPOUT {self.lost_streak}'

        return SignalDecision(
            state=self.state,
            reason=reason,
            red_streak=self.red_streak,
            green_streak=self.green_streak,
            lost_streak=self.lost_streak,
            red_seen=self.red_seen,
            evidence=evidence,
        )

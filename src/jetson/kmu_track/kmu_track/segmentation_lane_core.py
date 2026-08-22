"""Plan a lane center from YOLO masks without ROS dependencies."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Iterable, Sequence

import numpy as np


@dataclass(frozen=True)
class SegmentationInstance:
    """One class-labelled probability mask produced by a segmenter."""

    class_name: str
    confidence: float
    mask: np.ndarray


@dataclass(frozen=True)
class SegmentationLaneConfig:
    """Geometry and fail-safe thresholds for segmentation lane planning."""

    center_class_name: str = 'center'
    boundary_class_name: str = 'lane'
    scan_rows: Sequence[float] = (
        0.50, 0.58, 0.66, 0.74, 0.82, 0.90, 0.96,
    )
    scan_band_half_height: int = 5
    mask_threshold: float = 0.50
    min_lane_width_ratio: float = 0.10
    min_mask_pixels_per_row: int = 3
    min_valid_rows: int = 3
    min_vertical_span_ratio: float = 0.15
    max_lookahead_extrapolation_ratio: float = 0.05
    look_ahead_ratio: float = 0.74
    heading_far_ratio: float = 0.58
    heading_near_ratio: float = 0.90
    center_consistency_tol: float = 0.10
    minimum_plan_confidence: float = 0.25

    def validate(self) -> None:
        if not self.center_class_name or not self.boundary_class_name:
            raise ValueError('class names must be non-empty')
        if not self.scan_rows:
            raise ValueError('scan_rows must not be empty')
        if any(not 0.0 <= float(row) <= 1.0 for row in self.scan_rows):
            raise ValueError('scan_rows must lie in [0, 1]')
        if self.scan_band_half_height < 0:
            raise ValueError('scan_band_half_height must be non-negative')
        if not 0.0 < self.mask_threshold <= 1.0:
            raise ValueError('mask_threshold must lie in (0, 1]')
        if not 0.0 < self.min_lane_width_ratio < 1.0:
            raise ValueError('min_lane_width_ratio must lie in (0, 1)')
        if self.min_mask_pixels_per_row < 1 or self.min_valid_rows < 2:
            raise ValueError('row count thresholds are too small')
        for name in (
            'min_vertical_span_ratio',
            'max_lookahead_extrapolation_ratio',
            'look_ahead_ratio',
            'heading_far_ratio',
            'heading_near_ratio',
            'center_consistency_tol',
            'minimum_plan_confidence',
        ):
            value = float(getattr(self, name))
            if not 0.0 <= value <= 1.0:
                raise ValueError(f'{name} must lie in [0, 1]')
        if self.heading_far_ratio >= self.heading_near_ratio:
            raise ValueError(
                'heading_far_ratio must be below heading_near_ratio')


class SegmentationLanePlanner:
    """Fit a short image-space path from center and outer-lane masks."""

    def __init__(self, config: SegmentationLaneConfig | None = None) -> None:
        self.config = config or SegmentationLaneConfig()
        self.config.validate()

    def _sample_x(self, mask: np.ndarray, row_y: int) -> float | None:
        half = self.config.scan_band_half_height
        low = max(0, row_y - half)
        high = min(mask.shape[0], row_y + half + 1)
        _ys, xs = np.nonzero(mask[low:high] >= self.config.mask_threshold)
        if xs.size < self.config.min_mask_pixels_per_row:
            return None
        return float(np.median(xs))

    @staticmethod
    def _fit_path(points: list[dict]) -> tuple[np.ndarray, int]:
        y = np.asarray([point['ratio'] for point in points], dtype=np.float64)
        x = np.asarray(
            [point['target_norm'] for point in points], dtype=np.float64)
        weights = np.sqrt(np.asarray(
            [max(0.05, point['confidence']) for point in points],
            dtype=np.float64,
        ))
        degree = 2 if len(points) >= 5 else 1
        columns = [np.ones_like(y), y]
        if degree == 2:
            columns.append(y * y)
        design = np.column_stack(columns)
        weighted_design = design * weights[:, None]
        weighted_x = x * weights
        coefficients, _residuals, _rank, _singular = np.linalg.lstsq(
            weighted_design, weighted_x, rcond=None)
        return coefficients, degree

    @staticmethod
    def _evaluate(coefficients: np.ndarray, ratio: float) -> float:
        powers = np.asarray(
            [float(ratio) ** power for power in range(len(coefficients))],
            dtype=np.float64,
        )
        return float(np.dot(coefficients, powers))

    def plan(
        self,
        instances: Iterable[SegmentationInstance],
        image_shape: Sequence[int],
    ) -> dict:
        """Return a JSON-safe path and normalized controller errors."""
        if len(image_shape) < 2:
            raise ValueError('image_shape must contain height and width')
        height, width = int(image_shape[0]), int(image_shape[1])
        if height < 2 or width < 2:
            raise ValueError('image dimensions must be at least 2x2')

        selected: list[SegmentationInstance] = []
        for instance in instances:
            if instance.class_name not in {
                self.config.center_class_name,
                self.config.boundary_class_name,
            }:
                continue
            mask = np.asarray(instance.mask)
            if mask.ndim != 2 or mask.shape != (height, width):
                raise ValueError(
                    f'mask shape {mask.shape} does not match '
                    f'{(height, width)}')
            selected.append(SegmentationInstance(
                class_name=str(instance.class_name),
                confidence=float(np.clip(instance.confidence, 0.0, 1.0)),
                mask=mask,
            ))

        boundary_instances = [
            item for item in selected
            if item.class_name == self.config.boundary_class_name
        ]
        center_instances = [
            item for item in selected
            if item.class_name == self.config.center_class_name
        ]
        rows: list[dict] = []
        minimum_lane_width = width * self.config.min_lane_width_ratio
        consistency_gaps: list[float] = []

        for ratio_value in sorted(float(row) for row in self.config.scan_rows):
            row_y = int(round(ratio_value * (height - 1)))
            boundary_hits = []
            for instance in boundary_instances:
                x = self._sample_x(instance.mask, row_y)
                if x is not None:
                    boundary_hits.append((x, instance.confidence))
            boundary_hits.sort(key=lambda item: item[0])

            left = right = None
            if len(boundary_hits) >= 2:
                candidate_left = boundary_hits[0]
                candidate_right = boundary_hits[-1]
                if (
                    candidate_right[0] - candidate_left[0]
                    >= minimum_lane_width
                ):
                    left, right = candidate_left, candidate_right

            expected_center = (
                (left[0] + right[0]) * 0.5
                if left is not None and right is not None
                else width * 0.5
            )
            center_hits = []
            for instance in center_instances:
                x = self._sample_x(instance.mask, row_y)
                if x is not None:
                    center_hits.append((x, instance.confidence))
            center = (
                min(
                    center_hits,
                    key=lambda item: abs(item[0] - expected_center),
                )
                if center_hits else None
            )

            target_x = None
            source = 'NONE'
            row_confidence = 0.0
            consistency_gap = None
            if left is not None and right is not None:
                target_x = expected_center
                source = 'BOTH_LANES'
                row_confidence = float(np.sqrt(left[1] * right[1]))
                if center is not None:
                    consistency_gap = abs(center[0] - target_x) / max(
                        1.0, width * 0.5)
                    consistency_gaps.append(consistency_gap)
                    if consistency_gap > self.config.center_consistency_tol:
                        row_confidence *= 0.65
            elif center is not None:
                target_x = center[0]
                source = 'CENTER_MARKING'
                row_confidence = center[1] * 0.90

            rows.append({
                'ratio': ratio_value,
                'y': row_y,
                'left_x': None if left is None else float(left[0]),
                'center_x': None if center is None else float(center[0]),
                'right_x': None if right is None else float(right[0]),
                'target_x': None if target_x is None else float(target_x),
                'target_source': source,
                'confidence': float(row_confidence),
                'consistency_gap': (
                    None if consistency_gap is None else float(consistency_gap)
                ),
            })

        measured = []
        for row in rows:
            if row['target_x'] is None:
                continue
            measured.append({
                **row,
                'target_norm': row['target_x'] / max(1.0, width - 1.0),
            })

        valid_rows = len(measured)
        vertical_span = (
            0.0 if valid_rows < 2
            else measured[-1]['ratio'] - measured[0]['ratio']
        )
        coverage = valid_rows / max(1, len(rows))
        mean_row_confidence = (
            0.0 if not measured
            else float(np.mean([row['confidence'] for row in measured]))
        )
        confidence = mean_row_confidence * (0.65 + 0.35 * coverage)

        result = {
            'valid': False,
            'image_w': width,
            'image_h': height,
            'image_center_x': width * 0.5,
            'center_error': None,
            'heading_error': None,
            'lane_center_x': None,
            'look_ahead_ratio': self.config.look_ahead_ratio,
            'target_source': 'NONE',
            'lane_state': (
                'BOTH' if len(boundary_instances) >= 2
                else 'ONE' if boundary_instances else 'NONE'
            ),
            'scan_rows': rows,
            'fit_path': [],
            'valid_rows': valid_rows,
            'vertical_span_ratio': float(vertical_span),
            'coverage': float(coverage),
            'confidence': float(np.clip(confidence, 0.0, 1.0)),
            'consistency_warning': any(
                gap > self.config.center_consistency_tol
                for gap in consistency_gaps
            ),
            'center_consistency_gap': (
                None if not consistency_gaps else float(max(consistency_gaps))
            ),
            'boundary_instances': len(boundary_instances),
            'center_instances': len(center_instances),
        }
        if (
            valid_rows < self.config.min_valid_rows
            or vertical_span < self.config.min_vertical_span_ratio
        ):
            return result

        observed_low = measured[0]['ratio']
        observed_high = measured[-1]['ratio']
        extrapolation = self.config.max_lookahead_extrapolation_ratio
        look = self.config.look_ahead_ratio
        if (
            look < observed_low - extrapolation
            or look > observed_high + extrapolation
        ):
            return result

        coefficients, degree = self._fit_path(measured)
        target_norm = float(np.clip(
            self._evaluate(coefficients, look), 0.0, 1.0))
        target_x = target_norm * (width - 1.0)
        center_error = float(np.clip(
            (target_x - width * 0.5) / max(1.0, width * 0.5),
            -1.0,
            1.0,
        ))

        far_ratio = max(observed_low, self.config.heading_far_ratio)
        near_ratio = min(observed_high, self.config.heading_near_ratio)
        heading_error = None
        if near_ratio - far_ratio >= 0.10:
            far_x = self._evaluate(coefficients, far_ratio) * (width - 1.0)
            near_x = self._evaluate(coefficients, near_ratio) * (width - 1.0)
            heading_error = float(np.clip(
                (far_x - near_x) / max(1.0, width * 0.5),
                -1.0,
                1.0,
            ))

        fit_path = []
        for ratio_value in np.linspace(observed_low, observed_high, 20):
            x = float(np.clip(
                self._evaluate(coefficients, float(ratio_value)), 0.0, 1.0
            ) * (width - 1.0))
            fit_path.append({
                'x': x,
                'y': int(round(float(ratio_value) * (height - 1))),
            })

        sources = [row['target_source'] for row in measured]
        both_count = sources.count('BOTH_LANES')
        target_source = (
            'BOTH_LANES' if both_count >= (valid_rows + 1) // 2
            else 'CENTER_MARKING'
        )
        plan_valid = confidence >= self.config.minimum_plan_confidence
        result.update({
            'valid': bool(plan_valid),
            'center_error': center_error,
            'heading_error': heading_error,
            'lane_center_x': target_x,
            'target_source': target_source,
            'fit_degree': degree,
            'fit_path': fit_path,
        })
        return result

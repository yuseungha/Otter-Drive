"""Scan-line lane geometry extraction without ROS dependencies."""

from __future__ import annotations

from dataclasses import dataclass
from itertools import groupby
from typing import Dict, Iterable, List, Optional, Sequence, Tuple

import numpy as np


Box = Optional[Sequence[float]]


@dataclass(frozen=True)
class ScanCandidate:
    """One locally bright low-high-low run on a scan row."""

    x: float
    width_px: float
    reference_width_px: float
    contrast: float
    start_x: int
    end_x: int


@dataclass
class _AlphaBetaTrack:
    """Small constant-velocity tracker for one line at one scan row."""

    process_gain: float
    velocity_gain: float
    x: Optional[float] = None
    velocity: float = 0.0
    miss_count: int = 0

    def predict(self) -> Optional[float]:
        """Return the next constant-velocity prediction."""
        if self.x is None:
            return None
        return self.x + self.velocity

    def update(self, measurement: Optional[float]) -> Optional[float]:
        """Correct with a measurement or advance prediction-only state."""
        predicted = self.predict()
        if measurement is None:
            if predicted is not None:
                self.x = predicted
                self.miss_count += 1
            return self.x
        value = float(measurement)
        if predicted is None:
            self.x = value
            self.velocity = 0.0
        else:
            residual = value - predicted
            self.x = predicted + self.process_gain * residual
            self.velocity += self.velocity_gain * residual
        self.miss_count = 0
        return self.x


def _contiguous_runs(mask: np.ndarray) -> Iterable[Tuple[int, int]]:
    indices = np.flatnonzero(mask)
    for _key, values in groupby(enumerate(indices), lambda item: item[1] - item[0]):
        group = [item[1] for item in values]
        yield group[0], group[-1] + 1


def _inside_box(x: float, y: int, box: Box) -> bool:
    if box is None or len(box) != 4:
        return False
    x1, y1, x2, y2 = (float(value) for value in box)
    return x1 <= x <= x2 and y1 <= y <= y2


def find_scanline_candidates(
    gray: np.ndarray,
    row_y: int,
    boxes: Sequence[Box] = (),
    contrast_offset: float = 35.0,
    min_line_width_px: float = 4.0,
    max_line_width_px: float = 26.0,
    far_row_width_scale: float = 0.6,
    row_ratio_min: float = 0.62,
    row_ratio_max: float = 0.90,
    paired_edge_required: bool = True,
    reference_width_px: float = 640.0,
) -> List[ScanCandidate]:
    """Find locally contrasted, paired-edge bright runs on one image row."""
    if gray is None or gray.ndim != 2 or gray.size == 0:
        raise ValueError('gray must be a non-empty HxW image')
    height, width = gray.shape
    if not 0 <= int(row_y) < height:
        raise ValueError('row_y is outside the image')

    band_low = max(0, int(row_y) - 1)
    band_high = min(height, int(row_y) + 2)
    row = np.median(gray[band_low:band_high].astype(np.float32), axis=0)
    floor = float(np.median(row))
    bright = row >= floor + float(contrast_offset)

    # Join one-pixel holes caused by compression noise without widening lines.
    if width >= 3:
        holes = (~bright[1:-1]) & bright[:-2] & bright[2:]
        bright[1:-1] |= holes

    ratio = float(row_y) / max(1.0, float(height - 1))
    span = max(1e-6, float(row_ratio_max) - float(row_ratio_min))
    progress = float(np.clip((ratio - row_ratio_min) / span, 0.0, 1.0))
    perspective_scale = far_row_width_scale + (1.0 - far_row_width_scale) * progress
    image_scale = width / max(1.0, float(reference_width_px))
    minimum = max(1.0, min_line_width_px * image_scale * perspective_scale)
    maximum = max(minimum, max_line_width_px * image_scale * perspective_scale)
    candidates: List[ScanCandidate] = []

    active_boxes = [box for box in boxes if box is not None and len(box) == 4]
    for start, end in _contiguous_runs(bright):
        run_width = float(end - start)
        if run_width < minimum or run_width > maximum:
            continue
        center = (start + end - 1) / 2.0
        if active_boxes and not any(
            _inside_box(center, int(row_y), box) for box in active_boxes
        ):
            continue

        side_width = max(2, min(8, int(round(run_width))))
        left = row[max(0, start - side_width):start]
        right = row[end:min(width, end + side_width)]
        if left.size == 0 or right.size == 0:
            continue
        run = row[start:end]
        run_level = float(np.mean(run))
        left_level = float(np.median(left))
        right_level = float(np.median(right))
        paired_contrast = run_level - max(left_level, right_level)
        required_drop = max(6.0, float(contrast_offset) * 0.55)
        if paired_contrast < required_drop:
            continue
        if paired_edge_required:
            edge_width = max(1, min(3, int(round(run_width / 3.0))))
            left_edge = float(np.mean(run[:edge_width])) - float(np.mean(left[-edge_width:]))
            right_edge = float(np.mean(run[-edge_width:])) - float(np.mean(right[:edge_width]))
            edge_minimum = max(4.0, float(contrast_offset) * 0.30)
            if left_edge < edge_minimum or right_edge < edge_minimum:
                continue
        candidates.append(ScanCandidate(
            x=center,
            width_px=run_width,
            reference_width_px=run_width / max(1e-6, image_scale),
            contrast=run_level - (left_level + right_level) / 2.0,
            start_x=start,
            end_x=end,
        ))
    return candidates


class LaneFeatureTracker:
    """Validate three lane markings and maintain Lambda-ROI predictions."""

    ROLES = ('left', 'center', 'right')

    def __init__(
        self,
        scan_rows: Sequence[float] = (0.62, 0.70, 0.80, 0.90),
        look_ahead_ratio: float = 0.80,
        contrast_offset: float = 35.0,
        min_line_width_px: float = 4.0,
        max_line_width_px: float = 26.0,
        far_row_width_scale: float = 0.6,
        paired_edge_required: bool = True,
        target_mode: str = 'road_center',
        center_consistency_tol: float = 0.08,
        lane_half_width_px: float = 0.0,
        lambda_roi_margin_px: float = 40.0,
        lambda_roi_decay_frames: int = 15,
        track_process_var: float = 4.0,
        track_measure_var: float = 25.0,
        max_predicted_frames: int = 10,
        reference_width_px: float = 640.0,
    ) -> None:
        if not scan_rows:
            raise ValueError('scan_rows cannot be empty')
        if target_mode not in {'road_center', 'left_lane', 'right_lane'}:
            raise ValueError('unsupported target_mode')
        self.scan_rows = tuple(float(np.clip(value, 0.0, 1.0)) for value in scan_rows)
        self.look_ahead_ratio = float(look_ahead_ratio)
        self.contrast_offset = float(contrast_offset)
        self.min_line_width_px = float(min_line_width_px)
        self.max_line_width_px = float(max_line_width_px)
        self.far_row_width_scale = float(far_row_width_scale)
        self.paired_edge_required = bool(paired_edge_required)
        self.target_mode = target_mode
        self.center_consistency_tol = abs(float(center_consistency_tol))
        self.lane_half_width_px = max(0.0, float(lane_half_width_px))
        self.lambda_roi_margin_px = max(1.0, float(lambda_roi_margin_px))
        self.lambda_roi_decay_frames = max(0, int(lambda_roi_decay_frames))
        self.max_predicted_frames = max(0, int(max_predicted_frames))
        self.reference_width_px = max(1.0, float(reference_width_px))

        total_variance = max(1e-6, float(track_process_var) + float(track_measure_var))
        alpha = float(np.clip(track_process_var / total_variance + 0.35, 0.20, 0.85))
        beta = float(np.clip(alpha * 0.25, 0.05, 0.30))
        self._tracks: Dict[Tuple[int, str], _AlphaBetaTrack] = {
            (index, role): _AlphaBetaTrack(alpha, beta)
            for index in range(len(self.scan_rows))
            for role in self.ROLES
        }

    def reset(self) -> None:
        """Clear all accumulated line state."""
        for track in self._tracks.values():
            track.x = None
            track.velocity = 0.0
            track.miss_count = 0

    @staticmethod
    def _box_center(box: Box) -> Optional[float]:
        if box is None or len(box) != 4:
            return None
        return (float(box[0]) + float(box[2])) / 2.0

    def _anchors(self, width: int, left_box: Box, right_box: Box) -> Dict[str, float]:
        image_center = width / 2.0
        if left_box is not None and right_box is not None:
            seam = (float(left_box[2]) + float(right_box[0])) / 2.0
        else:
            seam = image_center
        return {
            'left': float(left_box[0]) if left_box is not None else width * 0.12,
            'center': seam,
            'right': float(right_box[2]) if right_box is not None else width * 0.88,
        }

    def _assign_candidates(
        self,
        row_index: int,
        candidates: Sequence[ScanCandidate],
        width: int,
        left_box: Box,
        right_box: Box,
    ) -> Dict[str, ScanCandidate]:
        anchors = self._anchors(width, left_box, right_box)
        seam = anchors['center']
        scale = width / self.reference_width_px
        margin = max(2.0, self.lambda_roi_margin_px * scale)
        assigned: Dict[str, ScanCandidate] = {}
        used = set()
        scored = []
        for role in self.ROLES:
            track = self._tracks[(row_index, role)]
            prediction = track.predict()
            anchor = prediction if prediction is not None else anchors[role]
            for candidate_index, candidate in enumerate(candidates):
                if role == 'left' and candidate.x >= seam - width * 0.04:
                    continue
                if role == 'right' and candidate.x <= seam + width * 0.04:
                    continue
                if role == 'center' and abs(candidate.x - seam) > width * 0.24:
                    continue
                distance = abs(candidate.x - anchor)
                lambda_active = (
                    prediction is not None
                    and track.miss_count <= self.lambda_roi_decay_frames
                )
                penalty = margin * 3.0 if lambda_active and distance > margin else 0.0
                initial_limit = max(margin * 1.5, width * 0.20)
                if prediction is None and distance > initial_limit:
                    continue
                scored.append((distance + penalty, role, candidate_index))
        for _score, role, candidate_index in sorted(scored):
            if role in assigned or candidate_index in used:
                continue
            assigned[role] = candidates[candidate_index]
            used.add(candidate_index)
        return assigned

    def _box_fallback(self, left_box: Box, right_box: Box) -> Optional[float]:
        left_center = self._box_center(left_box)
        right_center = self._box_center(right_box)
        if left_center is not None and right_center is not None:
            return (left_center + right_center) / 2.0
        if left_center is not None:
            return left_center + self.lane_half_width_px
        if right_center is not None:
            return right_center - self.lane_half_width_px
        return None

    def _target_for_row(
        self,
        points: Dict[str, dict],
        left_box: Box,
        right_box: Box,
        width: int,
    ) -> dict:
        left = points.get('left')
        center = points.get('center')
        right = points.get('right')
        consistency_gap = None
        warning = False

        if self.target_mode == 'road_center':
            if left is not None and right is not None:
                target_x = (left['x'] + right['x']) / 2.0
                source = 'BOTH_EDGES'
                used = (left, right)
                if center is not None:
                    consistency_gap = abs(target_x - center['x']) / max(1.0, width / 2.0)
                    warning = consistency_gap > self.center_consistency_tol
            elif center is not None:
                target_x = center['x']
                source = 'DASHED'
                used = (center,)
            elif left is not None and self.lane_half_width_px > 0.0:
                target_x = left['x'] + self.lane_half_width_px * width / self.reference_width_px
                source = 'ONE_EDGE'
                used = (left,)
            elif right is not None and self.lane_half_width_px > 0.0:
                target_x = right['x'] - self.lane_half_width_px * width / self.reference_width_px
                source = 'ONE_EDGE'
                used = (right,)
            else:
                target_x = self._box_fallback(left_box, right_box)
                source = 'BOX'
                used = ()
        else:
            edge = right if self.target_mode == 'right_lane' else left
            if center is not None and edge is not None:
                target_x = (center['x'] + edge['x']) / 2.0
                source = 'BOTH_EDGES'
                used = (center, edge)
            elif center is not None:
                offset = self.lane_half_width_px * width / self.reference_width_px / 2.0
                target_x = center['x'] + (offset if self.target_mode == 'right_lane' else -offset)
                source = 'ONE_EDGE'
                used = (center,)
            else:
                target_x = self._box_fallback(left_box, right_box)
                source = 'BOX'
                used = ()

        predicted = any(point.get('source') == 'predicted' for point in used)
        measured = [point for point in used if point.get('source') == 'measured']
        width_px = None
        contrast = None
        if measured:
            widths = [
                point['width_px_reference']
                for point in measured
                if point['width_px_reference'] is not None
            ]
            contrasts = [point['contrast'] for point in measured if point['contrast'] is not None]
            width_px = float(np.mean(widths)) if widths else None
            contrast = float(np.mean(contrasts)) if contrasts else None
        row_source = (
            'box_fallback'
            if source == 'BOX'
            else ('predicted' if predicted else 'measured')
        )
        return {
            'target_x': target_x,
            'target_source': source,
            'source': row_source,
            'predicted': predicted,
            'width_px': width_px,
            'contrast': contrast,
            'center_consistency_gap': consistency_gap,
            'consistency_warning': warning,
        }

    def process(
        self,
        gray: np.ndarray,
        left_box: Box,
        right_box: Box,
    ) -> dict:
        """Extract tracked geometry and a road-center target from one frame."""
        if gray is None or gray.ndim != 2 or gray.size == 0:
            raise ValueError('gray must be a non-empty HxW image')
        height, width = gray.shape
        rows = []
        for row_index, row_ratio in enumerate(self.scan_rows):
            row_y = int(round(row_ratio * max(1, height - 1)))
            candidates = find_scanline_candidates(
                gray,
                row_y,
                boxes=(left_box, right_box),
                contrast_offset=self.contrast_offset,
                min_line_width_px=self.min_line_width_px,
                max_line_width_px=self.max_line_width_px,
                far_row_width_scale=self.far_row_width_scale,
                row_ratio_min=min(self.scan_rows),
                row_ratio_max=max(self.scan_rows),
                paired_edge_required=self.paired_edge_required,
                reference_width_px=self.reference_width_px,
            )
            assigned = self._assign_candidates(
                row_index, candidates, width, left_box, right_box)
            points: Dict[str, dict] = {}
            for role in self.ROLES:
                candidate = assigned.get(role)
                track = self._tracks[(row_index, role)]
                value = track.update(None if candidate is None else candidate.x)
                if value is not None:
                    track.x = float(np.clip(value, 0.0, width - 1.0))
                    track.velocity = float(np.clip(
                        track.velocity, -width * 0.25, width * 0.25))
                    value = track.x
                if candidate is not None:
                    points[role] = {
                        'x': float(value),
                        'source': 'measured',
                        'width_px': candidate.width_px,
                        'width_px_reference': candidate.reference_width_px,
                        'contrast': candidate.contrast,
                    }
                elif value is not None and track.miss_count <= self.max_predicted_frames:
                    points[role] = {
                        'x': float(value),
                        'source': 'predicted',
                        'width_px': None,
                        'width_px_reference': None,
                        'contrast': None,
                    }
            target = self._target_for_row(points, left_box, right_box, width)
            rows.append({
                'ratio': row_ratio,
                'y': row_y,
                'left_x': None if points.get('left') is None else points['left']['x'],
                'center_x': None if points.get('center') is None else points['center']['x'],
                'right_x': None if points.get('right') is None else points['right']['x'],
                'points': points,
                **target,
                'lambda_roi_active': any(
                    self._tracks[(row_index, role)].x is not None
                    and self._tracks[(row_index, role)].miss_count
                    <= self.lambda_roi_decay_frames
                    for role in self.ROLES
                ),
            })

        look_index = min(
            range(len(rows)),
            key=lambda index: abs(rows[index]['ratio'] - self.look_ahead_ratio),
        )
        look = rows[look_index]
        look['look_ahead'] = True
        for index, row in enumerate(rows):
            if index != look_index:
                row['look_ahead'] = False

        far = rows[0]
        near = rows[-1]
        heading_error = None
        if (
            far['target_x'] is not None
            and near['target_x'] is not None
            and far['source'] == 'measured'
            and near['source'] != 'box_fallback'
        ):
            heading_error = float(np.clip(
                (far['target_x'] - near['target_x']) / max(1.0, width / 2.0),
                -1.0,
                1.0,
            ))

        outer_left = look['points'].get('left')
        outer_right = look['points'].get('right')
        if outer_left is not None and outer_right is not None:
            lane_state = 'BOTH'
        elif outer_left is not None:
            lane_state = 'LEFT-ONLY'
        elif outer_right is not None:
            lane_state = 'RIGHT-ONLY'
        else:
            lane_state = 'NONE'

        source_confidence = {
            'BOTH_EDGES': 1.0,
            'DASHED': 0.90,
            'ONE_EDGE': 0.65,
            'BOX': 0.45,
        }.get(look['target_source'], 0.0)
        if look['predicted']:
            source_confidence *= 0.60
        target_x = look['target_x']
        center_error = None
        if target_x is not None:
            center_error = float(np.clip(
                (target_x - width / 2.0) / max(1.0, width / 2.0),
                -1.0,
                1.0,
            ))
        used_single_side = look['target_source'] == 'ONE_EDGE'
        return {
            'image_w': width,
            'image_h': height,
            'image_center_x': width / 2.0,
            'target_mode': self.target_mode,
            'target_source': look['target_source'],
            'center_consistency_gap': look['center_consistency_gap'],
            'consistency_warning': look['consistency_warning'],
            'lane_center_x': target_x,
            'center_error': center_error,
            'heading_error': heading_error,
            'left_box': None if left_box is None else [float(v) for v in left_box],
            'right_box': None if right_box is None else [float(v) for v in right_box],
            'scan_rows': rows,
            'lane_state': lane_state,
            'used_single_side': used_single_side,
            'single_side_assumed_px': (
                self.lane_half_width_px if used_single_side else 0.0),
            'predicted': bool(look['predicted']),
            'feature_validated': look['target_source'] != 'BOX',
            'feature_confidence_scale': source_confidence,
            'look_ahead_ratio': look['ratio'],
            'lambda_roi_active': bool(look['lambda_roi_active']),
        }

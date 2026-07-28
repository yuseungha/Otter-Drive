import cv2
import math
import numpy as np


class LaneDetector:
    """BEV lane extraction with glare rejection and temporally stable curve fits."""

    def __init__(self, parameters):
        self.p = parameters
        self._previous_fits = [None, None]

    def _filter_components(self, mask):
        count, labels, stats, _ = cv2.connectedComponentsWithStats(mask, connectivity=8)
        filtered = np.zeros_like(mask)
        min_area = int(self.p['min_component_area'])
        min_aspect = float(self.p['min_component_aspect'])
        for label in range(1, count):
            x, y, width, height, area = stats[label]
            aspect = height / max(width, 1)
            if area >= min_area and aspect >= min_aspect:
                filtered[labels == label] = 255
        return filtered

    def _robust_fit(self, ys, xs, previous):
        if len(xs) < self.p['min_pixels']:
            return previous, False, 0.0
        fit = np.polyfit(ys, xs, 2)
        residual = np.abs(xs - np.polyval(fit, ys))
        inliers = residual < float(self.p['fit_residual_px'])
        inlier_ratio = float(np.count_nonzero(inliers)) / float(len(xs))
        if np.count_nonzero(inliers) >= self.p['min_pixels']:
            fit = np.polyfit(ys[inliers], xs[inliers], 2)
        if previous is not None:
            alpha = float(self.p['temporal_alpha'])
            fit = alpha * fit + (1.0 - alpha) * previous
        confidence = min(1.0, len(xs) / float(self.p['min_pixels'] * 4)) * inlier_ratio
        return fit, True, confidence

    def detect(self, image):
        height, width = image.shape[:2]
        p = self.p
        src = (np.float32(p['src_pts']).reshape(4, 2) * np.float32([width, height])).astype(np.float32)
        dst = (np.float32(p['dst_pts']).reshape(4, 2) * np.float32([width, height])).astype(np.float32)
        bev = cv2.warpPerspective(image, cv2.getPerspectiveTransform(src, dst), (width, height))

        hls = cv2.cvtColor(bev, cv2.COLOR_BGR2HLS)
        hsv = cv2.cvtColor(bev, cv2.COLOR_BGR2HSV)
        white_mask = cv2.inRange(hls, np.array(p['white_hls_low']), np.array(p['white_hls_high']))
        yellow_mask = cv2.inRange(hsv, np.array(p['yellow_hsv_low']), np.array(p['yellow_hsv_high']))
        color_mask = cv2.bitwise_or(white_mask, yellow_mask)

        # Glare is bright but often lacks a lane-like edge. Preserve only colour
        # candidates that are supported by a local luminance edge.
        lightness = hls[:, :, 1]
        edges = cv2.Canny(lightness, int(p['canny_low']), int(p['canny_high']))
        edge_support = cv2.dilate(edges, np.ones((3, 3), np.uint8), iterations=1)
        mask = cv2.bitwise_and(color_mask, edge_support)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, np.ones((3, 9), np.uint8))
        mask = self._filter_components(mask)

        ys, xs = mask.nonzero()
        midpoint = width // 2
        fits, observed, confidences = [], [], []
        for index, (low, high) in enumerate(((0, midpoint), (midpoint, width))):
            keep = (xs >= low) & (xs < high)
            fit, seen, confidence = self._robust_fit(ys[keep], xs[keep], self._previous_fits[index])
            fits.append(fit)
            observed.append(seen)
            confidences.append(confidence)
            if fit is not None:
                self._previous_fits[index] = fit
        left, right = fits

        lookahead_y = int(height * p['lookahead_ratio'])
        if left is not None and right is not None:
            lane_x = (np.polyval(left, lookahead_y) + np.polyval(right, lookahead_y)) / 2.0
            fit = (left + right) / 2.0
            heading = math.atan(2.0 * fit[0] * lookahead_y + fit[1])
        else:
            lane_x = width / 2.0
            heading = 0.0

        offset = float(lane_x - width / 2.0)
        steering = p['offset_gain'] * offset / (width / 2.0) + p['heading_gain'] * heading
        debug = bev.copy()
        cv2.line(debug, (width // 2, height), (width // 2, int(height * .55)), (0, 0, 255), 2)
        cv2.circle(debug, (int(np.clip(lane_x, 0, width - 1)), lookahead_y), 6, (0, 255, 0), -1)
        # Previous fits may keep the visualization smooth, but never turn a
        # missing current observation into a control-valid lane estimate.
        lane_valid = bool(observed[0] and observed[1])
        confidence = min(confidences) if lane_valid else 0.0
        return offset, heading, float(steering), debug, mask, lane_valid, confidence

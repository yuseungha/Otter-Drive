#!/usr/bin/env python3
# rubbercone_node.py (xycar_planning package)
#
# 시뮬레이션에서 검증된 두 접근(콘 페어링 + Pure Pursuit, Follow-the-Gap 폴백)을
# 오늘 실제 하드웨어 테스트로 검증한 안전장치(모양 필터, 최소 간격 검사, 목표점
# 급변 제한)와 합친 버전. S자 커브 대응력 향상이 핵심 목표.
#
# 변경 핵심:
#   1. 고정/자동 거리구간(bin) 스캔 방식 -> "가장 가까운 왼쪽 콘 + 그와 전후방
#      거리(x)가 가장 비슷한 오른쪽 콘"을 짝짓는 방식으로 교체. S자 커브에서
#      같은 열 콘끼리 잘못 짝지어지는 문제를 구조적으로 줄임.
#   2. 단순 PD 대신 휠베이스 기반 Pure Pursuit 조향식 사용.
#   3. 콘을 하나도 못 찾으면 Follow-the-Gap으로 안전하게 폴백.
#   4. 부채꼴(각도+거리) 기반 구간 진입/이탈 감지, 연속 프레임 카운팅으로
#      노이즈에 의한 오탐 방지. '/rubbercone/zone_active'로 발행 (나중에
#      mission_node가 참고할 수 있음).
#
# 실측 검증된 것 (오늘 하드웨어 테스트로 확인):
#   - 라이다 실제 드라이버: xycar_lidar_node (YDLIDAR, /dev/ttyUSB1, 230400bps)
#   - 모터 토픽: 'xycar_motor', Float32MultiArray, data=[angle, speed]
#
# 아직 실측 필요:
#   - wheelbase_m: 기본값은 추정치. 실제 차량 앞뒤 축간거리를 자로 재서 넣을 것.
#   - steer_gain, angle_offset_deg: 시뮬레이션 값 그대로 가져온 것. 실차에서
#     angle_offset_deg부터 먼저 보정(정면에 물체 하나 놓고 y=0 확인)한 뒤,
#     steer_gain은 실제 조향 반응 보고 튜닝할 것.
#
# IMPORTANT: lane_control_node와 동시에 켜지 마세요. 둘 다 'xycar_motor'에
# 발행해서 충돌합니다.

import math
import os
import numpy as np
import cv2
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import LaserScan, Image
from std_msgs.msg import Float32MultiArray, Bool


class RubberconeNode(Node):
    def __init__(self):
        super().__init__('rubbercone_node')

        # ---- 공통 ----
        self.declare_parameter('scan_topic', 'scan')
        self.declare_parameter('drive_topic', 'xycar_motor')  # launch with -p drive_topic:=/planning/cone_cmd under mission_node
        # Debug 폴더에서는 KMU_AutoDriving의 차량 제어와 충돌하지 않도록
        # 기본적으로 모터 명령을 발행하지 않는다.
        self.declare_parameter('publish_drive_command', False)
        self.declare_parameter('debug_topic', '/rubbercone/debug_image')
        self.declare_parameter('zone_topic', '/rubbercone/zone_active')
        self.declare_parameter('show_debug_window', True)
        self.declare_parameter('debug_window_name', 'Rubbercone LiDAR Pair Debug')

        # ---- 좌표계 보정 (실측 필요, 오늘 라이다 검증 때 쓰던 것과 동일) ----
        self.declare_parameter('angle_offset_deg', 0.0)

        # ---- 관심영역(ROI) - 콘 탐지용 ----
        self.declare_parameter('forward_min', 0.15)
        self.declare_parameter('forward_max', 2.5)
        self.declare_parameter('max_angle_deg', 100.0)

        # ---- 클러스터링 + 콘 모양 필터 ----
        self.declare_parameter('cluster_gap_m', 0.15)
        self.declare_parameter('cluster_min_points', 2)
        self.declare_parameter('cluster_max_span_m', 0.4)  # 이보다 크면 벽/사람 등으로 간주해 버림
        self.declare_parameter('cone_merge_dist_m', 0.25)  # 이보다 가까운 콘 중심점끼리는 같은 콘으로 보고 합침

        # ---- 콘 페어링 (핵심 변경 지점) ----
        self.declare_parameter('pair_max_x_diff_m', 0.5)   # 좌우 콘의 전후방 거리 차가 이보다 크면 짝 안 지음
        self.declare_parameter('min_gap_m', 0.5)            # 짝지은 폭이 이보다 좁으면 같은 열 콘으로 보고 버림
        self.declare_parameter('single_side_offset_m', 0.35)  # 한쪽만 보일 때 반대쪽으로 띄우는 폭

        # ---- Pure Pursuit 조향 ----
        self.declare_parameter('wheelbase_m', 0.26)   # 실측 필요
        self.declare_parameter('lookahead_min_m', 0.3)
        self.declare_parameter('steer_gain', 120.0)   # rad -> angle 스케일 (시뮬레이션 값, 실차 재튜닝 필요)
        self.declare_parameter('angle_limit', 50.0)
        self.declare_parameter('invert_steer', False)

        # ---- 목표점 안정화 (오늘 실제로 필요했던 안전장치) ----
        self.declare_parameter('target_smoothing_alpha', 0.5)
        self.declare_parameter('max_target_step_m', 0.12)

        # ---- 속도 ----
        self.declare_parameter('base_speed', 8.0)  # 실측 확인: 8 이상에서 급커브 멈춤 없이 안정적 (min_absolute_speed=5와 조합)
        self.declare_parameter('min_speed_ratio', 0.4)  # 급조향 시 base_speed 대비 최소 남기는 비율
        self.declare_parameter('min_absolute_speed', 5.0)  # 위 비율 계산과 무관하게 이 값 밑으로는 안 내려감 (모터 저속 한계 이상으로 유지)
        self.declare_parameter('lost_speed_ratio', 0.7)  # 콘을 못 찾아 FTG 폴백할 때 감속 비율

        # ---- Follow-the-Gap 폴백 ----
        self.declare_parameter('car_width_m', 0.3)
        self.declare_parameter('max_steering_angle_rad', 0.4)

        # ---- 구간 진입/이탈 감지 (부채꼴: 각도+거리) ----
        self.declare_parameter('zone_near_m', 0.2)
        self.declare_parameter('zone_far_m', 1.5)
        self.declare_parameter('zone_half_angle_deg', 55.0)
        self.declare_parameter('zone_point_threshold', 20)
        self.declare_parameter('zone_enter_frames', 5)
        self.declare_parameter('zone_exit_frames', 5)

        p = self.get_parameter
        self.scan_topic = p('scan_topic').value
        self.drive_topic = p('drive_topic').value
        self.publish_drive_command = bool(p('publish_drive_command').value)
        self.debug_topic = p('debug_topic').value
        self.zone_topic = p('zone_topic').value
        self.show_debug_window = bool(p('show_debug_window').value)
        self.debug_window_name = str(p('debug_window_name').value)

        # OpenCV HighGUI는 표시 서버가 없는 SSH 세션에서 프로세스 전체를
        # 종료시킬 수 있으므로 DISPLAY/Wayland가 있을 때만 창을 연다.
        self.debug_window_enabled = self.show_debug_window and bool(
            os.environ.get('DISPLAY') or os.environ.get('WAYLAND_DISPLAY'))
        self.debug_window_initialized = False
        if self.show_debug_window and not self.debug_window_enabled:
            self.get_logger().warning(
                'Debug window disabled: DISPLAY/WAYLAND_DISPLAY is not set. '
                'Use SSH X11 forwarding (ssh -X or ssh -Y), or view '
                f'{self.debug_topic} with rqt_image_view on the laptop.')

        self.angle_offset = math.radians(p('angle_offset_deg').value)

        self.forward_min = p('forward_min').value
        self.forward_max = p('forward_max').value
        self.max_angle = math.radians(p('max_angle_deg').value)

        self.cluster_gap_m = p('cluster_gap_m').value
        self.cluster_min_points = p('cluster_min_points').value
        self.cluster_max_span_m = p('cluster_max_span_m').value
        self.cone_merge_dist_m = p('cone_merge_dist_m').value

        self.pair_max_x_diff_m = p('pair_max_x_diff_m').value
        self.min_gap_m = p('min_gap_m').value
        self.single_side_offset_m = p('single_side_offset_m').value

        self.wheelbase = p('wheelbase_m').value
        self.lookahead_min = p('lookahead_min_m').value
        self.steer_gain = p('steer_gain').value
        self.angle_limit = p('angle_limit').value
        self.invert_steer = p('invert_steer').value

        self.target_smoothing_alpha = p('target_smoothing_alpha').value
        self.max_target_step_m = p('max_target_step_m').value

        self.base_speed = p('base_speed').value
        self.min_speed_ratio = p('min_speed_ratio').value
        self.min_absolute_speed = p('min_absolute_speed').value
        self.lost_speed_ratio = p('lost_speed_ratio').value

        self.car_width = p('car_width_m').value
        self.max_steering_angle_rad = p('max_steering_angle_rad').value

        self.zone_near_m = p('zone_near_m').value
        self.zone_far_m = p('zone_far_m').value
        self.zone_half_angle_rad = math.radians(p('zone_half_angle_deg').value)
        self.zone_point_threshold = p('zone_point_threshold').value
        self.zone_enter_frames = p('zone_enter_frames').value
        self.zone_exit_frames = p('zone_exit_frames').value

        self.smoothed_y = None
        self.last_time = self.get_clock().now()
        self.zone_active = False
        self.dense_count = 0
        self.sparse_count = 0

        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.scan_sub = self.create_subscription(LaserScan, self.scan_topic, self.scan_callback, qos)

        self.drive_pub = None
        if self.publish_drive_command:
            self.drive_pub = self.create_publisher(
                Float32MultiArray, self.drive_topic, 1)
        self.debug_pub = self.create_publisher(Image, self.debug_topic, 10)
        self.zone_pub = self.create_publisher(Bool, self.zone_topic, 10)

        drive_status = (
            self.drive_topic if self.publish_drive_command else 'DISABLED (debug only)')
        self.get_logger().info(
            'rubbercone_node started. scan: ' + self.scan_topic
            + ' -> drive: ' + drive_status)

    # ==================== 라이다 -> 차량 로컬 좌표 (전방=+x, 좌측=+y) ====================
    def _scan_to_points(self, msg: LaserScan):
        """ROI 없이 전체 유효 포인트를 반환 (구간 감지용으로도 재사용)."""
        points = []
        angle = msg.angle_min
        for r in msg.ranges:
            a = angle
            angle += msg.angle_increment
            if math.isinf(r) or math.isnan(r):
                continue
            if r < msg.range_min or r > msg.range_max:
                continue
            a_corrected = a + self.angle_offset
            x = r * math.cos(a_corrected)
            y = r * math.sin(a_corrected)
            points.append((x, y))
        return points

    def _apply_roi(self, points):
        return [(x, y) for (x, y) in points
                if self.forward_min <= x <= self.forward_max
                and abs(math.atan2(y, x)) <= self.max_angle]

    # ==================== 클러스터링 + 콘 모양 필터 ====================
    def _cluster_and_filter(self, points):
        if not points:
            return []
        pts = sorted(points, key=lambda p: math.atan2(p[1], p[0]))

        clusters = []
        current = [pts[0]]
        for pt in pts[1:]:
            if math.hypot(pt[0] - current[-1][0], pt[1] - current[-1][1]) <= self.cluster_gap_m:
                current.append(pt)
            else:
                clusters.append(current)
                current = [pt]
        clusters.append(current)

        cones = []
        for c in clusters:
            if len(c) < self.cluster_min_points:
                continue
            xs = [pt[0] for pt in c]
            ys = [pt[1] for pt in c]
            span = math.hypot(max(xs) - min(xs), max(ys) - min(ys))
            if span > self.cluster_max_span_m:
                continue  # 벽/사람 등 큰 물체는 콘이 아니라고 보고 버림
            cones.append((sum(xs) / len(xs), sum(ys) / len(ys)))
        return self._merge_nearby_cones(cones)

    def _merge_nearby_cones(self, cones):
        """라바콘 표면 반사가 고르지 않아 하나의 콘이 두 개 이상의 클러스터로
        쪼개지는 경우가 있음. 모양 필터를 통과한 작은 클러스터 중심점끼리
        cone_merge_dist_m 이내로 가까우면 같은 콘으로 보고 평균 위치로 합침
        (single-linkage 방식, 더 이상 합쳐질 게 없을 때까지 반복)."""
        merged = list(cones)
        changed = True
        while changed and len(merged) > 1:
            changed = False
            for i in range(len(merged)):
                for j in range(i + 1, len(merged)):
                    d = math.hypot(merged[i][0] - merged[j][0], merged[i][1] - merged[j][1])
                    if d <= self.cone_merge_dist_m:
                        mx = (merged[i][0] + merged[j][0]) / 2.0
                        my = (merged[i][1] + merged[j][1]) / 2.0
                        merged.pop(j)
                        merged.pop(i)
                        merged.append((mx, my))
                        changed = True
                        break
                if changed:
                    break
        return merged

    # ==================== 콘 페어링: 가장 가까운 왼쪽 콘 + x가 가장 비슷한 오른쪽 콘 ====================
    def _find_pair_target(self, cones):
        left = sorted([(x, y) for (x, y) in cones if y > 0.1], key=lambda p: p[0])
        right = sorted([(x, y) for (x, y) in cones if y < -0.1], key=lambda p: p[0])

        if left and right:
            lc = left[0]
            rc = min(right, key=lambda p: abs(p[0] - lc[0]))
            x_diff = abs(lc[0] - rc[0])
            gap = lc[1] - rc[1]  # 왼쪽 y(+) - 오른쪽 y(-) = 통로 폭
            if x_diff <= self.pair_max_x_diff_m and gap >= self.min_gap_m:
                return (
                    (lc[0] + rc[0]) / 2.0,
                    (lc[1] + rc[1]) / 2.0,
                    left,
                    right,
                    (lc, rc),
                    'PAIR_VALID',
                )

        if left:
            lx, ly = left[0]
            return lx, ly - self.single_side_offset_m, left, right, None, 'SINGLE_LEFT'
        if right:
            rx, ry = right[0]
            return rx, ry + self.single_side_offset_m, left, right, None, 'SINGLE_RIGHT'
        return None, None, left, right, None, 'NO_CONE'

    # ==================== Pure Pursuit 조향 ====================
    def _steer_to_target(self, tx, ty):
        dist = math.hypot(tx, ty)
        if dist < 1e-3:
            return 0.0
        ld = max(dist, self.lookahead_min)
        delta_rad = math.atan2(2.0 * self.wheelbase * (ty / dist), ld)
        angle = delta_rad * self.steer_gain
        return float(np.clip(angle, -self.angle_limit, self.angle_limit))

    # ==================== Follow-the-Gap 폴백 (콘을 하나도 못 찾았을 때) ====================
    def _gap_follow_angle(self, msg: LaserScan):
        ranges = np.array(msg.ranges, dtype=np.float32)
        ranges = np.where(np.isnan(ranges), 0.0, ranges)
        ranges = np.where(np.isinf(ranges), msg.range_max, ranges)
        ranges = np.clip(ranges, 0.0, msg.range_max)

        valid_mask = ranges > self.forward_min
        if not valid_mask.any():
            return 0.0
        masked = np.where(valid_mask, ranges, np.inf)
        closest_idx = int(np.argmin(masked))
        closest_dist = ranges[closest_idx]

        bubble_angle_rad = math.atan2(self.car_width / 2.0, max(closest_dist, 0.05))
        bubble_radius_idx = int(bubble_angle_rad / msg.angle_increment)
        b_start = max(0, closest_idx - bubble_radius_idx)
        b_end = min(len(ranges) - 1, closest_idx + bubble_radius_idx)
        processed = ranges.copy()
        processed[b_start:b_end + 1] = 0.0

        occ = (processed > self.forward_min).astype(np.int8)
        diff = np.diff(np.concatenate(([0], occ, [0])))
        gap_starts, gap_ends = np.where(diff == 1)[0], np.where(diff == -1)[0]
        if len(gap_starts) == 0:
            return 0.0
        largest = int(np.argmax(gap_ends - gap_starts))
        gs, ge = int(gap_starts[largest]), int(gap_ends[largest])
        if gs >= ge:
            best_idx = len(ranges) // 2
        else:
            seg = ranges[gs:ge]
            idxs = np.arange(gs, ge)
            best_idx = int(np.average(idxs, weights=seg)) if seg.sum() > 0 else (gs + ge) // 2

        steering_angle_rad = msg.angle_min + best_idx * msg.angle_increment
        mapped = -(steering_angle_rad / self.max_steering_angle_rad) * self.angle_limit
        return float(np.clip(mapped, -self.angle_limit, self.angle_limit))

    # ==================== 구간 진입/이탈 감지 (부채꼴: 각도+거리) ====================
    def _update_zone(self, all_points):
        if not all_points:
            sector_count = 0
        else:
            xs = np.array([p[0] for p in all_points])
            ys = np.array([p[1] for p in all_points])
            dist = np.hypot(xs, ys)
            ang = np.arctan2(ys, xs)
            sector = (dist >= self.zone_near_m) & (dist <= self.zone_far_m) & \
                     (np.abs(ang) <= self.zone_half_angle_rad)
            sector_count = int(np.sum(sector))

        dense = sector_count >= self.zone_point_threshold
        if dense:
            self.dense_count += 1
            self.sparse_count = 0
        else:
            self.sparse_count += 1
            self.dense_count = 0

        if not self.zone_active and self.dense_count >= self.zone_enter_frames:
            self.zone_active = True
            self.get_logger().info('CONE_ZONE entered')
        elif self.zone_active and self.sparse_count >= self.zone_exit_frames:
            self.zone_active = False
            self.get_logger().info('CONE_ZONE exited')

        flag = Bool()
        flag.data = self.zone_active
        self.zone_pub.publish(flag)

    # ==================== 메인 콜백 ====================
    def scan_callback(self, msg: LaserScan):
        all_points = self._scan_to_points(msg)
        roi_points = self._apply_roi(all_points)
        cones = self._cluster_and_filter(roi_points)

        tx, ty, left, right, selected_pair, target_mode = self._find_pair_target(cones)

        if ty is not None:
            # 목표점 급변 방지: 프레임당 최대 이동량 clamp + 약한 스무딩
            if self.smoothed_y is None:
                self.smoothed_y = ty
            else:
                delta = ty - self.smoothed_y
                if abs(delta) > self.max_target_step_m:
                    ty = self.smoothed_y + math.copysign(self.max_target_step_m, delta)
                a = self.target_smoothing_alpha
                self.smoothed_y = a * self.smoothed_y + (1.0 - a) * ty
            ty = self.smoothed_y

            angle = self._steer_to_target(tx, ty)
            speed = self.base_speed
            source = target_mode.lower()
        else:
            angle = self._gap_follow_angle(msg)
            speed = self.base_speed * self.lost_speed_ratio
            source = 'gap_fallback'

        if self.invert_steer:
            angle = -angle
        angle = float(np.clip(angle, -self.angle_limit, self.angle_limit))

        # 조향각이 클수록(급조향일수록) 속도를 줄임. angle_limit에서 min_speed_ratio까지
        # 선형으로 감속하고, 그 아래로는 안 떨어지게 함.
        steer_ratio = abs(angle) / self.angle_limit if self.angle_limit > 0 else 0.0
        speed_factor = max(self.min_speed_ratio, 1.0 - steer_ratio)
        speed = speed * speed_factor
        speed = max(speed, self.min_absolute_speed)  # 모터가 멈추지 않는 최소 속도 보장

        if self.drive_pub is not None:
            motor_msg = Float32MultiArray()
            motor_msg.data = [angle, float(speed)]
            self.drive_pub.publish(motor_msg)

        self._update_zone(all_points)
        self._publish_debug(
            roi_points, left, right, selected_pair, target_mode,
            tx, ty, angle, speed, source)

    # ==================== 디버그 시각화 ====================
    def _publish_debug(self, roi_points, left, right, selected_pair, target_mode,
                       tx, ty, angle, speed, source):
        size = 500
        scale = 150.0
        origin = (size // 2, size - 30)
        img = np.zeros((size, size, 3), dtype=np.uint8)

        for radius_m in (0.5, 1.0, 1.5, 2.0):
            cv2.circle(img, origin, int(radius_m * scale), (40, 40, 40), 1)

        def to_px(x, y):
            return int(origin[0] - y * scale), int(origin[1] - x * scale)

        for (x, y) in roi_points:
            px, py = to_px(x, y)
            if 0 <= px < size and 0 <= py < size:
                img[py, px] = (90, 90, 90)

        for (x, y) in left:
            cv2.circle(img, to_px(x, y), 7, (255, 0, 0), -1)   # 파랑 = 왼쪽 콘
        for (x, y) in right:
            cv2.circle(img, to_px(x, y), 7, (0, 255, 255), -1)  # 노랑 = 오른쪽 콘

        # 실제 페어링에 사용한 두 콘은 초록색 테두리와 연결선으로 강조한다.
        if selected_pair is not None:
            selected_left, selected_right = selected_pair
            left_px = to_px(*selected_left)
            right_px = to_px(*selected_right)
            cv2.line(img, left_px, right_px, (0, 255, 0), 3)
            cv2.circle(img, left_px, 11, (0, 255, 0), 2)
            cv2.circle(img, right_px, 11, (0, 255, 0), 2)

        cv2.circle(img, origin, 6, (0, 255, 0), -1)  # 차량 위치

        if tx is not None and ty is not None:
            cv2.circle(img, to_px(tx, ty), 8, (255, 0, 255), -1)  # 자주 = 목표점
            cv2.line(img, origin, to_px(tx, ty), (255, 0, 255), 1)

        if target_mode == 'PAIR_VALID':
            status_color = (0, 255, 0)
        elif target_mode.startswith('SINGLE_'):
            status_color = (0, 165, 255)
        else:
            status_color = (0, 0, 255)

        cv2.rectangle(img, (0, 0), (size, 66), (20, 20, 20), -1)
        cv2.putText(
            img, target_mode.replace('_', ' '), (10, 22),
            cv2.FONT_HERSHEY_SIMPLEX, 0.65, status_color, 2)
        cv2.putText(
            img, 'L=%d R=%d src=%s' % (len(left), len(right), source),
            (10, 43), cv2.FONT_HERSHEY_SIMPLEX, 0.48, (255, 255, 255), 1)
        cv2.putText(
            img, 'angle=%+.1f speed=%.1f zone=%s' %
            (angle, speed, self.zone_active),
            (10, 62), cv2.FONT_HERSHEY_SIMPLEX, 0.48, (255, 255, 255), 1)

        cv2.putText(img, 'LEFT', (size - 142, 22), cv2.FONT_HERSHEY_SIMPLEX,
                    0.42, (255, 0, 0), 1)
        cv2.putText(img, 'RIGHT', (size - 92, 22), cv2.FONT_HERSHEY_SIMPLEX,
                    0.42, (0, 255, 255), 1)

        # cv_bridge는 이 장비의 NumPy 2.x와 ABI가 맞지 않으므로, 단순한
        # bgr8 이미지는 ROS Image 메시지로 직접 변환한다.
        out = Image()
        out.height = int(img.shape[0])
        out.width = int(img.shape[1])
        out.encoding = 'bgr8'
        out.is_bigendian = False
        out.step = int(img.shape[1] * img.shape[2])
        out.data = img.tobytes()
        self.debug_pub.publish(out)

        if self.debug_window_enabled:
            try:
                if not self.debug_window_initialized:
                    cv2.namedWindow(self.debug_window_name, cv2.WINDOW_NORMAL)
                    cv2.resizeWindow(self.debug_window_name, size, size)
                    self.debug_window_initialized = True
                    self.get_logger().info(
                        f'Debug window opened: {self.debug_window_name} '
                        '(press q or ESC to close)')
                cv2.imshow(self.debug_window_name, img)
                key = cv2.waitKey(1) & 0xFF
                if key in (ord('q'), 27):
                    cv2.destroyWindow(self.debug_window_name)
                    self.debug_window_enabled = False
                    self.debug_window_initialized = False
                    self.get_logger().info('Debug window closed by user')
            except cv2.error as exc:
                self.debug_window_enabled = False
                self.debug_window_initialized = False
                self.get_logger().error(
                    f'Failed to show debug window; image topic remains active: {exc}')

    def destroy_node(self):
        if self.debug_window_initialized:
            try:
                cv2.destroyWindow(self.debug_window_name)
                cv2.waitKey(1)
            except cv2.error:
                pass
            self.debug_window_initialized = False
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = RubberconeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

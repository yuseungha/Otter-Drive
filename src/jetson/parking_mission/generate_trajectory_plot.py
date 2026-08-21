#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
import math
import shutil
import csv
import numpy as np
import matplotlib.pyplot as plt
import cv2
import yaml

def generate_trajectory_plot():
    # Load waypoints
    waypoints = {}
    with open('waypoints.csv', 'r') as f:
        reader = csv.DictReader(f)
        for row in reader:
            idx = int(row['index'])
            waypoints[idx] = {
                'x': float(row['x']),
                'y': float(row['y']),
                'yaw': float(row['yaw']),
                'speed': float(row['speed']) if 'speed' in row else 0.5
            }

    # Load Map
    with open('parking_map.yaml', 'r') as f:
        map_cfg = yaml.safe_load(f)
    map_img = cv2.imread('parking_map.pgm', cv2.IMREAD_GRAYSCALE)
    res = map_cfg.get('resolution', 0.05)
    origin = map_cfg.get('origin', [-2.15, -1.05, 0.0])
    h, w = map_img.shape

    # Simulation setup
    x = waypoints[1]['x']
    y = waypoints[1]['y']
    yaw = waypoints[1]['yaw']

    wheelbase = 0.315
    max_steer_rad = math.radians(35.0)
    forward_speed = 0.15
    reverse_speed = -0.15
    forward_lookahead = 0.50
    reverse_lookahead = 0.40
    waypoint_tol = 0.22
    parking_tol = 0.16
    parking_wait_sec = 3.0

    state = 'PHASE_1_FWD'
    sim_time = 0.0
    dt = 0.05
    wait_start_time = None

    traj_x = []
    traj_y = []
    traj_state = []

    def compute_forward_pp(curr_x, curr_y, curr_yaw, target_slice):
        min_idx = target_slice[0]
        min_dist = float('inf')
        for idx in target_slice:
            wp = waypoints[idx]
            d = math.hypot(wp['x'] - curr_x, wp['y'] - curr_y)
            if d < min_dist:
                min_dist = d
                min_idx = idx

        start_pos = target_slice.index(min_idx)
        lookahead_pt = None
        for idx in target_slice[start_pos:]:
            wp = waypoints[idx]
            dist = math.hypot(wp['x'] - curr_x, wp['y'] - curr_y)
            dx = wp['x'] - curr_x
            dy = wp['y'] - curr_y
            lx = math.cos(curr_yaw) * dx + math.sin(curr_yaw) * dy
            ly = -math.sin(curr_yaw) * dx + math.cos(curr_yaw) * dy
            if dist >= forward_lookahead and lx > 0.0:
                lookahead_pt = (wp['x'], wp['y'], lx, ly, dist)
                break

        if lookahead_pt is None:
            last_wp = waypoints[target_slice[-1]]
            dx = last_wp['x'] - curr_x
            dy = last_wp['y'] - curr_y
            lx = math.cos(curr_yaw) * dx + math.sin(curr_yaw) * dy
            ly = -math.sin(curr_yaw) * dx + math.cos(curr_yaw) * dy
            lookahead_pt = (last_wp['x'], last_wp['y'], lx, ly, math.hypot(dx, dy))

        _, _, lx, ly, ld = lookahead_pt
        ld_eff = max(ld, 0.20)
        alpha = math.atan2(ly, lx)
        steer = math.atan2(2.0 * wheelbase * math.sin(alpha), ld_eff)
        return np.clip(steer, -max_steer_rad, max_steer_rad)

    def compute_reverse_pp(curr_x, curr_y, curr_yaw, target_slice, target_yaw=None):
        min_idx = target_slice[0]
        min_dist = float('inf')
        for idx in target_slice:
            wp = waypoints[idx]
            d = math.hypot(wp['x'] - curr_x, wp['y'] - curr_y)
            if d < min_dist:
                min_dist = d
                min_idx = idx

        start_pos = target_slice.index(min_idx)
        lookahead_pt = None
        for idx in target_slice[start_pos:]:
            wp = waypoints[idx]
            dist = math.hypot(wp['x'] - curr_x, wp['y'] - curr_y)
            dx = wp['x'] - curr_x
            dy = wp['y'] - curr_y
            lx = math.cos(curr_yaw) * dx + math.sin(curr_yaw) * dy
            ly = -math.sin(curr_yaw) * dx + math.cos(curr_yaw) * dy
            if dist >= reverse_lookahead and lx < 0.1:
                lookahead_pt = (wp['x'], wp['y'], lx, ly, dist)
                break

        if lookahead_pt is None:
            last_wp = waypoints[target_slice[-1]]
            dx = last_wp['x'] - curr_x
            dy = last_wp['y'] - curr_y
            lx = math.cos(curr_yaw) * dx + math.sin(curr_yaw) * dy
            ly = -math.sin(curr_yaw) * dx + math.cos(curr_yaw) * dy
            lookahead_pt = (last_wp['x'], last_wp['y'], lx, ly, math.hypot(dx, dy))

        _, _, lx, ly, ld = lookahead_pt
        ld_eff = max(ld, 0.20)
        alpha_rev = math.atan2(ly, -lx)
        steer_pos = math.atan2(2.0 * wheelbase * math.sin(alpha_rev), ld_eff)

        if target_yaw is not None:
            last_wp = waypoints[target_slice[-1]]
            dist_to_final = math.hypot(curr_x - last_wp['x'], curr_y - last_wp['y'])
            yaw_err = math.atan2(math.sin(curr_yaw - target_yaw), math.cos(curr_yaw - target_yaw))
            steer_head = math.atan2(2.5 * wheelbase * yaw_err, 0.35)
            if dist_to_final < 0.40:
                w = float(np.clip((0.40 - dist_to_final) / 0.40, 0.0, 0.50))
                steer = (1.0 - w) * steer_pos + w * steer_head
            else:
                steer = steer_pos
        else:
            steer = steer_pos

        return np.clip(steer, -max_steer_rad, max_steer_rad)

    def compute_alignment_control(target_x, target_y, target_yaw, curr_center_x, curr_center_y, curr_yaw, dist_tol=0.10, yaw_tol=0.10):
        dx = target_x - curr_center_x
        dy = target_y - curr_center_y
        dist = math.hypot(dx, dy)
        local_x = math.cos(curr_yaw) * dx + math.sin(curr_yaw) * dy
        local_y = -math.sin(curr_yaw) * dx + math.cos(curr_yaw) * dy
        yaw_err = math.atan2(
            math.sin(curr_yaw - target_yaw),
            math.cos(curr_yaw - target_yaw)
        )
        is_aligned = (abs(yaw_err) <= yaw_tol) and (dist <= dist_tol)
        if is_aligned:
            return 0.0, 0.0, dist, yaw_err, True, "ALIGNED"

        align_speed = 0.15
        if dist <= dist_tol and abs(yaw_err) > yaw_tol:
            if local_x <= 0.02:
                cmd_spd = align_speed * 0.8
                cmd_st = float(-np.sign(yaw_err) * max_steer_rad)
                dir_str = "MICRO_FWD_TURN"
            else:
                cmd_spd = -align_speed * 0.8
                cmd_st = float(np.sign(yaw_err) * max_steer_rad)
                dir_str = "MICRO_REV_TURN"
        elif local_x > 0.02:
            cmd_spd = align_speed
            raw_st = math.atan2(-2.5 * wheelbase * yaw_err + 1.5 * local_y, 0.35)
            cmd_st = float(np.clip(raw_st, -max_steer_rad, max_steer_rad))
            dir_str = "FORWARD"
        elif local_x < -0.02:
            cmd_spd = -align_speed
            raw_st = math.atan2(2.5 * wheelbase * yaw_err - 1.5 * local_y, 0.35)
            cmd_st = float(np.clip(raw_st, -max_steer_rad, max_steer_rad))
            dir_str = "REVERSE"
        else:
            cmd_spd = align_speed * 0.8
            cmd_st = float(-np.sign(yaw_err) * max_steer_rad)
            dir_str = "MICRO_TURN"
        return cmd_spd, cmd_st, dist, yaw_err, False, dir_str

    phase1_overshoot_dist = 0.30  # Reduced by 0.1s (-0.05m -> 0.30m)
    phase1_overshoot_active = False
    phase1_overshoot_start = None

    phase3_overshoot_dist = 0.50  # Increased by 0.3s (+0.15m -> 0.50m)
    phase3_overshoot_active = False
    phase3_overshoot_start = None

    # Ground Truth Target Zone Poses
    start_pose_x = 1.80
    start_pose_y = 0.90
    start_pose_yaw = 3.141592
    parking_a_x = 0.00
    parking_a_y = 4.20
    parking_a_target_yaw = 0.00
    parking_a_tol = 0.10      # Stricter tolerance for A (meters)
    parking_a_yaw_tol = 0.10  # Stricter yaw tolerance for A (radians)

    parking_b_x = 2.10
    parking_b_y = 3.30
    parking_b_target_yaw = -1.5708
    parking_b_tol = 0.10      # Stricter tolerance for B (tightened to 0.10m to match A)
    parking_b_yaw_tol = 0.10  # Stricter yaw tolerance for B (tightened to 0.10 rad to match A)
    start_tol = 0.10          # Stricter tolerance for Start Zone (tightened to 0.10m to match A)

    for _ in range(5000):
        traj_x.append(x)
        traj_y.append(y)
        traj_state.append(state)

        cmd_speed = 0.0
        cmd_steer = 0.0

        # Vehicle Center
        center_x = x + (wheelbase / 2.0) * math.cos(yaw)
        center_y = y + (wheelbase / 2.0) * math.sin(yaw)

        if state == 'PHASE_1_FWD':
            dist = math.hypot(x - waypoints[27]['x'], y - waypoints[27]['y'])
            if not phase1_overshoot_active and dist <= waypoint_tol:
                phase1_overshoot_active = True
                phase1_overshoot_start = (x, y)

            if phase1_overshoot_active:
                traveled = math.hypot(x - phase1_overshoot_start[0], y - phase1_overshoot_start[1])
                if traveled >= phase1_overshoot_dist:
                    state = 'PHASE_2_REV_A'
                    phase1_overshoot_active = False
                else:
                    cmd_speed = forward_speed
                    cmd_steer = 0.0
            else:
                cmd_steer = compute_forward_pp(x, y, yaw, list(range(1, 28)))
                cmd_speed = forward_speed

        elif state == 'PHASE_2_REV_A':
            dist = math.hypot(center_x - parking_a_x, center_y - parking_a_y)
            if dist <= parking_a_tol:
                state = 'PHASE_2_ALIGN_A'
            else:
                cmd_steer = compute_reverse_pp(x, y, yaw, list(range(28, 34)), target_yaw=parking_a_target_yaw)
                cmd_speed = reverse_speed

        elif state == 'PHASE_2_ALIGN_A':
            cmd_spd, cmd_st, dist, yaw_err, is_aligned, dir_str = compute_alignment_control(
                parking_a_x, parking_a_y, parking_a_target_yaw,
                center_x, center_y, yaw,
                dist_tol=parking_a_tol, yaw_tol=parking_a_yaw_tol
            )
            if is_aligned:
                state = 'PHASE_2_WAIT'
                wait_start_time = sim_time
            else:
                cmd_speed = cmd_spd
                cmd_steer = cmd_st

        elif state == 'PHASE_2_WAIT':
            if (sim_time - wait_start_time) >= parking_wait_sec:
                state = 'PHASE_3_FWD'

        elif state == 'PHASE_3_FWD':
            dist = math.hypot(x - waypoints[55]['x'], y - waypoints[55]['y'])
            if not phase3_overshoot_active and dist <= waypoint_tol:
                phase3_overshoot_active = True
                phase3_overshoot_start = (x, y)

            if phase3_overshoot_active:
                traveled = math.hypot(x - phase3_overshoot_start[0], y - phase3_overshoot_start[1])
                if traveled >= phase3_overshoot_dist:
                    state = 'PHASE_4_REV_B'
                    phase3_overshoot_active = False
                else:
                    cmd_speed = forward_speed
                    cmd_steer = 0.0
            else:
                cmd_steer = compute_forward_pp(x, y, yaw, list(range(34, 56)))
                cmd_speed = forward_speed

        elif state == 'PHASE_4_REV_B':
            dist = math.hypot(center_x - parking_b_x, center_y - parking_b_y)
            if dist <= parking_b_tol:
                state = 'PHASE_4_ALIGN_B'
            else:
                cmd_steer = compute_reverse_pp(x, y, yaw, list(range(56, 62)), target_yaw=parking_b_target_yaw)
                cmd_speed = reverse_speed

        elif state == 'PHASE_4_ALIGN_B':
            cmd_spd, cmd_st, dist, yaw_err, is_aligned, dir_str = compute_alignment_control(
                parking_b_x, parking_b_y, parking_b_target_yaw,
                center_x, center_y, yaw,
                dist_tol=parking_b_tol, yaw_tol=parking_b_yaw_tol
            )
            if is_aligned:
                state = 'PHASE_4_WAIT'
                wait_start_time = sim_time
            else:
                cmd_speed = cmd_spd
                cmd_steer = cmd_st

        elif state == 'PHASE_4_WAIT':
            if (sim_time - wait_start_time) >= parking_wait_sec:
                state = 'PHASE_5_FWD'

        elif state == 'PHASE_5_FWD':
            dist = math.hypot(x - waypoints[79]['x'], y - waypoints[79]['y'])
            if dist <= waypoint_tol:
                state = 'PHASE_6_RETURN'
            else:
                cmd_steer = compute_forward_pp(x, y, yaw, list(range(62, 80)))
                cmd_speed = forward_speed

        elif state == 'PHASE_6_RETURN':
            dist = math.hypot(center_x - start_pose_x, center_y - start_pose_y)
            if dist <= start_tol:
                state = 'MISSION_COMPLETE'
                break
            else:
                cmd_steer = compute_forward_pp(x, y, yaw, [1])
                cmd_speed = forward_speed

        x += cmd_speed * math.cos(yaw) * dt
        y += cmd_speed * math.sin(yaw) * dt
        yaw += (cmd_speed / wheelbase) * math.tan(cmd_steer) * dt
        yaw = math.atan2(math.sin(yaw), math.cos(yaw))
        sim_time += dt

    # Plotting
    plt.figure(figsize=(10, 10), dpi=150)
    plt.style.use('dark_background')

    # Map extent: origin_x, origin_x + w*res, origin_y, origin_y + h*res
    extent = [origin[0], origin[0] + w * res, origin[1], origin[1] + h * res]
    plt.imshow(np.flipud(map_img), cmap='gray', extent=extent, alpha=0.6, origin='lower')

    # Plot All Waypoints
    wp_x = [wp['x'] for wp in waypoints.values()]
    wp_y = [wp['y'] for wp in waypoints.values()]
    plt.scatter(wp_x, wp_y, c='#4a5568', s=15, alpha=0.7, label='Waypoints (1-79)')

    # Plot Trajectory by Phase
    color_map = {
        'PHASE_1_FWD': '#00ffff',     # Cyan
        'PHASE_2_REV_A': '#ff007f',   # Magenta / Pink
        'PHASE_2_WAIT': '#ffd700',    # Gold
        'PHASE_3_FWD': '#00ff88',     # Spring Green
        'PHASE_4_REV_B': '#ff3333',   # Red
        'PHASE_4_WAIT': '#ffaa00',    # Orange
        'PHASE_5_FWD': '#3399ff',     # Blue
        'PHASE_6_RETURN': '#cc00ff',  # Purple
    }

    # Group segments
    traj_x = np.array(traj_x)
    traj_y = np.array(traj_y)
    traj_state = np.array(traj_state)

    for st, col in color_map.items():
        mask = (traj_state == st)
        if np.any(mask):
            label = st.replace('_', ' ')
            plt.plot(traj_x[mask], traj_y[mask], color=col, linewidth=2.5, label=label)

    # Highlight Key Waypoints & Target Zones
    plt.plot(waypoints[27]['x'], waypoints[27]['y'], 's', color='#ff007f', markersize=7, label='Rev Transition A (WP 27)')
    plt.plot(waypoints[55]['x'], waypoints[55]['y'], 's', color='#ff3333', markersize=7, label='Rev Transition B (WP 55)')

    # Draw target designated zones using the measured vehicle footprint.
    import matplotlib.patches as patches
    ax = plt.gca()
    start_rect = patches.Rectangle(
        (start_pose_x - 0.35, start_pose_y - 0.15), 0.70, 0.30,
        linewidth=2, edgecolor='#00ff44', facecolor='#00ff44', alpha=0.25, label='Start Zone (0.7x0.3m)'
    )
    ax.add_patch(start_rect)

    zone_a_rect = patches.Rectangle(
        (parking_a_x - 0.265, parking_a_y - 0.13), 0.53, 0.26,
        linewidth=2, edgecolor='#ff00c8', facecolor='#ff00c8', alpha=0.25, label='Parking Zone A (0.53x0.26m)'
    )
    ax.add_patch(zone_a_rect)

    zone_b_rect = patches.Rectangle(
        (parking_b_x - 0.13, parking_b_y - 0.265), 0.26, 0.53,
        linewidth=2, edgecolor='#ffaa00', facecolor='#ffaa00', alpha=0.25, label='Parking Zone B (0.26x0.53m)'
    )
    ax.add_patch(zone_b_rect)

    # Function to draw vehicle body frame (55x30cm) & center point
    def draw_vehicle_box(ax, x_center, y_center, car_yaw, box_col='#00e5ff', label_text=None):
        L_veh = 0.53
        W_veh = 0.26
        corners_local = np.array([
            [ L_veh / 2.0,  W_veh / 2.0],
            [ L_veh / 2.0, -W_veh / 2.0],
            [-L_veh / 2.0, -W_veh / 2.0],
            [-L_veh / 2.0,  W_veh / 2.0],
            [ L_veh / 2.0,  W_veh / 2.0]
        ])
        R = np.array([
            [math.cos(car_yaw), -math.sin(car_yaw)],
            [math.sin(car_yaw),  math.cos(car_yaw)]
        ])
        corners_world = (R @ corners_local.T).T + np.array([x_center, y_center])
        ax.plot(corners_world[:, 0], corners_world[:, 1], color=box_col, linewidth=2.2, linestyle='-', label=label_text)
        # Vehicle Center Dot (Yellow)
        ax.plot(x_center, y_center, 'o', color='#ffff00', markersize=6, markeredgecolor='#000000', markeredgewidth=1.0)
        # Heading direction arrow
        fx = x_center + 0.20 * math.cos(car_yaw)
        fy = y_center + 0.20 * math.sin(car_yaw)
        ax.plot([x_center, fx], [y_center, fy], color='#00ff44', linewidth=2.2)

    # Draw at Start (x=1.80, y=0.90, yaw=3.14)
    draw_vehicle_box(ax, start_pose_x, start_pose_y, start_pose_yaw, box_col='#00e5ff', label_text='Vehicle Body (55x30cm)')
    # Draw at Parking A (Center: x=0.00, y=4.20, yaw=0.0 rad)
    draw_vehicle_box(ax, parking_a_x, parking_a_y, parking_a_target_yaw, box_col='#ff00c8')
    # Draw at Parking B (Center: x=2.10, y=3.30, yaw=-1.57 rad)
    draw_vehicle_box(ax, parking_b_x, parking_b_y, parking_b_target_yaw, box_col='#ffaa00')

    # Legend for Vehicle Center
    plt.plot([], [], 'o', color='#ffff00', markeredgecolor='#000000', markersize=6, label='Vehicle Center Point')

    plt.title('Autonomous Vehicle 6-Phase Waypoint Following & Parking Trajectory\n(Target: Start Zone (1.8,0.9) | Zone A (0.0,4.2,0°) | Zone B (2.1,3.3,-90°))', fontsize=12, pad=12, fontweight='bold')
    plt.xlabel('Map X [meters]', fontsize=11)
    plt.ylabel('Map Y [meters]', fontsize=11)
    plt.grid(True, linestyle='--', alpha=0.3)
    plt.legend(loc='lower left', fontsize=7.2, framealpha=0.85)
    plt.tight_layout()

    out_file = os.path.join(
        os.path.dirname(os.path.abspath(__file__)),
        'trajectory_verification.png',
    )
    plt.savefig(out_file)
    print(f"[PLOT] Trajectory image saved to {out_file}")

    # Copy to artifact folder
    artifact_dir = '/home/sy/.gemini/antigravity-ide/brain/3564724c-3391-44b0-8845-5e5ac9be3979'
    if os.path.exists(artifact_dir):
        shutil.copy(out_file, os.path.join(artifact_dir, 'trajectory_verification.png'))
        print(f"[PLOT] Copied to artifact directory: {artifact_dir}")

if __name__ == '__main__':
    generate_trajectory_plot()

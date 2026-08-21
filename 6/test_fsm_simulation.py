#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Automated Simulation Test for 6-Phase Parking Mission
Simulates vehicle kinematics with mission_waypoint_follower logic,
verifying that all phases complete correctly and logging the trajectory.
"""

import math
import numpy as np
import csv

def run_simulation():
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

    print(f"[TEST] Successfully loaded {len(waypoints)} waypoints.")

    # Simulation state
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

    history = []
    phase_transitions = []

    def compute_forward_pp(curr_x, curr_y, curr_yaw, target_slice):
        lookahead_pt = None
        min_diff = 999.0
        for idx in target_slice:
            wp = waypoints[idx]
            dist = math.hypot(wp['x'] - curr_x, wp['y'] - curr_y)
            dx = wp['x'] - curr_x
            dy = wp['y'] - curr_y
            lx = math.cos(curr_yaw) * dx + math.sin(curr_yaw) * dy
            ly = -math.sin(curr_yaw) * dx + math.cos(curr_yaw) * dy
            if lx > 0.0:
                diff = abs(dist - forward_lookahead)
                if diff < min_diff:
                    min_diff = diff
                    lookahead_pt = (wp['x'], wp['y'], lx, ly, dist)
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

    max_steps = 5000
    for step in range(max_steps):
        history.append((sim_time, x, y, yaw, state))
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
                print(f"[{sim_time:6.2f}s] Reached WP 27 -> Extra forward clearance ({phase1_overshoot_dist:.2f}m)")

            if phase1_overshoot_active:
                traveled = math.hypot(x - phase1_overshoot_start[0], y - phase1_overshoot_start[1])
                if traveled >= phase1_overshoot_dist:
                    phase_transitions.append((sim_time, state, 'PHASE_2_REV_A'))
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
                phase_transitions.append((sim_time, state, 'PHASE_2_ALIGN_A'))
                state = 'PHASE_2_ALIGN_A'
                print(f"[{sim_time:6.2f}s] Reached Parking Zone A ({parking_a_x}, {parking_a_y}) -> Starting Horizontal Alignment (가로 정렬)")
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
                phase_transitions.append((sim_time, state, 'PHASE_2_WAIT'))
                state = 'PHASE_2_WAIT'
                wait_start_time = sim_time
                print(f"[{sim_time:6.2f}s] Horizontal Alignment Complete (yaw_err={math.degrees(yaw_err):.1f}°, dist={dist:.2f}m) -> Starting 3.0s Parking Wait")
            else:
                cmd_speed = cmd_spd
                cmd_steer = cmd_st

        elif state == 'PHASE_2_WAIT':
            cmd_speed = 0.0
            cmd_steer = 0.0
            if (sim_time - wait_start_time) >= parking_wait_sec:
                phase_transitions.append((sim_time, state, 'PHASE_3_FWD'))
                state = 'PHASE_3_FWD'

        elif state == 'PHASE_3_FWD':
            dist = math.hypot(x - waypoints[55]['x'], y - waypoints[55]['y'])
            if not phase3_overshoot_active and dist <= waypoint_tol:
                phase3_overshoot_active = True
                phase3_overshoot_start = (x, y)
                print(f"[{sim_time:6.2f}s] Reached WP 55 -> Extra forward clearance ({phase3_overshoot_dist:.2f}m)")

            if phase3_overshoot_active:
                traveled = math.hypot(x - phase3_overshoot_start[0], y - phase3_overshoot_start[1])
                if traveled >= phase3_overshoot_dist:
                    phase_transitions.append((sim_time, state, 'PHASE_4_REV_B'))
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
                phase_transitions.append((sim_time, state, 'PHASE_4_ALIGN_B'))
                state = 'PHASE_4_ALIGN_B'
                print(f"[{sim_time:6.2f}s] Reached Parking Zone B ({parking_b_x}, {parking_b_y}) -> Starting Vertical Alignment (세로 정렬)")
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
                phase_transitions.append((sim_time, state, 'PHASE_4_WAIT'))
                state = 'PHASE_4_WAIT'
                wait_start_time = sim_time
                print(f"[{sim_time:6.2f}s] Vertical Alignment Complete (yaw_err={math.degrees(yaw_err):.1f}°, dist={dist:.2f}m) -> Starting 3.0s Parking Wait")
            else:
                cmd_speed = cmd_spd
                cmd_steer = cmd_st

        elif state == 'PHASE_4_WAIT':
            cmd_speed = 0.0
            cmd_steer = 0.0
            if (sim_time - wait_start_time) >= parking_wait_sec:
                phase_transitions.append((sim_time, state, 'PHASE_5_FWD'))
                state = 'PHASE_5_FWD'

        elif state == 'PHASE_5_FWD':
            dist = math.hypot(x - waypoints[79]['x'], y - waypoints[79]['y'])
            if dist <= waypoint_tol:
                phase_transitions.append((sim_time, state, 'PHASE_6_RETURN'))
                state = 'PHASE_6_RETURN'
            else:
                cmd_steer = compute_forward_pp(x, y, yaw, list(range(62, 80)))
                cmd_speed = forward_speed

        elif state == 'PHASE_6_RETURN':
            dist = math.hypot(center_x - start_pose_x, center_y - start_pose_y)
            if dist <= start_tol:
                phase_transitions.append((sim_time, state, 'MISSION_COMPLETE'))
                state = 'MISSION_COMPLETE'
                print(f"[TEST SUCCESS] Vehicle Center reached Start Zone ({start_pose_x}, {start_pose_y}) (dist={dist:.3f}m) at t={sim_time:.2f}s!")
                break
            else:
                cmd_steer = compute_forward_pp(x, y, yaw, [1])
                cmd_speed = forward_speed

        # Kinematic integration
        x += cmd_speed * math.cos(yaw) * dt
        y += cmd_speed * math.sin(yaw) * dt
        yaw += (cmd_speed / wheelbase) * math.tan(cmd_steer) * dt
        yaw = math.atan2(math.sin(yaw), math.cos(yaw))
        sim_time += dt

    print("\n--- Phase Transitions ---")
    for t_stamp, from_s, to_s in phase_transitions:
        print(f"[{t_stamp:6.2f}s] {from_s} -> {to_s}")

    assert state == 'MISSION_COMPLETE', f"Simulation did not finish in MISSION_COMPLETE, ended in {state}"
    print("\n✅ All 6 FSM phases and 2 parking waits verified successfully!")

if __name__ == '__main__':
    run_simulation()

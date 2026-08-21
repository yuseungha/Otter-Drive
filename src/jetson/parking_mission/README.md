# 1/10 RC 카 자율주행 및 2회 후진 주차 미션 ROS 2 패키지 (`parking_mission`)

본 패키지는 NVIDIA Jetson Nano 기반 1/10 RC 모형차에서 경기장 트랙 맵(`parking_map.yaml` / `.pgm`)과 `waypoints.csv`를 기반으로 6단계 FSM 자율주행 및 2회 후진 주차(각 3초 대기) 시퀀스를 수행하는 ROS 2 Humble 패키지입니다.

---

## 1. 패키지 구성 및 주요 기능

1. **`mission_waypoint_follower`**:
   - `waypoints.csv` 로드 및 6단계 FSM 시퀀스 자동 제어
   - **전진 Pure Pursuit ($v > 0$)**: 전방 Lookahead ($L_{d,fwd}$) 기반 부드러운 트랙 주행
   - **후진 Pure Pursuit ($v < 0$)**: 후방 Lookahead ($L_{d,rev}$) 및 기구학 보정 조향각 계산
   - **정밀 주차 타이머**: 주차 목표 지점(WP 33, WP 51) 도달 시 정확히 3.0초 정지 대기
   - **안전 시스템**: LiDAR 전/후방 긴급 제동 (E-Stop, 0.3m 이내), 카메라 기반 주황색 콘 감속
   - **RViz 2 시각화**: 3D 실시간 상태 텍스트 마커, 타깃 구(Sphere) 마커, 활성 경로 및 전체 경로 표시

2. **`map_server_node`**:
   - `parking_map.yaml` & `parking_map.pgm`을 로드하여 `/map` 토픽으로 Latched (Transient Local) QoS 퍼블리시
   - 별도 nav2 의존성 없이도 RViz 2에 즉각적이고 안정적인 맵 렌더링 지원

3. **`sim_vehicle_node`**:
   - 실제 하드웨어 연결 없이도 RViz 2 상에서 전체 6단계 주행 시퀀스를 실시간으로 검증할 수 있는 Ackermann 운동학 시뮬레이터

---

## 2. 6단계 FSM 시퀀스 흐름

| 단계 (Phase) | 구간 웨이포인트 | 주행 모드 | 속도 ($v$) | 종료/전환 조건 |
| :--- | :--- | :--- | :--- | :--- |
| **Phase 1** | Index 1 ~ 27 | 전진 Pure Pursuit | $0.15\text{ m/s}$ | Index 27번 도달 후 전진 여유거리 확보 시 Phase 2 전환 |
| **Phase 2 (미션 A)** | Index 28 ~ 33 | **후진 Pure Pursuit** | $-0.15\text{ m/s}$ | 주차 A 정렬 후 **3초간 완전 정지 (Parking Wait)** 후 Phase 3 전환 |
| **Phase 3** | Index 34 ~ 55 | 전진 Pure Pursuit | $0.15\text{ m/s}$ | Index 55번 도달 후 전진 여유거리 확보 시 Phase 4 전환 |
| **Phase 4 (미션 B)** | Index 56 ~ 61 | **후진 Pure Pursuit** | $-0.15\text{ m/s}$ | 주차 B 정렬 후 **3초간 완전 정지 (Parking Wait)** 후 Phase 5 전환 |
| **Phase 5** | Index 62 ~ 79 | 전진 Pure Pursuit | $0.15\text{ m/s}$ | Index 79번 도달 시 Phase 6 전환 |
| **Phase 6** | Index 79 ~ 1 (시작점) | 전진 Pure Pursuit | $0.15\text{ m/s}$ | **시작점 도달 시 차량 완전 정지 및 미션 종료 (Shutdown)** |

---

## 3. 빌드 및 실행 방법

### 3.1 패키지 빌드
```bash
cd /home/sandi/6
source /opt/ros/humble/setup.bash
colcon build --packages-select parking_mission
source install/setup.bash
```

### 3.2 시뮬레이션 모드로 전체 시퀀스 즉시 실행 (RViz 2 포함)
```bash
ros2 launch parking_mission bringup_mission.launch.py use_sim:=true
```

### 3.3 실제 차량 (Jetson Nano) 하드웨어 주행 시 실행
```bash
ros2 launch parking_mission bringup_mission.launch.py use_sim:=false
```

---

## 4. 파라미터 튜닝 (`config/mission_params.yaml`)

| 파라미터 명 | 기본값 | 설명 |
| :--- | :--- | :--- |
| `wheelbase` | `0.315` | 차량 축거 $L$ (단위: m) |
| `vehicle_length/width/height` | `0.530/0.260/0.220` | 실측 차체 크기 (단위: m) |
| `max_steer_rad` | `0.610865` | 최대 조향각 $\pm35^\circ$ |
| `forward_speed` | `0.15` | 전진 주행 기본 속도 (0.54 km/h) |
| `reverse_speed` | `-0.15` | 후진 주행 기본 속도 (-0.54 km/h) |
| `xycar_max_steer_command` | `650` | 서보 명령 절댓값 상한 |
| `xycar_max_speed_command` | `650` | 모터 명령 절댓값 상한 |
| `forward_lookahead` | `0.50` | 전진 주행 Lookahead 거리 $L_{d,fwd}$ (m) |
| `reverse_lookahead` | `0.40` | 후진 주행 Lookahead 거리 $L_{d,rev}$ (m) |
| `waypoint_tolerance` | `0.22` | 일반 웨이포인트 도달 판정 거리 (m) |
| `parking_tolerance` | `0.16` | 주차 정지 지점 도달 판정 거리 (m) |
| `parking_wait_sec` | `3.0` | 주차 완료 후 정지 대기 시간 (초) |
| `obstacle_stop_dist` | `0.30` | 전/후방 긴급 제동 거리 (m) |
| `cone_slowdown_scale`| `0.50` | 카메라 주황색 콘 감지 시 감속 비율 |

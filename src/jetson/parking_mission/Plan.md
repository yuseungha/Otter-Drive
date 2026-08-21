# Role & Objective
당신은 모형차 자율주행 SW를 개발하는 시니어 로보틱스/임베디드 엔지니어입니다.
Jetson Nano 환경의 1/10 RC 카에서 사전 생성된 경기장 맵(`Parking_map`)을 RViz 2에 띄우고, `waypoints.csv` 기반의 전진/후진/주차(3초 대기) FSM 시퀀스를 실행하는 완전한 ROS 2 패키지를 작성해주세요.

---

## 1. 시스템 사양
- **Compute**: NVIDIA Jetson Nano (Ubuntu 20.04/22.04, ROS 2 Humble/Foxy)
- **Actuator/ESC**: 1/10 RC Car Body (Ackermann Steering), Brushless Motor + VESC (`/ackermann_cmd`)
- **Sensors & Localization**: 
  - 2D LiDAR + IMU + VESC 오도메트리 기반 2D Pose 추정 (`/odom` 또는 `/map` -> `/base_link` TF)
  - 170° 어안렌즈 USB 카메라 (주황색 콘 감지)
  - 초음파 센서 (HC-SR04)
- **Map File**: `Parking_map.yaml` & `Parking_map.pgm` (트랙 경기장 맵)

---

## 2. 정확한 주행 시퀀스 및 FSM 흐름

차량은 `waypoints.csv` 파일의 데이터를 기반으로 다음 6단계 FSM을 순차 실행합니다:

1. **Phase 1 (전진 추종)**:
   - Index **1번 ~ 27번**까지 **전진 Pure Pursuit** 주행 ($v > 0$)
2. **Phase 2 (미션 A 후진 주차)**:
   - 27번 도달 즉시 후진 모드로 전환
   - Index **28번 ~ 33번**까지 **후진 Pure Pursuit** 추종 ($v < 0$)
   - Index 33번 도달 시 **완전 정지 후 3초간 대기 (Parking Wait)**
3. **Phase 3 (전진 추종)**:
   - 3초 대기 완료 후 전진 모드로 전환
   - Index **34번 ~ 55번**까지 **전진 Pure Pursuit** 주행 ($v > 0$)
4. **Phase 4 (미션 B 후진 주차)**:
   - 55번 도달 즉시 후진 모드로 전환
   - Index **56번 ~ 61번**까지 **후진 Pure Pursuit** 추종 ($v < 0$)
   - Index 61번 도달 시 **완전 정지 후 3초간 대기 (Parking Wait)**
5. **Phase 5 (전진 추종)**:
   - 3초 대기 완료 후 전진 모드로 전환
   - Index **62번 ~ 79번**까지 **전진 Pure Pursuit** 주행 ($v > 0$)
6. **Phase 6 (시작점 복귀 및 최종 정지)**:
   - Index 79번 도달 후, 시작 위치인 **Index 1번 웨이포인트**를 향해 전진 추종
   - **Index 1번에 도달하면 차량을 완전 정지하고 전체 자율주행 시퀀스 종료 (Mission Complete & Shutdown)**

---

## 3. RViz 2 및 맵 서버(Map Server) 요구사항
- `nav2_map_server`(또는 `map_server`)를 실행하여 `Parking_map.yaml`을 `/map` 토픽으로 퍼블리시
- RViz 2 기본 설정(`.rviz`):
  - **Map Display**: 경기장 지도 (`Parking_map`) 렌더링
  - **Path Display**: 전체 `waypoints.csv` 경로 (`nav_msgs/msg/Path`)
  - **Marker Display**: 
    - 현재 주행 상태 텍스트 (예: `FORWARD (1-27)`, `REVERSE (28-33)`, `PARKING WAIT (3s)` 등)
    - 현재 추종 중인 타깃 웨이포인트 구(Sphere) 마커
  - **TF / Robot Model**: 차량 축거 및 헤딩 실시간 표시

---

## 4. 핵심 제어 알고리즘 및 안전 기능
1. **전진/후진 Pure Pursuit 제어기**:
   - **전진 ($v > 0$)**: 전방 Lookahead Distance ($L_{d,fwd}$) 기준 타깃 탐색 및 Ackermann 조향각($\delta$) 계산
   - **후진 ($v < 0$)**: 후방 Lookahead Distance ($L_{d,rev}$) 기준 타깃 탐색 및 Ackermann 후진 조향각($\delta_{rev}$) 계산
2. **안전 기능**:
   - LiDAR/초음파 센서로 진행 방향(전진 시 전방, 후진 시 후방) 위험 거리(0.3m 이내) 진입 시 긴급 제동(Emergency Brake)
   - 카메라 기반 주황색 콘 근접 감지 시 감속

---

## 5. 요청 산출물
1. **`mission_waypoint_follower.py`**:
   - CSV 로드, FSM 구간 분기, 전진/후진 Pure Pursuit, 3초 타이머 및 RViz Marker 퍼블리시
2. **`config/mission_params.yaml`**:
   - Wheelbase ($L$), 전진/후진 $L_d$, 전진/후진 속도, 도달 판정 거리(Tolerance), 맵 파일 경로
3. **`launch/bringup_mission.launch.py`**:
   - `Parking_map` Map Server 실행 + Lifecycle Manager 활성화
   - `mission_waypoint_follower` 노드 실행
   - 사전 정의된 설정이 적용된 RViz 2 동시 실행
4. **`rviz/parking_mission.rviz`** (설정 가이드 포함)
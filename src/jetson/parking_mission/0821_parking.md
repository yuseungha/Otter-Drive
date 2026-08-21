# 0821 주차 미션 정리

작성일: 2026-08-21  
프로젝트 위치: `/home/sandi/6`  
원본 압축 파일: `/home/sandi/Downloads/6.tar.gz`

## 1. 프로젝트 개요

이 프로젝트는 ROS 2 Humble 기반 RC카 주차 미션 패키지이다. 제공받은 정적 지도와 웨이포인트를 이용하여 전진 주행, 두 차례의 후진 주차, 각 주차 위치에서 3초 정차, 시작점 복귀를 수행한다.

현재 구조는 SLAM으로 지도를 새로 생성하는 방식이 아니다.

```text
parking_map.pgm / parking_map.yaml
                 +
            waypoints.csv
                 ↓
      Pure Pursuit + 주차 FSM
                 ↓
        속도·조향 명령 생성
```

- `parking_map.pgm`: 점유 지도 이미지
- `parking_map.yaml`: 지도 해상도, 원점, 장애물 판정값
- `waypoints.csv`: 차량이 따라갈 좌표와 순서
- `mission_waypoint_follower.py`: 경로 추종 및 주차 FSM
- `sim_vehicle_node.py`: Ackermann 자전거 모델 기반 단순 차량 시뮬레이터
- RViz2: 지도, 차량, 경로, 목표점 시각화

Gazebo 물리 시뮬레이터는 사용하지 않는다. 현재 시뮬레이터에는 관성, 타이어 미끄러짐, 마찰, 모터 지연, 실제 충돌 등이 반영되지 않는다.

## 2. 주요 파일

| 파일 | 역할 |
|---|---|
| `config/mission_params.yaml` | 차량 제원, 속도, 조향각, 안전거리, ROS 토픽 설정 |
| `parking_mission/mission_waypoint_follower.py` | Pure Pursuit, 주차 정렬, FSM, 모터 명령 생성 |
| `parking_mission/sim_vehicle_node.py` | 속도와 조향각을 차량 위치로 변환하는 운동학 시뮬레이터 |
| `parking_mission/map_server_node.py` | PGM/YAML 지도를 `/map`으로 발행 |
| `launch/bringup_mission.launch.py` | 지도, 제어기, 시뮬레이터, RViz 통합 실행 |
| `test_fsm_simulation.py` | ROS 없이 전체 FSM을 빠르게 검증 |
| `generate_trajectory_plot.py` | 예상 궤적 이미지 생성 |
| `trajectory_verification.png` | 최신 차량 제원 기준 궤적 검증 결과 |

## 3. 적용한 실제 차량 제원

입력받은 치수는 mm, 속도 `0.54km`는 `0.54km/h`로 해석하였다.

| 항목 | 입력값 | 적용값 |
|---|---:|---:|
| 차체 길이 | 530 mm | 0.530 m |
| 차체 폭 | 260 mm | 0.260 m |
| 차체 높이 | 220 mm | 0.220 m |
| 앞뒤 차축 거리 | 315 mm | 0.315 m |
| 최대 조향각 | ±35° | ±0.610865 rad |
| 전진 속도 | 0.54 km/h | 0.15 m/s |
| 후진 속도 | 0.54 km/h | -0.15 m/s |
| 모터 명령 범위 | ±650 | ±650 |
| 서보 명령 범위 | ±650 | ±650 |

현재 명령 변환은 선형으로 가정한다.

```text
모터 +650  ↔  +0.15 m/s  ↔  +0.54 km/h
모터 -650  ↔  -0.15 m/s  ↔  -0.54 km/h
서보 +650  ↔  한쪽 최대 조향 35°
서보 -650  ↔  반대쪽 최대 조향 35°
```

설정된 변환 계수:

- `xycar_speed_scale`: `0.000230769 m/s/command`
- `xycar_steer_scale`: `0.000939793 rad/command`
- `xycar_max_speed_command`: `650`
- `xycar_max_steer_command`: `650`

실차에서 명령값과 실제 속도·조향각의 관계가 선형이라는 보장은 없다. 바퀴를 지면에서 띄운 상태로 방향과 중립값을 먼저 확인한 뒤 실측 보정해야 한다.

## 4. 현재 시뮬레이션 동작

```text
지도 + 웨이포인트
        ↓
mission_waypoint_follower
        ↓ /cmd_vel
sim_vehicle_node
        ↓ /odom, map → base_link TF
RViz2
```

`mission_waypoint_follower`가 속도와 조향각을 계산하고 `/cmd_vel`을 발행한다. `sim_vehicle_node`가 이를 받아 차량의 위치와 방향을 계산한다. RViz2는 계산된 결과를 보여준다.

시뮬레이터는 장애물이 없는 `5.0m` 거리의 가상 `/scan` 데이터도 발행한다. 따라서 현재 시뮬레이션은 실제 라이다 장애물이나 초음파 센서를 재현하지 않는다.

## 5. FSM 주행 순서

1. 웨이포인트 1~27 전진
2. 여유거리 확보 후 주차 구역 A로 후진
3. 구역 A 중심과 각도 정렬 후 3초 정차
4. 웨이포인트 34~55 전진
5. 여유거리 확보 후 주차 구역 B로 후진
6. 구역 B 중심과 각도 정렬 후 3초 정차
7. 웨이포인트 62~79 전진
8. 시작 위치로 복귀 후 정지

## 6. 검증 결과

새 차량 제원을 적용한 독립 FSM 테스트와 ROS 노드 통합 테스트가 모두 통과하였다.

- 지도: 143 × 149 pixel
- 지도 해상도: 0.05 m/pixel
- 웨이포인트: 79개
- 주차 A 중심 오차: 약 0.10 m
- 주차 B 중심 오차: 약 0.10 m
- 시작점 복귀 오차: 약 0.10 m
- 전체 예상 미션 시간: 약 106초
- 두 주차 구역에서 각각 3초 정차 확인
- 6단계 FSM 및 시작점 복귀 확인

## 7. 빌드 및 시뮬레이션 실행

이 PC에는 Ubuntu 22.04, ROS 2 Humble Desktop, RViz2, colcon, OpenCV 등이 설치되어 있다.

사용자 영역의 NumPy 2.x와 ROS Humble `cv_bridge` 사이에 ABI 충돌이 있으므로 `PYTHONNOUSERSITE=1`을 사용한다.

### 빌드

```bash
cd /home/sandi/6
source /opt/ros/humble/setup.bash
export PYTHONNOUSERSITE=1

colcon --log-base log_sandi build \
  --base-paths /home/sandi/6 \
  --build-base build_sandi \
  --install-base install_sandi \
  --symlink-install \
  --packages-select parking_mission
```

### RViz 시뮬레이션

```bash
cd /home/sandi/6
source /opt/ros/humble/setup.bash
source install_sandi/setup.bash
export PYTHONNOUSERSITE=1

ROS_DOMAIN_ID=198 ros2 launch parking_mission bringup_mission.launch.py \
  use_sim:=true \
  rviz:=true
```

### RViz 없이 실행

```bash
ROS_DOMAIN_ID=198 ros2 launch parking_mission bringup_mission.launch.py \
  use_sim:=true \
  rviz:=false
```

## 8. 엔코더와 IMU가 없는 실차 구성

실차에는 엔코더와 IMU가 없으므로 현재 `sim_vehicle_node`가 만드는 완벽한 위치값을 사용할 수 없다. 라이다를 주 위치 센서로 사용하고 초음파를 근거리 주차 및 안전 센서로 사용하는 구성이 적합하다.

권장 TF 및 위치 추정 흐름:

```text
2D LiDAR /scan
      ↓
RF2O 라이다 오도메트리
      ↓  odom → base_link
AMCL + 제공받은 PGM/YAML 지도
      ↓  map → odom
map → base_link 완성
      ↓
mission_waypoint_follower
      ↓
모터·서보 명령
```

### 라이다 역할

- 연속 스캔 정합으로 단기 이동량과 회전량 추정
- 제공된 PGM/YAML 지도에서 차량의 전역 위치 추정
- 전방 및 후방 장애물 긴급정지
- 위치 추정 품질이 낮거나 스캔이 끊기면 차량 정지

PGM/YAML 정적 지도가 있으므로 전역 위치 추정에는 AMCL이 적합하다. 그러나 AMCL은 `odom → base_link` 변환을 필요로 한다. 엔코더 대신 RF2O 같은 2D 라이다 오도메트리로 이 변환을 공급하는 방안을 검토한다.

### 초음파 역할

- 전방 센서: 충돌 방지 및 긴급정지
- 후방 센서: 후진 주차 최종 정지거리
- 좌우 센서: 주차 구역 중심과 측면 여유거리 보정
- 같은 측면의 앞·뒤 센서: 거리 차이로 벽과 차체의 상대 각도 추정 가능

초음파 하나의 측정값은 한 방향 거리만 제공하므로 초음파만으로 지도상의 `x`, `y`, `yaw`를 안정적으로 추정하기는 어렵다. 초음파는 라이다 위치 추정의 대체재가 아니라 마지막 수십 cm 구간의 주차 보조 센서로 사용한다.

## 9. 현재 코드에서 실차용으로 추가할 사항

- `use_sim:=false`에서 `sim_vehicle_node` 비활성화
- 실제 라이다 드라이버로 `/scan` 발행
- `base_link → lidar_link` 정적 TF 추가
- RF2O 또는 동등한 라이다 오도메트리 노드 추가
- AMCL과 정적 지도 서버 추가
- AMCL 초기 위치 설정
- `/xycar_ultrasonic` 실제 구독 및 메시지 파싱 구현
- 주차 A/B 정렬 단계에 후방·좌우 초음파 거리 조건 추가
- 라이다/초음파 데이터 timeout 시 즉시 정지
- 위치 추정 유실 또는 TF 오류 시 즉시 정지
- 모터 및 서보의 중립값, 방향, 실제 변환 계수 보정
- 수동 비상정지와 원격 강제 정지 수단 추가

현재 코드에서 라이다 `/scan`은 전후방 장애물 검사에만 사용한다. `ultrasonic_topic`은 YAML에 선언되어 있지만 실제 초음파 구독 및 제어 로직은 아직 없다.

## 10. 실차 적용 전에 필요한 정보

- 라이다 제조사와 정확한 모델명
- 라이다 ROS 토픽명과 `LaserScan` frame ID
- 라이다 장착 위치: 차체 기준 x, y, z 및 방향
- 초음파 센서 개수
- 각 초음파 센서의 위치와 방향
- 초음파 ROS 메시지 형식과 토픽명
- 모터 및 서보 중립 명령값
- 전진·후진 명령 부호
- 좌·우 조향 명령 부호
- 여러 모터 명령값에서 측정한 실제 속도
- 여러 서보 명령값에서 측정한 실제 바퀴 조향각
- 주차 구역에서 요구되는 전후좌우 최소 여유거리

## 11. 권장 실차 시험 순서

1. 바퀴를 띄운 상태에서 모터와 서보 방향 확인
2. 낮은 명령값부터 실제 속도와 조향각 측정
3. 라이다 정지 상태 `/scan` 품질 및 TF 확인
4. 차량을 손으로 천천히 이동하며 라이다 오도메트리 확인
5. AMCL로 지도상 위치가 유지되는지 확인
6. 모터를 연결하지 않은 상태에서 제어 명령값만 기록
7. 넓고 통제된 공간에서 저속 직진·정지 시험
8. 전방/후방 긴급정지 시험
9. 주차 A만 단독 저속 시험
10. 주차 B만 단독 저속 시험
11. 전체 미션 통합 시험

실차 시험 중 라이다 위치 추정이 튀거나 초음파 데이터가 사라지는 경우 차량은 계속 진행하지 않고 정지하도록 구성해야 한다.

## 12. 참고 자료

- [Nav2 AMCL 설정 문서](https://docs.nav2.org/configuration/packages/configuring-amcl.html)
- [Nav2 TF 구성 문서](https://docs.nav2.org/setup_guides/transformation/setup_transforms.html)
- [RF2O Laser Odometry](https://github.com/MAPIRlab/rf2o_laser_odometry)
- [ROS 2 Humble SLAM Toolbox 문서](https://docs.ros.org/en/humble/p/slam_toolbox/)


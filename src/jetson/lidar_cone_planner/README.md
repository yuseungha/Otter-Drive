# lidar_cone_planner

RPLIDAR A1M8 같은 2D LiDAR의 `sensor_msgs/LaserScan`에서 라바콘 후보를 찾고,
양쪽 라바콘 열 사이의 로컬 중앙 경로를 만든 뒤 속도 적응형 Pure Pursuit로
아커만 명령까지 계산하는 ROS 2 Python 패키지입니다. Arduino·ESC·조향 모터로
직접 시리얼 명령을 보내지는 않습니다.

이 버전은 경로를 찾지 못했거나 스캔·TF가 끊기면 **빈 Path를 발행**하고,
동봉된 제어기는 빈 Path·오래된 stamp·오류 상태·입력 단절 중 하나라도 확인되면
가감속 제한을 우회해 속도와 조향을 즉시 0으로 만듭니다. 제어기는 기본적으로
비활성화되어 있으며 실측 차량 치수를 확인하기 전에는 활성화할 수 없습니다.

## 처리 흐름

1. LaserScan의 유효 거리와 센서 메타데이터 검사
2. scan stamp 시각의 TF로 점들을 `planning_frame`으로 변환
3. 전방 ROI 안에서 인접 반사점을 거리 적응형으로 군집화
4. 짧은 1-beam 누락을 조건부로 연결하고 물리 폭·각폭·깊이로 콘 후보 선별
5. A1 angle compensation의 동일거리 복제와 단발 반사를 분리
6. 두 번 이상 **연속** 관측된 후보만 확인된 라바콘으로 사용
7. 가능한 좌우 쌍들을 만들고 여러 경로 가설을 동시에 비교
8. 폭 변화, 양쪽 경계 연속성, 진행 방향, 콘 간격으로 최적 경계열 선택
9. 콘 배치가 엇갈려 직접 쌍이 부족하면 좌우 두 경계선을 공통 전방 station에서
   보간하고, 유효 폭이 유지되는 연속 구간의 중점열만 생성
10. 완전한 실제 쌍 3개 이후 한쪽 끝 경계가 잠깐 끊기면 최대 2개 가상 경계 생성
11. 경계 중점열을 제한된 범위에서만 평활화하고 균일 간격으로 재표본화
12. 확인된 같은 열의 라바콘 사이를 가상 펜스로 연결해 경계 횡단을 금지
13. 콘 여부와 무관하게 모든 전방 LiDAR 반사점에 차량 폭 복도 충돌 검사
14. 차량 폭, 가상 경계 불확실성, 경로 길이, 곡률, confidence를 검사
15. Path와 같은 stamp의 상태를 발행하고, 제어기에서 둘을 정확히 짝지음
16. 속도 적응 lookahead, 곡률·정지거리·confidence 제한으로 조향·속도 계산

한 개의 가까운 쌍을 탐욕적으로 고르지 않습니다. 실차 설정은 첫 쌍이 차량의
좌우를 가로지르도록 강제하므로, 한쪽 라바콘 선 너머의 물체를 반대 경계로 잘못
선택해 코스 밖으로 진입하지 않습니다. 한 station의 좌우 콘이 모두
사라진 경우에는 다음 완전한 쌍까지 제한된 거리 안에서 건너뜁니다. 한쪽만
보이는 짧은 꼬리는 이미 확인된 경계 접선의 법선 방향으로 복원하지만, 가상 구간은
최대 개수·비율·추가 clearance·confidence 감점·저속 제한을 받습니다. 좌우 판단의
근거가 없는 처음부터 완전한 단일 경계만으로는 경로를 만들지 않습니다.

## 설치

ROS 2 Humble 워크스페이스의 `src` 아래에 이 폴더를 복사합니다.

```bash
cd ~/xycar_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install --packages-select lidar_cone_planner
source install/setup.bash
```

RPLIDAR 드라이버는 Slamtec의
[sllidar_ros2](https://github.com/Slamtec/sllidar_ros2)를 사용합니다. A1M8은 보통
115200 baud이며 공식 A1 launch는 `angle_compensate=true`입니다. 이 보정은 한
원시 반사점을 여러 각도 bin에 복제할 수 있으므로 포인트 수만으로 콘을 판단하지
않고 실제 rosbag으로 폭·각폭 임계값을 맞춰야 합니다.

장치 경로는 `/dev/ttyUSB0`보다 `/dev/serial/by-id/...` 또는 전용 udev symlink를
권장합니다. 사용자를 `dialout` 그룹에 넣고, 임시 `chmod 777`에 의존하지 마세요.

## 좌표계 설정 — 필수

기본 `planning_frame`은 `base_link`입니다. `base_link`의 원점은 Pure Pursuit 같은
후속 제어기가 사용하는 차량 기준점, 일반적으로 후륜축 중앙에 두는 것이 좋습니다.
LiDAR 프레임이 `laser`라면 실제 장착 위치와 yaw를 반영한 TF가 있어야 합니다.

예를 들어 LiDAR가 기준점보다 앞 0.18 m, 왼쪽 0.01 m이고 회전이 없다면:

```bash
ros2 run tf2_ros static_transform_publisher \
  --x 0.18 --y 0.01 --z 0.20 \
  --yaw 0.0 --pitch 0.0 --roll 0.0 \
  --frame-id base_link --child-frame-id laser
```

수치는 예시일 뿐입니다. 줄자와 장착 방향으로 직접 측정하십시오. TF가 없으면
플래너는 `TF_ERROR`와 빈 Path를 발행합니다.

경계와 중앙선은 TF 변환이 끝난 `planning_frame`에서만 계산합니다. 각 station의
진행 접선을 정규화해 `t=(tx, ty)`, 좌측 법선을 `n=(-ty, tx)`로 두며, 양쪽 경계가
있으면 `(left+right)/2`, 왼쪽만 있으면 `left-n*estimated_track_width/2`, 오른쪽만
있으면 `right+n*estimated_track_width/2`를 사용합니다. 이 offset은 곡선을 따라
회전하는 local normal이므로 고정된 y축 offset이 아닙니다. 임의의 curvature 기반
racing-line offset도 이 단계에서는 적용하지 않습니다. `/cone_planner/status`에는
`scan_transform_time=scan_stamp`, `center_offset_policy=local_tangent_left_normal`,
`racing_line_offset=disabled`가 함께 표시됩니다.

## 실행

### A1 + 정적 TF + 플래너 + OpenCV BEV (1단계 권장)

이 launch에는 모터·ESC·제어기 노드가 포함되지 않습니다. A1 드라이버는 115200 baud,
`angle_compensate=true`, `Sensitivity` 모드로 시작하고 예상 조향값만 진단으로 계산합니다.

먼저 `config/cone_lidar_cv.yaml`의 `lidar_x_m`, `lidar_y_m`, `lidar_z_m`,
`lidar_yaw_rad`를 실제 장착 치수로 바꿉니다. 기본 0은 측정 전 안전 placeholder입니다.

```bash
ros2 launch lidar_cone_planner cone_lidar_cv.launch.py \
  serial_port:=/dev/serial/by-id/<DEVICE> \
  planning_frame:=base_link \
  viewer_enabled:=true
```

SSH처럼 `DISPLAY`가 없는 환경에서 `viewer_enabled:=true`이면 자동 headless 모드로
전환해 `viewer_record_path`에 최신 프레임 또는 영상을 기록합니다. 경로를 생략하면
`/tmp/cone_cv_viewer_latest.png`를 사용합니다.

```bash
ros2 launch lidar_cone_planner cone_lidar_cv.launch.py \
  serial_port:=/dev/serial/by-id/<DEVICE> \
  viewer_enabled:=true \
  viewer_record_path:=/home/sandi/cone_cv_capture.png
```

GUI와 기록을 모두 끄되 viewer 노드를 안전하게 유지하려면
`viewer_enabled:=false viewer_record_path:=""`를 사용합니다. GUI 창에서는 `q`로
종료합니다. BEV는 차량 원점이 화면 아래 중앙, 위쪽이 전방, 왼쪽이 차량 좌측입니다.

### 플래너만 실행

RPLIDAR 드라이버를 먼저 실행한 뒤:

```bash
ros2 launch lidar_cone_planner cone_line_planner.launch.py
```

토픽과 기준 프레임을 launch 인자로 바꿀 수 있습니다.

```bash
ros2 launch lidar_cone_planner cone_line_planner.launch.py \
  scan_topic:=/scan planning_frame:=base_link
```

시뮬레이션 시간이 필요하면 `use_sim_time:=true`를 추가합니다. 설정 파일 전체를
바꾸려면 `config:=/absolute/path/to/cone_planner.yaml`을 지정합니다.

### 플래너와 제어기 함께 실행

```bash
ros2 launch lidar_cone_planner cone_drive.launch.py \
  scan_topic:=/scan planning_frame:=base_link
```

제어기는 시작되어도 `geometry_confirmed: false`, `enabled_on_startup: false`이므로
정지 명령만 발행합니다. 먼저 `config/cone_controller.yaml`에서 다음을 실제 차량으로
측정해 바꿉니다.

- `wheelbase_m`: 후륜축 중심부터 전륜축 중심까지 거리
- `max_steering_angle_rad`: 핸들 모터 값이 아닌 실제 앞바퀴 최대 조향각
- `planner_max_curvature_1pm`: `tan(max_steering_angle_rad) / wheelbase_m` 이하
- `geometry_confirmed: true`: 위 두 값을 직접 확인한 뒤에만 변경
- `require_odometry: true`: 실차 기본값. odom stamp와 `base_link` child frame도 검사

설정 후에도 기본값은 운전 비허가입니다. 차량 바퀴를 바닥에서 띄우고 Path·상태·
명령을 확인한 뒤 다음 서비스로 활성화합니다.

```bash
ros2 service call /cone_controller/set_enabled std_srvs/srv/SetBool "{data: true}"
```

활성화 후에도 **동일 stamp로 짝지어진** Path와 상태가 기본 3회 연속 들어와야
출발합니다. 유효 쌍 사이의 공백이 receipt timeout을 넘으면 횟수는 다시 0이 됩니다.
비활성화는 같은 서비스에 `{data: false}`를 보내며 즉시 속도와 조향을 0으로 만듭니다.
오류 정지 뒤에는 odometry가 없을 때의 관성 이동을 고려해 기본 0.5초
`minimum_stop_hold_s` 동안 새 경로가 와도 재출발하지 않습니다.

### 수동주행 구동계를 사용하는 자율주행

`cone_autonomous_drive.launch.py`는 LiDAR 중앙 경로, Pure Pursuit, SI 단위 명령을
수동주행 프로토콜의 `[throttle, steering]`으로 바꾸는 어댑터를 함께 실행합니다.
기본값은 Arduino를 열지 않고 `/rc_car/drive_cmd_preview`만 발행합니다.

```bash
ros2 launch lidar_cone_planner cone_autonomous_drive.launch.py \
  lidar_port:=/dev/serial/by-id/<LIDAR_DEVICE> \
  planner_config:=/home/sandi/KMU_AutoDriving/configs/cone/cone_planner.yaml \
  system_config:=/home/sandi/KMU_AutoDriving/configs/cone/cone_lidar_cv.yaml \
  controller_config:=/home/sandi/KMU_AutoDriving/configs/cone/cone_controller.yaml \
  dry_run:=true serial_bridge:=false
```

Pure Pursuit는 여전히 시작 시 비활성입니다. 경로와 preview가 정상인지 확인한 뒤:

```bash
ros2 service call /cone_controller/set_enabled \
  std_srvs/srv/SetBool "{data: true}"
ros2 topic echo /rc_car/drive_cmd_preview
ros2 topic echo /vehicle/autonomous_drive_status
```

어댑터는 `cone_controller/status`가 `enabled=true`이면서 `OK` 또는 `OK_VIRTUAL`이고,
명령과 상태가 각각 0.2초보다 최신일 때만 값을 전달합니다. 그 밖의 경우 `[0, 0]`을
즉시 발행합니다. 실차 모드에서는 같은 조건으로 serial bridge의 ARM/deadman도
관리하며, preview 모드에서는 두 신호를 항상 false로 유지합니다. `ackermann_msgs`가
없는 환경에서는 동일한 SI 단위를 담는 `cone_controller/command_vector`
(`x=속도 m/s`, `y=실제 바퀴 조향각 rad`, `z=0`)를 자동으로 사용합니다.

실차 실행 전 `cone_controller.yaml`의 wheelbase·최대 실제 바퀴 조향각과
`autonomous_drive.yaml`의 속도/조향 한계 및 throttle/steering counts를 실제 측정값으로
일치시켜야 합니다. 확인 후에만 다음처럼 세 개의 live interlock을 모두 명시합니다.

```bash
ros2 launch lidar_cone_planner cone_autonomous_drive.launch.py \
  lidar_port:=/dev/serial/by-id/<LIDAR_DEVICE> \
  arduino_port:=/dev/serial/by-id/<ARDUINO_DEVICE> \
  dry_run:=false hardware_confirmed:=true serial_bridge:=true
```

위 명령만으로는 출발하지 않습니다. serial ready와 중립 명령을 확인한 뒤 별도로
`/cone_controller/set_enabled`를 true로 호출해야 합니다. false 호출, 경로 유실,
오래된 명령, 범위 초과 입력은 throttle과 steering을 모두 0으로 만듭니다.

## 출력

| 토픽 | 형식 | 내용 |
|---|---|---|
| `cone_planner/center_path` | `nav_msgs/Path` | 유효 중앙 경로. 무효 시 poses가 빈 Path |
| `cone_planner/cones` | `geometry_msgs/PoseArray` | 연속 관측으로 확인된 현재 콘 후보 |
| `cone_planner/raw_cones` | `geometry_msgs/PoseArray` | 단일 scan의 크기 기반 raw 후보(경로에는 직접 사용하지 않음) |
| `cone_planner/markers` | `visualization_msgs/MarkerArray` | 원시 후보·확인 콘·경계·중앙 경로 |
| `cone_planner/status` | `diagnostic_msgs/DiagnosticArray` | 상태, scan age, 처리시간, confidence 등 |
| `cone_controller/command` | `ackermann_msgs/AckermannDriveStamped` | 표준 속도·실제 바퀴 조향각 명령 |
| `cone_controller/command_vector` | `geometry_msgs/Vector3Stamped` | 명시적으로 허용해야 하는 전용 bridge 명령 |
| `cone_controller/status` | `diagnostic_msgs/DiagnosticArray` | 정지 이유, lookahead, 곡률, 입력 나이 등 |

`command_vector`는 일반 `cmd_vel`이 아닙니다. 전용 bridge 계약은 `vector.x`가 m/s
단위 전진속도, `vector.y`가 rad 단위 실제 바퀴 조향각이며 `vector.z`는 예약값입니다.
기본 `allow_compat_command: false`에서는 이 토픽만으로 운전을 허가하지 않습니다.
정상 설치에서는 아래 명령 또는 `rosdep install`로 표준 Ackermann 메시지를 준비하는
것을 권장합니다.

```bash
sudo apt install ros-humble-ackermann-msgs
```

주요 상태는 다음과 같습니다.

- `OK`: 모든 안전 조건을 통과한 실제 양측 Path
- `OK_VIRTUAL`: 제한된 한쪽 누락 복원을 사용한 저신뢰·저속 Path
- `NOT_ENOUGH_CONES`, `NO_VALID_PAIR`, `INSUFFICIENT_PAIRS`
- `VIRTUAL_LIMIT_EXCEEDED`: 가상 station 개수/비율 한계를 넘은 한쪽 누락
- `INSUFFICIENT_CLEARANCE`, `PATH_OUTSIDE_CORRIDOR`
- `CONE_BOUNDARY_CROSSING`: 확인된 라바콘 열 사이의 가상 펜스를 경로가 횡단
- `OBSTACLE_ON_PATH`: 비라바콘 물체를 포함한 LiDAR 반사점이 차량 폭 복도 안에 있음
- `PATH_TOO_SHORT`, `CURVATURE_LIMIT`, `LOW_CONFIDENCE`
- `NO_SCAN`, `SCAN_TIMEOUT`, `STALE_SCAN`, `OUT_OF_ORDER_SCAN`
- `TF_ERROR`, `BAD_SCAN_GEOMETRY`, `PROCESSING_ERROR`

상태 key에는 `lookahead_x_m`, `lookahead_y_m`, `target_heading_rad`,
`pure_pursuit_curvature_1pm`, `expected_steering_angle_rad`, `scan_hz`, 처리시간과
실제/가상 station 수가 포함됩니다. `cone_center_bias_note`는 관측 중심이 센서 쪽
표면으로 치우친다는 점을 명시하며, 단일 콘 bag 실측 후
`cone_center_radial_offset_m`에 양의 보정 거리를 설정할 수 있습니다.

제어기 상태에는 `DISABLED`, `WAITING_FOR_VALID_FRAMES`, `INPUT_RECEIPT_TIMEOUT`,
`UNCOMPENSATED_PATH_MOTION`, `STEERING_LIMIT` 등 구체적인 즉시 정지 이유가 나옵니다.

## RViz

Fixed Frame을 `base_link`로 놓고 다음을 추가합니다.

`cone_planner/markers`의 `observed_boundaries`는 차량 좌우에서 시작한 두 경계열을
각각 전방으로 추적해 선으로 잇습니다. 각 단계에서 진행 각도와 간격에 가장 잘 맞는
후보 하나만 선택하므로 경계 바깥 물체로 선이 갈라지지 않습니다. 한 콘 검출 누락은
표시선만 이어 주되, 실제 경로 차단용 펜스는 더 짧은 간격으로 보수적으로 검사합니다.
OpenCV viewer에는 이 두 경계선이 노란색으로 표시됩니다.

- `LaserScan`: `/scan`
- `MarkerArray`: `/cone_planner/markers`
- `Path`: `/cone_planner/center_path`

색상은 연한 주황=원시 cluster, 진한 주황=시간축 확인 콘, 파랑=좌측 경계,
빨강=우측 경계, 자홍=원시 중점, 초록=최종 경로입니다. 가상 경계점은 크게 표시되며
좌측은 하늘색, 우측은 분홍색입니다.

## 실제 치수로 반드시 바꿀 값

`config/cone_planner.yaml`에서 다음을 우선 측정·수정합니다.

- `track_width_m`: 좌우 **라바콘 중심 사이** 평균 폭
- `track_width_min_m`, `track_width_max_m`: 실제 배치 편차를 포함한 허용 폭
- `expected_cone_spacing_m`: 같은 쪽 콘의 전후 간격
- `vehicle_width_m`: 범퍼·바퀴를 포함한 차량 최대 외폭
- `safety_margin_m`: 각 측면에 추가로 확보할 여유
- `cone_obstacle_radius_m`: 모든 관측 콘 주위에 더할 선택적 반경. 기본 0은
  `track_width_m`가 콘 중심 간 거리라는 현재 계약을 따른 값이며, 올리기 전에 실제
  통로 폭이 `vehicle_width + 2 * (safety_margin + cone radius)`보다 큰지 확인
- `boundary_fence_max_gap_m`: 같은 열로 연결할 라바콘 사이 최대 거리. 반대편 열과
  연결되지 않도록 `track_width_min_m`보다 작게 두고 실제 전후 간격보다 약간 크게 설정
- `boundary_row_confidence_weight`: 엇갈린 좌우 경계선을 보간해 만든 중앙 경로의
  confidence 감점. 두 경계가 모두 확인돼도 직접 콘 쌍보다 낮게 평가
- `max_path_curvature_1pm`: 차량 최대 조향각과 축간거리에서 계산한 곡률 한계
- `min_path_length_m`: 제동거리와 제어 lookahead보다 길게 설정
- `max_virtual_pairs`, `max_virtual_fraction`: 한쪽 누락 복원의 절대·비율 한계
- `virtual_boundary_uncertainty_m`: 가상 경계 사용 시 추가할 측면 오차

차량 폭 0.50 m, 콘 중심 간 폭 0.60 m라면 기하학적 여유는 한쪽 0.05 m뿐입니다.
기본 안전 여유 0.02 m를 제외하면 센서·콘 배치·제어 오차에 남는 값은 0.03 m입니다.
실측 오차가 이보다 작다는 자료가 없으면 모터를 연결하지 마세요.

제어기에서는 `wheelbase_m`, `max_steering_angle_rad`, 실제 중립 정지거리로 구한
`max_decel_mps2`가 핵심입니다. 기본 0.20 m와 35도는 실행 가능한 예시일 뿐 실측값이
아니며, 그래서 `geometry_confirmed`가 기본 false입니다.

## A1M8 실데이터 튜닝 순서

1. 차량을 고정하고 `/scan`, `/tf`, planner 출력을 rosbag2로 기록합니다.
2. 0.3, 0.5, 1, 2, 3 m의 단일 콘으로 검출 폭과 점 개수를 확인합니다.
3. `cluster_min_independent_ranges`가 실제 콘은 통과시키고 동일거리 복제 반사는
   제거하는지 `angle_compensate=true/false` bag을 비교합니다.
4. 벽·의자 다리·차체가 콘으로 통과하지 않게 `cone_max_width_m`, 각폭, 깊이를 조절합니다.
5. 콘 중앙의 프레임 간 흔들림으로 `track_match_distance_m`를 정합니다.
6. 직선·곡선에서 실제 중심 오차와 false-valid 비율을 측정합니다.
7. USB 분리와 드라이버 종료 후 `scan_timeout_s` 안에 빈 Path가 나오는지 확인합니다.
8. 저속 수동 시험 후에만 Pure Pursuit/조향 제어기에 연결합니다.

`angle_compensate`, scan mode, 회전 속도를 바꾸면 cluster 포인트 수와 각폭 분포도
바뀌므로 다시 기록하고 튜닝해야 합니다. `LaserScan`에는 보정 전 원시 반사의 출처가
남지 않기 때문에 `cluster_min_independent_ranges`는 거리값 차이에 기반한 휴리스틱이지
독립 반사를 완전히 복원하는 판별기가 아닙니다.

## 정지 상태 rosbag 기록 절차

차량 구동 전 LiDAR를 고정하고 각 장면을 별도 bag으로 기록합니다.

1. 빈 공간
2. 콘 한 개를 0.5 m, 1.0 m, 1.5 m에 각각 배치
3. 직선 좌우 콘 4쌍 이상
4. 차량 기준 5 cm, 10 cm 좌우 편심
5. 좌회전과 우회전
6. 한쪽 콘 한 개 누락
7. 벽, 기둥, 의자 다리
8. 기록 중 LiDAR USB 분리

```bash
ros2 bag record /scan /tf /tf_static /odom \
  /cone_planner/raw_cones \
  /cone_planner/cones \
  /cone_planner/center_path \
  /cone_planner/status
```

재생 시 `/scan` stamp와 `/tf`를 그대로 사용하므로 실시간과 동일한 fail-closed
판정을 수행합니다. `/clock`을 기록하고 `use_sim_time:=true`로 재생하는 경우에는
모든 노드에 같은 설정을 적용해야 합니다. 각 bag에서 scan 주기, raw/confirmed 수,
유효 경로 비율, 중앙선 lateral jitter, 처리시간 p50/p95/max와 false-valid를
기록합니다. USB 분리 장면은 0.40초 이내 빈 Path와 `SCAN_TIMEOUT`이어야 합니다.

## 시험

### 실물 라바콘 없는 합성 폐루프

GUI 없이 빠르게 알고리즘만 반복 검증하려면 ray-cast 센서, 실제 플래너 코어,
Pure Pursuit, 자전거 모델을 한 프로세스에서 실행합니다.

```bash
ros2 run lidar_cone_planner synthetic_validation all
ros2 run lidar_cone_planner synthetic_validation straight --scan-dropout-step 70
```

출력 JSON에는 진행거리, 경로 유효 비율, 횡오차 p95/최대값, 최소 콘 여유,
충돌 수, 최대 속도·조향, fault 이후 양수 명령과 추가 이동이 포함됩니다.

실제 ROS 토픽, TF, odometry, watchdog까지 포함한 폐루프는 다음과 같이 실행합니다.
실차 노드와 섞이지 않도록 기본 namespace는 `/sim`입니다.

```bash
ros2 launch lidar_cone_planner synthetic_closed_loop.launch.py \
  scenario:=straight
```

지원 코스는 `straight`, `left_arc`, `right_arc`, `s_bend`입니다. 스캔 단절을
자동 주입하려면 다음처럼 실행합니다.

```bash
ros2 launch lidar_cone_planner synthetic_closed_loop.launch.py \
  scenario:=straight drop_scan_after_s:=15.0 drop_scan_duration_s:=8.0
```

기본 ROS 합성 프로필은 A1과 비슷한 10 Hz scan, 최고 0.10 m/s로 제한합니다.
단절 시험은 실제로 출발한 뒤 fault가 걸리도록 15초 이후로 설정하는 편이 안전합니다.
정상 직선 시험에서는 진행거리 증가, 충돌 0, 명령 거부 0을 확인합니다. 단절 중에는
월드가 `SCAN_DROP_ACTIVE`, 플래너가 `SCAN_TIMEOUT`, 제어기가 속도·조향 0을 보여야
합니다. 스캔 재개 뒤에는 새 유효 경로 세 장을 확인하기 전까지 재출발하지 않습니다.

관찰할 핵심 토픽:

- `/sim/synthetic_world/status`: 진행거리·횡오차·여유거리·충돌·명령 상태
- `/sim/cone_planner/status`: 콘 검출과 경로 유효 상태
- `/sim/cone_controller/status`: 주행 또는 정지 이유
- `/sim/synthetic_world/markers`: RViz용 콘·정답 중심선·차량 표시
- `/sim/cone_planner/markers`: 플래너가 인식한 콘과 계획 경로 표시

합성 월드 설정은 `config/synthetic_world.yaml`, 시뮬레이션 전용 제어 설정은
`config/sim_controller.yaml`입니다. 후자는 custom `Vector3Stamped` 명령을 명시적으로
허용하지만, 실차용 `cone_controller.yaml`의 잠금 설정은 변경하지 않습니다.

ROS 없이 core 시험:

```bash
cd ~/xycar_ws/src/lidar_cone_planner
python3 -m unittest discover -s test -v
```

현재 시험은 직선·곡선·편심 진입·잡음 쌍·완전/한쪽 station 누락·가상 경계 한계·
A1 동일거리 복제·scan hole·TF 수치 변환·시간축 확인·경로 clearance·곡률 제한을
검사합니다. 제어기는 좌우 조향 부호, arc-length lookahead, 조향·가속 변화율,
곡률·정지거리·confidence 속도 제한과 즉시 정지를 검사합니다.
현재 ROS 2 Humble 회귀시험은 local-normal station 수식, scan-stamp TF, 실제 DDS
단절 시험과 합성 센서·폐루프·wrapper를 포함해 95개입니다.

ROS 2가 준비된 워크스페이스에서는 다음 명령으로 wrapper 시험까지 실행합니다.

```bash
cd ~/xycar_ws
colcon test --packages-select lidar_cone_planner --event-handlers console_direct+
colcon test-result --verbose
```

ROS 환경에서 추가로 확인해야 할 합격 조건:

- 마지막 scan 이후 `scan_timeout_s` 안에 빈 Path와 `SCAN_TIMEOUT` 발행
- zero/stale/out-of-order stamp와 TF 오류가 모두 빈 Path를 만듦
- 출력 frame이 항상 `planning_frame`과 일치
- p95 처리시간이 실제 scan 주기보다 충분히 짧음
- Path와 status stamp가 맞지 않으면 제어기가 움직이지 않음
- 제어기가 빈/오래된 Path와 수신 단절에서 실제로 즉시 정지함
- A1 scan 시각부터 현재까지의 미보정 이동량이 설정 한계를 넘으면 정지함

## 차량 연결 단계

이 패키지는 안전한 속도·조향각 목표까지만 발행합니다. 다음 단계는
`AckermannDriveStamped` 또는 문서화된 호환 토픽을 Arduino의
`D <throttle> <steering>` 같은 실제 프로토콜로 변환하는 별도 bridge입니다.
bridge도 ROS 명령 수신 watchdog과 Arduino 자체 300 ms 이하 watchdog을 모두 가져야
하며, bridge 종료·USB 분리·ROS 노드 종료 중 어느 경우에도 ESC 중립이 되어야 합니다.

첫 시험 순서는 바퀴 공중→스로틀 0 조향만→최대 0.05 m/s 직선→넓은 코스
0.10 m/s→실측 정지거리 반영 순서를 권장합니다. 현재 기본 최대속도 0.15 m/s를
검증 전부터 그대로 사용하지 마십시오.

## 참고한 오픈소스

- [FaSTTUBe Path Planning](https://github.com/papalotis/ft-fsd-path-planning):
  한쪽 경계 누락과 가상 콘 아이디어
- [laser_segmentation](https://github.com/ajtudela/laser_segmentation):
  거리 변화에 대응하는 LaserScan jump-distance 분할 방식
- [PythonRobotics Pure Pursuit](https://github.com/AtsushiSakai/PythonRobotics/tree/master/PathTracking/pure_pursuit):
  후륜축 기준 Pure Pursuit 수식과 속도 연동 lookahead
- [Nav2 Regulated Pure Pursuit](https://github.com/ros-navigation/navigation2/tree/main/nav2_regulated_pure_pursuit_controller):
  곡률·충돌 위험에 따른 속도 제한 구조

외부 소스 파일을 복사하거나 vendoring하지 않았고, 위 아이디어를 현재의 NumPy 기반
로컬 좌표계·fail-closed 계약에 맞춰 독립 구현했습니다. 상세 비교와 라이선스는
`OPEN_SOURCE_REFERENCES.md`에 정리되어 있습니다.

# LANE/OBSTACLE_AVOID/CONE 통합 Pure Pursuit 주행

## 구성

통합 주행은 아래 경로를 사용한다.

1. `yolo_seg_lane_detector`
   - 카메라 영상에서 YOLO `center`/`lane` 마스크를 검출한다.
   - 양쪽 차선의 가운데를 우선 사용하고 중앙선으로 보강한다.
   - 4점 원근변환으로 영상 경로를 `base_link` 미터 좌표로 변환한다.
   - `/planning/lane_path`를 발행한다.
2. `cone_line_planner`
   - RPLIDAR 군집에서 라바콘을 검출한다.
   - 좌·우 두 줄을 연결하고 대응하는 라바콘 쌍의 가운데를 계산한다.
   - `/perception/cone_path`와 `cone_planner/cones`를 발행한다.
3. `unified_autonomy`
   - `LANE`, `OBSTACLE_AVOID`, `CONE` 중 사용할 경로를 선택한다.
   - 선택된 경로에 동일한 Pure Pursuit를 적용한다.
   - `[throttle, steering]`을 `/rc_car/drive_cmd`로 계속 발행한다.

LiDAR driver와 라바콘 플래너는 모든 상태에서 계속 실행한다. YOLO 모델은
GPU에 올려 둔 채 `LANE`과 `OBSTACLE_AVOID`에서 카메라를 구독하고 추론한다.
`CONE` 진입 시 영상 구독과 추론을 멈추고, 차선 상태 복귀 시 재구독한다.
현재 활성 여부는 `/perception/yolo_subscription_active`에서 확인할 수 있다.

## 상태 전환

- 초기 상태는 항상 `LANE`이다.
- LiDAR 유효 포인트 중 라바콘 주변을 제외한 점들을 군집화한다.
- 현재 차선 경로 좌우 `0.15 m` 안에 비라바콘 군집이 있으면 장애물 차량으로
  판단하고 `OBSTACLE_AVOID`로 전환한다.
- 장애물이 경로 왼쪽에 있으면 오른쪽 차선, 오른쪽에 있으면 왼쪽 차선을
  선택한다. 중앙에 있으면 `preferred_offset_sign` 방향을 사용한다.
- 우회 경로는 원래 YOLO 차선 경로를 `opposite_lane_offset_m`만큼 부드럽게
  횡이동해 생성한다. 기본 차선 이동량은 `0.55 m`다.
- 장애물 차량이 `clear_sec` 동안 보이지 않으면 `LANE`으로 복귀한다.
- 좌측 `y > 0` 라바콘과 우측 `y < 0` 라바콘 한 쌍을 찾는다.
- 두 라바콘의 전후 위치 차이와 폭이 설정 범위에 있고 중앙 경로도 유효하며,
  쌍의 중심이 `enter_distance_m` 안으로 들어오면 `CONE`으로 전환한다.
- `CONE`에서 양쪽 라바콘 쌍 또는 중앙 경로가 `exit_missing_sec` 동안
  보이지 않으면 `LANE`으로 복귀한다.
- 모드 전환 시 중립 명령을 삽입하지 않는다.
- 선택 경로가 잠깐 없어져도 마지막 조향값과 해당 모드의 전진 속도를 유지한다.

기본값은 진입 거리 `0.90 m`, 종료 판정 `0.80 s`이며
`src/jetson/kmu_track/config/unified_autonomy.yaml`에서 조정한다.

## 빌드

```bash
cd /home/sandi/KMU_AutoDriving
source /opt/ros/humble/setup.bash
PYTHONNOUSERSITE=1 colcon build \
  --build-base .colcon/build \
  --install-base .colcon/install \
  --packages-select kmu_track lidar_cone_planner rc_car_teleop \
  --symlink-install
source .colcon/install/setup.bash
```

## 명령 출력 확인

Arduino 대신 preview 토픽으로 결과를 확인한다.

```bash
./scripts/run_competition.sh --unified-dry-run
```

```bash
ros2 topic echo /mission/state
ros2 topic echo /planning/active_path
ros2 topic echo /planning/pure_pursuit_target
ros2 topic echo /perception/lidar_obstacle_points
ros2 topic echo /perception/obstacle_vehicle
ros2 topic echo /vehicle/unified_autonomy_status
ros2 topic echo /rc_car/drive_cmd_preview
```

## Arduino 연결

일반 안전 프로필:

```bash
./scripts/run_competition.sh --unified-live
```

경기 전용 무정지 프로필:

```bash
./scripts/run_competition.sh --unified-no-stop-live
```

`--unified-live`는 기존 안전 동작을 보존한다. IRE 명령이 만료되거나 host
명령이 `0.20 s` 이상 끊기면 정차한다. `--unified-no-stop-live`만
`competition_no_stop_enabled=true`를 통합 노드와 시리얼 브리지에 동시에
적용한다. 정상 양의 throttle이 한 번 전달된 뒤에는 다음 소프트웨어 입력
손실에서 마지막 양의 throttle과 steering을 유지하며, 동적 arm/deadman
false도 무시한다. 카메라 또는 lane controller 프로세스 종료도 전체 launch
종료로 전파하지 않는다. 시작 전에는 여전히 `[0,0]` 상태로 대기한다.

두 프로필 모두 Arduino 통신 계층의 reset/펌웨어 확인이 끝나
`/rc_car/serial_ready=true`가 되는 즉시 통합 노드가 arm/deadman을 자동으로
활성화한다. 별도의 수동 arm 명령 없이 최소 전진 출력부터 모터로 전달된다.
명령 범위와 serial reset 확인 절차는 펌웨어 호환을 위해 그대로 사용한다.

무정지 프로필에서도 아래 경로는 정차를 유지한다.

- `/vehicle/estop=true`
- 시리얼 단선·partial write·펌웨어 watchdog
- steering feedback fault 또는 설정 범위 위반
- 프로세스 종료 및 전원 상실

따라서 소프트웨어 테스트는 일시적인 perception/IRE/host command 손실에서
`[0,0]` 또는 `X`를 만들지 않는다는 범위의 보증이다. 실제 코스 연속 주행은
리프트 테스트, 저속 직선 테스트, 모드 전환 실차 테스트 완료 후 판단한다.

활성 여부 확인:

```bash
ros2 param get /unified_autonomy competition_no_stop_enabled
ros2 param get /serial_bridge competition_no_stop_enabled
ros2 topic echo /vehicle/unified_autonomy_status
ros2 topic echo /rc_car/tx_stats
```

## 먼저 보정할 값

- `segmentation_lane.yaml`의 영상 사다리꼴 4점과 실제 지면 좌표
- `unified_autonomy.yaml`의 `wheelbase_m`
- `maximum_steering_angle_rad`, `maximum_steering_counts`, `steering_sign`
- `enter_distance_m`, 라바콘 폭 범위, `maximum_pair_dx_m`
- 차선/라바콘 lookahead와 throttle counts

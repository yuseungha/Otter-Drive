# 2026-08-21 주행 상태 인식 CV 정리

## 1. 목적

카메라와 LiDAR를 동시에 받아 현재 차량이 어떤 방식으로 주행해야 하는지
하나의 OpenCV 창에서 확인한다.

```text
BRIO 카메라 → YOLO 차선 인식 ┐
                              ├→ 통합 상태 판단 → DRIVE MODE 표시
RPLIDAR → 라바콘·장애물 인식 ┘
```

이 창은 다음 세 주행 모드를 표시한다.

| 표시 | 의미 |
|---|---|
| `LANE` | 카메라 YOLO 차선 경로를 따라 주행 |
| `CONE` | LiDAR가 만든 라바콘 중앙 경로를 따라 주행 |
| `OBSTACLE_AVOID` | LiDAR 장애물을 피해 반대 차선 경로로 주행 |

## 2. 입력과 처리 흐름

### 카메라·YOLO

- 장치: Logitech BRIO `/dev/video0`
- 안정 장치 경로:
  `/dev/v4l/by-id/usb-046d_Logitech_BRIO_5FD2713E-video-index0`
- 입력 설정: `1920x1080`, `15 FPS`, `MJPG`
- 모델: `models/center_lane_best.pt`
- 모델 클래스: `center`, `lane`
- 차선 경로: `/planning/lane_path`
- CV 오버레이: `/lane/lane_overlay`

### LiDAR

- 장치: RPLIDAR, CP2102 `/dev/ttyUSB1`
- 안정 장치 경로:
  `/dev/serial/by-id/usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0001-if00-port0`
- 스캔 토픽: `/scan`
- 확정 라바콘: `/cone_planner/cones`
- 라바콘 중앙 경로: `/perception/cone_path`
- 장애물 점: `/perception/lidar_obstacle_points`

### 상태 판단

`unified_autonomy`가 카메라 차선 경로와 LiDAR 결과를 함께 판단한다.
선택된 모드는 `/mission/state`에 발행한다.

```text
기본 상태                                      → LANE
가까운 라바콘 쌍과 유효한 라바콘 경로 감지    → CONE
현재 경로를 차지하는 비라바콘 군집 감지        → OBSTACLE_AVOID
```

라바콘 경로가 `0.80초` 동안 끊기면 `CONE`에서 `LANE`으로 돌아오고,
장애물이 `0.60초` 동안 사라지면 `OBSTACLE_AVOID`에서 `LANE`으로 돌아온다.

## 3. 통합 CV 창

창 이름은 `KMU Unified Perception`이다.

- 왼쪽: BRIO 원본 영상과 YOLO 차선 segmentation 결과
- 오른쪽: LiDAR 점, 라바콘 후보·확정점, 라바콘 중앙 경로
- 왼쪽 아래: `CAMERA/YOLO`와 `LIDAR`의 `LIVE/STALE` 상태
- 상단과 왼쪽 아래: `DRIVE MODE`
- 경로를 사용할 수 없으면: `STOP / INVALID`

`DRIVE MODE`와 실제 주행 가능 여부는 별개다. 예를 들어 기본 선택 모드가
`LANE`이어도 차선 인식 결과가 `LOST`이면 화면에 `STOP / INVALID`이 함께
표시되며, 이때는 주행하면 안 된다.

## 4. 실행 방법

Arduino와 모터 출력을 연결하지 않는 인식·판단 전용 실행:

```bash
cd /home/sandi/KMU_AutoDriving
./scripts/run_competition.sh --unified-dry-run
```

`.env`의 `KMU_DISPLAY=true`가 필요하다. 이 모드는 실제
`/rc_car/drive_cmd`가 아니라 `/rc_car/drive_cmd_preview`만 발행하며
`serial_bridge`를 실행하지 않는다.

종료는 실행 터미널에서 `Ctrl+C`를 누른다.

현재 모드만 별도 확인하려면 다음을 실행한다.

```bash
export ROS_DOMAIN_ID=86
source /opt/ros/humble/setup.bash
source /home/sandi/KMU_AutoDriving/.colcon/install/setup.bash
ros2 topic echo /mission/state
```

## 5. 2026-08-21 실제 확인 결과

- RPLIDAR 연결 및 health `OK`
- RPLIDAR `Sensitivity` 모드, 장치 기준 약 `10 Hz`
- `/scan` 발행자 1개와 실시간 메시지 확인
- BRIO `1920x1080@15 MJPG` 오픈 성공
- YOLO segmentation CUDA 로드 성공
- `/lane/lane_overlay` 약 `5 Hz` 확인
- 통합 CV 창 생성 및 카메라·LiDAR 양쪽 `LIVE` 확인
- `LANE`, `CONE`, `OBSTACLE_AVOID` 상태 전환 확인
- 구동 출력은 연결하지 않은 dry-run으로 검증

검증 로그:

```text
/home/sandi/KMU_AutoDriving/logs/20260821-101837/competition.log
```

관련 패키지 `lidar_cone_planner`, `kmu_ire_track` 증분 빌드에 성공했고,
통합 viewer 회귀 테스트 5개가 모두 통과했다.

## 6. 확인된 주의점

실내에서 의자, 사람, 벽을 바라본 상태에서는 LiDAR 군집이 라바콘이나
장애물로 해석되어 모드가 자주 바뀌었다. 실제 코스가 아닌 실내 결과만으로
상태 판단 정확도를 평가하면 안 된다.

실제 주행 전에 다음 항목을 코스에서 조정해야 한다.

1. 라바콘 크기·간격 필터와 진입 거리
2. 장애물 군집 크기와 지속 프레임 수
3. `LANE ↔ CONE ↔ OBSTACLE_AVOID` 전환 유지 시간
4. 카메라 ROI와 영상-지면 좌표 투영값
5. 모드 전환 중 `STOP / INVALID` 안전 조건

현재 상태 판단 검증이 끝날 때까지 `--unified-live`는 사용하지 않고
`--unified-dry-run`으로만 시험한다.

## 7. 주요 구현 파일

- `src/jetson/kmu_ire_track/launch/ire_unified_autonomy.launch.py`
  - 카메라, YOLO, RPLIDAR, 라바콘 플래너, 상태 판단 통합 실행
- `src/jetson/kmu_track/kmu_track/unified_autonomy_node.py`
  - 주행 모드 선택과 `/mission/state` 발행
- `src/jetson/lidar_cone_planner/lidar_cone_planner/cone_cv_viewer.py`
  - 카메라 YOLO와 LiDAR를 한 창으로 합성하고 `DRIVE MODE` 표시
- `src/jetson/kmu_track/config/unified_autonomy.yaml`
  - 모드 전환과 장애물 판단 설정
- `configs/cone/cone_planner.yaml`
  - LiDAR 라바콘 검출과 중앙 경로 설정
- `scripts/run_competition.sh`
  - dry-run 실행 및 X11 통합 CV 창 전달

## 8. 현재 실행 상태

문서 작성 시점에는 통합 CV, Docker 컨테이너, 카메라와 LiDAR 점유를 모두
종료한 상태다.

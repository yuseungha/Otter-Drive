# ROS 2 Humble 트랙 알고리즘 워크스페이스

## 빌드

Windows PowerShell에서 WSL에 진입한 다음 실행한다.

```bash
wsl -d Ubuntu-22.04
cd "$(git rev-parse --show-toplevel)/seungha/laptop/ros2_ws"
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

## 하드웨어 없이 FSM 검증

```bash
ros2 launch kmu_track track_bringup.launch.py demo:=true
```

다른 터미널에서 상태를 확인한다.

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 topic echo /mission/state
```

데모는 출발 신호와 각 구간 완료 이벤트를 자동 발행하며 세 번째 `lap_complete` 뒤 `FINISHED`가 된다.

데모 모드에서는 카메라 입력이 없으므로 영상 노드는 기동하지 않는다. 실제 센서 모드에서만 신호등·차선 노드가 함께 시작된다.

## 실제 센서 모드

```bash
ros2 launch kmu_track track_bringup.launch.py demo:=false
```

HSV 조절창과 전체 영상 파이프라인을 함께 열려면 다음처럼 실행한다.

```bash
ros2 launch kmu_track track_bringup.launch.py demo:=false display:=true
```

카메라 드라이버는 `/camera/front/image_raw`에 `sensor_msgs/msg/Image`를 발행해야 한다. ROI와 임계값은 `src/kmu_track/config/perception.yaml`에서 트랙 영상에 맞게 조정한다.

현재 WSL 사용자 영역에는 NumPy 2.x가 설치되어 있지만 ROS 2 Humble의 `cv_bridge`는 Ubuntu 시스템 NumPy 1.x ABI를 사용한다. YOLO 노드는 `cv_bridge` 없이 카메라 메시지를 직접 변환하고 ROI·BEV·HSV 전처리까지 한 프로세스에서 처리한다. 이에 따라 BEV와 마스크를 노드 사이에서 복사하던 병목과 NumPy ABI 충돌을 함께 제거했다.

## MP4 영상으로 창에서 확인

Git에 포함하지 않은 시험 영상과 모델의 절대 경로를 실행 인자로 전달해 카메라 대신
재생하고 전체 처리 단계와 YOLO 결과를 별도 창에 표시한다.

```bash
ros2 launch kmu_track track_video.launch.py \
  video_path:=/absolute/path/to/Track_drive_mp4.mp4 \
  model_path:=/absolute/path/to/road_best.pt
```

창 조작키:

- `Space`: 일시정지 또는 재생
- `R`: 영상 처음부터 다시 시작
- `Q` 또는 `Esc`: 창과 launch 종료

`KMU Track Vision` 창은 필요한 세 화면만 한 줄로 표시한다.

```text
원본 | 흰색·노란색 결합/노이즈 제거 마스크 | YOLO lane1/lane2 및 중심 오차
```

ROI/BEV와 개별 흰색·노란색 마스크는 내부에서 계산하지만 기본 화면과 토픽에는 내보내지 않는다. 점검이 필요하면 `perception.yaml`의 `publish_bev_debug` 또는 `publish_individual_masks`를 `true`로 바꾼다.

별도의 `HSV Threshold Controls` 창에는 효과가 큰 값 여섯 개만 표시한다. 흰색은 밝기 하한과 채도 상한, 노란색은 색상 범위와 채도·밝기 하한을 조절한다. 나머지 안전한 상·하한은 자동으로 고정되며 `D` 키로 기본값을 복원할 수 있다. 값이 바뀌면 `/lane/hsv_thresholds/set`을 통해 전처리 노드에 즉시 반영된다. 확정한 값은 `src/kmu_track/config/perception.yaml`의 `white_*_hsv`, `yellow_*_hsv`에 기록한다.

YOLO 장치는 `auto`가 기본이다. CUDA GPU가 보이면 GPU와 FP16을 사용하고, 없으면 CPU로 자동 폴백한다. WSL의 RTX 5050에서 실제 MP4 통합 테스트 결과 추론 약 20 ms, 출력 약 15.05 Hz로 확인했다. YOLO 화면 왼쪽 위에는 현재 추론 시간과 이론 FPS가 표시된다.

`road_best.pt`는 `lane1`, `lane2` 클래스의 detection 모델이다. 실제 샘플에서 원본은 0.96/0.95, BEV 컬러는 0.89/0.86 신뢰도로 두 차선을 검출했지만 binary 및 마스크 적용 컬러 입력에서는 검출하지 못했다. 따라서 YOLO에는 BEV 컬러를 입력하고, 결합 binary는 검출 박스 안에 실제 흰색·노란색 차선 픽셀이 존재하는지 검증하는 용도로 사용한다. HSV 값이 아직 맞지 않을 때는 검출을 즉시 버리지 않고 신뢰도를 0.75배로 낮춰 사용하는 소프트 폴백이 기본이다. UI에서 값을 확정한 뒤 `strict_mask_validation: true`로 바꾸면 마스크 검증을 통과한 차선만 중심 계산에 사용한다.

### 창이 투명하거나 `[WARN:COPY MODE]`가 표시될 때

창 제목이 `[WARN:COPY MODE] KMU Track Vision (Ubuntu-22.04)`로 바뀌고 영상 대신 뒤쪽 Windows 창이 보이면 ROS/OpenCV 문제가 아니라 WSLg 공유 메모리 연결 실패다. `/mnt/wslg/weston.log`의 `rdp_allocate_shared_memory ... Input/output error`를 확인한 뒤, 실행 중인 WSL 작업을 저장하고 Windows PowerShell에서 다음을 실행한다.

```powershell
wsl --update
wsl --shutdown
```

`wsl --shutdown`은 모든 WSL 배포판과 실행 중인 ROS 프로세스를 종료한다. 이후 Ubuntu를 다시 열고 워크스페이스를 source한 다음 영상 launch를 재실행한다.

현재 `/etc/wsl.conf`에서 `appendWindowsPath=true`가 `[boot]` 아래에 있으면 별도의 `Unknown key 'boot.appendWindowsPath'` 경고가 발생한다. 이 값은 다음처럼 `[interop]` 아래에 둔다.

```ini
[boot]
systemd=true

[interop]
appendWindowsPath=true

[user]
default=xytron
```

이 잘못된 키 배치는 COPY-MODE의 직접 원인은 아니지만 WSL 시작 경고를 없애기 위해 함께 수정하는 것이 좋다.

원본 영상은 640×480, 15 FPS, 약 258.4초다. WSL의 ROS 이미지 복사량을 줄이기 위해 재생 노드는 기본적으로 320×240으로 축소해 발행하며, YOLO의 BEV 출력도 320×240을 사용한다. 다른 영상을 사용할 때는 WSL 절대 경로를 넘긴다.

```bash
ros2 launch kmu_track track_video.launch.py \
  video_path:="/mnt/c/path/to/another_video.mp4" \
  playback_rate:=1.0 loop:=true
```

## 핵심 입력 토픽

| 토픽 | 타입 | 의미 |
|---|---|---|
| `/camera/front/image_raw` | `sensor_msgs/msg/Image` | 전방 카메라 |
| `/vehicle/speed_mps` | `std_msgs/msg/Float32` | 실측 차량 속도 |
| `/perception/start_signal` | `std_msgs/msg/Bool` | 빨간불을 본 뒤 출발 녹색불 확정 |
| `/perception/left_signal` | `std_msgs/msg/Bool` | 좌회전 화살표 ROI 점등 확정 |
| `/mission/event` | `std_msgs/msg/String` | 구간 전환 이벤트 |

`/mission/event`에서 허용하는 정상 전이 이벤트는 다음과 같다.

```text
cone_complete
static_obstacle_zone
fixed_obstacle_clear
overtake_complete
shortcut_complete
lap_complete
reset
```

수동 이벤트 주입 예시:

```bash
ros2 topic pub --once /mission/event std_msgs/msg/String "{data: cone_complete}"
```

## 핵심 출력 토픽

| 토픽 | 타입 | 의미 |
|---|---|---|
| `/mission/state` | `std_msgs/msg/String` | 현재 FSM 상태, transient-local |
| `/mission/lap` | `std_msgs/msg/UInt8` | 완료한 랩 수 |
| `/mission/elapsed_sec` | `std_msgs/msg/Float32` | 출발 이후 경과 시간 |
| `/mission/remaining_sec` | `std_msgs/msg/Float32` | 안전 마진을 반영한 잔여 시간 |
| `/lane/center_error` | `std_msgs/msg/Float32` | 정규화된 차선 중심 오차; 우측이 양수 |
| `/lane/valid` | `std_msgs/msg/Bool` | 차선 검출 유효 여부 |
| `/lane/confidence` | `std_msgs/msg/Float32` | 차선 검출 신뢰도 |
| `/lane/bev_image` | `sensor_msgs/msg/Image` | 선택적 BEV 디버그 영상 (`publish_bev_debug`) |
| `/lane/white_mask` | `sensor_msgs/msg/Image` | 선택적 흰색 마스크 디버그 영상 |
| `/lane/yellow_mask` | `sensor_msgs/msg/Image` | 선택적 노란색 마스크 디버그 영상 |
| `/lane/debug_binary` | `sensor_msgs/msg/Image` | 결합 및 노이즈 제거된 binary 마스크 |
| `/lane/yolo_debug` | `sensor_msgs/msg/Image` | YOLO 박스와 계산된 중심선 |
| `/lane/yolo_detections` | `std_msgs/msg/String` | 클래스·신뢰도·마스크 중첩률 JSON |
| `/lane/inference_ms` | `std_msgs/msg/Float32` | 프레임별 YOLO 추론 시간(ms) |
| `/safety/stall_detected` | `std_msgs/msg/Bool` | 설정 시간 이상 정지 감지 |
| `/safety/recovery_requested` | `std_msgs/msg/Bool` | 경로/제어 계층에 재출발 요청 |
| `/vehicle/stop_requested` | `std_msgs/msg/Bool` | 완주 또는 안전 가드에 따른 정지 요청 |

## FSM 상태

```text
WAIT_START
  -> CONE_SLALOM
  -> LANE_FOLLOW
  -> STATIC_AVOID
  -> OVERTAKE
  -> SHORTCUT_WAIT
  -> SHORTCUT_LEFT
  -> LAP_RUN
  -> (다음 랩의 CONE_SLALOM 또는 FINISHED)
```

`ABORTED`는 55초 정지 가드 또는 235초 미션 시간 가드가 작동했을 때 진입한다. 60초 및 240초 규정에 각각 5초의 안전 여유를 둔 기본값이며 YAML에서 변경할 수 있다.

## 다음 연결 지점

1. MCU bridge가 `/vehicle/speed_mps`를 발행하고 `/vehicle/stop_requested`를 최우선으로 소비한다.
2. 라바콘/YOLO 노드가 각 구간 완료 조건을 판단해 `/mission/event`를 발행한다.
3. 조향 제어기는 `/lane/center_error`를 입력으로 사용하되, `/mission/state`에 따라 차선·회피·추월 제어기를 선택한다.
4. 실제 카메라 마운트 확정 후 `perception.yaml`의 ROI를 재보정하고 데이터를 다시 수집한다.

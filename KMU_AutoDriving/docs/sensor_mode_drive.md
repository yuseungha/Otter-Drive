# 카메라/라이다 perception 모드 전환 주행

이 launch는 카메라 publisher와 RPLIDAR driver를 항상 실행한다. 센서 전원이나
driver를 상태 전환에 사용하지 않는다. 대신 YOLO 이미지 subscription과 라이다
군집화 `LaserScan` subscription을 상호 배타적으로 생성·해제한다.

## 빌드

사용자 site-packages의 최신 setuptools와 ROS 2 Humble의 symlink-install이
충돌할 수 있으므로 다음처럼 빌드한다.

```bash
cd /home/sandi/KMU_AutoDriving
source /opt/ros/humble/setup.bash
PYTHONNOUSERSITE=1 colcon build \
  --build-base .colcon/build \
  --install-base .colcon/install \
  --packages-select rc_car_teleop lidar_cone_planner kmu_track \
  --symlink-install
```

## 센서 모드 dry-run

권장 실행기는 카메라와 LiDAR 장치 경로, 모델 해시, 기존 컨테이너 충돌을 먼저
검사하고 actuator bridge를 만들지 않는다.

```bash
./scripts/run_competition.sh --sensor-mode-dry-run
```

동일 launch를 직접 실행해야 할 때만 아래 명령을 사용한다.

```bash
cd /home/sandi/KMU_AutoDriving
source /opt/ros/humble/setup.bash
source .colcon/install/setup.bash
set -a; source .env; set +a

ros2 launch kmu_track sensor_mode_drive.launch.py \
  camera_device:="$KMU_CAMERA_DEVICE" \
  model_path:="$KMU_MODEL_PATH" \
  lidar_port:="$KMU_LIDAR_DEVICE" \
  dry_run:=true \
  serial_bridge:=false
```

## 실차 출력

아래 명령은 조향 방향·조향각·스로틀 한계와 라이다 TF를 실측한 뒤에만 사용한다.

```bash
ros2 launch kmu_track sensor_mode_drive.launch.py \
  camera_device:="$KMU_CAMERA_DEVICE" \
  model_path:="$KMU_MODEL_PATH" \
  lidar_port:="$KMU_LIDAR_DEVICE" \
  arduino_port:="$KMU_SERIAL_DEVICE" \
  dry_run:=false \
  hardware_confirmed:=true \
  steering_only:=false \
  cone_geometry_confirmed:=true \
  serial_bridge:=true \
  throttle_max:=300
```

별도 터미널에서 기존 operator arm/deadman 토픽을 유지해야 실제 출력이 허용된다.

```bash
source /opt/ros/humble/setup.bash
source /home/sandi/KMU_AutoDriving/.colcon/install/setup.bash
ros2 topic pub -r 10 /rc_car/operator_armed \
  std_msgs/msg/Bool '{data: true}' &
ros2 topic pub -r 10 /rc_car/operator_deadman \
  std_msgs/msg/Bool '{data: true}'
```

즉시 정지:

```bash
ros2 topic pub --once /vehicle/estop std_msgs/msg/Bool '{data: true}'
```

상태 확인:

```bash
ros2 topic echo /mission/state
ros2 topic echo /mission/sensor_mode_status
ros2 topic echo /vehicle/command_mux_status
```

## FSM 라바콘 모드 전환

상태 토픽을 직접 덮어쓰지 않고 FSM의 합법 전이만 요청한다. 라바콘 모드 진입
요청은 먼저 `CONE_INIT`으로 이동해 출력을 중립으로 유지한다. 카메라 구독이
종료되고 LiDAR 구독이 활성화된 뒤 유효한 `/perception/cone_path`가 들어와야
FSM이 `CONE_SLALOM`으로 넘어가고 cone 명령이 mux를 통과한다.

```bash
ros2 service call /sensor_mode_manager/set_cone_mode \
  std_srvs/srv/SetBool '{data: true}'
```

라바콘 모드를 취소하거나 종료하면 `LANE_REACQUIRE`로 이동한다. LiDAR 구독을
내리고 카메라 구독을 되살린 뒤, 설정된 횟수만큼 연속으로 유효한 차선을
확인해야 `LANE_FOLLOW`로 복귀한다.

```bash
ros2 service call /sensor_mode_manager/set_cone_mode \
  std_srvs/srv/SetBool '{data: false}'
```

`SAFE_STOP`에서는 이 서비스로 복구하지 않는다. 원인을 제거하고 전체 런치를
재시작해야 한다.

현재 `automatic_cone_transition_enabled`는 `false`다. 실장치 드라이런에서
일반 주황색 영역이 2.08% 면적의 라바콘으로 오인된 사례가 있어, 실제 코스에서
카메라 임계값을 다시 측정하기 전까지는 서비스 전환만 사용한다.

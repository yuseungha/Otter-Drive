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

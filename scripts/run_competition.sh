#!/usr/bin/env bash
set -Eeuo pipefail

project_root=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd -P)
install_setup=${project_root}/.colcon/install/setup.bash
mode=dry-run
video_path=''

usage() {
  cat <<'EOF'
Usage:
  ./scripts/run_competition.sh --check
  ./scripts/run_competition.sh --dry-run
  ./scripts/run_competition.sh --cone-dry-run
  ./scripts/run_competition.sh --sensor-mode-dry-run
  ./scripts/run_competition.sh --sensor-cone-live
  ./scripts/run_competition.sh --unified-dry-run
  ./scripts/run_competition.sh --unified-live
  ./scripts/run_competition.sh --video /absolute/path/to/video.mp4
  ./scripts/run_competition.sh --live

--dry-run is the default and never starts the serial bridge.
--cone-dry-run starts RPLIDAR perception only and never starts an actuator bridge.
--sensor-mode-dry-run starts camera, RPLIDAR, FSM, both controllers, and the
command mux in preview-only mode; it never starts an actuator bridge.
--sensor-cone-live starts the FSM in CONE_INIT and connects the confirmed
Arduino bridge with the same limits used by lane driving.
--unified-dry-run runs the IRE lane controller with obstacle/cone planning and
publishes only /rc_car/drive_cmd_preview.
--unified-live connects the IRE-based integrated stack to the Arduino bridge.
--live requires KMU_HARDWARE_CONFIRMED=true and a real serial device.
EOF
}

while (($#)); do
  case "$1" in
    --check) mode=check ;;
    --dry-run) mode=dry-run ;;
    --cone-dry-run) mode=cone-dry-run ;;
    --sensor-mode-dry-run) mode=sensor-mode-dry-run ;;
    --sensor-cone-live) mode=sensor-cone-live ;;
    --unified-dry-run) mode=unified-dry-run ;;
    --unified-live) mode=unified-live ;;
    --live) mode=live ;;
    --video)
      [[ $# -ge 2 ]] || { echo 'ERROR: --video requires a path.' >&2; exit 2; }
      mode=video
      video_path=$2
      shift
      ;;
    -h|--help) usage; exit 0 ;;
    *) echo "ERROR: unknown argument: $1" >&2; usage >&2; exit 2 ;;
  esac
  shift
done

if [[ -r ${project_root}/.env ]]; then
  set -a
  # shellcheck disable=SC1091
  source "${project_root}/.env"
  set +a
fi

image=${KMU_CONTAINER_IMAGE:-sandikookmin:cuda126}
model=${KMU_MODEL_PATH:-${project_root}/models/road_best.pt}
model_sha=${KMU_MODEL_SHA256:-b54bb33713d753ac7860ebad33c2f166ce9230f63fdf5c30a0528bac45ea779c}
seg_model=${KMU_SEG_MODEL_PATH:-${project_root}/models/last_3x.pt}
seg_model_sha=${KMU_SEG_MODEL_SHA256:-9d2797b3513e633ac944f55ac15b75344d26a9d1751f5c555df175cb0bd548d0}
if [[ ${mode} == unified-dry-run || ${mode} == unified-live ]]; then
  model=${seg_model}
  model_sha=${seg_model_sha}
fi
camera_requested=${KMU_CAMERA_DEVICE:-/dev/v4l/by-id/usb-046d_Logitech_BRIO_5FD2713E-video-index0}
camera=${camera_requested}
lidar_requested=${KMU_LIDAR_DEVICE:-/dev/serial/by-id/usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0001-if00-port0}
lidar=${lidar_requested}
serial_requested=${KMU_SERIAL_DEVICE:-/dev/serial/by-id/REPLACE_WITH_ARDUINO_DEVICE}
serial=${serial_requested}
domain_id=${KMU_ROS_DOMAIN_ID:-86}
display=${KMU_DISPLAY:-false}
steering_only=${KMU_STEERING_ONLY:-true}
initial_mode=${KMU_INITIAL_MODE:-LANE_FOLLOW}
confirmed=${KMU_HARDWARE_CONFIRMED:-false}
container_name=kmu-autodriving-runtime

fail() { echo "ERROR: $*" >&2; exit 1; }
command -v docker >/dev/null || fail 'docker is missing'
docker image inspect "${image}" >/dev/null 2>&1 || fail "container image is missing: ${image}"
[[ -r ${model} ]] || fail "model is missing: ${model}"
[[ -r ${install_setup} ]] || fail 'workspace is not built; run ./scripts/setup_jetson.sh'

actual_sha=$(sha256sum "${model}" | awk '{print $1}')
[[ ${actual_sha} == "${model_sha}" ]] || fail "model SHA-256 mismatch: ${actual_sha}"

if [[ ${mode} == dry-run || ${mode} == live || ${mode} == sensor-mode-dry-run || ${mode} == sensor-cone-live || ${mode} == unified-dry-run || ${mode} == unified-live ]]; then
  [[ -e ${camera_requested} ]] || fail "camera device is missing: ${camera_requested}"
  camera=$(readlink -f -- "${camera_requested}")
  [[ ${camera} == /dev/video* ]] || fail "camera did not resolve to /dev/video*: ${camera}"
fi
if [[ ${mode} == video ]]; then
  [[ -r ${video_path} ]] || fail "video is missing or unreadable: ${video_path}"
fi
if [[ ${mode} == cone-dry-run || ${mode} == sensor-mode-dry-run || ${mode} == sensor-cone-live || ${mode} == unified-dry-run || ${mode} == unified-live ]]; then
  [[ -e ${lidar_requested} ]] || fail "LiDAR device is missing: ${lidar_requested}"
  lidar=$(readlink -f -- "${lidar_requested}")
  [[ ${lidar} == /dev/ttyUSB* || ${lidar} == /dev/ttyACM* ]] || \
    fail "LiDAR did not resolve to /dev/ttyUSB* or /dev/ttyACM*: ${lidar}"
fi
if [[ ${mode} == live || ${mode} == sensor-cone-live || ${mode} == unified-live ]]; then
  [[ ${confirmed} == true ]] || fail 'set KMU_HARDWARE_CONFIRMED=true only after the hardware runbook'
  [[ -e ${serial_requested} ]] || fail "serial device is missing: ${serial_requested}"
  [[ ${serial_requested} == /dev/serial/by-id/* ]] || \
    fail "Arduino must use a stable /dev/serial/by-id path: ${serial_requested}"
  serial_device=$(readlink -f -- "${serial_requested}")
  [[ ${serial_device} == /dev/ttyUSB* || ${serial_device} == /dev/ttyACM* ]] || \
    fail "Arduino did not resolve to /dev/ttyUSB* or /dev/ttyACM*: ${serial_device}"
  if [[ ${mode} != live && ${serial_device} == "${lidar}" ]]; then
    fail "Arduino and LiDAR resolved to the same device: ${serial_device}"
  fi
  if fuser "${serial_device}" >/dev/null 2>&1; then
    fail "serial device is already in use: ${serial_device}"
  fi
  docker run --rm "${image}" python3 -c 'import serial' >/dev/null 2>&1 || \
    fail 'container image is missing PySerial; build Dockerfile.jetson first'
fi
if docker ps -a --format '{{.Names}}' | grep -Fxq "${container_name}"; then
  fail "container name is already in use: ${container_name}"
fi

docker_base=(
  docker run --rm --name "${container_name}" --init
  --runtime=nvidia --privileged --network host --ipc host
  -e NVIDIA_VISIBLE_DEVICES=all
  -e ROS_DOMAIN_ID="${domain_id}"
  -e KMU_PROJECT_ROOT="${project_root}"
  -e KMU_MODEL_PATH="${model}"
  -e KMU_SEG_MODEL_PATH="${seg_model}"
  -e KMU_VIDEO_PATH="${video_path}"
  -e YOLO_CONFIG_DIR=/tmp/kmu-yolo
  -e HOME=/tmp/kmu-home
  -v "${project_root}:${project_root}"
  -v /dev/serial/by-id:/dev/serial/by-id:ro
  -w "${project_root}"
)

if [[ ${display} == true ]]; then
  x11_display=${DISPLAY:-:0}
  x11_authority=${XAUTHORITY:-/run/user/$(id -u)/gdm/Xauthority}
  x11_number=${x11_display#:}
  x11_number=${x11_number%%.*}
  [[ -S /tmp/.X11-unix/X${x11_number} ]] || \
    fail "X11 display socket is missing for DISPLAY=${x11_display}"
  [[ -r ${x11_authority} ]] || \
    fail "X11 authority file is missing or unreadable: ${x11_authority}"
  docker_base+=(
    -e DISPLAY="${x11_display}"
    -e XAUTHORITY=/tmp/kmu-xauthority
    -e QT_X11_NO_MITSHM=1
    -v /tmp/.X11-unix:/tmp/.X11-unix:rw
    -v "${x11_authority}:/tmp/kmu-xauthority:ro"
  )
fi

if [[ ${mode} == check ]]; then
  "${docker_base[@]}" "${image}" bash -lc '
    mkdir -p "$HOME" "$YOLO_CONFIG_DIR"
    source /opt/ros/humble/setup.bash
    source "$KMU_PROJECT_ROOT/.colcon/install/setup.bash"
    python3 - <<PY
import os
import torch
from ultralytics import YOLO

assert torch.cuda.is_available()
model = YOLO(os.environ["KMU_MODEL_PATH"])
assert model.task == "detect"
assert set(model.names.values()) == {"lane1", "lane2"}
print("CUDA:", torch.cuda.get_device_name(0))
print("MODEL:", model.task, model.names)
PY
    ros2 pkg prefix kmu_track
    ros2 pkg prefix rc_car_teleop
  '
  echo 'PREFLIGHT=OK'
  exit 0
fi

if [[ ${mode} == cone-dry-run ]]; then
  launch_command=(ros2 launch lidar_cone_planner cone_lidar_cv.launch.py
    serial_port:="${lidar}"
    planner_config:="${project_root}/configs/cone/cone_planner.yaml"
    system_config:="${project_root}/configs/cone/cone_lidar_cv.yaml"
    planning_frame:=base_link
    viewer_enabled:=false
    viewer_record_path:=/tmp/kmu-cone-viewer-latest.png)
elif [[ ${mode} == unified-dry-run || ${mode} == unified-live ]]; then
  launch_command=(ros2 launch kmu_ire_track ire_unified_autonomy.launch.py
    camera_device:="${camera}"
    model_path:="${model}"
    lidar_port:="${lidar}"
    arduino_port:="${serial}"
    camera_config:="${project_root}/src/jetson/kmu_ire_track/config/ire_camera.yaml"
    segmentation_config:="${project_root}/src/jetson/kmu_ire_track/config/ire_segmentation_lane.yaml"
    control_config:="${project_root}/src/jetson/kmu_track/config/lane_control.yaml"
    autonomy_config:="${project_root}/src/jetson/kmu_track/config/unified_autonomy.yaml"
    cone_planner_config:="${project_root}/configs/cone/cone_planner.yaml"
    lidar_system_config:="${project_root}/configs/cone/cone_lidar_cv.yaml"
    viewer:="${display}")
  if [[ ${mode} == unified-live ]]; then
    launch_command+=(output_topic:=/rc_car/drive_cmd serial_bridge:=true
      dry_run:=false hardware_confirmed:=true)
  else
    launch_command+=(output_topic:=/rc_car/drive_cmd_preview
      serial_bridge:=false dry_run:=true hardware_confirmed:=false)
  fi
elif [[ ${mode} == sensor-mode-dry-run || ${mode} == sensor-cone-live ]]; then
  launch_command=(ros2 launch kmu_track sensor_mode_drive.launch.py
    camera_device:="${camera}"
    model_path:="${model}"
    lidar_port:="${lidar}"
    camera_config:="${project_root}/configs/camera.yaml"
    perception_config:="${project_root}/configs/perception.yaml"
    lane_control_config:="${project_root}/configs/lane_control.yaml"
    sensor_mode_config:="${project_root}/src/jetson/kmu_track/config/sensor_mode.yaml"
    cone_planner_config:="${project_root}/configs/cone/cone_planner.yaml"
    cone_controller_config:="${project_root}/src/jetson/lidar_cone_planner/config/cone_controller.yaml"
    lidar_system_config:="${project_root}/configs/cone/cone_lidar_cv.yaml"
    adapter_config:="${project_root}/src/jetson/rc_car_teleop/config/autonomous_drive.yaml"
    require_odometry:=false throttle_max:=550
    steering_min:=-900 steering_max:=900)
  if [[ ${mode} == sensor-cone-live ]]; then
    launch_command+=(dry_run:=false hardware_confirmed:=true
      steering_only:="${steering_only}" initial_mode:=CONE_INIT
      serial_bridge:=false arduino_port:="${serial}"
      cone_geometry_confirmed:=true lane_stack_enabled:=false)
  else
    launch_command+=(dry_run:=true hardware_confirmed:=false
      steering_only:=true initial_mode:="${initial_mode}"
      serial_bridge:=false cone_geometry_confirmed:=false)
  fi
elif [[ ${mode} == video ]]; then
  launch_command=(ros2 launch kmu_track lane_drive_video.launch.py
    perception_config:="${project_root}/configs/perception.yaml"
    control_config:="${project_root}/configs/lane_control.yaml"
    video_config:="${project_root}/configs/video.yaml"
    video_path:="${video_path}"
    model_path:="${model}"
    display:="${display}"
    enabled:=true dry_run:=true hardware_confirmed:=false
    steering_only:=true serial_bridge:=false loop:=false)
else
  launch_command=(ros2 launch kmu_track lane_drive_live.launch.py
    camera_config:="${project_root}/configs/camera.yaml"
    perception_config:="${project_root}/configs/perception.yaml"
    control_config:="${project_root}/configs/lane_control.yaml"
    video_config:="${project_root}/configs/video.yaml"
    camera_device:="${camera}"
    model_path:="${model}"
    display:="${display}"
    enabled:=true
    steering_only:="${steering_only}")
  if [[ ${mode} == live ]]; then
    launch_command+=(dry_run:=false hardware_confirmed:=true serial_bridge:=true serial_port:="${serial}")
  else
    launch_command+=(dry_run:=true hardware_confirmed:=false serial_bridge:=false)
  fi
fi

timestamp=$(date '+%Y%m%d-%H%M%S')
log_dir=${project_root}/logs/${timestamp}
mkdir -p "${log_dir}"

cleanup() {
  if docker ps --format '{{.Names}}' | grep -Fxq "${container_name}"; then
    docker stop --time 5 "${container_name}" >/dev/null 2>&1 || true
  fi
}
trap cleanup EXIT INT TERM

echo "MODE=${mode}"
echo "LOG=${log_dir}/competition.log"
"${docker_base[@]}" "${image}" bash -lc '
  mkdir -p "$HOME" "$YOLO_CONFIG_DIR"
  source /opt/ros/humble/setup.bash
  source "$KMU_PROJECT_ROOT/.colcon/install/setup.bash"
  exec "$@"
' bash "${launch_command[@]}" 2>&1 | tee "${log_dir}/competition.log"

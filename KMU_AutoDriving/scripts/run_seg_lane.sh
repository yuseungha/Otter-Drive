#!/usr/bin/env bash
set -Eeuo pipefail

project_root=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd -P)
install_setup=${project_root}/.colcon/install/setup.bash
mode=check
video_path=''
serial_bridge_pid=''

usage() {
  cat <<'EOF'
Usage:
  ./scripts/run_seg_lane.sh --check
  ./scripts/run_seg_lane.sh --video /absolute/path/to/video.mp4
  ./scripts/run_seg_lane.sh --video-live /absolute/path/to/video.mp4
  ./scripts/run_seg_lane.sh --camera

--video and --camera publish preview commands only.
--video-live requires KMU_HARDWARE_CONFIRMED=true and uses the verified
Arduino limits: throttle 0..700 and steering -650..650.
EOF
}

while (($#)); do
  case "$1" in
    --check) mode=check ;;
    --camera) mode=camera ;;
    --video)
      [[ $# -ge 2 ]] || {
        echo 'ERROR: --video requires a path.' >&2
        exit 2
      }
      mode=video
      video_path=$2
      shift
      ;;
    --video-live)
      [[ $# -ge 2 ]] || {
        echo 'ERROR: --video-live requires a path.' >&2
        exit 2
      }
      mode=video-live
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
model=${KMU_SEG_MODEL_PATH:-${project_root}/models/center_lane_best.pt}
model_sha=${KMU_SEG_MODEL_SHA256:-89427ee98034e0fdcee772b194923d33f532f07aa825905ec10a205687d71996}
camera_requested=${KMU_CAMERA_DEVICE:-/dev/v4l/by-id/usb-046d_Logitech_BRIO_5FD2713E-video-index0}
display=${KMU_DISPLAY:-false}
domain_id=${KMU_ROS_DOMAIN_ID:-86}
serial=${KMU_SERIAL_DEVICE:-/dev/serial/by-id/REPLACE_WITH_ARDUINO_DEVICE}
confirmed=${KMU_HARDWARE_CONFIRMED:-false}
container_name=kmu-seg-lane-runtime

fail() { echo "ERROR: $*" >&2; exit 1; }
command -v docker >/dev/null || fail 'docker is missing'
docker image inspect "${image}" >/dev/null 2>&1 || \
  fail "container image is missing: ${image}"
[[ -r ${model} ]] || fail "segmentation model is missing: ${model}"
[[ -r ${install_setup} ]] || \
  fail 'workspace is not built; run ./scripts/setup_jetson.sh'
actual_sha=$(sha256sum "${model}" | awk '{print $1}')
[[ ${actual_sha} == "${model_sha}" ]] || \
  fail "model SHA-256 mismatch: ${actual_sha}"

if [[ ${mode} == video || ${mode} == video-live ]]; then
  [[ -r ${video_path} ]] || fail "video is missing or unreadable: ${video_path}"
fi
if [[ ${mode} == video-live ]]; then
  [[ ${confirmed} == true ]] || \
    fail 'set KMU_HARDWARE_CONFIRMED=true only after the airborne hardware check'
  [[ -e ${serial} ]] || fail "serial device is missing: ${serial}"
  if fuser "${serial}" >/dev/null 2>&1; then
    fail "serial device is already in use: ${serial}"
  fi
fi
if [[ ${mode} == camera ]]; then
  [[ -e ${camera_requested} ]] || \
    fail "camera device is missing: ${camera_requested}"
  camera=$(readlink -f -- "${camera_requested}")
  [[ ${camera} == /dev/video* ]] || \
    fail "camera did not resolve to /dev/video*: ${camera}"
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
  -e KMU_SEG_MODEL_PATH="${model}"
  -e YOLO_CONFIG_DIR=/tmp/kmu-yolo
  -e HOME=/tmp/kmu-home
  -v "${project_root}:${project_root}"
  -w "${project_root}"
)
if [[ ${display} == true ]]; then
  [[ -n ${DISPLAY:-} ]] || fail 'KMU_DISPLAY=true but DISPLAY is unset'
  [[ -d /tmp/.X11-unix ]] || fail 'X11 socket directory is missing'
  docker_base+=(
    -e DISPLAY="${DISPLAY}"
    -v /tmp/.X11-unix:/tmp/.X11-unix:rw
  )
  if [[ -n ${XAUTHORITY:-} && -r ${XAUTHORITY} ]]; then
    docker_base+=(
      -e XAUTHORITY="${XAUTHORITY}"
      -v "${XAUTHORITY}:${XAUTHORITY}:ro"
    )
  fi
fi
if [[ ${mode} == video || ${mode} == video-live ]]; then
  docker_base+=(-v "${video_path}:${video_path}:ro")
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
model = YOLO(os.environ["KMU_SEG_MODEL_PATH"])
assert model.task == "segment"
assert set(model.names.values()) == {"center", "lane"}
print("CUDA:", torch.cuda.get_device_name(0))
print("MODEL:", model.task, model.names)
PY
    ros2 pkg prefix kmu_track
  '
  echo 'SEG_LANE_PREFLIGHT=OK'
  exit 0
fi

if [[ ${mode} == video || ${mode} == video-live ]]; then
  launch_command=(ros2 launch kmu_track seg_lane_drive_video.launch.py
    video_path:="${video_path}"
    model_path:="${model}"
    segmentation_config:="${project_root}/configs/segmentation_lane.yaml"
    control_config:="${project_root}/configs/lane_control.yaml"
    video_config:="${project_root}/configs/video.yaml"
    display:="${display}"
    loop:=false)
  if [[ ${mode} == video-live ]]; then
    launch_command+=(
      enabled:=true dry_run:=false hardware_confirmed:=true
      steering_only:=false manage_serial_gate:=true)
  else
    launch_command+=(
      enabled:=true dry_run:=true hardware_confirmed:=false
      steering_only:=true manage_serial_gate:=false)
  fi
else
  launch_command=(ros2 launch kmu_track seg_lane_camera_dry_run.launch.py
    camera_device:="${camera}"
    model_path:="${model}"
    segmentation_config:="${project_root}/configs/segmentation_lane.yaml"
    control_config:="${project_root}/configs/lane_control.yaml"
    video_config:="${project_root}/configs/video.yaml"
    display:="${display}")
fi

timestamp=$(date '+%Y%m%d-%H%M%S')
log_dir=${project_root}/logs/${timestamp}-seg-lane
mkdir -p "${log_dir}"

cleanup() {
  if docker ps --format '{{.Names}}' | grep -Fxq "${container_name}"; then
    docker stop --time 5 "${container_name}" >/dev/null 2>&1 || true
  fi
  if [[ -n ${serial_bridge_pid} ]] && kill -0 "${serial_bridge_pid}" 2>/dev/null; then
    kill -INT "${serial_bridge_pid}" 2>/dev/null || true
    wait "${serial_bridge_pid}" 2>/dev/null || true
  fi
}
trap cleanup EXIT INT TERM

if [[ ${mode} == video-live ]]; then
  bridge_log=${log_dir}/serial-bridge.log
  (
    export ROS_DOMAIN_ID="${domain_id}"
    export ROS_LOCALHOST_ONLY=0
    set +u
    # shellcheck disable=SC1091
    source /opt/ros/humble/setup.bash
    # shellcheck disable=SC1090
    source "${install_setup}"
    set -u
    exec ros2 run rc_car_teleop serial_bridge --ros-args \
      -p serial_port:="${serial}" \
      -p drive_enabled:=true \
      -p limits_confirmed:=true \
      -p throttle_min:=0 \
      -p throttle_max:=700 \
      -p steering_min:=-650 \
      -p steering_max:=650
  ) >"${bridge_log}" 2>&1 &
  serial_bridge_pid=$!
  sleep 1
  kill -0 "${serial_bridge_pid}" 2>/dev/null || \
    fail "serial bridge exited during startup; see ${bridge_log}"
  echo "BRIDGE_LOG=${bridge_log}"
fi

echo "MODE=${mode}"
echo "LOG=${log_dir}/seg-lane.log"
"${docker_base[@]}" "${image}" bash -lc '
  mkdir -p "$HOME" "$YOLO_CONFIG_DIR"
  source /opt/ros/humble/setup.bash
  source "$KMU_PROJECT_ROOT/.colcon/install/setup.bash"
  exec "$@"
' bash "${launch_command[@]}" 2>&1 | tee "${log_dir}/seg-lane.log"

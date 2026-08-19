#!/usr/bin/env bash
set -eo pipefail

workspace=${WORKSPACE:-/workspace/test_ws/codex_actuation_validation_ws}
model=${MODEL_PATH:-${workspace}/models/road_best.pt}
camera=${CAMERA_DEVICE:-/dev/video0}
log=${workspace}/jetson_live_camera_dry_run.log
export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-0}

source /opt/ros/humble/setup.bash
source "${workspace}/install/setup.bash"
set -u

test -r "${model}"
test -e "${camera}"

launch_pid=''
cleanup() {
  if [[ -n "${launch_pid}" ]] && kill -0 "${launch_pid}" 2>/dev/null; then
    kill -TERM -- "-${launch_pid}" 2>/dev/null || true
    wait "${launch_pid}" 2>/dev/null || true
  fi
}
trap cleanup EXIT INT TERM

setsid ros2 launch kmu_track lane_drive_live.launch.py \
  camera_device:="${camera}" \
  model_path:="${model}" \
  display:=false \
  enabled:=true \
  dry_run:=true \
  hardware_confirmed:=false \
  steering_only:=true \
  serial_bridge:=false >"${log}" 2>&1 &
launch_pid=$!

topic_found=false
for _ in $(seq 1 60); do
  if ros2 topic list 2>/dev/null | grep -Fxq '/lane/yolo_detections'; then
    topic_found=true
    break
  fi
  if ! kill -0 "${launch_pid}" 2>/dev/null; then
    break
  fi
done

ready=false
if [[ "${topic_found}" == true ]] && python3 \
    "${workspace}/src/kmu_track/test/video_chain_probe.py" \
    >/tmp/codex_live_camera_probe.txt 2>&1; then
  ready=true
fi

echo "LIVE_CAMERA_PIPELINE_READY=${ready}"
if [[ -f /tmp/codex_live_camera_probe.txt ]]; then
  cat /tmp/codex_live_camera_probe.txt
fi
echo 'CAMERA_RATE'
timeout 8 ros2 topic hz /camera/front/image_raw || true
echo 'INFERENCE_RATE'
timeout 8 ros2 topic hz /lane/yolo_detections || true
echo 'SERIAL_OWNER'
fuser /dev/ttyACM0 2>/dev/null || true
echo 'LAUNCH_LOG_TAIL'
tail -100 "${log}"

if [[ "${ready}" != true ]]; then
  exit 1
fi

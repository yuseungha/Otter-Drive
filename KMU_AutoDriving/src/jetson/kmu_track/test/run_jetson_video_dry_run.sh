#!/usr/bin/env bash
set -eo pipefail

# All paths are injected. Defaults follow the KMU_AutoDriving layout.
workspace=${WORKSPACE:-/home/sandi/KMU_AutoDriving}
video=${KMU_VIDEO_PATH:-${VIDEO_PATH:-}}
model=${KMU_MODEL_PATH:-${MODEL_PATH:-${workspace}/models/road_best.pt}}
log=${workspace}/jetson_video_dry_run.log
export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-86}

source /opt/ros/humble/setup.bash
source "${workspace}/install/setup.bash"
set -u

test -r "${video}"
test -r "${model}"

launch_pid=''
cleanup() {
  if [[ -n "${launch_pid}" ]] && kill -0 "${launch_pid}" 2>/dev/null; then
    kill -TERM -- "-${launch_pid}" 2>/dev/null || true
    wait "${launch_pid}" 2>/dev/null || true
  fi
}
trap cleanup EXIT INT TERM

setsid ros2 launch kmu_track lane_drive_video.launch.py \
  video_path:="${video}" \
  model_path:="${model}" \
  display:=false \
  enabled:=true \
  dry_run:=true \
  hardware_confirmed:=false \
  steering_only:=true \
  serial_bridge:=false \
  loop:=true >"${log}" 2>&1 &
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
    >/tmp/codex_video_probe.txt 2>&1; then
  ready=true
fi

echo "VIDEO_PIPELINE_READY=${ready}"
if [[ "${ready}" != true ]]; then
  tail -120 "${log}"
  exit 1
fi

echo 'PIPELINE_METRICS'
cat /tmp/codex_video_probe.txt
echo 'CAMERA_RATE'
timeout 8 ros2 topic hz /camera/front/image_raw || true
echo 'INFERENCE_RATE'
timeout 8 ros2 topic hz /lane/yolo_detections || true
echo 'SERIAL_OWNER'
fuser /dev/ttyACM0 2>/dev/null || true
echo 'LAUNCH_LOG_TAIL'
tail -80 "${log}"

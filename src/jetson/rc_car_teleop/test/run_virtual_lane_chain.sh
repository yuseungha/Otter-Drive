#!/usr/bin/env bash
set -eo pipefail

workspace="$(cd "$(dirname "${BASH_SOURCE[0]}")/../../.." && pwd)"
cd "$workspace"
source /opt/ros/humble/setup.bash
source install/setup.bash
set -u

fake_log="$(mktemp)"
launch_log="$(mktemp)"
launch_pid=''
python3 src/rc_car_teleop/test/fake_arduino.py >"$fake_log" 2>&1 &
fake_pid=$!
cleanup() {
  if [[ -n "$launch_pid" ]]; then
    kill -INT "$launch_pid" 2>/dev/null || true
  fi
  kill "$fake_pid" 2>/dev/null || true
  wait "$fake_pid" 2>/dev/null || true
  printf 'FAKE_LOG=%s\nLAUNCH_LOG=%s\n' "$fake_log" "$launch_log"
}
trap cleanup EXIT

for _ in $(seq 1 100); do
  if [[ -s "$fake_log" ]]; then
    break
  fi
  sleep 0.05
done
serial_port="$(head -n 1 "$fake_log")"
if [[ "$serial_port" != /dev/pts/* ]]; then
  echo "fake Arduino did not provide a PTY" >&2
  exit 1
fi
echo "VIRTUAL_SERIAL_PORT=$serial_port"

timeout --signal=INT --kill-after=3s 30s \
  ros2 launch kmu_track lane_drive_video.launch.py \
  enabled:=true dry_run:=false hardware_confirmed:=true \
  steering_only:=true serial_bridge:=true serial_port:="$serial_port" \
  display:=false >"$launch_log" 2>&1 &
launch_pid=$!
sleep 14
echo '--- TX STATS ---'
timeout 5s ros2 topic echo --once --full-length /rc_car/tx_stats || true
echo '--- FEEDBACK ---'
timeout 5s ros2 topic echo --once /rc_car/feedback || true
echo '--- E-STOP STATS ---'
ros2 topic pub --once /vehicle/estop std_msgs/msg/Bool '{data: true}'
sleep 2
timeout 5s ros2 topic echo --once --full-length /rc_car/tx_stats || true
wait "$launch_pid" 2>/dev/null || true

grep -E 'Serial connected|Lane controller ready|serial_arming|Video ready' \
  "$launch_log" || true

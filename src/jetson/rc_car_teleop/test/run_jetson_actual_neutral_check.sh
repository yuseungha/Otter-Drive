#!/usr/bin/env bash
set -eo pipefail

workspace="${1:-/home/sandi/codex_actuation_validation_ws}"
serial_port="${2:-/dev/ttyACM0}"
cd "$workspace"
source /opt/ros/humble/setup.bash
source install/setup.bash

[[ -c "$serial_port" ]]
if fuser "$serial_port" >/dev/null 2>&1; then
  echo "serial port is already in use: $serial_port" >&2
  exit 1
fi

bridge_log="$(mktemp)"
bridge_pid=''
publisher_pid=''
cleanup() {
  if [[ -n "$publisher_pid" ]]; then
    kill -TERM -- "-$publisher_pid" 2>/dev/null || true
  fi
  if [[ -n "$bridge_pid" ]]; then
    kill -INT -- "-$bridge_pid" 2>/dev/null || true
    sleep 0.5
    kill -TERM -- "-$bridge_pid" 2>/dev/null || true
  fi
  echo "BRIDGE_LOG=$bridge_log"
}
trap cleanup EXIT

setsid ros2 run rc_car_teleop serial_bridge --ros-args \
  -p serial_port:="$serial_port" >"$bridge_log" 2>&1 &
bridge_pid=$!
sleep 0.8
setsid timeout 10s ros2 topic pub -r 20 /rc_car/drive_cmd \
  std_msgs/msg/Int32MultiArray '{data: [0, 0, -1]}' >/dev/null 2>&1 &
publisher_pid=$!

sleep 4.2
echo '--- SERIAL READY ---'
timeout 4s ros2 topic echo /rc_car/serial_ready --once
echo '--- FEEDBACK ---'
timeout 4s ros2 topic echo /rc_car/feedback --once
echo '--- STEERING ADC ---'
timeout 4s ros2 topic echo /rc_car/steering_adc --once
echo '--- TX STATS ---'
timeout 4s ros2 topic echo /rc_car/tx_stats --once || true
grep -E 'Serial connected|write failed|disconnected' "$bridge_log" || true

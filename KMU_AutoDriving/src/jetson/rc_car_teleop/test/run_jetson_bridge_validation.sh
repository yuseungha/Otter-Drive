#!/usr/bin/env bash
set -eo pipefail

workspace="${1:-/home/sandi/codex_actuation_validation_ws}"
cd "$workspace"
source /opt/ros/humble/setup.bash
source install/setup.bash

fake_log="$(mktemp)"
bridge_log="$(mktemp)"
fake_pid=''
bridge_pid=''
publisher_pid=''
cleanup() {
  if [[ -n "$publisher_pid" ]]; then
    kill -TERM -- "-$publisher_pid" 2>/dev/null || true
  fi
  if [[ -n "$bridge_pid" ]]; then
    kill -TERM -- "-$bridge_pid" 2>/dev/null || true
  fi
  if [[ -n "$fake_pid" ]]; then
    kill -TERM "$fake_pid" 2>/dev/null || true
  fi
  echo "FAKE_LOG=$fake_log"
  echo "BRIDGE_LOG=$bridge_log"
}
trap cleanup EXIT

python3 src/rc_car_teleop/test/fake_arduino.py >"$fake_log" 2>&1 &
fake_pid=$!
for _ in $(seq 1 100); do
  [[ -s "$fake_log" ]] && break
  sleep 0.05
done
serial_port="$(head -n 1 "$fake_log")"
[[ "$serial_port" == /dev/pts/* ]]
echo "VIRTUAL_SERIAL_PORT=$serial_port"

setsid ros2 run rc_car_teleop serial_bridge --ros-args \
  -p serial_port:="$serial_port" >"$bridge_log" 2>&1 &
bridge_pid=$!
sleep 0.8
setsid timeout 14s ros2 topic pub -r 20 /rc_car/drive_cmd \
  std_msgs/msg/Int32MultiArray '{data: [0, 0, -1]}' >/dev/null 2>&1 &
publisher_pid=$!

sleep 4.2
echo '--- SERIAL READY ---'
timeout 4s ros2 topic echo /rc_car/serial_ready --once
echo '--- NORMAL TX ---'
timeout 4s ros2 topic echo /rc_car/tx_stats --once || true

ros2 topic pub --once /vehicle/estop std_msgs/msg/Bool '{data: true}'
sleep 1.5
echo '--- E-STOP LATCH ---'
timeout 4s ros2 topic echo /vehicle/estop_latched --once
echo '--- E-STOP TX ---'
timeout 4s ros2 topic echo /rc_car/tx_stats --once || true

grep -E 'Serial connected|E-stop latched|write failed' "$bridge_log" || true

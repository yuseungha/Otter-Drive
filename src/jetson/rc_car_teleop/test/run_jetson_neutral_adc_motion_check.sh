#!/usr/bin/env bash
set -eo pipefail

workspace=${1:-/home/sandi/codex_actuation_validation_ws}
serial_port=${2:-/dev/ttyACM0}
sample_file=$(mktemp)
bridge_log=$(mktemp)
bridge_pid=''
publisher_pid=''

cd "${workspace}"
source /opt/ros/humble/setup.bash
source install/setup.bash

test -c "${serial_port}"
if fuser "${serial_port}" >/dev/null 2>&1; then
  echo "serial port is already in use: ${serial_port}" >&2
  exit 1
fi

cleanup() {
  if [[ -n "${publisher_pid}" ]]; then
    kill -TERM -- "-${publisher_pid}" 2>/dev/null || true
  fi
  if [[ -n "${bridge_pid}" ]]; then
    kill -TERM -- "-${bridge_pid}" 2>/dev/null || true
  fi
}
trap cleanup EXIT INT TERM

setsid ros2 run rc_car_teleop serial_bridge --ros-args \
  -p serial_port:="${serial_port}" >"${bridge_log}" 2>&1 &
bridge_pid=$!
sleep 0.8
setsid timeout 24s ros2 topic pub -r 20 /rc_car/drive_cmd \
  std_msgs/msg/Int32MultiArray '{data: [0, 0, -1]}' \
  >/dev/null 2>&1 &
publisher_pid=$!

sleep 4.2
timeout 15s ros2 topic echo /rc_car/steering_adc --field data \
  >"${sample_file}" 2>/dev/null || true

awk '
  /^[0-9]+$/ {
    value = $1
    if (count == 0 || value < minimum) minimum = value
    if (count == 0 || value > maximum) maximum = value
    count++
  }
  END {
    printf "ADC_SAMPLE_COUNT=%d\n", count
    if (count > 0) {
      printf "ADC_MIN=%d\nADC_MAX=%d\nADC_SPAN=%d\n", \
        minimum, maximum, maximum - minimum
    }
  }
' "${sample_file}"
echo 'FEEDBACK_LAST'
timeout 3s ros2 topic echo /rc_car/feedback --once || true
grep -E 'Serial connected|write failed|disconnected' "${bridge_log}" || true

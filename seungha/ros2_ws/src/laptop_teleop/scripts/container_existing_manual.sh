#!/usr/bin/env bash
set -eo pipefail

serial_port=/dev/serial/by-id/usb-Arduino__www.arduino.cc__0043_5583832383535181C1B0-if00
log_file=/tmp/rc_car_manual_bridge.log

if [[ ! -e "$serial_port" ]]; then
  echo "Arduino serial port not found: $serial_port" >&2
  exit 1
fi

source /opt/ros/humble/install/setup.bash
cd /root/ros2_ws
source install/setup.bash
set -u

bridge_pid=''
cleanup_done=false

cleanup() {
  if [[ "$cleanup_done" == true ]]; then
    return
  fi
  cleanup_done=true
  set +e
  timeout 1s ros2 topic pub --once \
    /rc_car/drive_cmd std_msgs/msg/Int32MultiArray \
    '{data: [0, 0]}' >/dev/null 2>&1
  if [[ -n "$bridge_pid" ]] && kill -0 "$bridge_pid" 2>/dev/null; then
    kill -INT "$bridge_pid"
    wait "$bridge_pid" 2>/dev/null
  fi
  echo
  echo '수동주행 종료: 중립 명령을 보내고 시리얼 브리지를 닫았습니다.'
}
trap cleanup EXIT
trap 'exit 130' INT TERM

ros2 launch rc_car_teleop manual_drive.launch.py \
  serial_port:="$serial_port" >"$log_file" 2>&1 &
bridge_pid=$!

echo 'Arduino 시리얼 브리지를 시작하는 중입니다...'
for _ in $(seq 1 50); do
  if ! kill -0 "$bridge_pid" 2>/dev/null; then
    echo '시리얼 브리지 시작 실패:' >&2
    tail -40 "$log_file" >&2
    exit 1
  fi
  if grep -q 'Serial connected:' "$log_file"; then
    break
  fi
  sleep 0.1
done

if ! grep -q 'Serial connected:' "$log_file"; then
  echo 'Arduino 연결을 확인하지 못했습니다:' >&2
  tail -40 "$log_file" >&2
  exit 1
fi

echo
echo '============================================================'
echo ' 기존 수동주행 알고리즘 실행 중'
echo ' 이 터미널을 클릭한 후 W/A/S/D를 누르세요.'
echo ' Space: 속도 정지 / X: 완전 정지 / Q: 안전 종료'
echo ' 처음에는 바퀴를 지면에서 띄운 상태로 시험하세요.'
echo '============================================================'
echo

ros2 run rc_car_teleop keyboard_teleop

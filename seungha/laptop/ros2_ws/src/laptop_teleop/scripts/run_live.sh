#!/usr/bin/env bash
set -euo pipefail

package_dir=/home/sandi/ros2_ws/src/laptop_teleop
confirmation_file="$package_dir/config/hardware_confirmed.env"

if ! grep -qx 'HARDWARE_CONFIRMED=YES' "$confirmation_file"; then
  echo '실차 실행이 차단되어 있습니다.'
  echo '보드 종류, 배선 핀, 조향 ADC, ESC 중립/방향을 확인한 뒤 hardware_confirmed.env를 변경하세요.'
  exit 2
fi

docker exec -it sanditest bash -lc '
  source /opt/ros/humble/install/setup.bash
  cd /root/ros2_ws
  source install/setup.bash
  exec ros2 launch laptop_teleop manual_drive.launch.py
'


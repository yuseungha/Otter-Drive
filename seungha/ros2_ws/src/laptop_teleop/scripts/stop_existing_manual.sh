#!/usr/bin/env bash
set -eo pipefail

mapfile -t launch_pids < <(
  docker exec sanditest pgrep -f \
    '^/usr/bin/python3 /opt/ros/humble/install/bin/ros2 launch rc_car_teleop manual_drive.launch.py' \
    2>/dev/null || true
)

for pid in "${launch_pids[@]}"; do
  docker exec sanditest /bin/kill -s INT "$pid" 2>/dev/null || true
done

for _ in $(seq 1 30); do
  if ! docker exec sanditest pgrep -f \
      '^/usr/bin/python3 /opt/ros/humble/install/bin/ros2 launch rc_car_teleop manual_drive.launch.py' \
      >/dev/null 2>&1; then
    break
  fi
  sleep 0.1
done

mapfile -t wrapper_pids < <(
  docker exec sanditest pgrep -f \
    '^bash /root/ros2_ws/src/laptop_teleop/scripts/container_existing_manual.sh$' \
    2>/dev/null || true
)

for pid in "${wrapper_pids[@]}"; do
  docker exec sanditest /bin/kill -s TERM "$pid" 2>/dev/null || true
done

echo 'Existing manual-drive processes stopped.'


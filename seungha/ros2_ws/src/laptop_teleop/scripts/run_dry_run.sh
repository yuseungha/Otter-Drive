#!/usr/bin/env bash
set -euo pipefail

docker exec -it sanditest bash -lc '
  source /opt/ros/humble/install/setup.bash
  cd /root/ros2_ws
  source install/setup.bash
  exec ros2 launch laptop_teleop web_dry_run.launch.py
'


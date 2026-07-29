#!/usr/bin/env bash
set -eo pipefail

pid_file=/tmp/laptop_teleop_dryrun.pid

if [[ -s "$pid_file" ]]; then
  old_pid="$(<"$pid_file")"
  if kill -0 "$old_pid" 2>/dev/null; then
    exit 0
  fi
fi

source /opt/ros/humble/install/setup.bash
cd /root/ros2_ws
source install/setup.bash
set -u

echo "$$" >"$pid_file"
exec ros2 launch laptop_teleop web_dry_run.launch.py \
  >>/tmp/laptop_teleop_dryrun.log 2>&1

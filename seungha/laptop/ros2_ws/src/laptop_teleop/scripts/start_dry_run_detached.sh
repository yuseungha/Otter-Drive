#!/usr/bin/env bash
set -euo pipefail

status_url=http://127.0.0.1:8765/api/status

if curl --fail --silent --show-error --max-time 1 "$status_url" >/dev/null 2>&1; then
  echo 'DRY-RUN server is already running.'
  exit 0
fi

docker exec -d sanditest \
  bash /root/ros2_ws/src/laptop_teleop/scripts/container_dry_run.sh

for _ in $(seq 1 20); do
  if curl --fail --silent --show-error --max-time 1 "$status_url" >/dev/null 2>&1; then
    echo 'DRY-RUN server is ready.'
    exit 0
  fi
  sleep 0.25
done

echo 'DRY-RUN server did not become ready.' >&2
exit 1


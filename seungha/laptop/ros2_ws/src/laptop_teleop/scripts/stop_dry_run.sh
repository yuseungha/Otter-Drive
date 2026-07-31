#!/usr/bin/env bash
set -euo pipefail

curl --fail --silent --show-error --max-time 1 \
  -X POST -H 'Content-Type: application/json' -d '{}' \
  http://127.0.0.1:8765/api/estop >/dev/null 2>&1 || true

docker exec sanditest bash -lc '
  pid_file=/tmp/laptop_teleop_dryrun.pid
  if [[ ! -s "$pid_file" ]]; then
    exit 0
  fi
  pid="$(<"$pid_file")"
  if kill -0 "$pid" 2>/dev/null; then
    kill -INT "$pid"
    for _ in $(seq 1 20); do
      kill -0 "$pid" 2>/dev/null || break
      sleep 0.1
    done
  fi
  if ! kill -0 "$pid" 2>/dev/null; then
    rm -f "$pid_file"
  fi
'

echo 'DRY-RUN server stopped.'


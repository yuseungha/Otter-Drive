#!/usr/bin/env bash
set -Eeuo pipefail

project_root=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd -P)
colcon_root=${project_root}/.colcon

if [[ ! -r /opt/ros/humble/setup.bash ]]; then
  echo 'ERROR: ROS 2 Humble is required. Run this script inside Ubuntu 22.04/WSL.' >&2
  exit 1
fi

set +u
source /opt/ros/humble/setup.bash
set -u
cd "${project_root}"
PYTHONNOUSERSITE=1 PYTHONDONTWRITEBYTECODE=1 colcon --log-base "${colcon_root}/log" build \
  --build-base "${colcon_root}/build" \
  --install-base "${colcon_root}/install" \
  --symlink-install \
  --base-paths src/jetson \
  --packages-select rc_car_teleop kmu_track
PYTHONNOUSERSITE=1 PYTHONDONTWRITEBYTECODE=1 PYTEST_ADDOPTS='-p no:cacheprovider' \
  colcon --log-base "${colcon_root}/log" test \
  --build-base "${colcon_root}/build" \
  --install-base "${colcon_root}/install" \
  --packages-select rc_car_teleop kmu_track
PYTHONNOUSERSITE=1 colcon --log-base "${colcon_root}/log" test-result \
  --test-result-base "${colcon_root}/build" \
  --verbose

echo 'SETUP_LAPTOP=OK'

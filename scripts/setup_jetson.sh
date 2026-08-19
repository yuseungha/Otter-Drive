#!/usr/bin/env bash
set -Eeuo pipefail

project_root=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd -P)
colcon_root=${project_root}/.colcon

if [[ -r ${project_root}/.env ]]; then
  set -a
  # shellcheck disable=SC1091
  source "${project_root}/.env"
  set +a
fi

image=${KMU_CONTAINER_IMAGE:-sandikookmin:cuda126}
expected_model_sha=${KMU_MODEL_SHA256:-b54bb33713d753ac7860ebad33c2f166ce9230f63fdf5c30a0528bac45ea779c}
model=${KMU_MODEL_PATH:-${project_root}/models/road_best.pt}

if [[ $(uname -m) != aarch64 ]]; then
  echo 'ERROR: setup_jetson.sh must run on the Jetson (aarch64).' >&2
  exit 1
fi
command -v docker >/dev/null || { echo 'ERROR: docker is missing.' >&2; exit 1; }
command -v colcon >/dev/null || { echo 'ERROR: colcon is missing.' >&2; exit 1; }
docker image inspect "${image}" >/dev/null 2>&1 || {
  echo "ERROR: required image is missing: ${image}" >&2
  exit 1
}
[[ -r ${model} ]] || { echo "ERROR: model is missing: ${model}" >&2; exit 1; }

actual_model_sha=$(sha256sum "${model}" | awk '{print $1}')
if [[ ${actual_model_sha} != "${expected_model_sha}" ]]; then
  echo "ERROR: model SHA-256 mismatch: ${actual_model_sha}" >&2
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
  --packages-select rc_car_teleop kmu_track lidar_cone_planner rplidar_ros
PYTHONNOUSERSITE=1 PYTHONDONTWRITEBYTECODE=1 PYTEST_ADDOPTS='-p no:cacheprovider' \
  colcon --log-base "${colcon_root}/log" test \
  --build-base "${colcon_root}/build" \
  --install-base "${colcon_root}/install" \
  --packages-select rc_car_teleop kmu_track lidar_cone_planner rplidar_ros \
  --event-handlers console_cohesion+
PYTHONNOUSERSITE=1 colcon --log-base "${colcon_root}/log" test-result \
  --test-result-base "${colcon_root}/build" \
  --verbose

docker run --rm --runtime=nvidia --privileged --network host \
  -e NVIDIA_VISIBLE_DEVICES=all \
  "${image}" \
  python3 -c 'import torch; assert torch.cuda.is_available(); print("CUDA:", torch.cuda.get_device_name(0))'

echo 'SETUP_JETSON=OK'
echo 'Next: ./scripts/run_competition.sh --check'

#!/usr/bin/env bash
# One-shot Jetson entry point for a folder that arrived by scp.
#
# scripts/run_competition.sh is still the launcher; it owns the docker
# invocation and the ros2 launch arguments.  This script only fills in what a
# freshly copied folder is missing before that launcher can work:
#
#   - .colcon/install/setup.bash   build output, never copied
#   - .env                         device paths; without it a live run aborts
#                                  on REPLACE_WITH_ARDUINO_DEVICE
#   - LF line endings              a Windows checkout copies CRLF, which breaks
#                                  both the shebang and "source .env"
#
# It never re-implements the launch itself.  See docs/unified_autonomy.md.
set -Eeuo pipefail

project_root=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd -P)
mode=dry-run
viewer=false
rebuild=false
full_setup=false
assume_yes=false

usage() {
  cat <<'USAGE'
Usage:
  ./scripts/jetson_drive.sh                 # dry-run: every node, no actuator
  ./scripts/jetson_drive.sh --live          # real driving, normal safety profile
  ./scripts/jetson_drive.sh --no-stop       # real driving, competition no-stop
  ./scripts/jetson_drive.sh --preflight     # checks only, launches nothing
  ./scripts/jetson_drive.sh --build         # build only

Options:
  --viewer        open the unified OpenCV window (needs X11 / DISPLAY)
  --rebuild       rebuild even if .colcon/install already exists
  --full-setup    run scripts/setup_jetson.sh instead (build + tests + CUDA)
  --yes           skip the interactive confirmation before a real drive

--live and --no-stop drive the vehicle.  Both refuse to start unless every
preflight check passes.  The default is dry-run, which never opens the Arduino
and publishes to /rc_car/drive_cmd_preview only.
USAGE
}

while (($#)); do
  case "$1" in
    --dry-run) mode=dry-run ;;
    --live) mode=live ;;
    --no-stop) mode=no-stop ;;
    --preflight) mode=preflight ;;
    --build) mode=build ;;
    --viewer) viewer=true ;;
    --rebuild) rebuild=true ;;
    --full-setup) full_setup=true ;;
    --yes) assume_yes=true ;;
    -h|--help) usage; exit 0 ;;
    *) echo "ERROR: unknown argument: $1" >&2; usage >&2; exit 2 ;;
  esac
  shift
done

fail() { echo "ERROR: $*" >&2; exit 1; }
step() { printf '\n== %s\n' "$*"; }
ok() { printf '   OK   %s\n' "$*"; }
warn() { printf '   WARN %s\n' "$*"; }

# --- 1. host ---------------------------------------------------------------
step 'host'
[[ $(uname -m) == aarch64 ]] || fail "this must run on the Jetson (aarch64), not $(uname -m)"
ok "$(uname -m) / $(hostname)"
command -v docker >/dev/null || fail 'docker is missing'
command -v colcon >/dev/null || fail 'colcon is missing'
[[ -r /opt/ros/humble/setup.bash ]] || fail 'ROS 2 Humble is missing at /opt/ros/humble'
ok 'docker, colcon, ROS 2 Humble present'

# --- 2. line endings -------------------------------------------------------
# A folder copied from a Windows checkout arrives with CRLF.  bash then fails
# with "bad interpreter", and a CRLF .env yields device paths with a trailing
# carriage return, which look like missing devices.
step 'line endings'
# Byte-exact on purpose: grep folds CR into the line terminator on some
# hosts, so it cannot be trusted to answer whether a file contains a CR.
has_cr() { [[ -n $(LC_ALL=C tr -dc '\r' < "$1" | head -c 1) ]]; }

crlf_fixed=0
strip_cr() {
  if [[ -f $1 ]] && has_cr "$1"; then
    sed -i 's/\r$//' "$1"
    crlf_fixed=$((crlf_fixed + 1))
  fi
}
while IFS= read -r candidate; do
  strip_cr "${candidate}"
done < <(find "${project_root}" -name '*.sh' -not -path '*/.git/*')
strip_cr "${project_root}/.env"
strip_cr "${project_root}/.env.example"
if ((crlf_fixed > 0)); then
  warn "converted ${crlf_fixed} file(s) from CRLF to LF"
else
  ok 'all shell files already LF'
fi
chmod +x "${project_root}"/scripts/*.sh

# --- 3. devices ------------------------------------------------------------
step 'devices'
first_glob() { compgen -G "$1" 2>/dev/null | head -n 1 || true; }

camera=$(first_glob '/dev/v4l/by-id/*video-index0')
[[ -n ${camera} ]] || fail 'no camera at /dev/v4l/by-id/*video-index0'
ok "camera  ${camera} -> $(readlink -f -- "${camera}")"

lidar=$(first_glob '/dev/serial/by-id/*CP2102*')
[[ -n ${lidar} ]] || fail 'no RPLIDAR (CP2102) under /dev/serial/by-id/'
ok "lidar   ${lidar} -> $(readlink -f -- "${lidar}")"

arduino=$(first_glob '/dev/serial/by-id/*Arduino*')
[[ -n ${arduino} ]] || arduino=$(first_glob '/dev/serial/by-id/*ACM*')
if [[ -z ${arduino} ]]; then
  while IFS= read -r candidate; do
    [[ ${candidate} == "${lidar}" ]] && continue
    arduino=${candidate}
    break
  done < <(compgen -G '/dev/serial/by-id/*' 2>/dev/null || true)
fi
if [[ -n ${arduino} ]]; then
  ok "arduino ${arduino} -> $(readlink -f -- "${arduino}")"
else
  warn 'no Arduino found; dry-run still works, --live and --no-stop will not'
fi

busy=()
for candidate in "${camera}" "${lidar}" ${arduino:+"${arduino}"}; do
  resolved=$(readlink -f -- "${candidate}")
  if fuser "${resolved}" >/dev/null 2>&1; then
    busy+=("${resolved}")
  fi
done
((${#busy[@]} == 0)) || \
  fail "device already held by another process: ${busy[*]} (stop the previous ROS session first)"
ok 'no device is held by another process'

# --- 4. .env ---------------------------------------------------------------
# run_competition.sh sources .env with "set -a", so whatever is written here
# wins over the environment.  KMU_HARDWARE_CONFIRMED is deliberately left out:
# this script supplies it per invocation, only after the operator confirms.
step 'env'
env_file=${project_root}/.env
if [[ -f ${env_file} ]]; then
  ok "using existing ${env_file} (not overwritten)"
  if grep -q 'REPLACE_WITH_ARDUINO_DEVICE' "${env_file}"; then
    fail ".env still holds the REPLACE_WITH_ARDUINO_DEVICE placeholder; set KMU_SERIAL_DEVICE=${arduino:-<arduino by-id path>}"
  fi
  if grep -qE '^[[:space:]]*KMU_HARDWARE_CONFIRMED=' "${env_file}"; then
    fail ".env sets KMU_HARDWARE_CONFIRMED, which would override the per-run confirmation; delete that line from ${env_file}"
  fi
else
  {
    echo '# Written by scripts/jetson_drive.sh. Machine-specific values only.'
    echo "KMU_CONTAINER_IMAGE=${KMU_CONTAINER_IMAGE:-sandikookmin:cuda126}"
    echo "KMU_MODEL_PATH=${project_root}/models/road_best.pt"
    echo "KMU_SEG_MODEL_PATH=${project_root}/models/lane_seg_v3_e37.pt"
    echo "KMU_CAMERA_DEVICE=${camera}"
    echo "KMU_LIDAR_DEVICE=${lidar}"
    if [[ -n ${arduino} ]]; then
      echo "KMU_SERIAL_DEVICE=${arduino}"
    fi
    echo "KMU_ROS_DOMAIN_ID=${KMU_ROS_DOMAIN_ID:-86}"
    echo 'KMU_DISPLAY=false'
    echo '# KMU_HARDWARE_CONFIRMED is intentionally absent: jetson_drive.sh'
    echo '# sets it for a single run after the operator confirms.'
    echo '# Model SHA-256 values live in configs/model_manifest.yaml.'
  } > "${env_file}"
  ok "wrote ${env_file}"
fi

# --- 5. models and container ----------------------------------------------
# configs/model_manifest.yaml is the single source of truth for the hashes, so
# this script carries no copies of them.
step 'models'
manifest=${project_root}/configs/model_manifest.yaml
[[ -r ${manifest} ]] || fail "manifest is missing: ${manifest}"

manifest_sha() {
  awk -v want="$1" '
    $1 == "filename:" { seen = $2 }
    $1 == "sha256:" && seen == want { print $2; exit }
  ' "${manifest}"
}

check_model() {
  local name=$1 requirement=$2 file expected actual
  file=${project_root}/models/${name}
  if [[ ! -r ${file} ]]; then
    if [[ ${requirement} == required ]]; then
      fail "model is missing: ${file}"
    fi
    warn "${name} is absent (only --check and the detect-only launches need it)"
    return 0
  fi
  expected=$(manifest_sha "${name}")
  [[ -n ${expected} ]] || fail "no sha256 recorded for ${name} in ${manifest}"
  actual=$(sha256sum "${file}" | awk '{print $1}')
  [[ ${actual} == "${expected}" ]] || \
    fail "${name} SHA-256 mismatch, transfer may be truncated: ${actual}"
  ok "${name} $(stat -c %s "${file}") bytes, sha256 matches the manifest"
}

check_model lane_seg_v3_e37.pt required
check_model road_best.pt optional

image=${KMU_CONTAINER_IMAGE:-sandikookmin:cuda126}
docker image inspect "${image}" >/dev/null 2>&1 || \
  fail "container image is missing: ${image} (scp does not carry it; build Dockerfile.jetson on this Jetson)"
ok "container image ${image}"

container_name=kmu-autodriving-runtime
if docker ps --format '{{.Names}}' | grep -Fxq "${container_name}"; then
  fail "${container_name} is still running; stop it first: docker stop ${container_name}"
fi
if docker ps -a --format '{{.Names}}' | grep -Fxq "${container_name}"; then
  docker rm "${container_name}" >/dev/null
  warn "removed a stale exited container named ${container_name}"
fi

# --- 6. build --------------------------------------------------------------
step 'workspace'
install_setup=${project_root}/.colcon/install/setup.bash
if [[ ${full_setup} == true ]]; then
  "${project_root}/scripts/setup_jetson.sh"
elif [[ ! -r ${install_setup} || ${rebuild} == true ]]; then
  # Same flags as scripts/setup_jetson.sh, minus its test and CUDA stages, so
  # that one unrelated failing test cannot block a run.
  set +u
  source /opt/ros/humble/setup.bash
  set -u
  (cd "${project_root}" && PYTHONNOUSERSITE=1 PYTHONDONTWRITEBYTECODE=1 \
    colcon --log-base "${project_root}/.colcon/log" build \
      --build-base "${project_root}/.colcon/build" \
      --install-base "${project_root}/.colcon/install" \
      --symlink-install \
      --base-paths src/jetson \
      --packages-select rc_car_teleop kmu_track kmu_ire_track \
      lidar_cone_planner rplidar_ros)
  ok 'colcon build finished'
else
  ok 'workspace already built (--rebuild to force)'
fi
[[ -r ${install_setup} ]] || fail "the build produced no ${install_setup}"

if [[ ${mode} == build ]]; then
  echo
  echo 'BUILD=OK'
  exit 0
fi
if [[ ${mode} == preflight ]]; then
  echo
  echo 'PREFLIGHT=OK'
  exit 0
fi

# --- 7. launch -------------------------------------------------------------
export KMU_DISPLAY=${viewer}
case ${mode} in
  dry-run) run_args=(--unified-dry-run) ;;
  live) run_args=(--unified-live) ;;
  no-stop) run_args=(--unified-no-stop-live) ;;
esac

if [[ ${mode} != dry-run ]]; then
  [[ -n ${arduino} ]] || fail 'a real drive needs the Arduino, which was not found'
  echo
  echo '  ==========================================================='
  echo '   REAL DRIVE: the wheels will turn.'
  echo "   profile : ${mode}"
  echo "   arduino : ${arduino}"
  echo '   output  : /rc_car/drive_cmd'
  if [[ ${mode} == no-stop ]]; then
    echo '   competition no-stop is ON: after the first forward command'
    echo '   a transient perception or command loss no longer stops the'
    echo '   car. E-stop, serial loss and firmware watchdog still do.'
  fi
  echo '   Confirm before continuing:'
  echo '     - the area is clear of people'
  echo '     - someone can physically cut power'
  echo '     - the wheels-up test has passed on this vehicle'
  echo '  ==========================================================='
  echo
  if [[ ${assume_yes} != true ]]; then
    [[ -t 0 ]] || fail 'a real drive needs an interactive terminal (ssh -t) or --yes'
    read -r -p 'Type DRIVE to continue: ' answer
    [[ ${answer} == DRIVE ]] || fail 'not confirmed; nothing was launched'
  fi
  export KMU_HARDWARE_CONFIRMED=true
fi

step "launching ${mode}"
exec "${project_root}/scripts/run_competition.sh" "${run_args[@]}"

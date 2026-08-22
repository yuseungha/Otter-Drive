#!/usr/bin/env bash
set -Eeuo pipefail

project_root=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd -P)

scan_topic=/scan
output_path=''
duration_s=0
wait_timeout_s=15
include_tf=true
compression=true
check_only=false

usage() {
  cat <<'EOF'
Usage:
  ./scripts/record_lidar_rosbag.sh [options]

Options:
  -t, --topic TOPIC       LiDAR topic to record (default: /scan)
  -o, --output PATH       Bag directory (default: data/lidar/rosbags/lidar_TIMESTAMP)
  -d, --duration SEC      Stop automatically after SEC seconds (default: until Ctrl+C)
      --wait-timeout SEC  Seconds to wait for the first LiDAR message (default: 15)
      --without-tf        Record only the LiDAR topic, without /tf and /tf_static
      --no-compression    Disable zstd file compression
      --check             Check ROS and the LiDAR topic without recording
  -h, --help              Show this help

Examples:
  ./scripts/record_lidar_rosbag.sh
  ./scripts/record_lidar_rosbag.sh --duration 30
  ./scripts/record_lidar_rosbag.sh --topic /lidar/scan --output data/test_scan

Start the LiDAR publisher first. Stop an unlimited recording with Ctrl+C.
EOF
}

fail() {
  echo "ERROR: $*" >&2
  exit 1
}

require_value() {
  local option=$1
  local count=$2
  ((count >= 2)) || fail "${option} requires a value"
}

is_nonnegative_number() {
  [[ $1 =~ ^([0-9]+([.][0-9]*)?|[.][0-9]+)$ ]]
}

is_positive_number() {
  is_nonnegative_number "$1" && awk -v value="$1" 'BEGIN { exit !(value > 0) }'
}

while (($#)); do
  case "$1" in
    -t|--topic)
      require_value "$1" "$#"
      scan_topic=$2
      shift
      ;;
    -o|--output)
      require_value "$1" "$#"
      output_path=$2
      shift
      ;;
    -d|--duration)
      require_value "$1" "$#"
      duration_s=$2
      shift
      ;;
    --wait-timeout)
      require_value "$1" "$#"
      wait_timeout_s=$2
      shift
      ;;
    --without-tf) include_tf=false ;;
    --no-compression) compression=false ;;
    --check) check_only=true ;;
    -h|--help) usage; exit 0 ;;
    *) echo "ERROR: unknown argument: $1" >&2; usage >&2; exit 2 ;;
  esac
  shift
done

[[ ${scan_topic} == /* ]] || fail 'the topic must be an absolute ROS topic beginning with /'
if [[ ${duration_s} != 0 ]]; then
  is_positive_number "${duration_s}" || fail '--duration must be a positive number of seconds'
fi
is_positive_number "${wait_timeout_s}" || fail '--wait-timeout must be a positive number of seconds'

if [[ -r ${project_root}/.env ]]; then
  set -a
  # shellcheck disable=SC1091
  source "${project_root}/.env"
  set +a
fi

export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-${KMU_ROS_DOMAIN_ID:-86}}
export ROS_LOCALHOST_ONLY=${ROS_LOCALHOST_ONLY:-0}

[[ -r /opt/ros/humble/setup.bash ]] || fail 'ROS 2 Humble is not installed at /opt/ros/humble'
set +u
# shellcheck disable=SC1091
source /opt/ros/humble/setup.bash
if [[ -r ${project_root}/.colcon/install/setup.bash ]]; then
  # shellcheck disable=SC1091
  source "${project_root}/.colcon/install/setup.bash"
fi
set -u

command -v ros2 >/dev/null || fail 'ros2 command is unavailable after sourcing ROS 2 Humble'
command -v timeout >/dev/null || fail 'timeout command is unavailable'
command -v awk >/dev/null || fail 'awk command is unavailable'

echo "Waiting up to ${wait_timeout_s}s for ${scan_topic} ..."
if ! timeout --foreground "${wait_timeout_s}s" \
  ros2 topic echo --no-daemon --once "${scan_topic}" >/dev/null 2>&1; then
  fail "no message received on ${scan_topic}; start the LiDAR publisher and check ROS_DOMAIN_ID=${ROS_DOMAIN_ID}"
fi

topic_type=$(ros2 topic type --no-daemon "${scan_topic}" 2>/dev/null || true)
case "${topic_type}" in
  sensor_msgs/msg/LaserScan|sensor_msgs/msg/PointCloud2) ;;
  '') fail "could not determine the message type of ${scan_topic}" ;;
  *) fail "${scan_topic} has type ${topic_type}, not LaserScan or PointCloud2" ;;
esac

echo "LIDAR_TOPIC=${scan_topic}"
echo "LIDAR_TYPE=${topic_type}"
echo "ROS_DOMAIN_ID=${ROS_DOMAIN_ID}"
echo "ROS_LOCALHOST_ONLY=${ROS_LOCALHOST_ONLY}"

if [[ ${check_only} == true ]]; then
  echo 'LIDAR_ROSBAG_CHECK=OK'
  exit 0
fi

if [[ -z ${output_path} ]]; then
  output_path=${project_root}/data/lidar/rosbags/lidar_$(date '+%Y%m%d-%H%M%S')
elif [[ ${output_path} != /* ]]; then
  output_path=${project_root}/${output_path}
fi

[[ ! -e ${output_path} ]] || fail "output path already exists: ${output_path}"
mkdir -p "$(dirname -- "${output_path}")"

topics=("${scan_topic}")
if [[ ${include_tf} == true ]]; then
  topics+=(/tf /tf_static)
fi

record_command=(
  ros2 bag record
  --output "${output_path}"
  --storage sqlite3
  --no-discovery
)
if [[ ${compression} == true ]]; then
  record_command+=(--compression-mode file --compression-format zstd)
fi
record_command+=("${topics[@]}")

echo "OUTPUT=${output_path}"
echo "TOPICS=${topics[*]}"
if [[ ${duration_s} == 0 ]]; then
  echo 'Recording LiDAR data. Press Ctrl+C to stop.'
else
  echo "Recording LiDAR data for ${duration_s}s."
fi

record_status=0
if [[ ${duration_s} == 0 ]]; then
  "${record_command[@]}" || record_status=$?
else
  timeout --foreground --signal=INT --kill-after=10s "${duration_s}s" \
    "${record_command[@]}" || record_status=$?
  if [[ ${record_status} -eq 124 ]]; then
    record_status=0
  fi
fi

[[ -r ${output_path}/metadata.yaml ]] || \
  fail "recording did not produce metadata.yaml (ros2 bag exit=${record_status})"

echo
ros2 bag info "${output_path}"
if [[ ${record_status} -ne 0 ]]; then
  fail "ros2 bag record exited with status ${record_status}"
fi

lidar_message_count=$(
  awk -v topic="${scan_topic}" '
    $1 == "name:" { lidar_topic = ($2 == topic) }
    lidar_topic && $1 == "message_count:" { print $2; exit }
  ' \
    "${output_path}/metadata.yaml"
)
[[ ${lidar_message_count} =~ ^[0-9]+$ ]] || \
  fail "could not read the ${scan_topic} message count from metadata.yaml"
((lidar_message_count > 0)) || \
  fail "the bag contains no ${scan_topic} messages; check the publisher and QoS"

echo "LIDAR_MESSAGES=${lidar_message_count}"
echo "LIDAR_ROSBAG_SAVED=${output_path}"

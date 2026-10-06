#!/usr/bin/env bash
# Collect the evidence needed to distinguish /clock, PX4 DDS, pose-frame and TF
# publication failures. Run this while multi_scout.launch.py is still running.
set -u

SCOUT_COUNT="${1:-4}"
OUTPUT_FILE="${2:-${PWD}/fire_scout_tf_diagnostic_$(date +%Y%m%d_%H%M%S).log}"

if ! [[ "${SCOUT_COUNT}" =~ ^[1-9][0-9]*$ ]]; then
  echo "Usage: $0 [scout_count] [output_file]" >&2
  exit 2
fi

mkdir -p "$(dirname "${OUTPUT_FILE}")"
: > "${OUTPUT_FILE}"

section() {
  printf '\n===== %s =====\n' "$1" | tee -a "${OUTPUT_FILE}"
}

capture() {
  printf '\n$' | tee -a "${OUTPUT_FILE}"
  printf ' %q' "$@" | tee -a "${OUTPUT_FILE}"
  printf '\n' | tee -a "${OUTPUT_FILE}"
  timeout 8s "$@" 2>&1 | tee -a "${OUTPUT_FILE}" || true
}

section "ENVIRONMENT"
{
  echo "date=$(date --iso-8601=seconds)"
  echo "ROS_DISTRO=${ROS_DISTRO:-<unset>}"
  echo "ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-<unset>}"
  echo "RMW_IMPLEMENTATION=${RMW_IMPLEMENTATION:-<default>}"
  echo "GZ_PARTITION=${GZ_PARTITION:-<unset>}"
  echo "FIRE_SCOUT_GZ_WORLD=${FIRE_SCOUT_GZ_WORLD:-<unset>}"
} | tee -a "${OUTPUT_FILE}"

section "CLOCK"
capture ros2 topic info --verbose /clock
capture ros2 topic echo --once /clock

section "ROS GRAPH"
capture ros2 node list
capture ros2 topic info --verbose /tf
capture ros2 topic info --verbose /tf_static

for ((index = 1; index <= SCOUT_COUNT; ++index)); do
  scout="scout${index}"
  px4_topic="/${scout}/fmu/out/vehicle_odometry"
  odom_topic="/${scout}/odom"
  node="/${scout}/px4_odom_adapter"

  section "${scout}: PX4 VEHICLE ODOMETRY"
  capture ros2 topic info --verbose "${px4_topic}"
  capture ros2 topic echo --once --field pose_frame "${px4_topic}"
  capture ros2 topic hz "${px4_topic}"

  section "${scout}: ADAPTER PARAMETERS"
  capture ros2 param get "${node}" runtime_version
  capture ros2 param get "${node}" use_sim_time
  capture ros2 param get "${node}" input_topic
  capture ros2 param get "${node}" output_topic
  capture ros2 param get "${node}" odom_frame
  capture ros2 param get "${node}" base_frame
  capture ros2 param get "${node}" accept_frd_pose_frame

  section "${scout}: ROS ODOMETRY AND TF"
  capture ros2 topic info --verbose "${odom_topic}"
  capture ros2 topic echo --once --field header "${odom_topic}"
  capture ros2 topic echo --once --field child_frame_id "${odom_topic}"
  capture ros2 run tf2_ros tf2_echo "${scout}/odom" "${scout}/base_link"
done

section "ROS DOCTOR"
capture ros2 doctor --report

section "INTERPRETATION"
cat <<'TEXT' | tee -a "${OUTPUT_FILE}"
pose_frame=1 is PX4 NED; pose_frame=2 is PX4 fixed FRD. Both are valid inputs
for the repaired adapter. Diagnose in this order:
1. /clock absent or zero: scout1 ros_gz_bridge/GZ_PARTITION problem.
2. PX4 topic has zero publishers/data: PX4 DDS namespace, XRCE agent, domain or
   px4_msgs version mismatch.
3. PX4 data exists but /scoutN/odom does not: adapter rejection/crash; inspect
   that adapter's console message and pose_frame.
4. /scoutN/odom exists but tf2_echo fails: duplicate/conflicting /tf publishers
   or wrong frame IDs.
TEXT

printf '\nSaved diagnostic report: %s\n' "${OUTPUT_FILE}"

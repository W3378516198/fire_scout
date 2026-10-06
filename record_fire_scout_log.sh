#!/usr/bin/env bash
set -eo pipefail

# Fire Scout flight log recorder
# Usage:
#   chmod +x ~/fire_scout_ws/record_fire_scout_log.sh
#   ~/fire_scout_ws/record_fire_scout_log.sh
#
# Start this in a separate terminal BEFORE sending the mission goal.
# Press Ctrl+C 5~10 s after the UAV finishes / gets stuck / oscillates.
# The script will stop rosbag cleanly and create a ZIP file.

WS="${HOME}/fire_scout_ws"
LOG_ROOT="${HOME}/fire_scout_logs"
STAMP="$(date +%Y%m%d_%H%M%S)"
RUN_DIR="${LOG_ROOT}/flight_${STAMP}"
BAG_DIR="${RUN_DIR}/rosbag"

mkdir -p "${RUN_DIR}"

# ROS / ament setup scripts may read variables that are unset.
# Keep `nounset` disabled while sourcing them, then enable it afterwards.
set +u
source /opt/ros/humble/setup.bash
if [ -f "${WS}/install/setup.bash" ]; then
  source "${WS}/install/setup.bash"
fi
set -u

echo "=============================================="
echo " Fire Scout flight log recorder"
echo " Output: ${RUN_DIR}"
echo "=============================================="

# ---------- Static environment / graph snapshot ----------
{
  echo "===== TIME ====="
  date -Ins
  echo

  echo "===== ROS ENV ====="
  echo "ROS_DISTRO=${ROS_DISTRO:-}"
  echo "ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-}"
  echo "RMW_IMPLEMENTATION=${RMW_IMPLEMENTATION:-}"
  echo

  echo "===== ROS NODES ====="
  ros2 node list || true
  echo

  echo "===== ROS TOPICS + TYPES ====="
  ros2 topic list -t || true
  echo
} > "${RUN_DIR}/system_snapshot.txt"

# Dump parameters of the nodes that matter most.
for NODE in \
  /radar_free_space_mapper \
  /safe_airfar_like_planner \
  /safe_airfar_path_follower
do
  SAFE_NAME="$(echo "${NODE}" | tr '/' '_' | sed 's/^_//')"
  ros2 param dump "${NODE}" > "${RUN_DIR}/params_${SAFE_NAME}.yaml" 2>&1 || true
done

# ---------- Candidate topics ----------
# The script records only topics that actually exist.
CANDIDATES=(
  "/clock"
  "/tf"
  "/tf_static"
  "/rosout"

  "/scout1/odom"

  "/scout1/planning/global_path"

  "/scout1/map/radar_free"
  "/scout1/map/radar_occupied"
  "/scout1/map/radar_cloud"

  "/scout1/radar/front/mmwave_points"
  "/scout1/radar/left/mmwave_points"
  "/scout1/radar/right/mmwave_points"

  "/scout1/ultrasonic/front/scan"

  "/scout1/fmu/in/offboard_control_mode"
  "/scout1/fmu/in/trajectory_setpoint"
  "/scout1/fmu/in/vehicle_command"

  "/scout1/fmu/out/vehicle_odometry"
  "/scout1/fmu/out/vehicle_status"
  "/scout1/fmu/out/vehicle_local_position"
  "/scout1/fmu/out/vehicle_local_position_v1"
  "/scout1/fmu/out/vehicle_attitude"
  "/scout1/fmu/out/vehicle_angular_velocity"
  "/scout1/fmu/out/vehicle_command_ack"
)

mapfile -t EXISTING_TOPICS < <(ros2 topic list)

FOUND_TOPICS=()
MISSING_TOPICS=()

topic_exists() {
  local target="$1"
  for t in "${EXISTING_TOPICS[@]}"; do
    if [ "$t" = "$target" ]; then
      return 0
    fi
  done
  return 1
}

for t in "${CANDIDATES[@]}"; do
  if topic_exists "$t"; then
    FOUND_TOPICS+=("$t")
  else
    MISSING_TOPICS+=("$t")
  fi
done

{
  echo "===== RECORDED TOPICS ====="
  printf '%s\n' "${FOUND_TOPICS[@]}"
  echo
  echo "===== CANDIDATES NOT PRESENT ====="
  printf '%s\n' "${MISSING_TOPICS[@]}"
} > "${RUN_DIR}/recorded_topics.txt"

if [ "${#FOUND_TOPICS[@]}" -eq 0 ]; then
  echo "ERROR: No candidate topics were found."
  exit 1
fi

echo
echo "Recording ${#FOUND_TOPICS[@]} topics:"
printf '  %s\n' "${FOUND_TOPICS[@]}"
echo
echo "Start the mission now."
echo "Press Ctrl+C 5~10 seconds AFTER the interesting behavior."
echo

# ---------- Optional lightweight topic-rate monitor ----------
(
  while true; do
    {
      echo "===== $(date -Ins) ====="
      for t in \
        "/scout1/odom" \
        "/scout1/planning/global_path" \
        "/scout1/ultrasonic/front/scan"
      do
        if topic_exists "$t"; then
          timeout 2s ros2 topic hz "$t" 2>&1 | tail -n 2 || true
        fi
      done
      echo
    } >> "${RUN_DIR}/topic_rate_monitor.txt"
    sleep 2
  done
) &
RATE_PID=$!

# ---------- rosbag ----------
ros2 bag record \
  --storage sqlite3 \
  -o "${BAG_DIR}" \
  "${FOUND_TOPICS[@]}" &
BAG_PID=$!

cleanup() {
  trap - INT TERM

  echo
  echo "Stopping recorder..."

  if kill -0 "${RATE_PID}" 2>/dev/null; then
    kill "${RATE_PID}" 2>/dev/null || true
  fi

  if kill -0 "${BAG_PID}" 2>/dev/null; then
    kill -INT "${BAG_PID}" 2>/dev/null || true
    wait "${BAG_PID}" 2>/dev/null || true
  fi

  sleep 1

  if [ -d "${BAG_DIR}" ]; then
    ros2 bag info "${BAG_DIR}" > "${RUN_DIR}/bag_info.txt" 2>&1 || true
  fi

  # Save one final graph snapshot.
  {
    echo "===== FINAL TIME ====="
    date -Ins
    echo
    echo "===== FINAL ROS NODES ====="
    ros2 node list || true
    echo
    echo "===== FINAL TOPICS + TYPES ====="
    ros2 topic list -t || true
  } > "${RUN_DIR}/final_snapshot.txt"

  ZIP_PATH="${RUN_DIR}.zip"
  cd "${LOG_ROOT}"
  zip -qr "${ZIP_PATH}" "$(basename "${RUN_DIR}")"

  echo
  echo "=============================================="
  echo " Recording complete"
  echo " Folder: ${RUN_DIR}"
  echo " ZIP:    ${ZIP_PATH}"
  echo "=============================================="
  echo
  echo "Please send me the ZIP file."
  exit 0
}

trap cleanup INT TERM

wait "${BAG_PID}"
cleanup

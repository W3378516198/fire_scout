#!/usr/bin/env bash
# Record a reproducible ROS 2 bringup start for intermittent TF/DDS failures.
# The script starts the supplied launch command, mirrors its output to the
# terminal, and saves lightweight graph/TF snapshots without recording radar
# point clouds.

set -uo pipefail

DURATION=55
PERIOD=10
OUT_DIR=""
RECORD_TF_BAG=0

usage() {
  cat <<'EOF'
Usage:
  record_bringup_startup.sh [options] -- <ros2 launch command...>

Options:
  -d, --duration SEC       Total capture duration (default: 55)
  -p, --period SEC         Snapshot interval (default: 10)
  -o, --output DIR         Output directory (default: ./startup_logs/<time>)
      --record-tf-bag      Also record /tf, /tf_static, /clock and odometry.
                            This adds one extra ROS participant, so use only
                            after a normal capture is inconclusive.
  -h, --help               Show this help

Example:
  ./record_bringup_startup.sh -d 70 -- \
    ros2 launch fire_scout_bringup multi_scout.launch.py use_rviz:=true
EOF
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    -d|--duration) DURATION="$2"; shift 2 ;;
    -p|--period) PERIOD="$2"; shift 2 ;;
    -o|--output) OUT_DIR="$2"; shift 2 ;;
    --record-tf-bag) RECORD_TF_BAG=1; shift ;;
    -h|--help) usage; exit 0 ;;
    --) shift; break ;;
    *) echo "Unknown option: $1" >&2; usage >&2; exit 2 ;;
  esac
done

if [[ $# -eq 0 ]]; then
  echo "A launch command is required after --." >&2
  usage >&2
  exit 2
fi

if ! command -v ros2 >/dev/null 2>&1; then
  echo "ros2 was not found. Source /opt/ros/humble/setup.bash and the workspace first." >&2
  exit 1
fi

case "$DURATION:$PERIOD" in
  *[!0-9:]*|0:*|*:0) echo "duration and period must be positive integers." >&2; exit 2 ;;
esac

if [[ -z "$OUT_DIR" ]]; then
  OUT_DIR="$PWD/startup_logs/bringup_$(date +%Y%m%d_%H%M%S)"
fi
mkdir -p "$OUT_DIR"
OUT_DIR="$(cd "$OUT_DIR" && pwd)"

LAUNCH_PID=""
TAIL_PID=""
BAG_PID=""
CAPTURE_FINISHED=0

stop_group() {
  local pid="$1"
  [[ -n "$pid" ]] || return 0
  kill -0 "$pid" 2>/dev/null || return 0
  kill -INT -- "-$pid" 2>/dev/null || true
  sleep 1
  kill -0 "$pid" 2>/dev/null && kill -TERM -- "-$pid" 2>/dev/null || true
}

cleanup() {
  echo "[$(date --iso-8601=seconds)] stopping capture" >> "$OUT_DIR/monitor.log"
  stop_group "$BAG_PID"
  [[ -n "$TAIL_PID" ]] && kill "$TAIL_PID" 2>/dev/null || true
  # On Ctrl+C or an error, stop the launch too.  On a completed capture leave
  # it running: this is still the user's normal simulation session.
  [[ "$CAPTURE_FINISHED" -eq 1 ]] || stop_group "$LAUNCH_PID"
}
trap cleanup EXIT INT TERM

{
  echo "capture_started=$(date --iso-8601=seconds)"
  echo "hostname=$(hostname)"
  echo "ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-<unset>}"
  echo "RMW_IMPLEMENTATION=${RMW_IMPLEMENTATION:-<default>}"
  echo "ROS_LOCALHOST_ONLY=${ROS_LOCALHOST_ONLY:-<unset>}"
  echo "command: $(printf '%q ' "$@")"
} > "$OUT_DIR/environment.txt"

ps -eo pid,ppid,stat,etime,args > "$OUT_DIR/processes_before.txt"

echo "Saving startup evidence to: $OUT_DIR"
echo "Starting launch: $(printf '%q ' "$@")"

# setsid lets Ctrl+C cleanly stop only the launch started by this script.
setsid "$@" > "$OUT_DIR/bringup.log" 2>&1 &
LAUNCH_PID=$!
tail --pid="$LAUNCH_PID" -n 0 -f "$OUT_DIR/bringup.log" &
TAIL_PID=$!

if [[ "$RECORD_TF_BAG" -eq 1 ]]; then
  setsid ros2 bag record -o "$OUT_DIR/tf_startup_bag" \
    /tf /tf_static /clock \
    /scout1/odom /scout2/odom /scout3/odom /scout4/odom \
    /scout1/fmu/out/vehicle_odometry /scout2/fmu/out/vehicle_odometry \
    /scout3/fmu/out/vehicle_odometry /scout4/fmu/out/vehicle_odometry \
    > "$OUT_DIR/tf_bag.log" 2>&1 &
  BAG_PID=$!
fi

snapshot() {
  local t="$1" snap="$OUT_DIR/t$(printf '%03d' "$1")"
  mkdir -p "$snap"
  {
    echo "snapshot_time=$(date --iso-8601=seconds)"
    echo "elapsed_seconds=$t"
    echo "launch_pid=$LAUNCH_PID"
    ps -p "$LAUNCH_PID" -o pid,ppid,stat,etime,args 2>&1
  } > "$snap/meta.txt"
  ps -eo pid,ppid,stat,etime,args > "$snap/processes.txt"

  timeout 4 ros2 node list --no-daemon --spin-time 1 > "$snap/nodes.txt" 2>&1 || true
  timeout 4 ros2 topic list --no-daemon > "$snap/topics.txt" 2>&1 || true
  timeout 4 ros2 topic info -v /tf > "$snap/tf_topic_info.txt" 2>&1 || true
  timeout 4 ros2 topic info -v /tf_static > "$snap/tf_static_topic_info.txt" 2>&1 || true

  for scout in scout1 scout2 scout3 scout4; do
    timeout 3 ros2 run tf2_ros tf2_echo world "$scout/odom" \
      > "$snap/world_to_${scout}_odom.txt" 2>&1 &
  done
  wait || true

  # The generated frames.pdf is the easiest artefact for comparing good/bad
  # runs. Do it every 20 s only, to minimise extra DDS discovery traffic.
  if (( t % 20 == 0 )); then
    (
      cd "$snap"
      timeout 8 ros2 run tf2_tools view_frames > view_frames.log 2>&1 || true
    )
  fi
}

elapsed=0
while (( elapsed <= DURATION )); do
  snapshot "$elapsed"
  if ! kill -0 "$LAUNCH_PID" 2>/dev/null; then
    echo "Launch exited before the capture window finished." | tee -a "$OUT_DIR/monitor.log"
    break
  fi
  sleep "$PERIOD"
  elapsed=$((elapsed + PERIOD))
done

ps -eo pid,ppid,stat,etime,args > "$OUT_DIR/processes_after.txt"
{
  echo "capture_finished=$(date --iso-8601=seconds)"
  echo "launch_pid=$LAUNCH_PID"
  echo "tf_bag_enabled=$RECORD_TF_BAG"
  echo "Recommended comparison files: bringup.log, t*/nodes.txt, t*/world_to_*_odom.txt, t*/frames.pdf"
} >> "$OUT_DIR/monitor.log"

echo
echo "Capture complete: $OUT_DIR"
echo "Compare bringup.log and each t*/world_to_*_odom.txt first."
CAPTURE_FINISHED=1

#!/usr/bin/env bash
set -eo pipefail

DURATION=900
SCOUTS=4
MAX_MB=24
OUTPUT_ROOT="${HOME}/fire_scout_logs"

usage() {
  echo "用法: $0 [--duration 秒] [--scouts 数量] [--max-mb 大小] [--output 目录]"
  echo "      --duration 0 表示一直录制到 Ctrl-C；默认 900 秒。"
}

while [ "$#" -gt 0 ]; do
  case "$1" in
    --duration) DURATION="$2"; shift 2 ;;
    --scouts) SCOUTS="$2"; shift 2 ;;
    --max-mb) MAX_MB="$2"; shift 2 ;;
    --output) OUTPUT_ROOT="$2"; shift 2 ;;
    -h|--help) usage; exit 0 ;;
    *) echo "未知参数: $1" >&2; usage >&2; exit 2 ;;
  esac
done

if [ -r /opt/ros/humble/setup.bash ]; then
  # Do not enable nounset: Humble/colcon setup files read optional variables.
  # shellcheck disable=SC1091
  source /opt/ros/humble/setup.bash
fi
if [ -r "${HOME}/fire_scout_ws/install/setup.bash" ]; then
  # shellcheck disable=SC1091
  source "${HOME}/fire_scout_ws/install/setup.bash"
fi
if ! command -v ros2 >/dev/null 2>&1; then
  echo "错误：找不到 ros2，请确认 ROS 2 Humble 已安装。" >&2
  exit 2
fi

SCRIPT_DIR=$(CDPATH= cd -- "$(dirname -- "$0")" && pwd)
LOGGER="${SCRIPT_DIR}/fire_scout_light_logger.py"
if ! python3 -m py_compile "$LOGGER"; then
  echo "错误：日志程序语法自检失败，未开始录制。" >&2
  exit 2
fi

STAMP=$(date -u +%Y%m%dT%H%M%SZ)
SESSION_NAME="fire_scout_log_${STAMP}"
SESSION_DIR="${OUTPUT_ROOT}/${SESSION_NAME}"
ARCHIVE="${OUTPUT_ROOT}/${SESSION_NAME}.tar.xz"
mkdir -p "$SESSION_DIR"

{
  echo "started_utc=$(date -u --iso-8601=seconds)"
  echo "hostname=$(hostname)"
  echo "kernel=$(uname -srmo)"
  echo "ros_distro=${ROS_DISTRO:-unknown}"
  echo "ros_domain_id=${ROS_DOMAIN_ID:-0}"
  echo "rmw_implementation=${RMW_IMPLEMENTATION:-default}"
  echo "scouts=${SCOUTS}"
  echo "duration_seconds=${DURATION}"
  echo "event_limit_mb=${MAX_MB}"
} > "${SESSION_DIR}/manifest.txt"

PACKAGE_PREFIX=$(ros2 pkg prefix fire_scout_planning_safe 2>/dev/null || true)
if [ -n "$PACKAGE_PREFIX" ] && \
   [ -r "${PACKAGE_PREFIX}/share/fire_scout_planning_safe/config/safe_airfar.yaml" ]; then
  cp "${PACKAGE_PREFIX}/share/fire_scout_planning_safe/config/safe_airfar.yaml" \
     "${SESSION_DIR}/effective_package_config.yaml"
fi

{
  date -u --iso-8601=seconds
  uptime
  free -h 2>/dev/null || true
  df -h "$OUTPUT_ROOT" 2>/dev/null || true
  ps -eo pid,ppid,stat,pcpu,pmem,etime,comm,args --sort=-pcpu | head -n 80 || true
} > "${SESSION_DIR}/system_start.txt" 2>&1

echo "开始轻量录制：${SESSION_DIR}"
echo "不依赖 ros2 daemon 预检查；即使话题尚未出现，也会记录 missing/stale 状态。"
echo "按 Ctrl-C 可提前停止并自动打包。"

LOGGER_PID=""
STOPPING=0
USER_STOP=0
request_stop() {
  if [ "$STOPPING" -eq 1 ]; then
    [ -n "$LOGGER_PID" ] && kill -KILL "$LOGGER_PID" 2>/dev/null || true
    return
  fi
  STOPPING=1
  USER_STOP=1
  echo "收到停止信号，正在安全结束录制并打包……"
  [ -n "$LOGGER_PID" ] && kill -TERM "$LOGGER_PID" 2>/dev/null || true
}
trap request_stop INT TERM

python3 "$LOGGER" --output "${SESSION_DIR}/events.jsonl" \
  --duration "$DURATION" --scouts "$SCOUTS" --max-mb "$MAX_MB" \
  > "${SESSION_DIR}/logger_stdout.txt" 2> "${SESSION_DIR}/logger_stderr.txt" &
LOGGER_PID=$!

READY=0
for _attempt in $(seq 1 100); do
  if grep -q '^LOGGER_READY$' "${SESSION_DIR}/logger_stdout.txt" 2>/dev/null; then
    READY=1
    break
  fi
  if ! kill -0 "$LOGGER_PID" 2>/dev/null; then
    break
  fi
  sleep 0.1 || true
done
if [ "$READY" -eq 1 ]; then
  echo "日志订阅器已就绪。"
else
  echo "错误：日志订阅器未能就绪；仍会保留并打包诊断文件。" >&2
  STOPPING=1
  kill -TERM "$LOGGER_PID" 2>/dev/null || true
fi

# Optional inventories cannot prevent direct rclpy recording.
timeout 4s ros2 topic list -t > "${SESSION_DIR}/topics_start.txt" 2>&1 || true
timeout 4s ros2 node list > "${SESSION_DIR}/nodes_start.txt" 2>&1 || true

STOP_TICKS=0
while kill -0 "$LOGGER_PID" 2>/dev/null; do
  if [ "$STOPPING" -eq 1 ]; then
    STOP_TICKS=$((STOP_TICKS + 1))
    if [ "$STOP_TICKS" -ge 40 ]; then
      echo "日志进程未及时退出，正在强制结束；已有内容仍会打包。" >&2
      kill -KILL "$LOGGER_PID" 2>/dev/null || true
    fi
  fi
  sleep 0.25 || true
done

set +e
wait "$LOGGER_PID"
LOGGER_STATUS=$?
set -e
LOGGER_PID=""

timeout 4s ros2 topic list -t > "${SESSION_DIR}/topics_end.txt" 2>&1 || true
timeout 4s ros2 node list > "${SESSION_DIR}/nodes_end.txt" 2>&1 || true
{
  echo "finished_utc=$(date -u --iso-8601=seconds)"
  echo "logger_exit_status=${LOGGER_STATUS}"
  echo "user_stop=${USER_STOP}"
  uptime
  free -h 2>/dev/null || true
  ps -eo pid,ppid,stat,pcpu,pmem,etime,comm,args --sort=-pcpu | head -n 80 || true
} > "${SESSION_DIR}/system_end.txt" 2>&1

trap '' INT TERM
XZ_OPT="-1 -T0" tar -C "$OUTPUT_ROOT" -cJf "$ARCHIVE" "$SESSION_NAME"
SIZE=$(du -h "$ARCHIVE" | awk '{print $1}')
echo "日志已打包：${ARCHIVE} (${SIZE})"
echo "请把这个 .tar.xz 文件发给我分析。"

if [ "$READY" -ne 1 ] || { [ "$LOGGER_STATUS" -ne 0 ] && [ "$USER_STOP" -ne 1 ]; }; then
  echo "警告：日志进程异常退出，请连同压缩包一起发送；错误见 logger_stderr.txt。" >&2
  exit 1
fi
exit 0

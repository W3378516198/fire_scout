#!/usr/bin/env bash
set -eo pipefail

PACKAGE_NAME="fire_scout_planning_safe"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd -P)"
PACKAGE_CANDIDATE="$(cd "${SCRIPT_DIR}/.." && pwd -P)"
PACKAGE_DIR=""
if [ "$(basename "${PACKAGE_CANDIDATE}")" = "${PACKAGE_NAME}" ] && \
   [ -f "${PACKAGE_CANDIDATE}/package.xml" ]; then
  PACKAGE_DIR="${PACKAGE_CANDIDATE}"
fi

if [ "$#" -gt 1 ]; then
  echo "用法：$0 [ROS2工作空间路径]" >&2
  exit 2
fi

if [ "$#" -eq 1 ]; then
  WORKSPACE="$1"
elif [ -n "${PACKAGE_DIR}" ] && [ "$(basename "$(dirname "${PACKAGE_DIR}")")" = "src" ]; then
  WORKSPACE="$(cd "${PACKAGE_DIR}/../.." && pwd -P)"
else
  WORKSPACE="${HOME}/fire_scout_ws"
fi

WORKSPACE="$(cd "${WORKSPACE}" 2>/dev/null && pwd -P)" || {
  echo "错误：工作空间不存在" >&2
  exit 2
}

if [ -z "${PACKAGE_DIR}" ]; then
  PACKAGE_DIR="${WORKSPACE}/src/${PACKAGE_NAME}"
fi
if [ ! -f "${PACKAGE_DIR}/package.xml" ]; then
  echo "错误：未找到 ${WORKSPACE}/src/${PACKAGE_NAME}/package.xml" >&2
  exit 2
fi

case "${PACKAGE_DIR}" in
  "${WORKSPACE}/src/${PACKAGE_NAME}") ;;
  *)
    echo "错误：节点包应位于 ${WORKSPACE}/src/${PACKAGE_NAME}" >&2
    exit 2
    ;;
esac

if pgrep -f '/(radar_free_space_mapper|multi_uav_map_fusion|racer_task_allocator|safe_airfar_like_planner|safe_airfar_path_follower|passage_coordinator)( |$)' >/dev/null 2>&1; then
  echo "错误：检测到仍在运行的旧节点。请先 Ctrl+C 结束旧 launch，再执行本脚本。" >&2
  exit 3
fi

source /opt/ros/humble/setup.bash

BUILD_TARGET="${WORKSPACE}/build/${PACKAGE_NAME}"
INSTALL_TARGET="${WORKSPACE}/install/${PACKAGE_NAME}"
case "${BUILD_TARGET}" in "${WORKSPACE}/build/${PACKAGE_NAME}") ;; *) exit 4 ;; esac
case "${INSTALL_TARGET}" in "${WORKSPACE}/install/${PACKAGE_NAME}") ;; *) exit 4 ;; esac

# Preserve old products rather than deleting a possibly useful working build.
# Backups outside src plus COLCON_IGNORE avoid duplicate-package discovery.
BACKUP_DIR="${WORKSPACE}/version_backups/$(date +%Y%m%d-%H%M%S)-$$"
mkdir -p -- "${BACKUP_DIR}"
touch "${WORKSPACE}/version_backups/COLCON_IGNORE"
if [ -d "${BUILD_TARGET}" ]; then mv -- "${BUILD_TARGET}" "${BACKUP_DIR}/build"; fi
if [ -d "${INSTALL_TARGET}" ]; then mv -- "${INSTALL_TARGET}" "${BACKUP_DIR}/install"; fi
echo "旧构建产物已保存在：${BACKUP_DIR}（相对symlink可能因移动而失效）"
cd "${WORKSPACE}"
colcon build --packages-select "${PACKAGE_NAME}" --allow-overriding "${PACKAGE_NAME}" \
  --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
source "${WORKSPACE}/install/setup.bash"

echo "构建完成：$(ros2 pkg prefix "${PACKAGE_NAME}")"
echo "请重新启动 launch；运行中的旧进程不会被 source 自动替换。"
echo "期望跟随器版本：2.1.2-geometric-cooperative"
echo "期望规划器版本：2.1.2-straight-route-cooperative"

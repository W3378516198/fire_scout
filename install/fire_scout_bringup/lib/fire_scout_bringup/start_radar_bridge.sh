#!/usr/bin/env bash
set -euo pipefail

SCOUT_NAME="${1:-scout1}"
MODEL_NAME="${2:-x500_lidar_2d_1}"
BRIDGE_CLOCK="${3:-false}"
WORLD_NAME="${FIRE_SCOUT_GZ_WORLD:-fire_scout_floor}"

CONFIG="/tmp/fire_scout_${SCOUT_NAME}_radar_bridge.yaml"

RADAR_RIGHT_GZ="/world/${WORLD_NAME}/model/${MODEL_NAME}/link/radar_right_link/sensor/fire_radar_right_3d/scan/points"
RADAR_LEFT_GZ="/world/${WORLD_NAME}/model/${MODEL_NAME}/link/radar_left_link/sensor/fire_radar_left_3d/scan/points"
RADAR_FRONT_GZ="/world/${WORLD_NAME}/model/${MODEL_NAME}/link/radar_front_link/sensor/fire_radar_front_3d/scan/points"
ULTRASONIC_GZ="/world/${WORLD_NAME}/model/${MODEL_NAME}/link/radar_front_link/sensor/front_ultrasonic/scan"
GT_ODOM_GZ="/model/${MODEL_NAME}/odometry"

cat > "${CONFIG}" <<YAML
- ros_topic_name: "/${SCOUT_NAME}/radar/right/raw_points"
  gz_topic_name: "${RADAR_RIGHT_GZ}"
  ros_type_name: "sensor_msgs/msg/PointCloud2"
  gz_type_name: "gz.msgs.PointCloudPacked"
  direction: GZ_TO_ROS

- ros_topic_name: "/${SCOUT_NAME}/radar/left/raw_points"
  gz_topic_name: "${RADAR_LEFT_GZ}"
  ros_type_name: "sensor_msgs/msg/PointCloud2"
  gz_type_name: "gz.msgs.PointCloudPacked"
  direction: GZ_TO_ROS

- ros_topic_name: "/${SCOUT_NAME}/radar/front/raw_points"
  gz_topic_name: "${RADAR_FRONT_GZ}"
  ros_type_name: "sensor_msgs/msg/PointCloud2"
  gz_type_name: "gz.msgs.PointCloudPacked"
  direction: GZ_TO_ROS

- ros_topic_name: "/${SCOUT_NAME}/ultrasonic/front/scan"
  gz_topic_name: "${ULTRASONIC_GZ}"
  ros_type_name: "sensor_msgs/msg/LaserScan"
  gz_type_name: "gz.msgs.LaserScan"
  direction: GZ_TO_ROS

- ros_topic_name: "/${SCOUT_NAME}/sim/ground_truth_odom"
  gz_topic_name: "${GT_ODOM_GZ}"
  ros_type_name: "nav_msgs/msg/Odometry"
  gz_type_name: "gz.msgs.Odometry"
  direction: GZ_TO_ROS
YAML

if [[ "${BRIDGE_CLOCK}" == "true" || "${BRIDGE_CLOCK}" == "1" ]]; then
cat >> "${CONFIG}" <<YAML

- ros_topic_name: "/clock"
  gz_topic_name: "/clock"
  ros_type_name: "rosgraph_msgs/msg/Clock"
  gz_type_name: "gz.msgs.Clock"
  direction: GZ_TO_ROS
YAML
fi

echo "============================================================"
echo " Fire Scout Gazebo -> ROS2 bridge"
echo " Scout      : ${SCOUT_NAME}"
echo " Gazebo model: ${MODEL_NAME}"
echo " World      : ${WORLD_NAME}"
echo " Clock      : ${BRIDGE_CLOCK}"
echo " GZ_PARTITION=${GZ_PARTITION:-<unset>}"
echo "============================================================"
cat "${CONFIG}"

exec ros2 run ros_gz_bridge parameter_bridge \
  --ros-args -p config_file:="${CONFIG}"
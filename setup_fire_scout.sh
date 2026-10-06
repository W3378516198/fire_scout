#!/bin/bash

source /opt/ros/humble/setup.bash

if [ -f "$HOME/fire_scout_ws/install/setup.bash" ]; then
    source "$HOME/fire_scout_ws/install/setup.bash"
fi

export ROS_DOMAIN_ID=0
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export GZ_PARTITION=fire_scout_sim

unset ROS_LOCALHOST_ONLY

echo "=========================================="
echo " Fire Scout ROS2 environment"
echo " ROS_DOMAIN_ID=$ROS_DOMAIN_ID"
echo " RMW_IMPLEMENTATION=$RMW_IMPLEMENTATION"
echo "=========================================="

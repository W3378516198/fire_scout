#!/usr/bin/env python3

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    share = get_package_share_directory("fire_scout_planning_safe")
    params = os.path.join(share, "config", "safe_airfar.yaml")
    multi = os.path.join(share, "launch", "multi_safe_airfar_navigation.launch.py")
    return LaunchDescription([
        DeclareLaunchArgument("params_file", default_value=params),
        DeclareLaunchArgument("scout_count", default_value="4"),
        DeclareLaunchArgument("first_scout_index", default_value="1"),
        DeclareLaunchArgument("first_target_system", default_value="2"),
        DeclareLaunchArgument("use_sim_time", default_value="true"),
        # multi_scout.launch.py only starts bridges/sensors/localization, so this
        # launch owns one evidence mapper for every requested scout.
        DeclareLaunchArgument("start_mapper", default_value="true"),
        DeclareLaunchArgument("world_frame", default_value="world"),
        DeclareLaunchArgument("publish_world_transforms", default_value="true"),
        DeclareLaunchArgument(
            "scout_origins",
            default_value="-12.5,-9.0,0.2,0.0;-12.5,-1.0,0.2,0.0;"
                          "-12.5,8.0,0.2,0.0;-12.5,17.0,0.2,0.0"),
        DeclareLaunchArgument("fusion_voxel_size", default_value=""),
        DeclareLaunchArgument("fusion_source_timeout", default_value="5.0"),
        DeclareLaunchArgument("fusion_publish_rate", default_value="1.0"),
        # Keep task dispatch and PX4 arming separate for staged validation.
        DeclareLaunchArgument("racer_enabled", default_value="false"),
        DeclareLaunchArgument("racer_task_mode", default_value="goal_guided"),
        DeclareLaunchArgument("start_passage_coordinator", default_value="true"),
        DeclareLaunchArgument("fire_goal_topic", default_value="/fire_goal"),
        DeclareLaunchArgument("auto_arm", default_value="false"),
        DeclareLaunchArgument("startup_spread_enabled", default_value="true"),
        DeclareLaunchArgument("startup_spread_spacing", default_value=""),
        DeclareLaunchArgument("startup_spread_distance", default_value=""),
        DeclareLaunchArgument("startup_spread_speed", default_value="0.80"),
        DeclareLaunchArgument("vehicle_wheelbase", default_value=""),
        DeclareLaunchArgument("propeller_diameter", default_value=""),
        DeclareLaunchArgument("max_speed_xy", default_value=""),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(multi),
            launch_arguments={
                "params_file": LaunchConfiguration("params_file"),
                "scout_count": LaunchConfiguration("scout_count"),
                "first_scout_index": LaunchConfiguration("first_scout_index"),
                "first_target_system": LaunchConfiguration("first_target_system"),
                "use_sim_time": LaunchConfiguration("use_sim_time"),
                "start_mapper": LaunchConfiguration("start_mapper"),
                "start_fusion": "true",
                "start_racer": "true",
                "racer_enabled": LaunchConfiguration("racer_enabled"),
                "racer_task_mode": LaunchConfiguration("racer_task_mode"),
                "start_passage_coordinator": LaunchConfiguration("start_passage_coordinator"),
                "fire_goal_topic": LaunchConfiguration("fire_goal_topic"),
                "start_planner": "true",
                "start_follower": "true",
                "auto_arm": LaunchConfiguration("auto_arm"),
                "startup_spread_enabled": LaunchConfiguration("startup_spread_enabled"),
                "startup_spread_spacing": LaunchConfiguration("startup_spread_spacing"),
                "startup_spread_distance": LaunchConfiguration("startup_spread_distance"),
                "startup_spread_speed": LaunchConfiguration("startup_spread_speed"),
                "world_frame": LaunchConfiguration("world_frame"),
                "publish_world_transforms": LaunchConfiguration("publish_world_transforms"),
                "scout_origins": LaunchConfiguration("scout_origins"),
                "fusion_voxel_size": LaunchConfiguration("fusion_voxel_size"),
                "fusion_source_timeout": LaunchConfiguration("fusion_source_timeout"),
                "fusion_publish_rate": LaunchConfiguration("fusion_publish_rate"),
                "vehicle_wheelbase": LaunchConfiguration("vehicle_wheelbase"),
                "propeller_diameter": LaunchConfiguration("propeller_diameter"),
                "max_speed_xy": LaunchConfiguration("max_speed_xy"),
            }.items(),
        ),
    ])

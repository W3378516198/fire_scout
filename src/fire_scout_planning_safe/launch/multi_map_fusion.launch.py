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
        DeclareLaunchArgument("use_sim_time", default_value="true"),
        DeclareLaunchArgument("world_frame", default_value="world"),
        DeclareLaunchArgument("publish_world_transforms", default_value="true"),
        DeclareLaunchArgument(
            "scout_origins",
            default_value="-12.5,-9.0,0.2,0.0;-12.5,-1.0,0.2,0.0;"
                          "-12.5,8.0,0.2,0.0;-12.5,17.0,0.2,0.0"),
        DeclareLaunchArgument("fusion_voxel_size", default_value=""),
        DeclareLaunchArgument("fusion_source_timeout", default_value="5.0"),
        DeclareLaunchArgument("fusion_publish_rate", default_value="1.0"),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(multi),
            launch_arguments={
                "params_file": LaunchConfiguration("params_file"),
                "scout_count": LaunchConfiguration("scout_count"),
                "first_scout_index": LaunchConfiguration("first_scout_index"),
                "use_sim_time": LaunchConfiguration("use_sim_time"),
                "start_mapper": "false",
                "start_fusion": "true",
                "start_racer": "false",
                "racer_enabled": "false",
                "start_planner": "false",
                "start_follower": "false",
                "auto_arm": "false",
                "world_frame": LaunchConfiguration("world_frame"),
                "publish_world_transforms": LaunchConfiguration("publish_world_transforms"),
                "scout_origins": LaunchConfiguration("scout_origins"),
                "fusion_voxel_size": LaunchConfiguration("fusion_voxel_size"),
                "fusion_source_timeout": LaunchConfiguration("fusion_source_timeout"),
                "fusion_publish_rate": LaunchConfiguration("fusion_publish_rate"),
            }.items(),
        ),
    ])

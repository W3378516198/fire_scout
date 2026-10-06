"""Use the planning package's evidence map as the single navigation map.

The original radar_mapping.launch.py remains available for the historical
left/right accumulated cloud. Do not run both when visualization_topic names
the same output. A nav launch paired with this one must use start_mapper:=false.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    default_params = os.path.join(
        get_package_share_directory("fire_scout_planning_safe"),
        "config", "safe_airfar.yaml",
    )
    return LaunchDescription([
        DeclareLaunchArgument("params_file", default_value=default_params),
        DeclareLaunchArgument("use_sim_time", default_value="true"),
        DeclareLaunchArgument(
            "visualization_topic", default_value="/scout1/map/radar_cloud",
            description="Occupied snapshot for RViz; never used as free-space evidence",
        ),
        Node(
            package="fire_scout_planning_safe",
            executable="radar_free_space_mapper",
            name="radar_free_space_mapper",
            output="screen",
            parameters=[LaunchConfiguration("params_file"), {
                "use_sim_time": ParameterValue(
                    LaunchConfiguration("use_sim_time"), value_type=bool),
                "visualization_topic": ParameterValue(
                    LaunchConfiguration("visualization_topic"), value_type=str),
            }],
        ),
    ])

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory
import os
import yaml


def _nodes(context):
    params = LaunchConfiguration("params_file").perform(context)
    start_mapper = LaunchConfiguration("start_mapper").perform(context).strip().lower()
    if start_mapper not in ("true", "false", "1", "0"):
        raise ValueError("start_mapper must be true or false")
    shared = {"use_sim_time": ParameterValue(LaunchConfiguration("use_sim_time"), value_type=bool)}
    # Empty CLI defaults preserve a custom YAML file's dimensions.
    for name in ("vehicle_wheelbase", "propeller_diameter"):
        value = LaunchConfiguration(name).perform(context)
        if value:
            shared[name] = float(value)
    with open(params, encoding="utf-8") as stream:
        settings = yaml.safe_load(stream) or {}
    profile = dict(settings.get("/**", {}).get("ros__parameters", {}))
    profile.update(settings.get("safe_airfar_path_follower", {}).get("ros__parameters", {}))
    result = []
    for executable in ['radar_free_space_mapper', 'safe_airfar_like_planner', 'safe_airfar_path_follower']:
        if executable == "radar_free_space_mapper" and start_mapper in ("false", "0"):
            continue
        overrides = dict(shared)
        if executable == "safe_airfar_like_planner":
            for key in ("lookahead", "max_speed_xy", "max_accel_xy", "max_jerk_xy",
                        "max_lateral_accel", "max_yaw_rate", "sharp_turn_stop_deg"):
                if key in profile:
                    overrides[key] = profile[key]
        if executable in ("safe_airfar_path_follower", "safe_airfar_like_planner"):
            speed = LaunchConfiguration("max_speed_xy").perform(context)
            if speed:
                overrides["max_speed_xy"] = float(speed)
        if executable == "safe_airfar_path_follower":
            overrides["passage_scheduler_enabled"] = False
            overrides["auto_arm"] = ParameterValue(LaunchConfiguration("auto_arm"), value_type=bool)
        result.append(Node(package="fire_scout_planning_safe", executable=executable,
                           name=executable, output="screen", parameters=[params, overrides]))
    return result


def generate_launch_description():
    default = os.path.join(get_package_share_directory("fire_scout_planning_safe"), "config", "safe_airfar.yaml")
    return LaunchDescription([
        DeclareLaunchArgument("params_file", default_value=default),
        DeclareLaunchArgument("use_sim_time", default_value="true"),
        DeclareLaunchArgument("start_mapper", default_value="true", description="false when planning_map.launch.py already runs the evidence mapper"),
        DeclareLaunchArgument("auto_arm", default_value="true"),
        DeclareLaunchArgument("vehicle_wheelbase", default_value="", description="Opposite motor-axis spacing in metres"),
        DeclareLaunchArgument("propeller_diameter", default_value="", description="Propeller diameter in metres"),
        DeclareLaunchArgument("max_speed_xy", default_value="", description="Straight-line speed ceiling in m/s"),
        OpaqueFunction(function=_nodes),
    ])

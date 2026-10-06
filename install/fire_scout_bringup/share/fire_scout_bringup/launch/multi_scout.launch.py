#!/usr/bin/env python3

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_prefix


def _make_nodes(context):
    count = int(LaunchConfiguration("scout_count").perform(context))
    first_model_index = int(LaunchConfiguration("first_model_index").perform(context))
    use_sim_time_text = LaunchConfiguration("use_sim_time").perform(context).strip().lower()
    use_rviz_text = LaunchConfiguration("use_rviz").perform(context).strip().lower()
    accept_frd_text = LaunchConfiguration(
        "accept_frd_pose_frame").perform(context).strip().lower()
    if use_sim_time_text not in ("true", "false", "1", "0"):
        raise ValueError("use_sim_time must be true or false")
    if use_rviz_text not in ("true", "false", "1", "0"):
        raise ValueError("use_rviz must be true or false")
    if accept_frd_text not in ("true", "false", "1", "0"):
        raise ValueError("accept_frd_pose_frame must be true or false")
    if count < 1:
        raise ValueError("scout_count must be >= 1")
    if first_model_index < 0:
        raise ValueError("first_model_index must be >= 0")
    use_sim_time = use_sim_time_text in ("true", "1")
    use_rviz = use_rviz_text in ("true", "1")
    accept_frd = accept_frd_text in ("true", "1")
    odom_input_timeout = float(
        LaunchConfiguration("odom_input_timeout").perform(context))
    rviz_delay = float(LaunchConfiguration("rviz_delay").perform(context))
    if odom_input_timeout <= 0.0:
        raise ValueError("odom_input_timeout must be > 0")
    if rviz_delay < 0.0 or rviz_delay > 30.0:
        raise ValueError("rviz_delay must be within 0..30 seconds")

    bringup_prefix = get_package_prefix("fire_scout_bringup")
    radar_bridge_script = os.path.join(
        bringup_prefix, "lib", "fire_scout_bringup", "start_radar_bridge.sh")

    actions = []

    for n in range(1, count + 1):
        scout = f"scout{n}"
        model_index = first_model_index + (n - 1)
        model_name = f"x500_lidar_2d_{model_index}"

        # One bridge process per simulated UAV. Only scout1 bridges /clock.
        actions.append(ExecuteProcess(
            cmd=[radar_bridge_script, scout, model_name,
                 "true" if n == 1 else "false"],
            output="screen",
            respawn=True,
            respawn_delay=2.0,
        ))

        # PX4 VehicleOdometry -> ROS ENU odometry + TF.
        actions.append(Node(
            package="fire_scout_localization",
            executable="px4_odom_adapter",
            namespace=scout,
            name="px4_odom_adapter",
            output="screen",
            respawn=True,
            respawn_delay=2.0,
            parameters=[{
                "use_sim_time": use_sim_time,
                "input_topic": f"/{scout}/fmu/out/vehicle_odometry",
                "output_topic": f"/{scout}/odom",
                "odom_frame": f"{scout}/odom",
                "base_frame": f"{scout}/base_link",
                "accept_frd_pose_frame": accept_frd,
                "input_timeout": odom_input_timeout,
                "diagnostic_period": 1.0,
            }],
        ))

        # Static sensor extrinsics.
        tf_specs = [
            ("radar_right_static_tf", "0.0", "-0.15", "0.12",
             "-1.5707963267948966", f"{scout}/radar_right_link"),
            ("radar_left_static_tf", "0.0", "0.15", "0.12",
             "1.5707963267948966", f"{scout}/radar_left_link"),
            ("radar_front_static_tf", "0.15", "0.0", "0.12",
             "0.0", f"{scout}/radar_front_link"),
            ("ultrasonic_front_static_tf", "0.20", "0.0", "0.07",
             "0.0", f"{scout}/ultrasonic_front_link"),
        ]
        for name, x, y, z, yaw, child in tf_specs:
            actions.append(Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                namespace=scout,
                name=name,
                output="screen",
                arguments=[
                    "--x", x, "--y", y, "--z", z,
                    "--yaw", yaw, "--pitch", "0.0", "--roll", "0.0",
                    "--frame-id", f"{scout}/base_link",
                    "--child-frame-id", child,
                ],
            ))

        common_mmwave = {
            "use_sim_time": use_sim_time,
            "odom_topic": f"/{scout}/sim/ground_truth_odom",
            "max_odom_time_diff": 0.03,
            "doppler_noise_std": 0.05,
            "snr_at_1m_db": 35.0,
            "snr_noise_std_db": 1.5,
            "min_snr_db": 5.0,
            "max_snr_db": 40.0,
            "rcs_mean_dbsm": 0.0,
            "rcs_std_db": 3.0,
        }

        mmwave_specs = [
            ("front_mmwave_enricher", "front", f"{scout}/radar_front_link",
             0.15, 0.0, 0.12, 0.0),
            ("right_mmwave_enricher", "right", f"{scout}/radar_right_link",
             0.0, -0.15, 0.12, -1.5707963267948966),
            ("left_mmwave_enricher", "left", f"{scout}/radar_left_link",
             0.0, 0.15, 0.12, 1.5707963267948966),
        ]

        for node_name, side, output_frame, rx, ry, rz, ryaw in mmwave_specs:
            params = dict(common_mmwave)
            params.update({
                "input_topic": f"/{scout}/radar/{side}/raw_points",
                "output_topic": f"/{scout}/radar/{side}/mmwave_points",
                "output_frame": output_frame,
                "radar_x": rx,
                "radar_y": ry,
                "radar_z": rz,
                "radar_yaw": ryaw,
            })
            actions.append(Node(
                package="fire_scout_sensors",
                executable="mmwave_enricher",
                namespace=scout,
                name=node_name,
                output="screen",
                parameters=[params],
            ))

        actions.append(Node(
            package="fire_scout_sensors",
            executable="ultrasonic_to_px4",
            namespace=scout,
            name="ultrasonic_to_px4",
            output="screen",
            parameters=[{
                "use_sim_time": use_sim_time,
                "input_topic": f"/{scout}/ultrasonic/front/scan",
                "output_topic": f"/{scout}/fmu/in/distance_sensor",
                "min_distance": 0.02,
                "max_distance": 5.0,
            }],
        ))

    if use_rviz:
        actions.append(TimerAction(
            period=rviz_delay,
            actions=[Node(
                package="rviz2",
                executable="rviz2",
                name="rviz2",
                output="screen",
                parameters=[{"use_sim_time": use_sim_time}],
            )],
        ))

    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("scout_count", default_value="4"),
        # With PX4 instances -i 1,-i 2,-i 3,-i 4, Gazebo model names are
        # normally x500_lidar_2d_1 ... x500_lidar_2d_4.
        DeclareLaunchArgument("first_model_index", default_value="1"),
        DeclareLaunchArgument("use_sim_time", default_value="true"),
        DeclareLaunchArgument("use_rviz", default_value="true"),
        # PX4 may legitimately use a fixed FRD local frame before/without
        # globally aligned heading. The adapter converts it to a ROS FLU frame.
        DeclareLaunchArgument("accept_frd_pose_frame", default_value="true"),
        DeclareLaunchArgument("odom_input_timeout", default_value="2.0"),
        # Avoid RViz reporting a transient disconnected tree before /clock and
        # the first PX4 odometry samples arrive.
        DeclareLaunchArgument("rviz_delay", default_value="2.0"),
        OpaqueFunction(function=_make_nodes),
    ])

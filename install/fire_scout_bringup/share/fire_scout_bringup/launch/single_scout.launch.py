#!/usr/bin/env python3

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_prefix


def generate_launch_description():
    use_rviz = LaunchConfiguration("use_rviz")

    bringup_prefix = get_package_prefix("fire_scout_bringup")
    radar_bridge_script = os.path.join(
        bringup_prefix,
        "lib",
        "fire_scout_bringup",
        "start_radar_bridge.sh",
    )

    radar_bridge = ExecuteProcess(
        cmd=[radar_bridge_script, "scout1"],
        output="screen",
    )

    localization = Node(
        package="fire_scout_localization",
        executable="px4_odom_adapter",
        name="px4_odom_adapter",
        output="screen",
        parameters=[{
            "use_sim_time": True,
            "input_topic": "/scout1/fmu/out/vehicle_odometry",
            "output_topic": "/scout1/odom",
            "odom_frame": "scout1/odom",
            "base_frame": "scout1/base_link",
        }],
    )

    radar_right_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="radar_right_static_tf",
        output="screen",
        arguments=[
            "--x", "0.0",
            "--y", "-0.15",
            "--z", "0.12",
            "--yaw", "-1.5707963267948966",
            "--pitch", "0.0",
            "--roll", "0.0",
            "--frame-id", "scout1/base_link",
            "--child-frame-id", "scout1/radar_right_link",
        ],
    )

    radar_left_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="radar_left_static_tf",
        output="screen",
        arguments=[
            "--x", "0.0",
            "--y", "0.15",
            "--z", "0.12",
            "--yaw", "1.5707963267948966",
            "--pitch", "0.0",
            "--roll", "0.0",
            "--frame-id", "scout1/base_link",
            "--child-frame-id", "scout1/radar_left_link",
        ],
    )

    right_mmwave = Node(
        package="fire_scout_sensors",
        executable="mmwave_enricher",
        name="right_mmwave_enricher",
        output="screen",
        parameters=[{
            "use_sim_time": True,
            "input_topic": "/scout1/radar/right/raw_points",
            "odom_topic": "/scout1/sim/ground_truth_odom",
            "output_topic": "/scout1/radar/right/mmwave_points",
            "output_frame": "scout1/radar_right_link",
            "radar_x": 0.0,
            "radar_y": -0.15,
            "radar_z": 0.12,
            "radar_yaw": -1.5707963267948966,
            "max_odom_time_diff": 0.03,
            "doppler_noise_std": 0.05,
            "snr_at_1m_db": 35.0,
            "snr_noise_std_db": 1.5,
            "min_snr_db": 5.0,
            "max_snr_db": 40.0,
            "rcs_mean_dbsm": 0.0,
            "rcs_std_db": 3.0,
        }],
    )

    left_mmwave = Node(
        package="fire_scout_sensors",
        executable="mmwave_enricher",
        name="left_mmwave_enricher",
        output="screen",
        parameters=[{
            "use_sim_time": True,
            "input_topic": "/scout1/radar/left/raw_points",
            "odom_topic": "/scout1/sim/ground_truth_odom",
            "output_topic": "/scout1/radar/left/mmwave_points",
            "output_frame": "scout1/radar_left_link",
            "radar_x": 0.0,
            "radar_y": 0.15,
            "radar_z": 0.12,
            "radar_yaw": 1.5707963267948966,
            "max_odom_time_diff": 0.03,
            "doppler_noise_std": 0.05,
            "snr_at_1m_db": 35.0,
            "snr_noise_std_db": 1.5,
            "min_snr_db": 5.0,
            "max_snr_db": 40.0,
            "rcs_mean_dbsm": 0.0,
            "rcs_std_db": 3.0,
        }],
    )

    ultrasonic = Node(
        package="fire_scout_sensors",
        executable="ultrasonic_to_px4",
        name="ultrasonic_to_px4",
        output="screen",
        parameters=[{
            "input_topic":
            "/scout1/ultrasonic/front/scan",

            "output_topic":
            "/scout1/fmu/in/distance_sensor",

            "min_distance":
            0.02,

            "max_distance":
            5.0
        }]
    )

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        parameters=[{"use_sim_time": True}],
        condition=IfCondition(use_rviz),
    )

    

    radar_front_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="radar_front_static_tf",
        arguments=[
            "0.15",
            "0",
            "0.12",
            "0",
            "0",
            "0",
            "scout1/base_link",
            "scout1/radar_front_link"
        ],
    )


    front_mmwave = Node(
        package="fire_scout_sensors",
        executable="mmwave_enricher",
        name="front_mmwave_enricher",
        parameters=[{
            "input_topic":
                "/scout1/radar/front/raw_points",

            "output_topic":
                "/scout1/radar/front/mmwave_points",

            "output_frame":
                "scout1/radar_front_link",

            "radar_x":0.15,
            "radar_y":0.0,
            "radar_z":0.12,
            "radar_yaw":0.0,

            "max_odom_time_diff":0.03,
            "doppler_noise_std":0.05,
            "snr_at_1m_db":35.0,
            "snr_noise_std_db":1.5,
            "min_snr_db":5.0,
            "max_snr_db":40.0,
            "rcs_mean_dbsm":0.0,
            "rcs_std_db":3.0
        }],
    )



    ultrasonic_front_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="ultrasonic_front_static_tf",
        output="screen",
        arguments=[
            "--x", "0.20",
            "--y", "0.0",
            "--z", "0.07",
            "--yaw", "0.0",
            "--pitch", "0.0",
            "--roll", "0.0",
            "--frame-id", "scout1/base_link",
            "--child-frame-id", "scout1/ultrasonic_front_link",
        ],
    )


    return LaunchDescription([
        DeclareLaunchArgument("use_rviz", default_value="true"),
        radar_bridge,
        localization,
        radar_right_tf,
        radar_left_tf,
        radar_front_tf,
        ultrasonic_front_tf,
        front_mmwave,
        right_mmwave,
        left_mmwave,
        ultrasonic,
        rviz,
    ])

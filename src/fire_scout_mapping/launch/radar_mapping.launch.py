from launch import LaunchDescription

from launch_ros.actions import Node


def generate_launch_description():

    radar_map_builder = Node(
        package='fire_scout_mapping',
        executable='radar_map_builder',
        name='radar_map_builder',
        output='screen',

        parameters=[{
            'use_sim_time': True,

            'target_frame':
                'scout1/odom',

            'left_topic':
                '/scout1/radar/left/mmwave_points',

            'right_topic':
                '/scout1/radar/right/mmwave_points',

            'map_topic':
                '/scout1/map/radar_cloud',

            'voxel_size':
                0.15,

            'max_voxels':
                300000,

            'publish_rate_hz':
                2.0,

            'tf_timeout_sec':
                0.10,
        }]
    )

    return LaunchDescription([
        radar_map_builder
    ])

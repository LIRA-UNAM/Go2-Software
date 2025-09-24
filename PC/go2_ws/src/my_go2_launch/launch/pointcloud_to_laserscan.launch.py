from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='pointcloud_to_laserscan',
            executable='pointcloud_to_laserscan_node',
            name='pointcloud_to_laserscan',
            remappings=[
                ('cloud_in', '/utlidar/cloud_deskewed'),
                ('scan', '/utlidar/scan'),
            ],
            parameters=[{
                'target_frame': 'odom',
                'transform_tolerance': 0.01,
                'min_height': -0.1,
                'max_height': 0.1,
                'angle_min': -3.14,
                'angle_max': 3.14,
                'angle_increment': 0.003,
                'scan_time': 0.1,
                'range_min': 0.05,
                'range_max': 30.0,
                'use_inf': True,
                'inf_epsilon': 1.0,
            }]
        )
    ])

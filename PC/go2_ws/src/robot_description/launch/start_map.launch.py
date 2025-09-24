from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='slam_toolbox',
            executable='sync_slam_toolbox_node',
            name='slam_toolbox',
            output='screen',
            parameters=[{
                'use_sim_time': False,
                'odom_frame': 'odom',
                'map_frame': 'map',
                'base_frame': 'base_link',
                'scan_topic': '/scan',          # coincide con el tópico del LIDAR
                'resolution': 0.05,
                'max_laser_range': 64.0,        # <= range_max del scan
                'minimum_time_interval': 0.05,  # 20 Hz
                'transform_publish_period': 0.05,
                'map_update_interval': 0.1,
                'minimum_travel_distance': 0.2,
                'use_scan_matching': True,
                'use_odometry': True            # pon False si no tienes odometría
            }]
        ),
    ])
#!/usr/bin/env python3
"""
Launch completo para crear mapas con SLAM.
Incluye: robot, odom, joystick, cámara, RViz y slam_toolbox.
NO incluye map_server ni AMCL (modo mapeo).
"""
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
import os


def generate_launch_description():
    urdf_file = os.path.join(
        get_package_share_directory('go2_description'),
        'urdf',
        'go2_description.urdf'
    )
    with open(urdf_file, 'r') as infp:
        robot_desc = infp.read()

    multicast_iface = LaunchConfiguration('multicast_iface')

    return LaunchDescription([
        DeclareLaunchArgument(
            'multicast_iface',
            default_value='enx6c1ff767936e',
            description='Interfaz de red para el stream de video del robot'
        ),

        # --- Robot model y estado ---
        Node(
            package='go2_description',
            executable='joint_state_relay',
            name='joint_state_relay',
            output='screen'
        ),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_desc}]
        ),

        # --- Teleoperación ---
        Node(
            package='joystick_teleop',
            executable='joystick_teleop',
            name='joystick_teleop',
            output='screen'
        ),

        # --- Odometría y TF ---
        Node(
            package='my_go2_launch',
            executable='odom_publisher',
            name='odom_to_tf_broadcaster',
            output='screen'
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='baselink_to_base_broadcaster',
            output='screen',
            arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'base']
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_tf_pub_laser',
            output='screen',
            arguments=['-0.15', '0', '0.05', '0', '3.14', '0', 'Head_upper', 'laser_frame']
        ),

        # --- Visualización ---
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen'
        ),

        # --- Cámara del robot ---
        Node(
            package='my_go2_launch',
            executable='img_publisher',
            name='img_publisher',
            output='screen',
            parameters=[{'multicast_iface': multicast_iface}]
        ),

        # --- SLAM (crea el mapa en tiempo real) ---
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
                'scan_topic': '/scan',
                'resolution': 0.05,
                'max_laser_range': 30.0,
                'minimum_time_interval': 0.05,
                'transform_publish_period': 0.05,
                'map_update_interval': 0.1,
                'minimum_travel_distance': 0.2,
                'use_scan_matching': True,
                'use_odometry': True
            }]
        ),
    ])
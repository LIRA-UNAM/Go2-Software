#!/usr/bin/env python3
"""
Go2 robot model en PC: joint_state_relay, robot_state_publisher, TF base_link->base, Head_upper->laser_frame.
Se ejecuta en el PC. Suscribe a /lowstate (robot) y publica /joint_states, /robot_description y TF.
unitree_go y go2_description están en el workspace del PC.
"""
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    urdf_file = os.path.join(
        get_package_share_directory('go2_description'),
        'urdf',
        'go2_description.urdf'
    )
    with open(urdf_file, 'r') as infp:
        robot_desc = infp.read()

    return LaunchDescription([
        # joint_state_relay: /lowstate (robot) -> /joint_states
        Node(
            package='go2_description',
            executable='joint_state_relay',
            name='joint_state_relay',
            output='screen',
            respawn=True,
            parameters=[{'lowstate_topic': '/lowstate'}]
        ),
        # robot_state_publisher: /joint_states -> TF base->legs, /robot_description
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            respawn=True,
            parameters=[{'robot_description': robot_desc}]
        ),
        # TF base_link -> base (conecta odom->base_link con el árbol del robot)
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='baselink_to_base_broadcaster',
            output='screen',
            arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'base']
        ),
        # TF Head_upper -> laser_frame (YDLidar en el robot)
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_tf_pub_laser',
            output='screen',
            arguments=['-0.15', '0', '0.05', '0', '3.14', '0', 'Head_upper', 'laser_frame']
        ),
    ])

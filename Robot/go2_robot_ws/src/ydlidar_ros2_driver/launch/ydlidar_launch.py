#!/usr/bin/python3
# Copyright 2020, EAIBOT
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import LifecycleNode
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

import os


def generate_launch_description():
    share_dir = get_package_share_directory('ydlidar_ros2_driver')
    parameter_file = LaunchConfiguration('params_file')
    node_name = 'ydlidar_ros2_driver_node'

    params_declare = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(share_dir, 'params', 'TG.yaml'),
        description='Path to the ROS2 parameters file to use.'
    )

    driver_node = LifecycleNode(
        package='ydlidar_ros2_driver',
        executable='ydlidar_ros2_driver_node',   # en Foxy es "executable" (no "node_executable")
        name=node_name,
        output='screen',
        emulate_tty=True,
        parameters=[parameter_file],
        namespace='/'
    )

    joy_node = Node(
        package='joy',
        executable='joy_node',
        name='joy_node',
        output='screen'
    )

    return LaunchDescription([
        params_declare,
        driver_node,
        joy_node
    ])

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
    gait_type = LaunchConfiguration('gait_type', default='2')
    foot_raise_height = LaunchConfiguration('foot_raise_height', default='0.04')
    speed_level = LaunchConfiguration('speed_level', default='0')
    continuous_gait = LaunchConfiguration('continuous_gait', default='true')
    share_dir = get_package_share_directory('ydlidar_ros2_driver')
    parameter_file = LaunchConfiguration('params_file')
    node_name = 'ydlidar_ros2_driver_node'

    params_declare = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(share_dir, 'params', 'TG.yaml'),
        description='Path to the ROS2 parameters file to use.'
    )
    gait_declare = DeclareLaunchArgument(
        'gait_type', default_value='2',
        description='Gait: 0=classic, 1=flat, 2=economic (pasos cortos, poco salto)'
    )
    foot_raise_declare = DeclareLaunchArgument(
        'foot_raise_height', default_value='0.04',
        description='Altura elevacion patas (m). Bajo=menos salto. Rango ~0.04-0.12'
    )
    speed_declare = DeclareLaunchArgument(
        'speed_level', default_value='0',
        description='Nivel velocidad: -1,0,1,2. Bajo=pasos mas cortos'
    )
    continuous_declare = DeclareLaunchArgument(
        'continuous_gait', default_value='true',
        description='true=run (pasos cortos), false=walk (pasos grandes). Con foot_raise bajo reduce salto'
    )

    # publish_tf:=false fijo: el PC publica odom->base_link (odom_publisher o odom_filter).
    # Asi el robot no cambia; el launch de navegacion en PC elige RF2O o odom robot.
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

    # Bridge cmd_vel -> Unitree Sport API (hace que el robot camine)
    # Marcha: continuous_gait=true (pasos cortos) + foot_raise=0.04 (poco salto)
    dogbase_node = Node(
        package='go2_demo',
        executable='dogbase',
        name='dog_base_node',
        output='screen',
        emulate_tty=True,
        parameters=[{
            'publish_tf': False,
            'gait_type': gait_type,
            'foot_raise_height': foot_raise_height,
            'speed_level': speed_level,
            'continuous_gait': continuous_gait
        }]
    )

    return LaunchDescription([
        params_declare,
        gait_declare,
        foot_raise_declare,
        speed_declare,
        continuous_declare,
        driver_node,
        joy_node,
        dogbase_node
    ])

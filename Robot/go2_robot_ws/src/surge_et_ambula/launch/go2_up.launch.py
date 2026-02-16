"""
go2_up: bringup del robot (ydlidar, dogbase, joystick, cámara, mapas).
Se ejecuta en el ROBOT. Publica /map, /fixed_prohibition_layer_map, /lowstate, /scan, /utlidar/robot_odom.
joint_state_relay, robot_state_publisher y TF base_link->base->laser_frame se ejecutan en el PC (go2_robot_model).
"""
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, TextSubstitution
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    surge_share = get_package_share_directory('surge_et_ambula')
    map_name = LaunchConfiguration('map_name', default='my_house')

    ydlidar_launch = os.path.join(
        get_package_share_directory('ydlidar_ros2_driver'),
        'launch',
        'ydlidar_launch.py'
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'map_name',
            default_value='my_house',
            description='Nombre del mapa (my_house, lab, etc.)'
        ),
        DeclareLaunchArgument(
            'launch_rviz2',
            default_value='false',
            description='Robot suele ser headless. true solo si tiene pantalla.'
        ),
        # Lidar, joy_node, dogbase (publica /lowstate, /scan, /utlidar/robot_odom; recibe /cmd_vel)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(ydlidar_launch)
        ),

        Node(
            package='joystick_teleop',
            executable='joystick_teleop',
            name='joystick_teleop',
        ),

        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            condition=IfCondition(LaunchConfiguration('launch_rviz2')),
        ),

        Node(
            package='my_go2_launch',
            executable='img_publisher',
            name='img_publisher',
            output='screen',
            additional_env={'LD_PRELOAD': '/lib/aarch64-linux-gnu/libgomp.so.1'},
            parameters=[{
                'gstreamer_pipeline': (
                    "udpsrc address=230.1.1.1 port=1720 multicast-iface=eth0 ! "
                    "application/x-rtp, media=video, encoding-name=H264 ! "
                    "rtph264depay ! h264parse ! avdec_h264 ! videoconvert ! "
                    "video/x-raw,width=1280,height=720,format=BGR ! appsink name=mysink drop=1 emit-signals=true"
                ),
                'topic_name': '/camera/image_raw',
                'frame_id': 'camera_link',
            }]
        ),

        # Map servers: cargan de surge_et_ambula y publican /map y /fixed_prohibition_layer_map
        # Navegación en PC se suscribe a estos topics (no carga archivos)
        Node(
            package='nav2_map_server',
            executable='map_server',
            name='static_map_server',
            output='screen',
            respawn=True,
            parameters=[{'yaml_filename': [TextSubstitution(text=surge_share), TextSubstitution(text='/maps/'), map_name, TextSubstitution(text='/map.yaml')]}]
        ),

        Node(
            package='nav2_map_server',
            executable='map_server',
            name='prohibition_map_server',
            output='screen',
            respawn=True,
            parameters=[{'yaml_filename': [TextSubstitution(text=surge_share), TextSubstitution(text='/maps/'), map_name, TextSubstitution(text='/prohibition_map.yaml')]}],
            remappings=[('/map', '/fixed_prohibition_layer_map')]
        ),

        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_map',
            output='screen',
            respawn=True,
            parameters=[{
                'use_sim_time': False,
                'autostart': True,
                'node_names': ['static_map_server', 'prohibition_map_server']
            }]
        ),
    ])

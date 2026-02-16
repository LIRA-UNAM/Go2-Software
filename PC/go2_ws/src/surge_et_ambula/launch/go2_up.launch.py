"""
go2_up: robot en PC (TF, joystick, camara).
Siempre igual; no depende de RF2O vs odom robot.
La odometria (odom_publisher o RF2O+odom_filter) la lanzan los launches de navegacion.
"""
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.conditions import IfCondition
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

    return LaunchDescription([
        DeclareLaunchArgument(
            'launch_rviz2',
            default_value='true',
            description='Si false, no lanzar rviz2 (cuando se incluye desde navegacion que ya lanza RViz).'
        ),

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

        Node(
            package='joystick_teleop',
            executable='joystick_teleop',
            name='joystick_teleop',
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
            parameters=[{
                'gstreamer_pipeline': "udpsrc address=230.1.1.1 port=1720 multicast-iface=enx207bd2565bdb ! "
                                       "application/x-rtp, media=video, encoding-name=H264 ! "
                                       "rtph264depay ! h264parse ! avdec_h264 ! videoconvert ! "
                                       "video/x-raw,width=1280,height=720,format=BGR ! appsink drop=1",
                'frame_id': 'camera_link'
            }]
        ),
    ])

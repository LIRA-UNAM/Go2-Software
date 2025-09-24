from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    urdf_file = LaunchConfiguration('urdf_file')

    return LaunchDescription([
        DeclareLaunchArgument(
            'urdf_file',
            default_value='go2_description.urdf',
            description='/home/unitree/ros2_ws/src/robot_description/urdf/go2_description.urdf'
        ),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            parameters=[{'robot_description': open(urdf_file.perform({})).read()}]
        ),
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui'
        )
    ])

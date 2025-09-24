from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Ruta del URDF
    urdf_file = os.path.join(
        get_package_share_directory('robot_description'),
        'urdf',
        'go2_description.urdf'
    )

    with open(urdf_file, 'r') as infp:
        robot_desc = infp.read()

    return LaunchDescription([
        # Publicador del estado del robot
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_desc}]
        ),

        # GUI para mover articulaciones
        Node(
            package='robot_description',
            executable='joint_state_publisher_gui',
            name='joint_state_publisher_gui',
            output='screen'
        ),

        # RViz2
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen'
        ),

        # Nodo para recibir video por GStreamer y publicarlo como imagen
        Node(
            package='my_go2_launch',
            executable='gstreamer_image_publisher',  # nombre del ejecutable que pusiste en setup.py
            name='gstreamer_image_publisher',
            output='screen',
            parameters=[{
                'gstreamer_pipeline': "udpsrc address=230.1.1.1 port=1720 multicast-iface=enx207bd2565bdb ! "
                                       "application/x-rtp, media=video, encoding-name=H264 ! "
                                       "rtph264depay ! h264parse ! avdec_h264 ! videoconvert ! "
                                       "video/x-raw,width=1280,height=720,format=BGR ! appsink drop=1",
                'frame_id': 'camera_link'
            }]
        ),

        # Nodo para convertir PointCloud2 a LaserScan con QoS ajustable
        Node(
            package='my_go2_launch',
            executable='pointcloud_to_laserscan_qos',
            name='pointcloud_to_laserscan_qos',
            output='screen',
            parameters=[{
                'target_frame': 'odom',
                'transform_tolerance': 0.01,
                'min_height': 0.2,
                'max_height': 0.5,
                'angle_min': -3.14,
                'angle_max': 3.14,
                'angle_increment': 0.0087,
                'scan_time': 0.033,
                'range_min': 0.45,
                'range_max': 10.0,
                'use_inf': True
            }]
        )
    ])

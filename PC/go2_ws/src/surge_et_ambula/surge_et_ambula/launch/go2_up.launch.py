from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.conditions import UnlessCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, TextSubstitution
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_share = get_package_share_directory('surge_et_ambula')

    # --- Arg: subcarpeta del mapa (lab, my_house, etc.) ---
    map_name = LaunchConfiguration('map_name')
    map_config_file = PathJoinSubstitution([
        TextSubstitution(text=pkg_share),
        TextSubstitution(text='maps'),
        map_name,
        TextSubstitution(text='map.yaml')
    ])

    # Ruta del URDF
    urdf_file = os.path.join(
        get_package_share_directory('go2_description'),
        'urdf',
        'go2_description.urdf'
    )

    with open(urdf_file, 'r') as infp:
        robot_desc = infp.read()

    return LaunchDescription([
        # Subcarpeta del mapa: lab, my_house, etc. (debe existir en maps/<map_name>/)
        DeclareLaunchArgument(
            'map_name',
            default_value='my_house',
            description='Subcarpeta del mapa: lab, my_house, etc.'
        ),

        # ---------------- NODOS ORIGINALES ----------------
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
            package='my_go2_launch',
            executable='odom_publisher',
            name='odom_to_tf_broadcaster',
            output='screen',
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

        # base_footprint a nivel de patas (map/odom en z=0). AMCL usa base_footprint para map->odom.
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_footprint_publisher',
            output='screen',
            arguments=['0', '0', '-0.28', '0', '0', '0', 'base_link', 'base_footprint']
        ),

        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen'
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

        # Node(
        #     package='my_go2_launch',
        #     executable='pointcloud_to_laserscan_qos',
        #     name='pointcloud_to_laserscan_qos',
        #     output='screen',
        #     parameters=[{
        #         'target_frame': 'base_link',
        #         'transform_tolerance': 0.1,
        #         'min_height': -0.1,
        #         'max_height': 0.1,
        #         'angle_min': -3.14,
        #         'angle_max': 3.14,
        #         'angle_increment': 0.0087,
        #         'scan_time': 0.033,
        #         'range_min': 0.45,
        #         'range_max': 10.0,
        #         'use_inf': True
        #     }]
        # ),

        # ---------------- MAP SERVER + AMCL ----------------
        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            output='screen',
            parameters=[{'yaml_filename': map_config_file}]
        ),

        Node(
            package='nav2_amcl',
            executable='amcl',
            name='amcl',
            output='screen',
            parameters=[
                {'base_frame_id': 'base_footprint'},  # Patas en z=0, map y odom al mismo nivel
                {'odom_frame_id': 'odom'},
                {'global_frame_id': 'map'},
                {'scan_topic': 'scan'},
                {'set_initial_pose': True},
                # Mayor ruido en odom = confiar mas en el lidar (evita perderse al mover con joystick)
                {'alpha1': 0.5},
                {'alpha2': 0.5},
                {'alpha3': 0.5},
                {'alpha4': 0.5},
                {'max_beams': 80},
            ]
        ),

        # --- LIFECYCLE MANAGER (sustituye a nav2_util/lifecycle_bringup) ---
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_localization',
            output='screen',
            parameters=[
                {'use_sim_time': False},
                {'autostart': True},
                {'node_names': ['map_server', 'amcl']}
            ]
        ),
    ])

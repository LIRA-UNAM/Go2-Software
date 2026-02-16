from launch import LaunchDescription
from launch_ros.actions import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy

def generate_launch_description():

    # Definimos QoS BEST_EFFORT para la suscripción
    qos_profile_best_effort = QoSProfile(depth=10)
    qos_profile_best_effort.reliability = ReliabilityPolicy.BEST_EFFORT

    return LaunchDescription([
        Node(
            package='pointcloud_to_laserscan',
            executable='pointcloud_to_laserscan_node',
            name='pointcloud_to_laserscan',
            remappings=[
                ('cloud_in', '/utlidar/cloud_deskewed'),
                ('scan', '/utlidar/scan')
            ],
            parameters=[{
                'target_frame': 'base_link',
                'transform_tolerance': 0.01,
                'min_height': -0.1,
                'max_height': 0.1,
                'angle_min': -3.14,
                'angle_max': 3.14,
                'angle_increment': 0.003,
                'scan_time': 0.1,
                'range_min': 0.05,
                'range_max': 30.0,
                'use_inf': True,
                'inf_epsilon': 1.0
            }],
            # Aquí el truco para QoS (que requiere ROS 2 > Foxy en general)
            # Si esta opción falla, la otra opción es hacer un nodo Python custom
            extra_arguments=[{'use_intra_process_comms': False}],
            # Nota: para QoS exacto, normalmente toca cambiar el código fuente del nodo
        )
    ])


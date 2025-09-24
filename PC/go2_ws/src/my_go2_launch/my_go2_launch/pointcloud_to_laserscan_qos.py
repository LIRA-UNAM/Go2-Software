# import rclpy
# from rclpy.node import Node
# from rclpy.qos import QoSProfile, ReliabilityPolicy,DurabilityPolicy
# from sensor_msgs.msg import PointCloud2, LaserScan
# from tf2_ros import Buffer, TransformListener, LookupException, ExtrapolationException
# import math
# import struct

# def read_points(cloud, field_names=None, skip_nans=False, uvs=[]):
#     # Función adaptada simplificada para leer puntos de PointCloud2 (solo XYZ)
#     fmt = 'fff'  # 3 floats (x,y,z)
#     point_step = cloud.point_step
#     offset_x = 0
#     offset_y = 4
#     offset_z = 8
#     data = cloud.data
#     width = cloud.width
#     height = cloud.height

#     for row in range(height):
#         for col in range(width):
#             i = row * cloud.row_step + col * point_step
#             x = struct.unpack_from('f', data, i + offset_x)[0]
#             y = struct.unpack_from('f', data, i + offset_y)[0]
#             z = struct.unpack_from('f', data, i + offset_z)[0]
#             if skip_nans and (math.isnan(x) or math.isnan(y) or math.isnan(z)):
#                 continue
#             yield (x, y, z)

# class PointCloudToLaserScanNode(Node):
#     def __init__(self):
#         super().__init__('pointcloud_to_laserscan_qos')

#         self.declare_parameter('pub_reliability', 'reliable')
#         self.declare_parameter('sub_reliability', 'reliable')
#         self.declare_parameter('min_height', -0.2)
#         self.declare_parameter('max_height', 0)

#         pub_reliability_str = self.get_parameter('pub_reliability').get_parameter_value().string_value
#         sub_reliability_str = self.get_parameter('sub_reliability').get_parameter_value().string_value
#         self.min_height = self.get_parameter('min_height').get_parameter_value().double_value
#         self.max_height = self.get_parameter('max_height').get_parameter_value().double_value

#         pub_reliability = ReliabilityPolicy.RELIABLE if pub_reliability_str == 'reliable' else ReliabilityPolicy.BEST_EFFORT
#         sub_reliability = ReliabilityPolicy.RELIABLE if sub_reliability_str == 'reliable' else ReliabilityPolicy.BEST_EFFORT

#         pub_qos = QoSProfile(depth=10, reliability=pub_reliability,durability=DurabilityPolicy.VOLATILE)
#         sub_qos = QoSProfile(depth=10, reliability=sub_reliability,durability=DurabilityPolicy.VOLATILE)

#         self.subscription = self.create_subscription(
#             PointCloud2,
#             '/utlidar/cloud_base',
#             self.pointcloud_callback,
#             sub_qos)

#         self.publisher = self.create_publisher(
#             LaserScan,
#             '/utlidar/scan',
#             pub_qos)

#         self.get_logger().info(f'Node started with pub QoS {pub_reliability_str}, sub QoS {sub_reliability_str}')
#         self.get_logger().info(f'Filtering points with height between {self.min_height} and {self.max_height}')

#         self.angle_min = -math.pi
#         self.angle_max = math.pi
#         self.angle_increment = 0.01
#         self.range_min = 0.05
#         self.range_max = 30.0

#         # Inicializar TF2
#         self.tf_buffer = Buffer()
#         self.tf_listener = TransformListener(self.tf_buffer, self)

#     def pointcloud_callback(self, msg):
#         points = list(read_points(msg, skip_nans=True))

#         num_ranges = int((self.angle_max - self.angle_min) / self.angle_increment)
#         ranges = [float('inf')] * num_ranges

#         for point in points:
#             x, y, z = point

#             if z < self.min_height or z > self.max_height:
#                 continue  # Filtrar puntos fuera del plano definido

#             angle = math.atan2(y, x)
#             if self.angle_min <= angle <= self.angle_max:
#                 distance = math.sqrt(x*x + y*y)
#                 if self.range_min < distance < self.range_max:
#                     index = int((angle - self.angle_min) / self.angle_increment)
#                     if 0 <= index < num_ranges:
#                         if distance < ranges[index]:
#                             ranges[index] = distance

#         ranges = [r if r != float('inf') else 0.0 for r in ranges]

#         try:
#             t = self.tf_buffer.lookup_transform('odom', 'base_link', rclpy.time.Time())
#         except (LookupException, ExtrapolationException):
#             self.get_logger().warn('TF odom → base_link no disponible todavía')
#             return  # TF no existe, no publicar todavía
#         scan_msg = LaserScan()
#         scan_msg.header = msg.header
#         scan_msg.angle_min = self.angle_min
#         scan_msg.angle_max = self.angle_max
#         scan_msg.angle_increment = self.angle_increment
#         scan_msg.time_increment = 0.0
#         scan_msg.scan_time = 0.1
#         scan_msg.range_min = self.range_min
#         scan_msg.range_max = self.range_max
#         scan_msg.ranges = ranges

#         self.publisher.publish(scan_msg)

# def main(args=None):
#     rclpy.init(args=args)
#     node = PointCloudToLaserScanNode()
#     rclpy.spin(node)
#     node.destroy_node()
#     rclpy.shutdown()

# if __name__ == '__main__':
#     main()

##############################################
# 
# 
# primera version 
##############################################


import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from sensor_msgs.msg import PointCloud2, LaserScan
from tf2_ros import Buffer, TransformListener, LookupException, ExtrapolationException
import math
import struct
from rclpy.time import Time 

def read_points(cloud, skip_nans=False):
    """Función simplificada para leer puntos XYZ de PointCloud2"""
    point_step = cloud.point_step
    offset_x, offset_y, offset_z = 0, 4, 8
    data = cloud.data
    width, height = cloud.width, cloud.height

    for row in range(height):
        for col in range(width):
            i = row * cloud.row_step + col * point_step
            x = struct.unpack_from('f', data, i + offset_x)[0]
            y = struct.unpack_from('f', data, i + offset_y)[0]
            z = struct.unpack_from('f', data, i + offset_z)[0]
            if skip_nans and (math.isnan(x) or math.isnan(y) or math.isnan(z)):
                continue
            yield (x, y, z)

class PointCloudToLaserScanNode(Node):
    def __init__(self):
        super().__init__('pointcloud_to_laserscan_qos')

        # --- Parámetros ---
        self.declare_parameter('pub_reliability', 'reliable')
        self.declare_parameter('sub_reliability', 'reliable')
        self.declare_parameter('min_height', -0.2)
        self.declare_parameter('max_height', 0)
        self.declare_parameter('angle_min', -math.pi)
        self.declare_parameter('angle_max', math.pi)
        self.declare_parameter('angle_increment', 0.01)
        self.declare_parameter('range_min', 0.05)
        self.declare_parameter('range_max', 30.0)

        pub_reliability_str = self.get_parameter('pub_reliability').get_parameter_value().string_value
        sub_reliability_str = self.get_parameter('sub_reliability').get_parameter_value().string_value
        self.min_height = self.get_parameter('min_height').get_parameter_value().double_value
        self.max_height = self.get_parameter('max_height').get_parameter_value().double_value
        self.angle_min = self.get_parameter('angle_min').get_parameter_value().double_value
        self.angle_max = self.get_parameter('angle_max').get_parameter_value().double_value
        self.angle_increment = self.get_parameter('angle_increment').get_parameter_value().double_value
        self.range_min = self.get_parameter('range_min').get_parameter_value().double_value
        self.range_max = self.get_parameter('range_max').get_parameter_value().double_value

        pub_reliability = ReliabilityPolicy.RELIABLE if pub_reliability_str == 'reliable' else ReliabilityPolicy.BEST_EFFORT
        sub_reliability = ReliabilityPolicy.RELIABLE if sub_reliability_str == 'reliable' else ReliabilityPolicy.BEST_EFFORT

        pub_qos = QoSProfile(depth=10, reliability=pub_reliability, durability=DurabilityPolicy.VOLATILE)
        sub_qos = QoSProfile(depth=10, reliability=sub_reliability, durability=DurabilityPolicy.VOLATILE)

        # --- Suscripción y publicación ---
        self.subscription = self.create_subscription(
            PointCloud2,
            '/utlidar/cloud_base',
            self.pointcloud_callback,
            sub_qos
        )

        self.publisher = self.create_publisher(
            LaserScan,
            '/utlidar/scan',
            pub_qos
        )

        self.get_logger().info(f'Node started with pub QoS {pub_reliability_str}, sub QoS {sub_reliability_str}')
        self.get_logger().info(f'Filtering points with height between {self.min_height} and {self.max_height}')

        # --- TF2 ---
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

    def pointcloud_callback(self, msg):
        points = read_points(msg, skip_nans=True)

        num_ranges = round((self.angle_max - self.angle_min) / self.angle_increment) + 1
        ranges = [float('inf')] * num_ranges
        valid_point_found = False

        for x, y, z in points:
            if z < self.min_height or z > self.max_height:
                continue

            angle = math.atan2(y, x)
            if self.angle_min <= angle <= self.angle_max:
                distance = math.sqrt(x*x + y*y)
                if self.range_min < distance < self.range_max:
                    index = int((angle - self.angle_min) / self.angle_increment)
                    if 0 <= index < num_ranges:
                        if distance < ranges[index]:
                            ranges[index] = distance
                            valid_point_found = True

        if not valid_point_found:
            return  # Evitar publicar scans vacíos

        # --- TF para base_link → odom ---
        try:
            _ = self.tf_buffer.lookup_transform('odom', 'base_link', Time())
        except (LookupException, ExtrapolationException):
            self.get_logger().warn('TF odom → base_link no disponible todavía')
            return

        # --- Publicar LaserScan ---
        scan_msg = LaserScan()
        #scan_msg.header.stamp = msg.header.stamp
        scan_msg.header.stamp = self.get_clock().now().to_msg()
        scan_msg.header.frame_id = 'base_link'
        scan_msg.angle_min = self.angle_min
        scan_msg.angle_max = self.angle_max
        scan_msg.angle_increment = self.angle_increment
        scan_msg.time_increment = 0.0
        scan_msg.scan_time = 0.1
        scan_msg.range_min = self.range_min
        scan_msg.range_max = self.range_max
        scan_msg.ranges = [r if r != float('inf') else 0.0 for r in ranges]

        self.publisher.publish(scan_msg)

def main(args=None):
    rclpy.init(args=args)
    node = PointCloudToLaserScanNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

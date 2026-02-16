import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from sensor_msgs.msg import LaserScan
from tf2_ros import Buffer, TransformListener, LookupException, ExtrapolationException

class LaserScanPassthrough(Node):
    def __init__(self):
        super().__init__('laser_scan_passthrough')

        # --- Parámetros ---
        self.declare_parameter('pub_reliability', 'reliable')
        self.declare_parameter('sub_reliability', 'reliable')
        self.declare_parameter('fixed_frame', 'odom')
        self.declare_parameter('child_frame', 'base_link')

        pub_reliability_str = self.get_parameter('pub_reliability').get_parameter_value().string_value
        sub_reliability_str = self.get_parameter('sub_reliability').get_parameter_value().string_value
        self.fixed_frame = self.get_parameter('fixed_frame').get_parameter_value().string_value
        self.child_frame = self.get_parameter('child_frame').get_parameter_value().string_value

        pub_reliability = ReliabilityPolicy.RELIABLE if pub_reliability_str == 'reliable' else ReliabilityPolicy.BEST_EFFORT
        sub_reliability = ReliabilityPolicy.RELIABLE if sub_reliability_str == 'reliable' else ReliabilityPolicy.BEST_EFFORT

        pub_qos = QoSProfile(depth=10, reliability=pub_reliability, durability=DurabilityPolicy.VOLATILE)
        sub_qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT, durability=DurabilityPolicy.VOLATILE)

        # --- TF ---
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # --- Suscripción y publicación ---
        self.subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            sub_qos
        )

        self.publisher = self.create_publisher(
            LaserScan,
            '/fixed_scan',
            pub_qos
        )

        self.get_logger().info(f'Passthrough node started with pub QoS {pub_reliability_str}, sub QoS {sub_reliability_str}')

    def scan_callback(self, msg: LaserScan):
        # Verifica TF entre fixed_frame y child_frame
        try:
            _ = self.tf_buffer.lookup_transform(self.fixed_frame, self.child_frame, msg.header.stamp)
        except (LookupException, ExtrapolationException):
            self.get_logger().debug(f'TF {self.fixed_frame} → {self.child_frame} no disponible todavía')
            return

        # Publica scan igual, solo cambiando frame_id si quieres
        new_scan = LaserScan()
        new_scan.header = msg.header
        new_scan.header.frame_id = self.child_frame  # puedes mantener base_link
        new_scan.angle_min = msg.angle_min
        new_scan.angle_max = msg.angle_max
        new_scan.angle_increment = msg.angle_increment
        new_scan.time_increment = msg.time_increment
        new_scan.scan_time = msg.scan_time
        new_scan.range_min = msg.range_min
        new_scan.range_max = msg.range_max
        new_scan.ranges = msg.ranges
        new_scan.intensities = msg.intensities

        self.publisher.publish(new_scan)

def main(args=None):
    rclpy.init(args=args)
    node = LaserScanPassthrough()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

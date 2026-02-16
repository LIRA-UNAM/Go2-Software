import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, LaserScan
from pointcloud_to_laserscan import PointcloudToLaserScan
from rclpy.qos import QoSProfile, ReliabilityPolicy


class PointCloudToLaserScanNode(Node):
    def __init__(self):
        super().__init__('pointcloud_to_laserscan_custom')

        # QoS BEST_EFFORT para la suscripción al lidar
        qos_profile = QoSProfile(depth=10)
        qos_profile.reliability = ReliabilityPolicy.BEST_EFFORT

        self.subscription = self.create_subscription(
            PointCloud2,
            '/utlidar/cloud_deskewed',
            self.pointcloud_callback,
            qos_profile
        )

        self.publisher = self.create_publisher(
            LaserScan,
            '/utlidar/scan',
            10
        )

        self.converter = PointcloudToLaserScan()
        self.converter.target_frame = 'base_link'  # Ajusta si usas otro frame
        self.converter.min_height = -0.1
        self.converter.max_height = 0.1
        self.converter.angle_min = -3.14
        self.converter.angle_max = 3.14
        self.converter.angle_increment = 0.003
        self.converter.scan_time = 0.1
        self.converter.range_min = 0.05
        self.converter.range_max = 30.0
        self.converter.use_inf = True
        self.converter.inf_epsilon = 1.0

        self.get_logger().info("PointCloudToLaserScan node started with BEST_EFFORT QoS")

    def pointcloud_callback(self, msg):
        scan_msg = LaserScan()
        if self.converter.convert_msg(msg, scan_msg):
            self.publisher.publish(scan_msg)


def main(args=None):
    rclpy.init(args=args)
    node = PointCloudToLaserScanNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

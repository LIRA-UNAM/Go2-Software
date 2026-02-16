#!/usr/bin/env python3
"""
Publica TF odom→base_link desde /utlidar/robot_odom.
"""
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
import tf2_ros

class OdomToTF(Node):
    def __init__(self):
        super().__init__('odom_to_tf_broadcaster')

        self.initialized = False

        from rclpy.qos import qos_profile_sensor_data
        self.subscription = self.create_subscription(
            Odometry,
            '/utlidar/robot_odom',
            self.odom_callback,
            qos_profile_sensor_data
        )

        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        # TF inicial hasta el primer odom (evita gaps al arranque)
        self._initial_tf_timer = self.create_timer(0.1, self._publish_initial_tf)

        self.get_logger().info("Odom → TF broadcaster listo (usando QoS sensor_data)")

    def _publish_initial_tf(self):
        """Publica odom→base_link en (0,0,0) hasta que llegue el primer mensaje del robot."""
        if self.initialized:
            self._initial_tf_timer.cancel()
            return
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_link'
        t.transform.translation.x = 0.0
        t.transform.translation.y = 0.0
        t.transform.translation.z = 0.0
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = 0.0
        t.transform.rotation.w = 1.0
        self.tf_broadcaster.sendTransform(t)

    def odom_callback(self, msg: Odometry):
        self.initialized = True
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = msg.header.frame_id if msg.header.frame_id else 'odom'
        t.child_frame_id = msg.child_frame_id if msg.child_frame_id else 'base_link'

        t.transform.translation.x = msg.pose.pose.position.x
        t.transform.translation.y = msg.pose.pose.position.y
        t.transform.translation.z = msg.pose.pose.position.z
        t.transform.rotation = msg.pose.pose.orientation

        self.tf_broadcaster.sendTransform(t)

def main(args=None):
    rclpy.init(args=args)
    node = OdomToTF()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

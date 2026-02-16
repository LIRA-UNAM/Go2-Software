#!/usr/bin/env python3
"""
Filtro de odometría con zona muerta.
Ignora movimientos pequeños (drift de RF2O cuando el robot está parado)
y publica odom + TF filtrados.
"""
import math
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster


class OdomFilterNode(Node):
    def __init__(self):
        super().__init__("odom_filter")

        self.declare_parameter("input_topic", "/odom_rf2o_raw")
        self.declare_parameter("output_topic", "/odom_rf2o")
        self.declare_parameter("linear_threshold", 0.015)   # 1.5 cm
        self.declare_parameter("angular_threshold", 0.02)    # ~1.1 grados
        self.declare_parameter("odom_frame_id", "odom")
        self.declare_parameter("base_frame_id", "base_link")
        self.declare_parameter("base_link_height", 0.28)  # Go2: cuerpo sobre patas, map/odom en z=0

        self.input_topic = self.get_parameter("input_topic").value
        self.output_topic = self.get_parameter("output_topic").value
        self.linear_thresh = self.get_parameter("linear_threshold").value
        self.angular_thresh = self.get_parameter("angular_threshold").value
        self.odom_frame = self.get_parameter("odom_frame_id").value
        self.base_frame = self.get_parameter("base_frame_id").value
        self.base_link_height = self.get_parameter("base_link_height").value

        self.last_pose = None
        self.last_yaw = 0.0
        self.initialized = False

        self.sub = self.create_subscription(
            Odometry, self.input_topic,
            self.odom_callback, 10)
        self.pub = self.create_publisher(Odometry, self.output_topic, 10)
        self.tf_broadcaster = TransformBroadcaster(self)

        # Publicar TF inicial periodicamente hasta que RF2O publique (tarda ~2 scans)
        self._initial_tf_timer = self.create_timer(0.1, self._publish_initial_tf)

    def _publish_initial_tf(self):
        """Publica odom->base_link en (0,0,0) hasta que llegue el primer mensaje de RF2O."""
        if self.initialized:
            self._initial_tf_timer.cancel()
            return
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = self.odom_frame
        t.child_frame_id = self.base_frame
        t.transform.translation.x = 0.0
        t.transform.translation.y = 0.0
        t.transform.translation.z = self.base_link_height
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = 0.0
        t.transform.rotation.w = 1.0
        self.tf_broadcaster.sendTransform(t)

    def odom_callback(self, msg):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        yaw = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z))

        if not self.initialized:
            self.last_pose = (x, y, yaw)
            self.last_yaw = yaw
            self.initialized = True
            self._publish(msg, x, y, yaw)
            return

        lx, ly, lyaw = self.last_pose
        dx = x - lx
        dy = y - ly
        dyaw = yaw - lyaw
        # Normalizar dyaw a [-pi, pi]
        while dyaw > math.pi:
            dyaw -= 2 * math.pi
        while dyaw < -math.pi:
            dyaw += 2 * math.pi

        linear_delta = math.sqrt(dx * dx + dy * dy)
        angular_delta = abs(dyaw)

        if linear_delta < self.linear_thresh and angular_delta < self.angular_thresh:
            # Zona muerta: mantener pose anterior, velocidad cero
            x, y, yaw = self.last_pose
            msg.pose.pose.position.x = x
            msg.pose.pose.position.y = y
            msg.pose.pose.position.z = self.base_link_height
            qw = math.cos(yaw / 2.0)
            qz = math.sin(yaw / 2.0)
            msg.pose.pose.orientation.x = 0.0
            msg.pose.pose.orientation.y = 0.0
            msg.pose.pose.orientation.z = qz
            msg.pose.pose.orientation.w = qw
            msg.twist.twist.linear.x = 0.0
            msg.twist.twist.linear.y = 0.0
            msg.twist.twist.linear.z = 0.0
            msg.twist.twist.angular.x = 0.0
            msg.twist.twist.angular.y = 0.0
            msg.twist.twist.angular.z = 0.0
        else:
            self.last_pose = (x, y, yaw)

        self._publish(msg, msg.pose.pose.position.x, msg.pose.pose.position.y,
                     math.atan2(2.0 * (msg.pose.pose.orientation.w * msg.pose.pose.orientation.z +
                              msg.pose.pose.orientation.x * msg.pose.pose.orientation.y),
                               1.0 - 2.0 * (msg.pose.pose.orientation.y ** 2 +
                                        msg.pose.pose.orientation.z ** 2)))

    def _publish(self, msg, x, y, yaw):
        msg.header.frame_id = self.odom_frame
        msg.child_frame_id = self.base_frame
        msg.pose.pose.position.z = self.base_link_height
        self.pub.publish(msg)

        t = TransformStamped()
        t.header.stamp = msg.header.stamp
        t.header.frame_id = self.odom_frame
        t.child_frame_id = self.base_frame
        t.transform.translation.x = x
        t.transform.translation.y = y
        t.transform.translation.z = self.base_link_height
        qz = math.sin(yaw / 2.0)
        qw = math.cos(yaw / 2.0)
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = qz
        t.transform.rotation.w = qw
        self.tf_broadcaster.sendTransform(t)


def main(args=None):
    rclpy.init(args=args)
    node = OdomFilterNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

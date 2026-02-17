#!/usr/bin/env python3
"""
Convierte /lowstate (Unitree LowState) a sensor_msgs/Imu en /imu.
Usado para fusión EKF con robot_localization (odom + IMU).
"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from unitree_go.msg import LowState


class LowstateToImuNode(Node):
    def __init__(self):
        super().__init__("lowstate_to_imu")

        self.declare_parameter("lowstate_topic", "/lowstate")
        self.declare_parameter("imu_topic", "/imu")
        self.declare_parameter("frame_id", "base_link")

        lowstate_topic = self.get_parameter("lowstate_topic").value
        imu_topic = self.get_parameter("imu_topic").value
        self.frame_id = self.get_parameter("frame_id").value

        self.sub = self.create_subscription(
            LowState, lowstate_topic, self.lowstate_callback, 10
        )
        self.pub = self.create_publisher(Imu, imu_topic, 10)

        self.get_logger().info(
            f"lowstate_to_imu: {lowstate_topic} -> {imu_topic} (frame: {self.frame_id})"
        )

    def lowstate_callback(self, msg: LowState):
        imu_msg = Imu()
        imu_msg.header.stamp = self.get_clock().now().to_msg()
        imu_msg.header.frame_id = self.frame_id

        # Orientación (quaternion: x, y, z, w)
        imu_msg.orientation.x = float(msg.imu_state.quaternion[0])
        imu_msg.orientation.y = float(msg.imu_state.quaternion[1])
        imu_msg.orientation.z = float(msg.imu_state.quaternion[2])
        imu_msg.orientation.w = float(msg.imu_state.quaternion[3])

        # Velocidad angular (rad/s)
        imu_msg.angular_velocity.x = float(msg.imu_state.gyroscope[0])
        imu_msg.angular_velocity.y = float(msg.imu_state.gyroscope[1])
        imu_msg.angular_velocity.z = float(msg.imu_state.gyroscope[2])

        # Aceleración lineal (m/s²)
        imu_msg.linear_acceleration.x = float(msg.imu_state.accelerometer[0])
        imu_msg.linear_acceleration.y = float(msg.imu_state.accelerometer[1])
        imu_msg.linear_acceleration.z = float(msg.imu_state.accelerometer[2])

        # Covarianza -1 = desconocida (robot_localization lo acepta)
        imu_msg.orientation_covariance[0] = -1.0
        imu_msg.angular_velocity_covariance[0] = -1.0
        imu_msg.linear_acceleration_covariance[0] = -1.0

        self.pub.publish(imu_msg)


def main(args=None):
    rclpy.init(args=args)
    node = LowstateToImuNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

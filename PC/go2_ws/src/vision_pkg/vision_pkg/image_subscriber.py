#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

RELIABLE_QOS = QoSProfile(depth=10)
RELIABLE_QOS.reliability = ReliabilityPolicy.RELIABLE
RELIABLE_QOS.history = HistoryPolicy.KEEP_LAST
RELIABLE_QOS.durability = DurabilityPolicy.VOLATILE

class ImageSubscriber(Node):
    def __init__(self):
        super().__init__('image_subscriber')

        # Declarar parámetro para el tópico de la cámara
        self.declare_parameter('image_topic', '/camera/image_raw')
        image_topic = self.get_parameter('image_topic').get_parameter_value().string_value
        
        # Puente ROS ↔ OpenCV
        self.bridge = CvBridge()

        self.subscription = self.create_subscription(
            Image,
            image_topic,
            self.listener_callback,
            RELIABLE_QOS
        )

        self.get_logger().info(f"📷 Subscrito al tópico: {image_topic}")

    def listener_callback(self, msg: Image):
        try:
            # Convertir ROS Image a OpenCV
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

            # Mostrar la imagen
            cv2.imshow("Camera View", cv_image)
            cv2.waitKey(1)

        except Exception as e:
            self.get_logger().error(f"Error al procesar imagen: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = ImageSubscriber()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()

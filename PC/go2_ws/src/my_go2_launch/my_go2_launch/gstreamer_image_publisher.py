#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2


class GStreamerImagePublisher(Node):
    def __init__(self):
        super().__init__('gstreamer_image_publisher')

        # Parámetros configurables
        self.declare_parameter("multicast_address", "230.1.1.1")
        self.declare_parameter("port", 1720)
        self.declare_parameter("multicast_iface", "enx207bd2565bdb")
        self.declare_parameter("width", 1280)
        self.declare_parameter("height", 720)
        self.declare_parameter("topic_name", "/camera/image_raw")

        multicast_address = self.get_parameter("multicast_address").value
        port = self.get_parameter("port").value
        multicast_iface = self.get_parameter("multicast_iface").value
        width = self.get_parameter("width").value
        height = self.get_parameter("height").value
        topic_name = self.get_parameter("topic_name").value

        # GStreamer pipeline string
        gstreamer_str = (
            f"udpsrc address={multicast_address} port={port} multicast-iface={multicast_iface} "
            "! application/x-rtp, media=video, encoding-name=H264 "
            "! rtph264depay ! h264parse ! avdec_h264 "
            "! videoconvert "
            f"! video/x-raw,width={width},height={height},format=BGR ! appsink drop=1"
        )

        # Abrir la captura con OpenCV + GStreamer
        self.cap = cv2.VideoCapture(gstreamer_str, cv2.CAP_GSTREAMER)
        if not self.cap.isOpened():
            self.get_logger().error("No se pudo abrir el stream GStreamer")
            raise RuntimeError("Error al abrir GStreamer pipeline")

        # Publisher ROS2
        self.publisher_ = self.create_publisher(Image, topic_name, 10)
        self.bridge = CvBridge()

        # Temporizador para leer frames
        self.timer = self.create_timer(0.033, self.timer_callback)  # ~30 FPS

        self.get_logger().info(f"Publicando imágenes en {topic_name}")

    def timer_callback(self):
        ret, frame = self.cap.read()
        if ret:
            # Convertir a mensaje ROS
            msg = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
            msg.header.stamp = self.get_clock().now().to_msg()
            self.publisher_.publish(msg)
        else:
            self.get_logger().warn("No se pudo leer frame del stream")

    def destroy_node(self):
        if hasattr(self, "cap") and self.cap.isOpened():
            self.cap.release()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = GStreamerImagePublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

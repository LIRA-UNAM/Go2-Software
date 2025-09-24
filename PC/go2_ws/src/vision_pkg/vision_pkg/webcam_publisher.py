#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import time
import cv2
from cv_bridge import CvBridge

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

RELIABLE_QOS = QoSProfile(depth=10)
RELIABLE_QOS.reliability = ReliabilityPolicy.RELIABLE
RELIABLE_QOS.history = HistoryPolicy.KEEP_LAST
RELIABLE_QOS.durability = DurabilityPolicy.VOLATILE


class WebcamPublisher(Node):
    def __init__(self):
        super().__init__('webcam_publisher')

        # Parámetros
        self.declare_parameter('camera_id', 0)
        self.declare_parameter('width', 640)
        self.declare_parameter('height', 480)
        self.declare_parameter('fps', 30.0)
        self.declare_parameter('topic', '/camera/image_raw')
        self.declare_parameter('convert_to_rgb', False)

        self.camera_id = int(self.get_parameter('camera_id').value)
        self.width     = int(self.get_parameter('width').value)
        self.height    = int(self.get_parameter('height').value)
        self.fps       = float(self.get_parameter('fps').value)
        self.topic     = self.get_parameter('topic').value
        self.to_rgb    = bool(self.get_parameter('convert_to_rgb').value)

        self.bridge = CvBridge()
        self.pub = self.create_publisher(
            msg_type=__import__('sensor_msgs.msg', fromlist=['Image']).Image,
            topic=self.topic,
            qos_profile=RELIABLE_QOS
        )

        self.cap = None
        self._open_camera()

        period = 1.0 / max(self.fps, 1.0)
        self.timer = self.create_timer(period, self._grab_and_publish)

        self.get_logger().info(
            f'📷 Publicando {self.topic}  '
            f'(id={self.camera_id}, {self.width}x{self.height}@{self.fps:.1f}fps)'
        )

    def _open_camera(self):
        if self.cap is not None:
            self.cap.release()
            self.cap = None

        # CAP_V4L2 funciona en muchos entornos; si falla, prueba sin segundo arg.
        self.cap = cv2.VideoCapture(self.camera_id, cv2.CAP_V4L2)
        if not self.cap.isOpened():
            self.get_logger().error(f'No se pudo abrir la cámara id={self.camera_id}. Reintentando...')
            time.sleep(0.5)
            self.cap.open(self.camera_id)

        if self.cap.isOpened():
            self.cap.set(cv2.CAP_PROP_FRAME_WIDTH,  self.width)
            self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
            self.cap.set(cv2.CAP_PROP_FPS,          self.fps)
        else:
            self.get_logger().error('Cámara no disponible; se seguirá reintentando.')

    def _grab_and_publish(self):
        if self.cap is None or not self.cap.isOpened():
            self._open_camera()
            return

        ok, frame = self.cap.read()
        if not ok or frame is None:
            self.get_logger().warn('Frame inválido. Reabriendo cámara...')
            self._open_camera()
            return

        if self.to_rgb:
            frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            msg = self.bridge.cv2_to_imgmsg(frame, encoding='rgb8')
        else:
            msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')

        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'camera_frame'
        self.pub.publish(msg)

    def destroy_node(self):
        if self.cap is not None:
            self.cap.release()
        super().destroy_node()


def main():
    rclpy.init()
    node = WebcamPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

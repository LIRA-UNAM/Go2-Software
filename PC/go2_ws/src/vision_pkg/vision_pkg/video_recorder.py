#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from datetime import datetime
import os

RELIABLE_QOS = QoSProfile(depth=10)
RELIABLE_QOS.reliability = ReliabilityPolicy.RELIABLE
RELIABLE_QOS.history = HistoryPolicy.KEEP_LAST
RELIABLE_QOS.durability = DurabilityPolicy.VOLATILE

# SensorDataQoS: BEST_EFFORT, compatible con la mayoría de drivers de cámara
SENSOR_QOS = QoSProfile(depth=10)
SENSOR_QOS.reliability = ReliabilityPolicy.BEST_EFFORT
SENSOR_QOS.history = HistoryPolicy.KEEP_LAST
SENSOR_QOS.durability = DurabilityPolicy.VOLATILE


class VideoRecorder(Node):
    def __init__(self):
        super().__init__('video_recorder')

        # Parámetros
        self.declare_parameter('image_topic', '/camera/image_raw')
        self.declare_parameter('output_dir', '/home/unitree')
        self.declare_parameter('fps', 30.0)

        image_topic = self.get_parameter('image_topic').get_parameter_value().string_value
        output_dir = os.path.expanduser(
            self.get_parameter('output_dir').get_parameter_value().string_value
        )
        self.fps = self.get_parameter('fps').get_parameter_value().double_value

        self.bridge = CvBridge()
        self.writer = None
        self.frame_count = 0
        self.output_path = None
        self.last_log_time = self.get_clock().now()
        self.first_frame_received = False

        # Crear directorio de salida si no existe
        os.makedirs(output_dir, exist_ok=True)

        # Nombre del archivo con timestamp
        timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        self.output_path = os.path.join(output_dir, f'recording_{timestamp}.mp4')

        # Suscribirse con AMBOS perfiles QoS para compatibilidad con cualquier cámara
        self.sub_reliable = self.create_subscription(
            Image, image_topic, self.listener_callback, RELIABLE_QOS
        )
        self.sub_sensor = self.create_subscription(
            Image, image_topic, self.listener_callback, SENSOR_QOS
        )

        qos_str = "RELIABLE + BEST_EFFORT (compatible con ambos)"
        self.get_logger().info(
            f"📹 Grabando video desde {image_topic} → {self.output_path}"
        )
        self.get_logger().info(f"   QoS: {qos_str}")
        self.get_logger().info("   Presiona Ctrl+C para detener la grabación.")

        # Timer para avisar si no llegan frames (posible QoS o tópico incorrecto)
        self.check_timer = self.create_timer(3.0, self._check_no_frames)

    def _check_no_frames(self):
        if not self.first_frame_received:
            self.get_logger().warn(
                "⚠️ No se han recibido frames. Verifica que algo publique en el tópico: "
                "ros2 topic hz /camera/image_raw"
            )
        self.check_timer.cancel()

    def listener_callback(self, msg: Image):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            h, w = cv_image.shape[:2]

            # Inicializar VideoWriter con el primer frame
            if self.writer is None:
                fourcc = cv2.VideoWriter_fourcc(*'mp4v')
                self.writer = cv2.VideoWriter(
                    self.output_path, fourcc, self.fps, (w, h)
                )
                self.get_logger().info(f"✅ Primer frame recibido. Grabando {w}x{h} @ {self.fps} fps")

            self.writer.write(cv_image)
            self.frame_count += 1
            self.first_frame_received = True

            # Log periódico cada ~5 segundos
            now = self.get_clock().now()
            dt = (now - self.last_log_time).nanoseconds / 1e9
            if dt >= 5.0:
                self.get_logger().info(f"   ... {self.frame_count} frames grabados")
                self.last_log_time = now

        except Exception as e:
            self.get_logger().error(f"Error al procesar imagen: {e}")

    def destroy_node(self):
        if self.writer is not None:
            self.writer.release()
            self.get_logger().info(
                f"✅ Video guardado: {self.output_path} ({self.frame_count} frames)"
            )
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = VideoRecorder()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

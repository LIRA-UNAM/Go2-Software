#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import time
import cv2
import rclpy
from rclpy.node import Node

from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from std_msgs.msg import String

# Servicio generado por object_classification/srv/Classify_dino.srv
from object_classification.srv import ClassifyDino


# QoS RELIABLE (Foxy)
RELIABLE_QOS = QoSProfile(depth=10)
RELIABLE_QOS.reliability = ReliabilityPolicy.RELIABLE
RELIABLE_QOS.history     = HistoryPolicy.KEEP_LAST
RELIABLE_QOS.durability  = DurabilityPolicy.VOLATILE


class DinoClient(Node):
    def __init__(self):
        super().__init__('dino_client')

        # Parámetros
        self.declare_parameter('image_topic',   '/camera/image_raw')
        self.declare_parameter('service_name',  'grounding_dino_detect')
        self.declare_parameter('prompt',        'person')  # cambia desde CLI
        self.declare_parameter('show',          True)     # mostrar ventanas
        self.declare_parameter('draw_bbox',     True)     # dibujar bbox devuelto

        image_topic  = self.get_parameter('image_topic').value
        self.srv_name = self.get_parameter('service_name').value
        self.prompt   = self.get_parameter('prompt').value
        self.show     = bool(self.get_parameter('show').value)
        self.draw_bbox= bool(self.get_parameter('draw_bbox').value)

        # ROS I/O
        self.bridge = CvBridge()
        self.last_frame = None
        self.last_annot = None
        self.pending = False
        self.req_count = 0

        self.create_subscription(Image, image_topic, self._image_cb, RELIABLE_QOS)
        self.cli = self.create_client(ClassifyDino, self.srv_name)

        # Ventanas
        self.win_in  = "DINO Input"
        self.win_out = "DINO Annotated"
        self.use_window = False
        if self.show:
            try:
                cv2.namedWindow(self.win_in,  cv2.WINDOW_NORMAL)
                cv2.namedWindow(self.win_out, cv2.WINDOW_NORMAL)
                cv2.resizeWindow(self.win_in,  640, 360)
                cv2.resizeWindow(self.win_out, 640, 360)
                self.use_window = True
            except Exception as e:
                self.get_logger().warn(f"Sin GUI: {e}. Ejecuta con -p show:=false si no tienes X11.")

        # Timer de UI / teclado
        self.timer = self.create_timer(1/30.0, self._ui_tick)

        self.get_logger().info(f"Suscrito a {image_topic}, servicio '{self.srv_name}', prompt='{self.prompt}'")
        self.get_logger().info("Pulsa 'd' para enviar la imagen actual al servidor DINO, 'q' para salir.")

    # ========= Callbacks =========
    def _image_cb(self, msg: Image):
        # Guardar último frame (BGR para OpenCV)
        try:
            self.last_frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f"Error convirtiendo imagen: {e}")

    def _ui_tick(self):
        # Mostrar ventanas
        if self.use_window and self.last_frame is not None:
            cv2.imshow(self.win_in, self.last_frame)
            if self.last_annot is not None:
                cv2.imshow(self.win_out, self.last_annot)
            key = cv2.waitKey(1) & 0xFF
            if key == ord('d'):
                self._trigger_request()
            elif key == ord('q'):
                self.get_logger().info("q -> salir")
                rclpy.shutdown()
                return

        # También dispara con “auto” si no hay GUI (envía cada ~2s como ejemplo)
        if not self.use_window:
            now = time.time()
            if getattr(self, "_last_auto", None) is None:
                self._last_auto = now
            if now - self._last_auto > 2.0:
                self._last_auto = now
                self._trigger_request()

    def _trigger_request(self):
        if self.pending:
            self.get_logger().warn("Petición anterior aún en curso.")
            return
        if self.last_frame is None:
            self.get_logger().warn("Aún no hay frame de cámara.")
            return
        if not self.cli.wait_for_service(timeout_sec=0.1):
            self.get_logger().warn(f"Servicio '{self.srv_name}' no disponible.")
            return

        # Imagen BGR -> ROS bgr8 (el server espera y convierte a BGR internamente)
        req = ClassifyDino.Request()
        req.image = self.bridge.cv2_to_imgmsg(self.last_frame, encoding='bgr8')
        p = String()
        p.data = self.prompt
        req.prompt = p

        self.pending = True
        self.req_count += 1
        self.get_logger().info(f"[{self.req_count}] Solicitando DINO con prompt: '{self.prompt}'")
        future = self.cli.call_async(req)
        future.add_done_callback(self._on_response)

    def _on_response(self, future):
        self.pending = False
        try:
            res = future.result()
        except Exception as e:
            self.get_logger().error(f"Falló la llamada al servicio: {e}")
            return

        # Imagen anotada (el server publica RGB8 si logró anotar)
        try:
            enc = getattr(res.image, "encoding", "bgr8").lower()
            if "rgb" in enc:
                annot = self.bridge.imgmsg_to_cv2(res.image, desired_encoding='rgb8')
                #annot = cv2.cvtColor(annot, cv2.COLOR_RGB2BGR)
            else:
                annot = self.bridge.imgmsg_to_cv2(res.image, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().warn(f"No pude convertir imagen anotada: {e}")
            annot = None

        # Dibujar bbox en la anotada (o sobre el último frame) si procede
        try:
            data = list(getattr(res.bounding_boxes, "data", []))
            if len(data) >= 4:
                x1, y1, x2, y2 = map(int, data[:4])
                target = annot if annot is not None else (self.last_frame.copy() if self.last_frame is not None else None)
                if target is not None:
                    cv2.rectangle(target, (x1, y1), (x2, y2), (0, 255, 0), 2)
                    if annot is None:
                        annot = target
                self.get_logger().info(f"BBox: [{x1}, {y1}, {x2}, {y2}]")
            else:
                self.get_logger().info("BBox vacío.")
        except Exception as e:
            self.get_logger().warn(f"Error procesando bbox: {e}")

        self.last_annot = annot
        if self.use_window and self.last_annot is not None:
            cv2.imshow(self.win_out, self.last_annot)
            cv2.waitKey(1)


def main():
    rclpy.init()
    node = DinoClient()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()
    try:
        cv2.destroyAllWindows()
    except Exception:
        pass


if __name__ == '__main__':
    main()

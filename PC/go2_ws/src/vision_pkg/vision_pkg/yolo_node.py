#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import os, time, numpy as np, cv2
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

RELIABLE_QOS = QoSProfile(depth=10)
RELIABLE_QOS.reliability = ReliabilityPolicy.RELIABLE
RELIABLE_QOS.history     = HistoryPolicy.KEEP_LAST
RELIABLE_QOS.durability  = DurabilityPolicy.VOLATILE

class YoloImageSubscriber(Node):
    def __init__(self):
        super().__init__('yolo_image_subscriber')

        # Parámetros
        self.declare_parameter('image_topic', '/camera/image_raw')
        self.declare_parameter('annotated_topic', '/camera/image_yolo')
        self.declare_parameter('model_path', 'yolov8n.pt')   # usa fichero local si lo tienes
        self.declare_parameter('device', 'cuda')             # 'cuda' o 'cpu'
        self.declare_parameter('conf', 0.25)
        self.declare_parameter('imgsz', 640)
        self.declare_parameter('show', True)
        self.declare_parameter('passthrough', False)         # si True, publica sin YOLO (debug)

        image_topic     = self.get_parameter('image_topic').value
        annotated_topic = self.get_parameter('annotated_topic').value
        self.model_path = self.get_parameter('model_path').value
        self.device     = self.get_parameter('device').value
        self.conf       = float(self.get_parameter('conf').value)
        self.imgsz      = int(self.get_parameter('imgsz').value)
        self.show       = bool(self.get_parameter('show').value)
        self.passthrough= bool(self.get_parameter('passthrough').value)

        # YOLO
        from ultralytics import YOLO
        import torch
        if self.device.startswith('cuda') and not torch.cuda.is_available():
            self.get_logger().warn("CUDA no disponible, usando CPU.")
            self.device = 'cpu'
        if not os.path.isfile(self.model_path) and self.model_path != 'yolov8n.pt':
            self.get_logger().warn(f"No existe {self.model_path}, usando 'yolov8n.pt' (requiere internet).")
            self.model_path = 'yolov8n.pt'
        self.model = YOLO(self.model_path)
        try: self.model.fuse()
        except: pass
        self.get_logger().info(f"Modelo: {self.model_path} | device={self.device}")

        # ROS
        self.bridge = CvBridge()
        self.sub = self.create_subscription(Image, image_topic, self.cb, RELIABLE_QOS)
        self.pub = self.create_publisher(Image, annotated_topic, RELIABLE_QOS)
        self.get_logger().info(f"Suscrito: {image_topic} | Publica: {annotated_topic}")

        # Ventana
        self.win = "YOLO View"
        self.use_window = False
        if self.show:
            try:
                cv2.namedWindow(self.win, cv2.WINDOW_NORMAL)
                cv2.resizeWindow(self.win, 960, 540)
                self.use_window = True
            except Exception as e:
                self.get_logger().warn(f"Sin GUI: {e}. Usa -p show:=false o rqt_image_view.")

        # Contadores debug
        self.rx = 0
        self.pubc = 0
        self._t0 = time.time()

    def cb(self, msg: Image):
        # RX FPS
        self.rx += 1
        now = time.time()
        if now - self._t0 >= 1.0:
            self.rx = 0; self.pubc = 0; self._t0 = now

        # Convertir a OpenCV (BGR)
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        if self.passthrough:
            annotated = frame
        else:
            # YOLO -> usar plot() para obtener imagen anotada robusta
            res = self.model.predict(source=frame, imgsz=self.imgsz, conf=self.conf,
                                     device=self.device, verbose=False)[0]
            annotated = res.plot()  # numpy BGR

        # Asegurar memoria contigua para cv_bridge
        annotated = np.ascontiguousarray(annotated)
        out = self.bridge.cv2_to_imgmsg(annotated, encoding='bgr8')
        out.header = msg.header
        self.pub.publish(out)
        self.pubc += 1

        if self.use_window:
            cv2.imshow(self.win, annotated)
            if (cv2.waitKey(1) & 0xFF) == ord('q'):
                self.get_logger().info("q -> salir")
                rclpy.shutdown()

def main():
    rclpy.init()
    node = YoloImageSubscriber()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()
    try: cv2.destroyAllWindows()
    except: pass

if __name__ == '__main__':
    main()

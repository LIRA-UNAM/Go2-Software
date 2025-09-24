#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
import sys
import cv2
import torch
import numpy as np

import rclpy
from rclpy.node import Node

from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from std_msgs.msg import String, Int32MultiArray, MultiArrayDimension

# Servicio correcto (generado por Classify_dino.srv -> ClassifyDino)
from object_classification.srv import ClassifyDino

# ===== Normalización GroundingDINO =====
MEAN = [0.485, 0.456, 0.406]
STD  = [0.229, 0.224, 0.225]


def preprocess_image_rgb(image_rgb, max_size=1333, stride=32, device="cuda"):
    """RGB (H,W,C uint8) -> tensor normalizado (C,H,W) en device."""
    h, w = image_rgb.shape[:2]
    scale = min(max_size / max(h, w), 1.0)
    new_h = max(int(round((h * scale) / stride) * stride), stride)
    new_w = max(int(round((w * scale) / stride) * stride), stride)

    img_resized = cv2.resize(image_rgb, (new_w, new_h), interpolation=cv2.INTER_LINEAR)

    t = torch.as_tensor(img_resized, dtype=torch.float32).permute(2, 0, 1) / 255.0
    mean = torch.tensor(MEAN, dtype=torch.float32).view(3, 1, 1)
    std  = torch.tensor(STD,  dtype=torch.float32).view(3, 1, 1)
    t = (t - mean) / std
    return t.to(device)


def create_multiarray_bbox(xmin, ymin, xmax, ymax):
    arr = Int32MultiArray()
    arr.data = [int(xmin), int(ymin), int(xmax), int(ymax)]
    dim = MultiArrayDimension()
    dim.label = "bounding_boxes"
    dim.size = len(arr.data)
    dim.stride = len(arr.data)
    arr.layout.dim.append(dim)
    arr.layout.data_offset = 0
    return arr


class GroundingDinoServer(Node):
    def __init__(self):
        super().__init__('grounding_dino_server')

        # ===== Parámetros =====
        self.declare_parameter('base_dir', '/home/unitree/Go2/GroundingDINO')  # AJUSTA si tu repo está en otro lado
        self.declare_parameter('config_rel', 'groundingdino/config/GroundingDINO_SwinT_OGC.py')
        self.declare_parameter('weights_rel', 'weights/groundingdino_swint_ogc.pth')
        self.declare_parameter('device', 'cuda')
        self.declare_parameter('box_threshold', 0.4)
        self.declare_parameter('text_threshold', 0.4)
        self.declare_parameter('service_name', 'grounding_dino_detect')

        base_dir     = self.get_parameter('base_dir').get_parameter_value().string_value
        config_rel   = self.get_parameter('config_rel').get_parameter_value().string_value
        weights_rel  = self.get_parameter('weights_rel').get_parameter_value().string_value
        self.device  = self.get_parameter('device').get_parameter_value().string_value
        self.box_th  = float(self.get_parameter('box_threshold').value)
        self.text_th = float(self.get_parameter('text_threshold').value)
        service_name = self.get_parameter('service_name').get_parameter_value().string_value

        self.config_path  = os.path.join(base_dir, config_rel)
        self.weights_path = os.path.join(base_dir, weights_rel)

        # ===== Importar groundingdino de forma robusta =====
        try:
            from groundingdino.util.inference import load_model, predict, annotate  # noqa
            self._gd_load = load_model
            self._gd_predict = predict
            self._gd_annotate = annotate
        except Exception as e:
            self.get_logger().warn(f"No se pudo importar groundingdino: {e}. "
                                   f"Intentando añadir base_dir a sys.path: {base_dir}")
            if base_dir not in sys.path:
                sys.path.insert(0, base_dir)
            # reintenta
            from groundingdino.util.inference import load_model, predict, annotate  # type: ignore
            self._gd_load = load_model
            self._gd_predict = predict
            self._gd_annotate = annotate

        # Bridge
        self.bridge = CvBridge()

        # Cargar modelo una vez
        # Fallback de device si no hay CUDA
        if self.device.startswith('cuda') and not torch.cuda.is_available():
            self.get_logger().warn("CUDA no disponible. Cambiando a CPU.")
            self.device = 'cpu'

        self.get_logger().info(f'Cargando GroundingDINO...\n  cfg={self.config_path}\n  wts={self.weights_path}\n  dev={self.device}')
        self.model = self._gd_load(self.config_path, self.weights_path)
        self.get_logger().info('Modelo cargado.')

        # Servicio
        self.srv = self.create_service(ClassifyDino, service_name, self.handle_detection)
        self.get_logger().info(f'Servicio listo: {service_name}')

    def handle_detection(self, request, response):
        try:
            cv_bgr = self.bridge.imgmsg_to_cv2(request.image, desired_encoding='bgr8')
            h, w, _ = cv_bgr.shape

            prompt_text = request.prompt.data.strip() if isinstance(request.prompt, String) else str(request.prompt).strip()
            if not prompt_text:
                self.get_logger().warn('Prompt vacío; devolviendo imagen original.')
                response.image = self.bridge.cv2_to_imgmsg(cv_bgr, encoding='bgr8')
                response.bounding_boxes = Int32MultiArray()
                return response

            cv_rgb = cv2.cvtColor(cv_bgr, cv2.COLOR_BGR2RGB)
            img_tensor = preprocess_image_rgb(cv_rgb, device=self.device)

            boxes, logits, phrases = self._gd_predict(
                model=self.model,
                image=img_tensor,
                caption=prompt_text,
                box_threshold=self.box_th,
                text_threshold=self.text_th
            )

            if boxes is None or len(boxes) == 0:
                self.get_logger().warn('Sin detecciones.')
                response.image = self.bridge.cv2_to_imgmsg(cv_bgr, encoding='bgr8')
                response.bounding_boxes = Int32MultiArray()
                return response

            logits_np = np.array([float(x) for x in np.asarray(logits).reshape(-1)])
            max_index = int(np.argmax(logits_np))
            cx, cy, bw, bh = [float(v) for v in boxes[max_index]]

            xmin = int((cx - bw / 2.0) * w)
            ymin = int((cy - bh / 2.0) * h)
            xmax = int((cx + bw / 2.0) * w)
            ymax = int((cy + bh / 2.0) * h)

            xmin = max(0, min(xmin, w - 1))
            xmax = max(0, min(xmax, w - 1))
            ymin = max(0, min(ymin, h - 1))
            ymax = max(0, min(ymax, h - 1))

            bbox_msg = create_multiarray_bbox(xmin, ymin, xmax, ymax)

            annotated_rgb = self._gd_annotate(
                cv_rgb,
                boxes=boxes[max_index:max_index+1],
                logits=logits_np[max_index:max_index+1],
                phrases=[phrases[max_index]]
            )

            if annotated_rgb is None or annotated_rgb.size == 0:
                self.get_logger().error('annotated_rgb vacío; devolviendo original.')
                response.image = self.bridge.cv2_to_imgmsg(cv_bgr, encoding='bgr8')
            else:
                response.image = self.bridge.cv2_to_imgmsg(np.ascontiguousarray(annotated_rgb), encoding='rgb8')

            response.bounding_boxes = bbox_msg
            return response

        except Exception as e:
            self.get_logger().error(f'Error en handle_detection: {e}')
            try:
                response.image = self.bridge.cv2_to_imgmsg(
                    self.bridge.imgmsg_to_cv2(request.image, desired_encoding='bgr8'),
                    encoding='bgr8'
                )
            except Exception:
                pass
            response.bounding_boxes = Int32MultiArray()
            return response


def main():
    rclpy.init()
    node = GroundingDinoServer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

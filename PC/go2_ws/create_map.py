#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy, QoSHistoryPolicy
from nav_msgs.msg import OccupancyGrid
from math import isnan

FREE_VAL = 254     # blanco (libre)
OCC_VAL  = 0       # negro (ocupado)
UNK_VAL  = 205     # gris (desconocido)

class MapSaver(Node):
    def __init__(self):
        super().__init__('map_saver_no_cv')
        qos = QoSProfile(
            depth=1,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST
        )
        self.create_subscription(OccupancyGrid, 'map', self.cb, qos)
        self.saved = False
        self.get_logger().info('Esperando un mensaje en /map (QoS transient_local)…')

    def cb(self, msg: OccupancyGrid):
        if self.saved:
            return
        self.saved = True

        w, h = msg.info.width, msg.info.height
        data = msg.data  # list[int] con -1 (desconocido), 0 (libre), 100 (ocupado) usualmente

        # Mapea a niveles de gris
        buf = bytearray(w * h)
        for i, v in enumerate(data):
            if v == 0:
                buf[i] = FREE_VAL
            elif v == 100:
                buf[i] = OCC_VAL
            elif v == -1 or v is None:
                buf[i] = UNK_VAL
            else:
                # valores “intermedios”: aplica umbrales típicos
                buf[i] = OCC_VAL if v >= 65 else (FREE_VAL if v <= 25 else UNK_VAL)

        # Escribe PGM binario (P5)
        with open('map.pgm', 'wb') as f:
            header = f'P5\n{w} {h}\n255\n'.encode('ascii')
            f.write(header)
            # Nota: OccupancyGrid es row-major origin (0,0) esquina inferior izq. Muchos visores
            # esperan (0,0) arriba-izq; si lo necesitas volteado verticalmente, invierte filas:
            for row in range(h-1, -1, -1): 
                f.write(buf[row*w:(row+1)*w])
            # Aquí lo dejamos tal cual:
            #f.write(buf)

        # Escribe YAML simple (sin PyYAML)
        origin_x = float(msg.info.origin.position.x)
        origin_y = float(msg.info.origin.position.y)
        resolution = float(msg.info.resolution)
        yaml_text = (
            "image: map.pgm\n"
            f"resolution: {resolution}\n"
            f"origin: [{origin_x}, {origin_y}, 0.0]\n"
            "negate: 0\n"
            "occupied_thresh: 0.65\n"
            "free_thresh: 0.25\n"
        )
        with open('map.yaml', 'w') as f:
            f.write(yaml_text)

        self.get_logger().info(f"Mapa guardado: map.pgm / map.yaml  ({w}x{h} @ {resolution} m/px)")
        rclpy.shutdown()

def main():
    rclpy.init()
    rclpy.spin(MapSaver())

if __name__ == '__main__':
    main()

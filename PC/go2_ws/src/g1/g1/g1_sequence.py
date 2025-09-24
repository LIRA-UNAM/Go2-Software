#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import json, math, time
from unitree_api.msg import Request  # asegúrate de tener unitree_api disponible

API_ID = 7105  # loco api

class LocoSequencer(Node):
    def __init__(self):
        super().__init__('loco_sequencer')
        self.pub = self.create_publisher(Request, '/api/loco/request', 10)
        self.identity_id = 100
        self.buffer = 0.5  # colchón entre comandos

    def send_loco(self, vx, vy, wz, duration_s, note=''):
        msg = Request()
        msg.header.identity.id = self.identity_id
        msg.header.identity.api_id = API_ID
        msg.header.lease.id = 0
        msg.header.policy.priority = 1
        msg.header.policy.noreply = False

        params = {"velocity": [float(vx), float(vy), float(wz)],
                  "duration": float(duration_s)}
        msg.parameter = json.dumps(params)
        msg.binary = []

        self.get_logger().info(
            f"[{note}] v=({vx:.2f},{vy:.2f},{wz:.2f}) t={duration_s:.2f}s id={self.identity_id}"
        )
        self.pub.publish(msg)
        self.identity_id += 1
        time.sleep(duration_s + self.buffer)

    def run_sequence(self):
        # 1) Avanzar 2 m a 0.5 m/s -> 4 s
        self.send_loco(0.5, 0.0, 0.0, 4.0, note="Avanzar 2 m")

        # 2) Girar 180° a 0.5 rad/s -> pi/0.5 ≈ 6.283 s
        turn_time = math.pi / 0.5
        self.send_loco(0.0, 0.0, 0.5, turn_time, note="Girar 180°")

        # 3) Avanzar otros 2 m
        self.send_loco(0.5, 0.0, 0.0, 4.0, note="Avanzar 2 m (otra vez)")

def main():
    rclpy.init()
    node = LocoSequencer()
    try:
        time.sleep(0.3)  # margen para conectar publisher
        node.run_sequence()
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import json
import time
from unitree_api.msg import Request

# API IDs:
# 7101: modo / órdenes de postura/arranque
# 7102: BalanceMode
PRIORITY = 5

class G1Up(Node):
    def __init__(self):
        super().__init__('g1_up')
        self.pub = self.create_publisher(Request, '/api/loco/request', 10)
        self.identity_id = 500  # emparejado con tus ejemplos
        self.buffer = 0.4       # colchón entre mensajes (s)

    def send_simple(self, api_id: int, data: int, note: str):
        msg = Request()
        msg.header.identity.id = self.identity_id
        msg.header.identity.api_id = api_id
        msg.header.lease.id = 0
        msg.header.policy.priority = PRIORITY
        msg.header.policy.noreply = False

        # El API espera un JSON string en 'parameter'
        msg.parameter = json.dumps({"data": int(data)})
        msg.binary = []

        self.pub.publish(msg)
        self.get_logger().info(
            f"{note}: api_id={api_id} data={data} id={self.identity_id}"
        )
        self.identity_id += 1
        time.sleep(self.buffer)

    def run_sequence(self):
        # 1) Damping mode (data=1)  api_id=7101
        self.send_simple(7101, 1, "Damping mode")
        time.sleep(3)
        # 2) StandUp (data=4)  api_id=7101
        self.send_simple(7101, 4, "StandUp")
        time.sleep(5)
        # 3) BalanceMode (data=1)  api_id=7102
        self.send_simple(7102, 1, "BalanceMode")
        time.sleep(3)
        # 4) Start (data=500)  api_id=7101
        self.send_simple(7101, 500, "Start")
        time.sleep(3)
def main():
    rclpy.init()
    node = G1Up()
    try:
        time.sleep(0.3)  # margen para que el publisher conecte
        node.run_sequence()
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

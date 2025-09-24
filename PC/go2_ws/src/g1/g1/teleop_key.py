#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import json, sys, termios, tty, select, time
from unitree_api.msg import Request

API_ID = 7105  # loco api

class RawKeyboard:
    """ Context manager para leer teclado en modo raw, no bloqueante. """
    def __enter__(self):
        self.fd = sys.stdin.fileno()
        self.old_settings = termios.tcgetattr(self.fd)
        tty.setcbreak(self.fd)
        return self

    def __exit__(self, exc_type, exc, tb):
        termios.tcsetattr(self.fd, termios.TCSADRAIN, self.old_settings)

    def getch_nowait(self):
        dr, _, _ = select.select([sys.stdin], [], [], 0.0)
        if dr:
            ch = sys.stdin.read(1)
            return ch
        return None

class TeleopKeys(Node):
    def __init__(self):
        super().__init__('teleop_keys')
        # Parámetros
        self.declare_parameter('linear_speed', 0.5)     # m/s
        self.declare_parameter('angular_speed', 0.6)    # rad/s
        self.declare_parameter('burst_duration', 0.3)   # s
        self.declare_parameter('idle_buffer', 0.05)     # s colchón

        self.vx = float(self.get_parameter('linear_speed').value)
        self.wz = float(self.get_parameter('angular_speed').value)
        self.dt = float(self.get_parameter('burst_duration').value)
        self.buf = float(self.get_parameter('idle_buffer').value)

        self.pub = self.create_publisher(Request, '/api/loco/request', 10)
        self.identity_id = 100

        self.get_logger().info(
            "Teleop listo. Teclas: w/s/a/d, x o espacio=stop, q=salir.\n"
            f"linear_speed={self.vx} m/s, angular_speed={self.wz} rad/s, burst={self.dt}s"
        )

        self.timer = self.create_timer(0.02, self.loop)  # 50 Hz
        self.kb = RawKeyboard().__enter__()  # entrar a modo raw
        self.last_cmd_time = 0.0

    def destroy_node(self):
        try:
            self.kb.__exit__(None, None, None)
        except Exception:
            pass
        super().destroy_node()

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

        self.pub.publish(msg)
        self.identity_id += 1
        if note:
            self.get_logger().info(
                f"{note}: v=({vx:.2f},0.00,{wz:.2f}) t={duration_s:.2f}s id={self.identity_id-1}"
            )
        # pequeño colchón para no saturar
        time.sleep(self.buf)

    def loop(self):
        ch = self.kb.getch_nowait()
        if not ch:
            return

        ch = ch.lower()
        now = time.time()

        # Evitar bombardeo si mantienes presionada la tecla:
        if now - self.last_cmd_time < 0.05:
            return

        if ch == 'w':            # adelante
            self.send_loco(self.vx, 0.0, 0.0, self.dt, "adelante")
        elif ch == 's':          # atrás
            self.send_loco(-self.vx, 0.0, 0.0, self.dt, "atrás")
        elif ch == 'a':          # girar izquierda
            self.send_loco(0.0, 0.0, self.wz, self.dt, "giro izq")
        elif ch == 'd':          # girar derecha
            self.send_loco(0.0, 0.0, -self.wz, self.dt, "giro der")
        elif ch == 'x' or ch == ' ':  # stop
            self.send_loco(0.0, 0.0, 0.0, 0.0, "stop")
        elif ch == 'q':          # salir
            self.get_logger().info("Saliendo…")
            rclpy.shutdown()
            return
        # ignora otras teclas

        self.last_cmd_time = now

def main():
    rclpy.init()
    node = TeleopKeys()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Header
from unitree_go.msg import LowState

JOINT_NAMES = [
    "FL_hip_joint", "FL_thigh_joint", "FL_calf_joint",
    "FR_hip_joint", "FR_thigh_joint", "FR_calf_joint",
    "RL_hip_joint", "RL_thigh_joint", "RL_calf_joint",
    "RR_hip_joint", "RR_thigh_joint", "RR_calf_joint"
]

# Pose por defecto (pie): hip=0, thigh=0.8, calf=-1.5 rad
DEFAULT_POSE = [0.0, 0.8, -1.5] * 4  # FL, FR, RL, RR


class JointStateRelay(Node):
    def __init__(self):
        super().__init__('joint_state_relay')
        self.declare_parameter('lowstate_topic', '/lowstate')
        self.declare_parameter('fallback_interval', 0.1)  # Publicar fallback cada 100 ms si no hay lowstate
        lowstate_topic = self.get_parameter('lowstate_topic').value
        self.fallback_interval = self.get_parameter('fallback_interval').value

        self.pub = self.create_publisher(JointState, '/joint_states', 10)
        self.sub = self.create_subscription(LowState, lowstate_topic, self.lowstate_callback, 10)
        self.last_lowstate_time = None
        self.fallback_timer = self.create_timer(self.fallback_interval, self.fallback_callback)

    def _publish_joint_state(self, positions, velocities=None, efforts=None):
        joint_state = JointState()
        joint_state.header = Header()
        joint_state.header.stamp = self.get_clock().now().to_msg()
        joint_state.header.frame_id = ''
        joint_state.name = JOINT_NAMES
        joint_state.position = list(positions)
        joint_state.velocity = velocities or [0.0] * 12
        joint_state.effort = efforts or [0.0] * 12
        self.pub.publish(joint_state)

    def fallback_callback(self):
        """Publica pose por defecto si no hay lowstate reciente."""
        if self.last_lowstate_time is None:
            self._publish_joint_state(DEFAULT_POSE)
        else:
            # Si recibimos lowstate hace más de 1 s, volver a fallback (robot apagado?)
            now = self.get_clock().now()
            elapsed = (now - self.last_lowstate_time).nanoseconds / 1e9
            if elapsed > 1.0:
                self._publish_joint_state(DEFAULT_POSE)
                self.last_lowstate_time = None

    def lowstate_callback(self, msg):
        self.last_lowstate_time = self.get_clock().now()
        # motor_state[0:12] = FL, FR, RL, RR (hip, thigh, calf cada uno)
        # Si mode==0 usar 0.0 para evitar desincronización nombre/posición
        positions = []
        velocities = []
        efforts = []
        for i in range(len(JOINT_NAMES)):
            if i < len(msg.motor_state) and msg.motor_state[i].mode != 0:
                positions.append(msg.motor_state[i].q)
                velocities.append(msg.motor_state[i].dq)
                efforts.append(msg.motor_state[i].tau_est)
            else:
                positions.append(DEFAULT_POSE[i])  # Usar pose por defecto si motor inactivo
                velocities.append(0.0)
                efforts.append(0.0)
        self._publish_joint_state(positions, velocities, efforts)

def main(args=None):
    rclpy.init(args=args)
    node = JointStateRelay()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

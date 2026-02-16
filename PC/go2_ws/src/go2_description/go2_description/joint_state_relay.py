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

class JointStateRelay(Node):
    def __init__(self):
        super().__init__('joint_state_relay')
        self.pub = self.create_publisher(JointState, '/joint_states', 10)
        self.sub = self.create_subscription(LowState, '/lowstate', self.lowstate_callback, 10)

    def lowstate_callback(self, msg):
        # Usar indice fijo: motor_state[0:12] = FL, FR, RL, RR (hip, thigh, calf cada uno)
        # Siempre publicar 12 posiciones para que robot_state_publisher muestre las 4 patas.
        # Si un motor tiene mode==0, usar 0.0 para evitar desincronizacion nombre/posicion.
        joint_state = JointState()
        joint_state.header = Header()
        joint_state.header.stamp = self.get_clock().now().to_msg()
        joint_state.name = JOINT_NAMES
        positions = []
        velocities = []
        efforts = []
        for i in range(len(JOINT_NAMES)):
            if i < len(msg.motor_state) and msg.motor_state[i].mode != 0:
                positions.append(msg.motor_state[i].q)
                velocities.append(msg.motor_state[i].dq)
                efforts.append(msg.motor_state[i].tau_est)
            else:
                positions.append(0.0)
                velocities.append(0.0)
                efforts.append(0.0)
        joint_state.position = positions
        joint_state.velocity = velocities
        joint_state.effort = efforts

        self.pub.publish(joint_state)

def main(args=None):
    rclpy.init(args=args)
    node = JointStateRelay()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

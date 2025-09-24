# #!/usr/bin/env python3


# import rclpy
# from rclpy.node import Node
# from sensor_msgs.msg import JointState

# class JointStateRelay(Node):
#     """
#     Nodo que se suscribe a los joint_states reales del robot y los publica
#     para que robot_state_publisher pueda generar los TF en tiempo real.
#     """
#     def __init__(self):
#         super().__init__('joint_state_relay')

#         # Suscripción a los joints reales
#         self.subscription = self.create_subscription(
#             JointState,
#             '/joint_states',  # topic de los joints reales
#             self.joint_callback,
#             10
#         )

#         # Publicador de JointState
#         self.publisher = self.create_publisher(JointState, '/joint_states_relay', 10)

#     def joint_callback(self, msg: JointState):
#         """
#         Publica los joints recibidos en un nuevo topic para robot_state_publisher
#         """
#         relay_msg = JointState()
#         relay_msg.header.stamp = self.get_clock().now().to_msg()
#         relay_msg.name = msg.name
#         relay_msg.position = msg.position
#         relay_msg.velocity = msg.velocity
#         relay_msg.effort = msg.effort

#         self.publisher.publish(relay_msg)

# def main(args=None):
#     rclpy.init(args=args)
#     node = JointStateRelay()
#     rclpy.spin(node)
#     node.destroy_node()
#     rclpy.shutdown()

# if __name__ == '__main__':
#     main()



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
        joint_state = JointState()
        joint_state.header = Header()
        joint_state.header.stamp = self.get_clock().now().to_msg()
        joint_state.name = JOINT_NAMES
        joint_state.position = [m.q for m in msg.motor_state if m.mode != 0]
        joint_state.velocity = [m.dq for m in msg.motor_state if m.mode != 0]
        joint_state.effort = [m.tau_est for m in msg.motor_state if m.mode != 0]

        self.pub.publish(joint_state)

def main(args=None):
    rclpy.init(args=args)
    node = JointStateRelay()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

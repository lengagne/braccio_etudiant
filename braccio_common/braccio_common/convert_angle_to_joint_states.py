#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState

class AnglesToJointStates(Node):
    def __init__(self):
        super().__init__('angles_to_joint_states')

        # Noms des joints (à adapter selon ton robot)
        self.joint_names = ['base_joint', 'shoulder_joint', 'elbow_joint', 'wrist_pitch_joint', 'wrist_roll_joint', 'gripper_joint']

        # Publisher pour /joint_states
        self.joint_state_pub = self.create_publisher(JointState, '/joint_states', 10)

        # Subscriber pour /angles
        self.angles_sub = self.create_subscription(Float64MultiArray,'/angles',self.angles_callback,10)

    def angles_callback(self, msg):
        # self.get_logger().info(f"Message reçu sur /angles : {msg.data}")
        joint_state = JointState()
        joint_state.header.stamp = self.get_clock().now().to_msg()

        # Copie les noms des joints
        joint_state.name = self.joint_names

        # Copie les positions depuis le message reçu
        joint_state.position = msg.data

        # Publie le message
        self.joint_state_pub.publish(joint_state)

def main(args=None):
    rclpy.init(args=args)
    node = AnglesToJointStates()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

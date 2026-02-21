#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Bool
import math


class JoystickTargetPose(Node):

    def __init__(self):
        super().__init__('joystick_target_pose')

        # Subscriber joystick
        self.joy_sub = self.create_subscription(Joy, '/joy', self.joy_callback, 10)

        # Publisher target pose
        self.pose_pub = self.create_publisher(PoseStamped, '/target_pose', 10)
        self.open_pub = self.create_publisher(Bool, '/open_gripper', 10)

        # Etat courant de la cible
        self.x = 0.1
        self.y = 0.1
        self.z = 0.1
        self.yaw = 0.0

        # Sensibilité
        self.pos_scale = 0.005
        self.rot_scale = 0.01

        self.get_logger().info("Joystick Target Pose Node Started")

        self.count = 0
        self.previous_push = 0
        self.targets = [
                [0.1,0.1,0.1],
                [0.1,0.1,0.05],
                [0.1,-0.1,0.05],
                [0.1,-0.1,0.1],
                [-0.1,-0.1,0.1],
                [-0.1,-0.1,0.05],
                [-0.1,0.1,0.05],
                [-0.1,0.1,0.1],
            ]

        self.open_gripper = False
        self.previous_push_gripper = 0

    def joy_callback(self, msg: Joy):

        # Mapping standard Xbox / Logitech
        # axes[0] = stick gauche horizontal
        # axes[1] = stick gauche vertical
        # axes[3] = stick droit horizontal
        # axes[4] = stick droit vertical

        if msg.buttons[0] == 1 and self.previous_push ==0:
            self.previous_push = 1
            self.count = self.count +1
            if self.count > len(self.targets)-1:
                self.count = 0
            self.x = self.targets[self.count][0]
            self.y = self.targets[self.count][1]
            self.z = self.targets[self.count][2]

        if msg.buttons[0] ==0:
            self.previous_push = 0

        if msg.buttons[9] == 1 and self.previous_push_gripper ==0:
            self.previous_push_gripper = 1
            self.open_gripper = not self.open_gripper
            if self.open_gripper:
                self.get_logger().info("Open Gripper")
            else:
                self.get_logger().info("Close Gripper")

        if msg.buttons[9] ==0:
            self.previous_push_gripper = 0




        self.x += msg.axes[1] * self.pos_scale
        self.y += msg.axes[0] * self.pos_scale
        self.z += (msg.buttons[5] -  msg.buttons[7]) * self.pos_scale

        self.yaw += (msg.buttons[3] -  msg.buttons[1]) * self.rot_scale

        # Création message PoseStamped
        pose = PoseStamped()
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.header.frame_id = "base_link"

        pose.pose.position.x = self.x
        pose.pose.position.y = self.y
        pose.pose.position.z = self.z

        # Conversion Euler -> Quaternion
        # q = quaternion_from_euler(0.0, 0.0, self.yaw)
        pose.pose.orientation.x = 0.0
        pose.pose.orientation.y = 0.0
        pose.pose.orientation.z = math.sin(self.yaw / 2.0)
        pose.pose.orientation.w = math.cos(self.yaw / 2.0)

        self.pose_pub.publish(pose)

        msg_gripper = Bool()
        msg_gripper.data = self.open_gripper
        self.open_pub.publish(msg_gripper)


def main(args=None):
    rclpy.init(args=args)
    node = JoystickTargetPose()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

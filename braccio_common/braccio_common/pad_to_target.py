#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from action_msgs.msg import GoalStatus
from sensor_msgs.msg import Joy
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Bool
from tag_msgs.action import MoveToPose  # Assure-toi que ce module existe
import math

class JoystickActionClient(Node):

    def __init__(self):
        super().__init__('joystick_action_client')

        # Subscriber joystick
        self.joy_sub = self.create_subscription(Joy, '/joy', self.joy_callback, 10)

        # Publisher pour le gripper
        self.open_pub = self.create_publisher(Bool, '/open_gripper', 10)

        # pour l'affichage
        self.pose_pub = self.create_publisher(PoseStamped, '/target_pose', 10)


        # Client d'action
        self.action_client = ActionClient(self, MoveToPose, 'move_to_pose')

        # Etat courant de la cible
        self.x = -0.05
        self.y = 0.1
        self.z = 0.1
        self.yaw = 0.0

        self.oldx = self.x
        self.oldy = self.y
        self.oldz = self.z
        self.oldyaw = self.yaw
        self.target_change = True

        # Sensibilité
        self.pos_scale = 0.005
        self.rot_scale = 0.01

        self.get_logger().info("Joystick Action Client Node Started")

        # Positions prédéfinies
        self.count = 0
        self.previous_push = 0
        self.targets = [
            [-0.05, 0.1, 0.1],
            [-0.05, 0.1, 0.05],
            [-0.05, -0.1, 0.05],
            [-0.05, -0.1, 0.1],
            [-0.1, -0.1, 0.1],
            [-0.1, -0.1, 0.05],
            [-0.1, 0.1, 0.05],
            [-0.1, 0.1, 0.1],
        ]

        # État du gripper
        self.open_gripper = False
        self.previous_push_gripper = 0

        # Goal actuel
        self.current_goal_handle = None

        self.send_goal(self.x, self.y, self.z)

    def send_goal(self, x, y, z):
        """Envoie un nouveau goal au serveur d'action et annule le précédent si nécessaire."""
        # Annule le goal précédent s'il existe et est toujours actif
        if self.current_goal_handle is not None:
            # Vérifie si le goal est toujours en cours
            if self.current_goal_handle.status != GoalStatus.STATUS_SUCCEEDED and \
               self.current_goal_handle.status != GoalStatus.STATUS_CANCELED and \
               self.current_goal_handle.status != GoalStatus.STATUS_ABORTED:
                self.get_logger().info("Annulation du goal précédent...")
                future = self.current_goal_handle.cancel_goal_async()
                future.add_done_callback(lambda _: self.get_logger().info("Goal précédent annulé"))

        # Crée un nouveau goal
        goal_msg = MoveToPose.Goal()
        goal_msg.target_pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.target_pose.header.frame_id = "base_link"
        goal_msg.target_pose.pose.position.x = x
        goal_msg.target_pose.pose.position.y = y
        goal_msg.target_pose.pose.position.z = z
        goal_msg.effector_frame = "MGD"

        # Orientation (quaternion)
        goal_msg.target_pose.pose.orientation.x = 0.0
        goal_msg.target_pose.pose.orientation.y = 0.0
        goal_msg.target_pose.pose.orientation.z = math.sin(self.yaw / 2.0)
        goal_msg.target_pose.pose.orientation.w = math.cos(self.yaw / 2.0)

        self.pose_pub.publish(goal_msg.target_pose)

        # Envoie le goal
        self.get_logger().info(f"Envoi d'un nouveau goal : x={x:.3f}, y={y:.3f}, z={z:.3f}")
        send_goal_future = self.action_client.send_goal_async(goal_msg)

        # Stocke le futur goal handle
        send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        """Callback appelé quand le serveur accepte/rejette le goal."""
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn("Goal rejeté par le serveur !")
            return

        self.current_goal_handle = goal_handle
        self.get_logger().info("Goal accepté par le serveur !")

        # Ajoute un callback pour les feedbacks
        goal_handle.get_result_async().add_done_callback(self.goal_result_callback)

    def goal_result_callback(self, future):
        """Callback appelé quand le goal est terminé."""
        result = future.result().result
        self.get_logger().info(f"Résultat du goal : {result.message}")
        self.current_goal_handle = None

    def joy_callback(self, msg: Joy):
        # Mapping standard Xbox / Logitech
        # axes[0] = stick gauche horizontal
        # axes[1] = stick gauche vertical
        # axes[3] = stick droit horizontal
        # axes[4] = stick droit vertical

        # Changement de cible prédéfinie
        if msg.buttons[0] == 1 and self.previous_push == 0:
            self.previous_push = 1
            self.count = (self.count + 1) % len(self.targets)
            self.x, self.y, self.z = self.targets[self.count]
            self.send_goal(self.x, self.y, self.z)

        if msg.buttons[0] == 0:
            self.previous_push = 0

        # Contrôle du gripper
        if msg.buttons[9] == 1 and self.previous_push_gripper == 0:
            self.previous_push_gripper = 1
            self.open_gripper = not self.open_gripper
            msg_gripper = Bool()
            msg_gripper.data = self.open_gripper
            self.open_pub.publish(msg_gripper)
            if self.open_gripper:
                self.get_logger().info("Ouverture du gripper")
            else:
                self.get_logger().info("Fermeture du gripper")

        if msg.buttons[9] == 0:
            self.previous_push_gripper = 0



        # Mise à jour de la position avec les sticks
        self.x += msg.axes[1] * self.pos_scale
        self.y += msg.axes[0] * self.pos_scale
        self.z += (msg.buttons[5] - msg.buttons[7]) * self.pos_scale

        # Mise à jour de l'orientation
        self.yaw += (msg.buttons[3] - msg.buttons[1]) * self.rot_scale


        if not (self.oldx == self.x and self.oldy == self.y and self.oldz == self.z and self.oldyaw == self.yaw):
            self.send_goal(self.x, self.y, self.z)

        self.oldx = self.x
        self.oldy = self.y
        self.oldz = self.z
        self.oldyaw = self.yaw


def main(args=None):
    rclpy.init(args=args)
    node = JoystickActionClient()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()


# #!/usr/bin/env python3
#
# import rclpy
# from rclpy.node import Node
# from sensor_msgs.msg import Joy
# from geometry_msgs.msg import PoseStamped
# from std_msgs.msg import Bool
# import math
#
#
# class JoystickTargetPose(Node):
#
#     def __init__(self):
#         super().__init__('joystick_target_pose')
#
#         # Subscriber joystick
#         self.joy_sub = self.create_subscription(Joy, '/joy', self.joy_callback, 10)
#
#         # Publisher target pose
#         self.pose_pub = self.create_publisher(PoseStamped, '/target_pose', 10)
#         self.open_pub = self.create_publisher(Bool, '/open_gripper', 10)
#
#         # Etat courant de la cible
#         self.x = 0.1
#         self.y = 0.1
#         self.z = 0.1
#         self.yaw = 0.0
#
#         # Sensibilité
#         self.pos_scale = 0.005
#         self.rot_scale = 0.01
#
#         self.get_logger().info("Joystick Target Pose Node Started")
#
#         self.count = 0
#         self.previous_push = 0
#         self.targets = [
#                 [0.1,0.1,0.1],
#                 [0.1,0.1,0.05],
#                 [0.1,-0.1,0.05],
#                 [0.1,-0.1,0.1],
#                 [-0.1,-0.1,0.1],
#                 [-0.1,-0.1,0.05],
#                 [-0.1,0.1,0.05],
#                 [-0.1,0.1,0.1],
#             ]
#
#         self.open_gripper = False
#         self.previous_push_gripper = 0
#
#     def joy_callback(self, msg: Joy):
#
#         # Mapping standard Xbox / Logitech
#         # axes[0] = stick gauche horizontal
#         # axes[1] = stick gauche vertical
#         # axes[3] = stick droit horizontal
#         # axes[4] = stick droit vertical
#
#         if msg.buttons[0] == 1 and self.previous_push ==0:
#             self.previous_push = 1
#             self.count = self.count +1
#             if self.count > len(self.targets)-1:
#                 self.count = 0
#             self.x = self.targets[self.count][0]
#             self.y = self.targets[self.count][1]
#             self.z = self.targets[self.count][2]
#
#         if msg.buttons[0] ==0:
#             self.previous_push = 0
#
#         if msg.buttons[9] == 1 and self.previous_push_gripper ==0:
#             self.previous_push_gripper = 1
#             self.open_gripper = not self.open_gripper
#             if self.open_gripper:
#                 self.get_logger().info("Open Gripper")
#             else:
#                 self.get_logger().info("Close Gripper")
#
#         if msg.buttons[9] ==0:
#             self.previous_push_gripper = 0
#
#
#
#
#         self.x += msg.axes[1] * self.pos_scale
#         self.y += msg.axes[0] * self.pos_scale
#         self.z += (msg.buttons[5] -  msg.buttons[7]) * self.pos_scale
#
#         self.yaw += (msg.buttons[3] -  msg.buttons[1]) * self.rot_scale
#
#         # Création message PoseStamped
#         pose = PoseStamped()
#         pose.header.stamp = self.get_clock().now().to_msg()
#         pose.header.frame_id = "base_link"
#
#         pose.pose.position.x = self.x
#         pose.pose.position.y = self.y
#         pose.pose.position.z = self.z
#
#         # Conversion Euler -> Quaternion
#         # q = quaternion_from_euler(0.0, 0.0, self.yaw)
#         pose.pose.orientation.x = 0.0
#         pose.pose.orientation.y = 0.0
#         pose.pose.orientation.z = math.sin(self.yaw / 2.0)
#         pose.pose.orientation.w = math.cos(self.yaw / 2.0)
#
#         self.pose_pub.publish(pose)
#
#         msg_gripper = Bool()
#         msg_gripper.data = self.open_gripper
#         self.open_pub.publish(msg_gripper)
#
#
# def main(args=None):
#     rclpy.init(args=args)
#     node = JoystickTargetPose()
#     rclpy.spin(node)
#     node.destroy_node()
#     rclpy.shutdown()
#
#
# if __name__ == '__main__':
#     main()

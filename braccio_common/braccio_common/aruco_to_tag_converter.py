#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from aruco_msgs.msg import MarkerArray  # ou apriltag_msgs si vous utilisez ça
from tag_msgs.msg import TagPose, TagPoseArray
from geometry_msgs.msg import PoseStamped


class ArucoToTagConverter(Node):
    """
    Convertit les messages ArUco (MarkerArray) en TagPoseArray
    """

    def __init__(self):
        super().__init__('aruco_to_tag_converter')

        # Subscriber ArUco
        self.aruco_sub = self.create_subscription(MarkerArray,'aruco_detector/markers',self.aruco_callback,10)

        # Publisher TagPoseArray
        self.tag_pub = self.create_publisher(TagPoseArray,'tag_list',10)

        self.get_logger().info(f'Convertisseur ArUco → TagPoseArray démarré')

    def aruco_callback(self, aruco_msg):
        """
        Callback pour convertir MarkerArray en TagPoseArray
        """
        # Créer le message de sortie
        tag_array = TagPoseArray()
        tag_array.header = aruco_msg.header

        # Convertir chaque marqueur ArUco en TagPose
        for marker in aruco_msg.markers:
            tag_pose = TagPose()
            tag_pose.id = marker.id

            # Créer le PoseStamped
            tag_pose.pose = PoseStamped()
            tag_pose.pose.header = aruco_msg.header

            # Copier la pose
            tag_pose.pose.pose = marker.pose.pose

            # Ajouter à la liste
            tag_array.tags.append(tag_pose)

        # Publier
        self.tag_pub.publish(tag_array)

        # if len(tag_array.tags) > 0:
        #     self.get_logger().info(
        #         f'Converti {len(tag_array.tags)} marqueurs ArUco en TagPoseArray',
        #         throttle_duration_sec=1.0
        #     )


def main(args=None):
    rclpy.init(args=args)
    node = ArucoToTagConverter()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

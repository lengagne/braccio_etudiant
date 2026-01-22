#!/usr/bin/env python3
"""
Utilitaires pour la gestion des transformations TF
"""
from rclpy.duration import Duration
from geometry_msgs.msg import TransformStamped, Pose, PoseStamped
from tf2_ros import TransformBroadcaster, StaticTransformBroadcaster, Buffer, TransformListener
from tf2_ros import LookupException, ConnectivityException, ExtrapolationException
import tf2_geometry_msgs
import math
import numpy as np

from ..student_work.transformation import quaternion_to_rotation_matrix
from ..student_work.transformation import Transformation




class TFPublisher:
    """
    Classe utilitaire pour publier des transformations TF
    Simplifie la publication de TF depuis un nœud ROS 2
    """

    def __init__(self, node):
        """
        Initialise le publisher TF

        Args:
            node: Instance du nœud ROS 2 (rclpy.node.Node)
        """
        self.node = node
        self.tf_broadcaster = TransformBroadcaster(node)
        self.logger = node.get_logger()

    def publish_transform(self, parent_frame, child_frame, pose, stamp=None):
        """
        Publie une transformation TF unique

        Args:
            parent_frame (str): Frame parent
            child_frame (str): Frame enfant
            pose (Pose): Pose du child_frame dans le parent_frame
            stamp (Time, optional): Timestamp. Si None, utilise l'heure actuelle

        Example:
            >>> tf_pub = TFPublisher(self)
            >>> pose = Pose()
            >>> pose.position.x = 1.0
            >>> pose.orientation.w = 1.0
            >>> tf_pub.publish_transform('world', 'robot', pose)
        """
        t = TransformStamped()

        # Timestamp
        if stamp is None:
            t.header.stamp = self.node.get_clock().now().to_msg()
        else:
            t.header.stamp = stamp

        # Frames
        t.header.frame_id = parent_frame
        t.child_frame_id = child_frame

        # Transformation
        # t.transform.translation.x = pose.position.x
        # t.transform.translation.y = pose.position.y
        # t.transform.translation.z = pose.position.z
        # t.transform.rotation = pose.orientation

        # Publier
        self.tf_broadcaster.sendTransform(t)


def pose_to_transformation (pose_stamped):
        """Crée une Transformation depuis un message PoseStamped ROS2"""
        # Extraire position
        position = np.array([
            pose_stamped.pose.position.x,
            pose_stamped.pose.position.y,
            pose_stamped.pose.position.z
        ])

        # Extraire quaternion et convertir en matrice de rotation
        q = pose_stamped.pose.orientation
        rotation = quaternion_to_rotation_matrix(q.x, q.y, q.z, q.w)

        return Transformation(rotation, position)

#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from apriltag_msgs.msg import AprilTagDetectionArray
from geometry_msgs.msg import PoseStamped, PoseArray
from sensor_msgs.msg import CameraInfo
from braccio_msg.msg import AprilTagPose, AprilTagPoseArray
import numpy as np
import cv2


class AprilTagPoseEstimator(Node):
    def __init__(self):
        super().__init__('apriltag_pose_estimator')

        # Paramètres
        self.declare_parameter('tag_size', 0.03)  # Taille du tag en mètres
        self.tag_size = self.get_parameter('tag_size').value

        # Matrice de caméra et coefficients de distorsion
        self.camera_matrix = None
        self.dist_coeffs = None

        # Subscribers
        self.detection_sub = self.create_subscription(
            AprilTagDetectionArray,
            '/detections',
            self.detection_callback,
            10
        )

        self.camera_info_sub = self.create_subscription(
            CameraInfo,
            '/usb_cam/camera_info',
            self.camera_info_callback,
            10
        )

        # Publishers
        self.pose_pub = self.create_publisher(
            AprilTagPoseArray,
            '/apriltag_poses',
            10
        )

        self.get_logger().info('AprilTag Pose Estimator started')
        self.get_logger().info(f'Tag size: {self.tag_size}m')

    def camera_info_callback(self, msg):
        """Récupérer les paramètres intrinsèques de la caméra"""
        if self.camera_matrix is None:
            self.camera_matrix = np.array(msg.k).reshape(3, 3)
            self.dist_coeffs = np.array(msg.d)
            self.get_logger().info('Camera parameters received')
            self.get_logger().info(f'Camera matrix:\n{self.camera_matrix}')

    def detection_callback(self, msg):
        """Calculer la pose de chaque tag détecté"""
        if self.camera_matrix is None:
            self.get_logger().warn('Waiting for camera info...', throttle_duration_sec=2.0)
            return

        pose_array = AprilTagPoseArray()
        pose_array.header = msg.header

        for detection in msg.detections:
            # Extraire les coins du tag (dans l'image)
            corners_2d = np.array([
                [detection.corners[0].x, detection.corners[0].y],
                [detection.corners[1].x, detection.corners[1].y],
                [detection.corners[2].x, detection.corners[2].y],
                [detection.corners[3].x, detection.corners[3].y]
            ], dtype=np.float32)

            # Définir les coins 3D du tag dans son propre repère
            # Le tag est centré à l'origine, dans le plan z=0
            half_size = self.tag_size / 2.0
            corners_3d = np.array([
                [-half_size, -half_size, 0],  # Coin bas-gauche
                [ half_size, -half_size, 0],  # Coin bas-droit
                [ half_size,  half_size, 0],  # Coin haut-droit
                [-half_size,  half_size, 0],  # Coin haut-gauche
            ], dtype=np.float32)

            # Résoudre PnP pour obtenir la pose (rotation + translation)
            success, rvec, tvec = cv2.solvePnP(
                corners_3d,
                corners_2d,
                self.camera_matrix,
                self.dist_coeffs,
                flags=cv2.SOLVEPNP_IPPE_SQUARE  # Méthode optimale pour les carrés plans
            )

            if success:
                # Convertir le vecteur de rotation en matrice de rotation
                rmat, _ = cv2.Rodrigues(rvec)

                # Convertir la matrice de rotation en quaternion
                quat = self.rotation_matrix_to_quaternion(rmat)

                # Créer le message Pose
                tagpose = AprilTagPose()
                tagpose.id = detection.id

                # Position (translation)
                tagpose.pose.pose.position.x = float(tvec[0][0])
                tagpose.pose.pose.position.y = float(tvec[1][0])
                tagpose.pose.pose.position.z = float(tvec[2][0])

                # Orientation (quaternion)
                tagpose.pose.orientation.x = quat[0]
                tagpose.pose.orientation.y = quat[1]
                tagpose.pose.orientation.z = quat[2]
                tagpose.pose.orientation.w = quat[3]

                pose_array.poses.append(tagpose)

                # Log de la pose
                self.get_logger().info(
                    f'Tag {detection.id}: '
                    f'pos=({tvec[0][0]:.3f}, {tvec[1][0]:.3f}, {tvec[2][0]:.3f}) '
                    f'dist={np.linalg.norm(tvec):.3f}m',
                    throttle_duration_sec=1.0
                )

        if len(pose_array.poses) > 0:
            self.pose_pub.publish(pose_array)

    def rotation_matrix_to_quaternion(self, R):
        """Convertir une matrice de rotation 3x3 en quaternion (x, y, z, w)"""
        trace = np.trace(R)

        if trace > 0:
            s = 0.5 / np.sqrt(trace + 1.0)
            w = 0.25 / s
            x = (R[2, 1] - R[1, 2]) * s
            y = (R[0, 2] - R[2, 0]) * s
            z = (R[1, 0] - R[0, 1]) * s
        elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
            s = 2.0 * np.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2])
            w = (R[2, 1] - R[1, 2]) / s
            x = 0.25 * s
            y = (R[0, 1] + R[1, 0]) / s
            z = (R[0, 2] + R[2, 0]) / s
        elif R[1, 1] > R[2, 2]:
            s = 2.0 * np.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2])
            w = (R[0, 2] - R[2, 0]) / s
            x = (R[0, 1] + R[1, 0]) / s
            y = 0.25 * s
            z = (R[1, 2] + R[2, 1]) / s
        else:
            s = 2.0 * np.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1])
            w = (R[1, 0] - R[0, 1]) / s
            x = (R[0, 2] + R[2, 0]) / s
            y = (R[1, 2] + R[2, 1]) / s
            z = 0.25 * s

        return np.array([x, y, z, w])


def main(args=None):
    rclpy.init(args=args)
    node = AprilTagPoseEstimator()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

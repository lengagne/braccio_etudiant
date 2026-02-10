#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
import yaml
import os
from pathlib import Path


class CameraCalibrator(Node):
    def __init__(self):
        super().__init__('camera_calibrator')

        # Paramètres du damier
        self.declare_parameter('pattern_width', 8)   # Nombre de coins intérieurs en largeur
        self.declare_parameter('pattern_height', 6)  # Nombre de coins intérieurs en hauteur
        self.declare_parameter('square_size', 0.0241)  # Taille d'un carré en mètres
        self.declare_parameter('num_images', 20)     # Nombre d'images à capturer
        self.declare_parameter('output_file', '~/camera_calibration.yaml')

        self.pattern_width = self.get_parameter('pattern_width').value
        self.pattern_height = self.get_parameter('pattern_height').value
        self.square_size = self.get_parameter('square_size').value
        self.num_images = self.get_parameter('num_images').value
        self.output_file = os.path.expanduser(self.get_parameter('output_file').value)

        self.bridge = CvBridge()

        # Critères pour la détection des coins
        self.criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

        # Préparer les points 3D du damier
        self.objp = np.zeros((self.pattern_height * self.pattern_width, 3), np.float32)
        self.objp[:, :2] = np.mgrid[0:self.pattern_width, 0:self.pattern_height].T.reshape(-1, 2)
        self.objp *= self.square_size

        # Stockage des points
        self.objpoints = []  # Points 3D dans l'espace réel
        self.imgpoints = []  # Points 2D dans l'image

        self.images_captured = 0
        self.img_shape = None

        # Subscriber
        self.subscription = self.create_subscription(
            Image,
            'image_raw',
            self.image_callback,
            10
        )

        self.get_logger().info('=== Camera Calibrator démarré ===')
        self.get_logger().info(f'Damier: {self.pattern_width}x{self.pattern_height}')
        self.get_logger().info(f'Taille des carrés: {self.square_size}m')
        self.get_logger().info(f'Images à capturer: {self.num_images}')
        self.get_logger().info('Appuyez sur ESPACE pour capturer une image')
        self.get_logger().info('Appuyez sur C pour calculer la calibration')
        self.get_logger().info('Appuyez sur Q pour quitter')

    def image_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')

            if cv_image is None or cv_image.size == 0:
                self.get_logger().warn('Image vide reçue, frame ignorée')
                return

            if self.img_shape is None:
                h, w = cv_image.shape[:2]
                self.img_shape = (w, h)
                # self.img_shape = cv_image.shape[:2]

            # Convertir en niveaux de gris pour la détection
            gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)

            # Chercher les coins du damier
            ret, corners = cv2.findChessboardCorners(
                gray,
                (self.pattern_width, self.pattern_height),
                None
            )

            # Dessiner les coins si trouvés
            display_image = cv_image.copy()
            if ret:
                # Affiner la position des coins
                corners2 = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), self.criteria)

                # Dessiner les coins
                cv2.drawChessboardCorners(
                    display_image,
                    (self.pattern_width, self.pattern_height),
                    corners2,
                    ret
                )

                # Texte en vert si damier détecté
                cv2.putText(display_image, 'Damier detecte! ESPACE pour capturer',
                           (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
            else:
                # Texte en rouge si pas de damier
                cv2.putText(display_image, 'Damier non detecte',
                           (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)

            # Afficher le nombre d'images capturées
            cv2.putText(display_image, f'Images: {self.images_captured}/{self.num_images}',
                       (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)

            if display_image is None:
                return
            cv2.imshow('Camera Calibration', display_image)

            # Gérer les touches clavier
            key = cv2.waitKey(1) & 0xFF

            if key == ord(' ') and ret:  # ESPACE - Capturer l'image
                self.objpoints.append(self.objp)
                self.imgpoints.append(corners2)
                self.images_captured += 1
                self.get_logger().info(f'Image {self.images_captured}/{self.num_images} capturée ✓')

                if self.images_captured >= self.num_images:
                    self.get_logger().info('Nombre d\'images requis atteint! Appuyez sur C pour calibrer')

            elif key == ord('c') or key == ord('C'):  # C - Calculer calibration
                if self.images_captured < 5:
                    self.get_logger().warn('Au moins 5 images sont nécessaires pour la calibration')
                else:
                    self.calibrate_camera()

            elif key == ord('q') or key == ord('Q'):  # Q - Quitter
                self.get_logger().info('Arrêt de la calibration')
                cv2.destroyAllWindows()
                rclpy.shutdown()

        except Exception as e:
            self.get_logger().error(f'Erreur: {e}')

    def calibrate_camera(self):
        """Effectue la calibration de la caméra"""
        self.get_logger().info('=== Début de la calibration ===')

        # Calibration OpenCV
        ret, camera_matrix, dist_coeffs, rvecs, tvecs = cv2.calibrateCamera(
            self.objpoints,
            self.imgpoints,
            self.img_shape[::-1],
            None,
            None
        )

        if ret:
            # Calculer l'erreur de reprojection
            total_error = 0
            for i in range(len(self.objpoints)):
                imgpoints2, _ = cv2.projectPoints(
                    self.objpoints[i],
                    rvecs[i],
                    tvecs[i],
                    camera_matrix,
                    dist_coeffs
                )
                error = cv2.norm(self.imgpoints[i], imgpoints2, cv2.NORM_L2) / len(imgpoints2)
                total_error += error

            mean_error = total_error / len(self.objpoints)

            self.get_logger().info('✓ Calibration réussie!')
            self.get_logger().info(f'Erreur moyenne de reprojection: {mean_error:.4f} pixels')

            # Afficher les résultats
            self.get_logger().info('=== Matrice intrinsèque ===')
            self.get_logger().info(f'\n{camera_matrix}')
            self.get_logger().info('=== Coefficients de distorsion ===')
            self.get_logger().info(f'{dist_coeffs.ravel()}')

            # Sauvegarder les résultats
            self.save_calibration(camera_matrix, dist_coeffs, self.img_shape, mean_error)
        else:
            self.get_logger().error('✗ Échec de la calibration')

    def save_calibration(self, camera_matrix, dist_coeffs, image_shape, mean_error):
        """Sauvegarde les paramètres de calibration au format YAML ROS"""

        # Format correct pour ROS2
        calibration_data = {
            'image_width': int(image_shape[1]),
            'image_height': int(image_shape[0]),
            'camera_name': 'usb_cam',
            'camera_matrix': {
                'rows': 3,
                'cols': 3,
                'data': [float(x) for x in camera_matrix.flatten()]  # <-- Convertir en float
            },
            'distortion_model': 'plumb_bob',
            'distortion_coefficients': {
                'rows': 1,
                'cols': 5,
                'data': [float(x) for x in dist_coeffs.flatten()]  # <-- Convertir en float
            },
            'rectification_matrix': {
                'rows': 3,
                'cols': 3,
                'data': [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
            },
            'projection_matrix': {
                'rows': 3,
                'cols': 4,
                'data': [
                    float(camera_matrix[0, 0]), 0.0, float(camera_matrix[0, 2]), 0.0,
                    0.0, float(camera_matrix[1, 1]), float(camera_matrix[1, 2]), 0.0,
                    0.0, 0.0, 1.0, 0.0
                ]
            }
        }

        # Créer le répertoire si nécessaire
        calib_dir = os.path.expanduser('~/.ros/camera_info')
        os.makedirs(calib_dir, exist_ok=True)
        calib_file = os.path.join(calib_dir, 'usb_cam.yaml')

        # Sauvegarder en YAML
        with open(calib_file, 'w') as f:
            yaml.dump(calibration_data, f, default_flow_style=False, sort_keys=False)

        self.get_logger().info(f'✓ Calibration sauvegardée dans: {calib_file}')


def main(args=None):
    rclpy.init(args=args)
    node = CameraCalibrator()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

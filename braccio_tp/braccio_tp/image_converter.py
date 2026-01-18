#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2


class ImageConverter(Node):
    def __init__(self):
        super().__init__('image_converter')

        self.bridge = CvBridge()

        # Subscriber sur l'image d'entrée
        self.subscription = self.create_subscription(
            Image,
            'image_in',
            self.image_callback,
            10
        )

        # Publisher sur l'image de sortie
        self.publisher = self.create_publisher(
            Image,
            'image_out',
            10
        )

        self.get_logger().info('Image Converter Node démarré')

    def image_callback(self, msg):
        try:
            # Convertir selon l'encodage d'entrée
            if msg.encoding in ['yuv422_yuy2', 'yuyv']:
                # Obtenir l'image YUV brute
                yuv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
                # Convertir en BGR
                cv_image = cv2.cvtColor(yuv_image, cv2.COLOR_YUV2BGR_YUY2)

            elif msg.encoding == 'uyvy':
                yuv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
                cv_image = cv2.cvtColor(yuv_image, cv2.COLOR_YUV2BGR_UYVY)

            elif msg.encoding == 'rgb8':
                # Convertir RGB en BGR
                rgb_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='rgb8')
                cv_image = cv2.cvtColor(rgb_image, cv2.COLOR_RGB2BGR)

            elif msg.encoding == 'mono8':
                # Convertir niveaux de gris en BGR
                mono_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='mono8')
                cv_image = cv2.cvtColor(mono_image, cv2.COLOR_GRAY2BGR)

            elif msg.encoding == 'bgr8':
                # Déjà en BGR, pas de conversion nécessaire
                cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

            else:
                # Essayer une conversion automatique
                self.get_logger().warn(f'Encodage non reconnu: {msg.encoding}, tentative de conversion automatique')
                cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

            # Convertir l'image OpenCV BGR en message ROS Image
            output_msg = self.bridge.cv2_to_imgmsg(cv_image, encoding='bgr8')

            # Copier le header original pour conserver le timestamp
            output_msg.header = msg.header

            # Publier l'image convertie
            self.publisher.publish(output_msg)

        except Exception as e:
            self.get_logger().error(f'Erreur de conversion: {e}')


def main(args=None):
    rclpy.init(args=args)
    node = ImageConverter()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

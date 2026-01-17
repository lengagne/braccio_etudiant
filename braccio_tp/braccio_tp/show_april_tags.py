#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from apriltag_msgs.msg import AprilTagDetectionArray
from cv_bridge import CvBridge
import cv2

class AprilTagVisualizer(Node):
    def __init__(self):
        super().__init__('show_april_tags')
        self.bridge = CvBridge()
        self.image_sub = self.create_subscription( Image, '/image_raw', self.image_callback, 10)
        self.detection_sub = self.create_subscription(AprilTagDetectionArray, '/detections', self.detection_callback, 10)
        self.image_pub = self.create_publisher(Image, '/detections_image', 10)
        self.latest_detections = None
        self.get_logger().info('AprilTag Visualizer started')
        mem_image = Image()

    def detection_callback(self, msg):
        self.latest_detections = msg
        self.get_logger().info(f'Received {len(msg.detections)} detections')

    def image_callback(self, msg):

        try:
            # self.get_logger().info(f'Encodage de l\'image: {msg.encoding}')

            # Convertir YUV422 en BGR
            if msg.encoding == 'yuv422_yuy2' or msg.encoding == 'yuyv':
                # Utiliser passthrough pour obtenir les données brutes
                yuv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')

                # Convertir YUV422 (YUY2) en BGR
                cv_image = cv2.cvtColor(yuv_image, cv2.COLOR_YUV2BGR_YUY2)

            elif msg.encoding == 'uyvy':
                yuv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
                cv_image = cv2.cvtColor(yuv_image, cv2.COLOR_YUV2BGR_UYVY)

            else:
                # Pour les autres encodages
                cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

            # self.get_logger().info(f'Taille image convertie: {cv_image.shape}')

        except Exception as e:
            self.get_logger().error(f'Erreur de conversion: {e}')

        # Vérifier que l'image n'est pas vide
        if cv_image is None or cv_image.size == 0:
            self.get_logger().warn('Received empty image')
            return

        # Dessiner les détections
        if self.latest_detections:
            for detection in self.latest_detections.detections:
                corners = detection.corners
                pts = [(int(c.x), int(c.y)) for c in corners]

                # Dessiner le contour du tag
                for i in range(4):
                    cv2.line(cv_image, pts[i], pts[(i+1)%4], (0, 255, 0), 2)

                # Dessiner les coins
                for pt in pts:
                    cv2.circle(cv_image, pt, 5, (255, 0, 0), -1)

                # Afficher l'ID au centre
                center_x = int(sum([c.x for c in corners]) / 4)
                center_y = int(sum([c.y for c in corners]) / 4)

                # Fond noir pour le texte
                text = f"ID: {detection.id}"
                text_size = cv2.getTextSize(text, cv2.FONT_HERSHEY_SIMPLEX, 0.7, 2)[0]
                cv2.rectangle(cv_image,
                            (center_x - 5, center_y - text_size[1] - 5),
                            (center_x + text_size[0] + 5, center_y + 5),
                            (0, 0, 0), -1)

                # Texte en rouge
                cv2.putText(cv_image, text, (center_x, center_y),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)

                # cv2.imshow("Ma Fenêtre d'Image", cv_image)

        # Publier l'image annotée
        try:
            out_msg = self.bridge.cv2_to_imgmsg(cv_image, encoding='bgr8')
            out_msg.header = msg.header  # Conserver le header original
            self.image_pub.publish(out_msg)
            #self.image_pub.publish(mem_image)
        except Exception as e:
            self.get_logger().error(f'Error publishing image: {e}')

def main(args=None):
    rclpy.init(args=args)
    node = AprilTagVisualizer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

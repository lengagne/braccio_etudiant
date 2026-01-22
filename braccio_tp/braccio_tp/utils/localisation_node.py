#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from aruco_msgs.msg import MarkerArray  # ou apriltag_msgs si vous utilisez ça
from tag_msgs.msg import TagPose, TagPoseArray

from braccio_tp.student_work.transformation import Transformation
from braccio_tp.student_work.localisation import compute_camera_pose

from braccio_tp.utils.static_marker_utils import StaticMarker
from braccio_tp.utils.tf_utils import pose_to_transformation

class Localisation(Node):
    def __init__(self):
        super().__init__('localisation')

        # Subscriber ArUco
        self.aruco_sub = self.create_subscription(TagPoseArray,'tag_list',self.localisation_callback,10)

        self.get_logger().info(f'Localisation démarré')

        self.static_markers = []
        self.static_markers.append(StaticMarker (31,True))

        self.camera_frame = Transformation()


    def localisation_callback(self, tag_msg):
        """
        Callback pour convertir MarkerArray en TagPoseArray
        """

        # on localise la camera en fonction d'un marker fixe connu
        for marker in tag_msg.tags:
            for static_marker in self.static_markers:
                if marker.id == static_marker.id and static_marker.seen==True:
                    marker_pose = pose_to_transformation( marker.pose)
                    self.camera_frame = compute_camera_pose( marker_pose,
                                                             static_marker.transform)
                    self.get_logger().info(f'On a trouvé le marker  {marker.id} on va pouvoir définir la pause de la caméra"')

                    break


                self.get_logger().info(f'Localisation found tag {marker.id}')



def main(args=None):
    rclpy.init(args=args)
    node = Localisation()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

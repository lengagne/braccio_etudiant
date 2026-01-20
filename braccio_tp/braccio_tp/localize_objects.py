#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from tag_msgs.msg import TagPose, TagPoseArray

from geometry_msgs.msg import TransformStamped
# from tf2_ros import TransformBroadcaster, StaticTransformBroadcaster


from braccio_tp.student_work.transformation import Transformation
from braccio_tp.utils.tf_utils import TFPublisher



class LocalizeObjects(Node):
    def __init__(self): # Constructeur qui est appelé quand le noeud est lancé.
        super().__init__('localize_objects')

        # Subscriber
        self.tag_pub = self.create_subscription(TagPoseArray,'tag_list',self.general_localizer,10)


        #publisher sur tf
        self.tf_broadcaster = TFPublisher(self)

        self.get_logger().info(f'localize_objects démarré')

    def general_localizer(self, tag_msg):
        # Créer le message
        t = Transformation()
        #
        # # Timestamp
        # t.header.stamp = self.get_clock().now().to_msg()
        #
        # # Frames
        # t.header.frame_id = 'parent_frame'  # Frame parent
        # t.child_frame_id = 'child_frame'     # Frame enfant
        #
        # # Position
        # t.transform.translation.x = 1.0
        # t.transform.translation.y = 2.0
        # t.transform.translation.z = 0.5
        #
        # # Orientation (quaternion)
        # t.transform.rotation.x = 0.0
        # t.transform.rotation.y = 0.0
        # t.transform.rotation.z = 0.0
        # t.transform.rotation.w = 1.0

        # Publier
        self.tf_broadcaster.publish_transform("world","frame1",t)



def main(args=None):
    rclpy.init(args=args)
    node = LocalizeObjects()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

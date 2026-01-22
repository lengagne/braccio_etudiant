#!/usr/bin/env python3
"""
Utilitaires pour la gestion des transformations TF
"""

from braccio_tp.student_work.transformation import Transformation


class StaticMarker:
    """
    Classe utilitaire pour publier des transformations TF
    Simplifie la publication de TF depuis un nœud ROS 2
    """

    def __init__(self, id, seen=False, transform=Transformation() ):
        """
        Initialise le publisher TF

        Args:
            node: Instance du nœud ROS 2 (rclpy.node.Node)
        """
        self.id = id
        self.seen = seen
        self.transform = transform



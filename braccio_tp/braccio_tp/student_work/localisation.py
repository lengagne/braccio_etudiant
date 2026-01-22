
from braccio_tp.student_work.transformation import Transformation


# Entrees
#   marker_transform : pose du marqueur dans le repere de la camera
#   static_transform : pose du marqueur dans le repere de reference
#   retourne la pose de la camera dans le repere de reference


def compute_camera_pose( marker_transform,
                         static_transform):

    T = static_transform * marker_transform.inverse()
    # T = Transformation()
    return T


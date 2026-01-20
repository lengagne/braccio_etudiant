from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():

    # Chemin vers les packages
    this_pkg = get_package_share_directory('braccio_common')

    # Déclarer l'argument pour choisir le type de marqueur
    marker_type_arg = DeclareLaunchArgument(
        'marker',
        default_value='aruco',
        description='Type de marqueur à détecter: aruco ou apriltag'
    )

    # Récupérer les configurations
    marker_type = LaunchConfiguration('marker')

   # Condition pour ArUco
    is_aruco = PythonExpression(['"', marker_type, '" == "aruco"'])

    # Condition pour AprilTag
    is_apriltag = PythonExpression(['"', marker_type, '" == "apriltag"'])

  # ============== Configuration ArUco ==============
    aruco_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(this_pkg, 'launch', 'ArucoDetection.launch.py')
        ),
        # launch_arguments={
        #     'camera_topic': camera_topic,
        #     'namespace': namespace,
        # }.items(),
        condition=IfCondition(is_aruco)
    )

    # ============== Configuration AprilTag ==============
    apriltag_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(this_pkg, 'launch', 'ApriltagDetection.launch.py')
        ),
        # launch_arguments={
        #     'camera_topic': camera_topic,
        #     'namespace': namespace,
        # }.items(),
        condition=IfCondition(is_apriltag)
    )

    # ============== Lancement de la caméra ==============


    camera_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(this_pkg, 'launch', 'RunCamera.launch.py')
        )
    )

    return LaunchDescription([
        # Arguments
        marker_type_arg,

        # Lancement conditionnel
        aruco_launch,
        apriltag_launch,

        # paquets communs
        camera_launch
    ])

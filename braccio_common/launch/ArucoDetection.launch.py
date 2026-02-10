from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():

    aruco_detector_node = Node(
        package='aruco_ros',  # ou votre package
        executable='marker_publisher',
        name='aruco_detector',
        # namespace=namespace,
        parameters=[
            # config_file,
        {
            'image_is_rectified': True,
            'marker_size': 0.024,  # taille en mètres
            # # Paramètres de visualisation (ca ne change rien) , FIXME a voir en le mettant dans un config file
            # 'axis_length': 0.5,      # Axes plus grands (en mètres)
            # 'axis_thickness': 3,     # Épaisseur des axes
            # 'text_scale': 0.1,       # Texte plus petit
            # 'text_thickness': 1,     # Épaisseur du texte

        }],
        remappings=[
            ('image', '/usb_cam/image_raw'),
            ('camera_info', '/usb_cam/camera_info'),
        ]
    )

    # Node usb_cam
    aruco_to_tag_converter_node = Node(
        package='braccio_common',
        executable='aruco_to_tag_converter',
    )

    return LaunchDescription([
        aruco_detector_node,
        aruco_to_tag_converter_node,
    ])

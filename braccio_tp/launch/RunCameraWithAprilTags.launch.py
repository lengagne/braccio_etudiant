from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Déclaration de l'argument pour choisir la caméra
    camera_device_arg = DeclareLaunchArgument(
        'camera_device',
        default_value='/dev/video0',
        description='Chemin du périphérique caméra (ex: /dev/video0, /dev/video2)'
    )

    # Configuration pour utiliser l'argument
    camera_device = LaunchConfiguration('camera_device')
    camera_info_url = 'file://' + os.path.expanduser('~/.ros/camera_info/usb_cam.yaml')


    # Chemin vers le fichier de config des april tags
    config_file = os.path.join(
        get_package_share_directory('braccio_common'),
        'config',
        'tags.yaml'
    )

    # Node usb_cam
    usb_cam_node = Node(
        package='usb_cam',
        executable='usb_cam_node_exe',
        name='usb_cam',
        namespace='usb_cam',
        parameters=[{
            'video_device': camera_device,
            'framerate': 30.0,
            'image_width': 640,
            'image_height': 480,
            'pixel_format': 'yuyv',
            'camera_frame_id': 'usb_cam',
            'io_method': 'mmap',
            'camera_name': 'usb_cam',  # <-- IMPORTANT: d
            'camera_info_url': camera_info_url,  # <-- Chemin vers le fichier de calibration
        }],
        output='screen'
    )

    # Node AprilTag
    apriltag_node = Node(
        package='apriltag_ros',
        executable='apriltag_node',
        name='apriltag_node',
        remappings=[
            ('image_rect', 'usb_cam/image_raw'),
            ('camera_info', 'usb_cam/camera_info'),
        ],
        parameters=[{
            'image_transport': 'raw',
            'publish_tf': False,
            'family': '36h11',
            'size': 0.03,
            'max_hamming': 0,
            'z_aligned': False,  # <-- Ajoutez ceci
        }],
        # parameters=[
        #     config_file,
        #     {
        #         'image_transport': 'raw',
        #         'publish_tf': False,  # <-- NE PAS publier les TF
        #     }
        # ],
        output='screen'
    )

    # Ajoutez ce node dans SeeAprilTags.launch.py
    pose_estimator_node = Node(
        package='braccio_common',
        executable='apriltag_pose_estimator',
        name='apriltag_pose_estimator',
        parameters=[{
            'tag_size': 0.03,  # <-- Mettez la vraie taille de vos tags
        }],
        output='screen'
    )


    # Node pour afficher les AprilTags
    show_april_tags_node = Node(
        package='braccio_common',
        executable='show_april_tags',
        name='show_april_tags',
        output='screen',
        remappings=[
            ('image_raw', 'usb_cam/image_raw'),
        ],
    )

    # Node rqt_image_view
    rqt_image_view_node = Node(
        package='rqt_image_view',
        executable='rqt_image_view',
        name='rqt_image_view',
        output='screen'
    )

    return LaunchDescription([
        camera_device_arg,
        usb_cam_node,
        apriltag_node,
        show_april_tags_node,
        rqt_image_view_node,
        pose_estimator_node,
    ])

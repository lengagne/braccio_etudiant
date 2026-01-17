from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # Déclaration de l'argument pour choisir la caméra
    camera_device_arg = DeclareLaunchArgument(
        'camera_device',
        default_value='/dev/video0',
        description='Chemin du périphérique caméra (ex: /dev/video0, /dev/video2)'
    )

    # Configuration pour utiliser l'argument
    camera_device = LaunchConfiguration('camera_device')

    # Node usb_cam
    usb_cam_node = Node(
        package='usb_cam',
        executable='usb_cam_node_exe',
        name='usb_cam',
        parameters=[{
            'video_device': camera_device,
            'framerate': 30.0,
            'image_width': 640,
            'image_height': 480,
            'pixel_format': 'yuyv',
            'camera_frame_id': 'usb_cam',
            'io_method': 'mmap',
        }],
        output='screen'
    )

    # Node AprilTag
    apriltag_node = Node(
        package='apriltag_ros',
        executable='apriltag_node',
        name='apriltag_node',
        remappings=[
            ('image_rect', '/image_raw'),
            ('camera_info', '/camera_info'),
        ],
        parameters=[{
            'image_transport': 'raw',
            'pose_estimation_method': '',
        }],
        output='screen'
    )

    # Node pour afficher les AprilTags
    show_april_tags_node = Node(
        package='braccio_tp',
        executable='show_april_tags',
        name='show_april_tags',
        output='screen'
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
    ])

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():

    pkg_braccion_common = get_package_share_directory('braccio_common')

    # Arguments
    camera_device_arg = DeclareLaunchArgument(
        'camera_device',
        default_value='/dev/video0',
        description='Chemin du périphérique camera_device'
    )

    pattern_width_arg = DeclareLaunchArgument(
        'pattern_width',
        default_value='8',
        description='Nombre de coins intérieurs en largeur du damier'
    )

    pattern_height_arg = DeclareLaunchArgument(
        'pattern_height',
        default_value='6',
        description='Nombre de coins intérieurs en hauteur du damier'
    )

    square_size_arg = DeclareLaunchArgument(
        'square_size',
        default_value='0.0241',
        description='Taille d\'un carré du damier en mètres'
    )

    num_images_arg = DeclareLaunchArgument(
        'num_images',
        default_value='20',
        description='Nombre d\'images à capturer pour la calibration'
    )

    # Configurations
    camera_device = LaunchConfiguration('camera_device')
    pattern_width = LaunchConfiguration('pattern_width')
    pattern_height = LaunchConfiguration('pattern_height')
    square_size = LaunchConfiguration('square_size')
    num_images = LaunchConfiguration('num_images')

    # Node de calibration OpenCV
    calibrator_node = Node(
        package='braccio_common',
        executable='camera_calibrator',
        name='camera_calibrator',
        parameters=[{
            'pattern_width': pattern_width,
            'pattern_height': pattern_height,
            'square_size': square_size,
            'num_images': num_images,
        }],
        remappings=[
            ('image_raw', 'usb_cam/image_raw'),
        ],
        output='screen'
    )

    camera_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_braccion_common, 'launch', 'RunCamera.launch.py')
        )
    )

    return LaunchDescription([
        camera_device_arg,
        pattern_width_arg,
        pattern_height_arg,
        square_size_arg,
        num_images_arg,
        # usb_cam_node,
        # image_converter_node,
        calibrator_node,
        camera_launch
    ])

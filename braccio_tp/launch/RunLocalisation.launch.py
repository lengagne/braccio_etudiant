from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
# from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Arguments

    pkg_braccion_common = get_package_share_directory('braccio_common')

    camera_with_tags_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_braccion_common, 'launch', 'RunCameraWithTags.launch.py')
        ),
    )

    localisation_node = Node(
        package='braccio_tp',
        executable='localisation',
        name='localisation',
    )

    return LaunchDescription([
        camera_with_tags_launch,
        localisation_node,
    ])

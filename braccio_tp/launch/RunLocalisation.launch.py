from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Arguments
    pkg_share = get_package_share_directory('braccio_tp')
    pkg_braccion_common = get_package_share_directory('braccio_common')
    pkg_braccion_robot = get_package_share_directory('braccio_robot')

    camera_with_tags_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_braccion_common, 'launch', 'RunCameraWithTags.launch.py')
        ),
    )

    rviz2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_braccion_robot, 'launch', 'SeeRobotOnRviz.launch.py')
        ),
    )

    # Chemin vers le fichier de markers
    default_markers_file = os.path.join(pkg_share, 'config', 'static_markers.yaml')

    static_markers_arg = DeclareLaunchArgument(
        'static_markers',
        default_value=default_markers_file,
        description='Path to static markers YAML file'
    )

    localisation_node = Node(
        package='braccio_tp',
        executable='marker_broadcast',
        name='localisation',
        parameters=[{
            'static_markers': LaunchConfiguration('static_markers')
        }],
        output='screen'
    )

    return LaunchDescription([
        static_markers_arg,
        camera_with_tags_launch,
        localisation_node,
        rviz2_launch

    ])

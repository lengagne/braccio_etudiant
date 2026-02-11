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

    rviz2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_braccion_robot, 'launch', 'SeeRobotOnRviz.launch.py')
        ),
    )

    joy_node = Node(
        package='joy',
        executable='joy_node',
        name='joy_node',
        output='screen'
    )

    pad_to_target_node = Node(
        package='braccio_common',
        executable='pad_to_target',
        name='pad_to_target',
        output='screen'
    )

    return LaunchDescription([
        joy_node,
        rviz2_launch,
        pad_to_target_node
    ])

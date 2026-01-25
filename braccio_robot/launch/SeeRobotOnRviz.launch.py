from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import Command
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_share = get_package_share_directory('braccio_robot')

    # Chemin vers le fichier URDF
    urdf_file = os.path.join(pkg_share, 'urdf', 'braccio_arm.urdf')
    # Ou si c'est un xacro :
    # urdf_file = os.path.join(pkg_share, 'urdf', 'braccio.xacro')

    # Charger l'URDF
    robot_description = ParameterValue(
        Command(['cat ', urdf_file]),
        value_type=str
    )
    # Si c'est un fichier xacro, utilisez plutôt :
    # robot_description = ParameterValue(
    #     Command(['xacro ', urdf_file]),
    #     value_type=str
    # )

    # Robot State Publisher - publie les transformations du robot
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description}]
    )

    # Joint State Publisher GUI - pour bouger les joints manuellement
    joint_state_publisher_gui_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        output='screen'
    )

    # Fichier de configuration RViz
    rviz_config_file = os.path.join(pkg_share, 'config', 'braccio_view.rviz')

    # RViz2
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file],
        output='screen'
    )

    return LaunchDescription([
        robot_state_publisher_node,
        joint_state_publisher_gui_node,
        rviz_node,
    ])

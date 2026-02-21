from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import Command
from launch.conditions import IfCondition, UnlessCondition
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_share = get_package_share_directory('braccio_robot')

    use_gui_arg = DeclareLaunchArgument(
        'use_gui',
        default_value='True',
        description='Whether to launch the joint_state_publisher_gui node'
    )

    use_robot_arg = DeclareLaunchArgument(
        'use_robot',
        default_value='False',
        description='Whether to launch the node to control the robot'
    )

    # Chemin vers le fichier URDF
    urdf_file = os.path.join(pkg_share, 'urdf', 'braccio_arm.urdf')
    cubes_file = os.path.join(pkg_share, 'urdf', 'Cubes.urdf')

    # Charger l'URDF
    robot_description = ParameterValue(
        Command(['cat ', urdf_file]),
        value_type=str
    )

    cubes_description = ParameterValue(
        Command(['cat ', cubes_file]),
        value_type=str
    )

    # Robot State Publisher - publie les transformations du robot
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description}]
    )

    cube_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher_cubes',
        namespace='cubes',
        output='screen',
        parameters=[{'robot_description': cubes_description}]
    )


    # Joint State Publisher GUI - pour bouger les joints manuellement
    joint_state_publisher_gui_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        output='screen',
        condition=IfCondition(LaunchConfiguration('use_gui')),
    )

    # Joint State Publisher GUI - pour bouger les joints manuellement
    braccio_node = Node(
        package='ros2_braccio',
        executable='braccio',
        name='braccio',
        output='screen',
        condition=IfCondition(LaunchConfiguration('use_robot'))
    )

    convert_angle_node = Node(
        package='ros2_braccio',
        executable='convert_joint_angle',
        name='convert_joint_angle',
        output='screen',
        condition=IfCondition(LaunchConfiguration('use_robot')),
        remappings=[
            ('braccio/joint_angles','joint_states'),
        ],
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
        use_gui_arg,
        use_robot_arg,
        robot_state_publisher_node,
        joint_state_publisher_gui_node,
        cube_state_publisher_node,
        rviz_node,
        braccio_node,
        convert_angle_node
    ])

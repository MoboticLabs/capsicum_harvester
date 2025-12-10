import os
import xacro
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from ament_index_python.packages import get_package_share_directory

def generate_launch_description():


    use_sim_time = LaunchConfiguration('use_sim_time')

    pkg_path = os.path.join(get_package_share_directory('robot_description'))
    xacro_file = os.path.join(pkg_path, 'urdf', 'robot_core.xacro')
    robot_desc = xacro.process_file(xacro_file).toxml()

    node_robot_state_publisher = Node(
        package = 'robot_state_publisher',
        executable = 'robot_state_publisher',
        name = 'robot_state_publisher',
        output = 'screen',
        parameters = [{'use_sim_time': use_sim_time, 'robot_description': robot_desc}]
    )

    node_joint_state_publisher = Node(
        package = 'joint_state_publisher',
        executable = 'joint_state_publisher',
        name = 'joint_state_publisher',
        output = 'screen'
    )

    node_rviz = Node(
        package = 'rviz2',
        executable = 'rviz2',
        name = 'rviz2',
        output = 'screen',
        arguments = ['-d' + os.path.join(get_package_share_directory('robot_description'), 'rviz', 'rviz_env.rviz')]
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value = 'false',
            description = 'use sim time if true'
        ),
        node_robot_state_publisher,
        #node_joint_state_publisher,
        node_rviz

    ])



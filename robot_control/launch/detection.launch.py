from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='robot_control',
            executable='detection_node.py',
            name='detection_node',
            output='screen'
        )
    ])

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    """
    Launch all nodes needed for the complete system:
    1. IK solver - converts target poses to joint angles
    2. Serial bridge - sends joint angles to Arduino
    """
    return LaunchDescription([
        # IK Solver Node
        Node(
            package='robot_control',
            executable='ik.py',
            name='inverse_kinematics',
            output='screen'
        ),
        
        # Serial Bridge Node
        Node(
            package='robot_control',
            executable='serial_bridge_node.py',
            name='serial_bridge',
            output='screen',
            parameters=[
                {'serial_port': '/dev/ttyACM0'},
                {'baud_rate': 115200},
                {'steps_per_rev': 1600}
            ]
        ),
        
        # Uncomment to enable automatic detection:
        Node(
            package='robot_control',
            executable='detection_node.py',
            name='detection_node',
            output='screen'
        ),
    ])

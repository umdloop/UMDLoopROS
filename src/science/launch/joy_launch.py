from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(package='keyboard', executable='keyboard', name="keyboard_node",         parameters=[
            {"allow_repeat": True},
            {"repeat_delay": 1}
        ])
    ])

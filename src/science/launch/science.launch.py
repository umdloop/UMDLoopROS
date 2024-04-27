from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='science',
            executable='science_node',
            name='science_node'
        )
    ])

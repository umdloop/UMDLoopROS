from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(package='keyboard', executable='keyboard', name="keyboard_node",         parameters=[
            {"allow_repeat": True},
            {"repeat_delay": 1}
        ]),
        #Node(package='joy', executable='joy_node', name='joy_node'),
        Node(
            package='science',
            executable='science_node',
            name='science_node'
        )
    ])

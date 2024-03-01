# Copyright 2020 ros2_control Development Team
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution, LaunchConfiguration

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory


from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    # Declare arguments
    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            "gui",
            default_value="true",
            description="Start RViz2 automatically with this launch file.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "use_mock_hardware",
            default_value="false",
            description="Start robot with mock hardware mirroring command to its states.",
        )
    )

    # Initialize Arguments
    gui = LaunchConfiguration("gui")
    use_mock_hardware = LaunchConfiguration("use_mock_hardware")



    config_file_path = PathJoinSubstitution(
                [FindPackageShare("diff_drive"), "config", "input_config.yaml"]
            )
    joy_params = PathJoinSubstitution(
                [FindPackageShare("diff_drive"), "config", "joystick.yaml"]
            )
    
    joy_node = Node(
            package='joy_linux', executable='joy_linux_node', name='joy_node',
            parameters=[joy_params])
        
    teleop_twist_joy_node = Node(
            package='teleop_twist_joy', executable='teleop_node',
            name='teleop_twist_joy_node', 
            parameters=[config_file_path],
            remappings={('/cmd_vel', '/joy_cmd_vel')},
            )
    teleop_twist_keyboard_node = Node(
            package='teleop_twist_keyboard', executable='teleop_twist_keyboard',
            name='teleop_twist_keyboard_node', 
            remappings={('/cmd_vel', '/key_cmd_vel')},
            output='screen',
            prefix = 'xterm -e',
            )
    twist_mux = Node(
            package="twist_mux",
            executable="twist_mux",
            parameters=[config_file_path],
            remappings=[('/cmd_vel_out','/diffbot_base_controller/cmd_vel_unstamped')]
        )
    

    nodes = [
        joy_node,
        teleop_twist_joy_node,
        teleop_twist_keyboard_node,
        twist_mux
        
    ]

    return LaunchDescription(declared_arguments + nodes)

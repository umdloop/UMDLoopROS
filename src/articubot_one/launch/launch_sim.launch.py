import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    package_name = 'diff_drive'
    joy_config = LaunchConfiguration('joy_config')
    joy_dev = LaunchConfiguration('joy_dev')

    declare_joy_vel = DeclareLaunchArgument('joy_vel', default_value='/test_twist_mux/cmd_vel')
    declare_joy_config = DeclareLaunchArgument('joy_config', default_value='ps3')
    declare_joy_dev = DeclareLaunchArgument('joy_dev', default_value='/dev/input/js0')
    robot_state_publisher = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory(package_name), 'launch', 'rsp.launch.py')
        ), 
        launch_arguments={'use_sim_time': 'true', 'use_ros2_control': 'true'}.items()
    )

    gazebo_params_file = os.path.join(get_package_share_directory(package_name), 'config', 'gazebo_params.yaml')

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')
        ),
        launch_arguments={'extra_gazebo_args': '--ros-args --params-file ' + gazebo_params_file}.items()
    )

    spawn_entity = Node(
        package='gazebo_ros', 
        executable='spawn_entity.py',
        arguments=['-topic', 'robot_description', '-entity', 'my_bot'],
        output='screen'
    )

    diff_drive_spawner = Node(
        package="controller_manager",
        executable="spawner.py",
        arguments=["diff_cont"],
    )

    joint_broad_spawner = Node(
        package="controller_manager",
        executable="spawner.py",
        arguments=["joint_broad"],
    )

    joy_node = Node(
        package='joy', 
        executable='joy_node', 
        name='joy_node',
        parameters=[{
            'dev': 'joy_dev',
            'deadzone': 0.3,
            'autorepeat_rate': 20.0,
        }]
    )
    
    config_file_path = os.path.join(get_package_share_directory('twist_mux_test'),'config', 'params.yaml')
    
    teleop_twist_joy_node = Node(
        package='teleop_twist_joy', 
        executable='teleop_node',
        name='teleop_twist_joy_node', 
        parameters=[config_file_path],
        remappings={('/cmd_vel', '/joy_cmd_vel')},
    )

    teleop_twist_keyboard_node = Node(
        package='teleop_twist_keyboard', 
        executable='teleop_twist_keyboard',
        name='teleop_twist_keyboard_node', 
        remappings={('/cmd_vel', '/key_cmd_vel')},
        output='screen',
        prefix='xterm -e',
    )

    twist_mux_node = Node(
        package='twist_mux',
        executable='twist_mux',
        output='screen',
        remappings={('/cmd_vel_out', LaunchConfiguration('joy_vel'))},
        parameters=[config_file_path],
    )

    return LaunchDescription([
        robot_state_publisher,
        gazebo,
        spawn_entity,
        diff_drive_spawner,
        joint_broad_spawner,
        joy_node,
        teleop_twist_joy_node,
        teleop_twist_keyboard_node,
        twist_mux_node
    ])

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    package_name = 'diff_drive'  # <-- CHANGE ME

    # Declare the launch arguments
    use_sim_time_arg = DeclareLaunchArgument('use_sim_time', default_value='true')
    use_ros2_control_arg = DeclareLaunchArgument('use_ros2_control', default_value='true')

    rsp = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory(package_name), 'launch', 'rsp.launch.py')),
        launch_arguments={'use_sim_time': LaunchConfiguration('use_sim_time'),
                          'use_ros2_control': LaunchConfiguration('use_ros2_control')}.items()
    )

    joystick = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory(package_name), 'launch', 'joystick.launch.py')),
        launch_arguments={'use_sim_time': LaunchConfiguration('use_sim_time')}.items()
    )

    twist_mux_params = os.path.join(get_package_share_directory(package_name), 'config', 'twist_mux.yaml')
    twist_mux = Node(
        package="twist_mux",
        executable="twist_mux",
        parameters=[twist_mux_params, {'use_sim_time': LaunchConfiguration('use_sim_time')}],
        remappings=[('/cmd_vel_out', '/diff_cont/cmd_vel_unstamped')]
    )

    gazebo_params_file = os.path.join(get_package_share_directory(package_name), 'config', 'gazebo_params.yaml')
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')),
        launch_arguments={'extra_gazebo_args': '--ros-args --params-file ' + gazebo_params_file}.items()
    )

    spawn_entity = Node(package='gazebo_ros', executable='spawn_entity.py',
                        arguments=['-topic', 'robot_description',
                                   '-entity', 'my_bot'],
                        output='screen')

    diff_drive_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["diff_cont"],
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_broad"]
    )

    return LaunchDescription([
        use_sim_time_arg,
        use_ros2_control_arg,
        rsp,
        joystick,
        twist_mux,
        gazebo,
        spawn_entity,
        diff_drive_spawner,
        joint_state_broadcaster_spawner
    ])

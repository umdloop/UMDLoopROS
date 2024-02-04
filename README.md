# UMDLoopROS

This is all written assuming an Ubuntu 22.04 LTS install with ROS2 Humble installed. 

You will need to install these packages additionally.


## Navigation Packages
```
sudo apt-get install ros-humble-navigation2
sudo apt-get install ros-humble-nav2-bringup
sudo apt-get install ros-humble-turtlebot3-gazebo
```

## Control Packages
```
sudo apt-get install ros-humble-ros2-control
sudo apt-get install ros-humble-ros2-controllers
sudo apt-get install ros-humble-gazebo-ros2-control
```

# Run Gazebo Simulation

```
colcon build
source install/setup.bash
ros2 launch diff_drive launch_sim.launch.py world:=src/diff_drive/worlds/empty.world

// to control it with the keyboard?

ros2 run teleop_twist_keyboard teleop_twist_keyboard
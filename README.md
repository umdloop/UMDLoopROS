# UMDLoopROS

This is all written assuming an Ubuntu 22.04 LTS install with ROS2 Humble installed. 

You will need to install these packages additionally.

## Getting Started

You need to install the myactuator_rmd package globally to get started (TODO: Make this not as terrible)
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

## Making Phoenix Not Mad

```
sudo nano /etc/ld.so.conf.d/randomLibs.conf
```

Add the path to the CTRE libs folder. Everything should work just fine after that!

```
/home/mdurrani/ROS/UMDLoopROS/src/diff_drive/hardware/lib/x86-64
```


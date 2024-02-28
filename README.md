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

## Low Latency Linux

### TODO - Look into adapting Tegra Linux to use a realtime-kernel.

Possibly:

```
sudo apt install linux-lowlatency
```

^ This may not work for tegra-linux (what we have on the Jetson Orin Nano), but should
work for most other Ubuntu installs. This makes the realtime capabilities of ros2-control
work better by reducing jitter in the main control loop.
I would recommend running the above command for personal use and doing the following.

### ROS2-Control requirements

ROS2-Control controller_manager configures the SCHED_FIFO of the main control loop
thread to 50 (the range is 1 (low priority) - 99 (high priority) for SCHED_FIFO
scheduling type). By default, users do not have permission to set such a high
scheduling priority for threads. To access those permissions, do the following:

Add a group named 'realtime' and add the user controlling the rover to this group:

```
sudo addgroup realtime
sudo usermod -a -G realtime $(whoami)
```

Afterwards, run the following commands:

```
echo "@realtime soft rtprio 99" >> /etc/security/limits.conf
echo "@realtime soft priority 99" >> /etc/security/limits.conf
echo "@realtime soft memlock 102400" >> /etc/security/limits.conf
echo "@realtime hard rtprio 99" >> /etc/security/limits.conf
echo "@realtime hard priority 99" >> /etc/security/limits.conf
echo "@realtime hard memlock 102400" >> /etc/security/limits.conf
```

These limits will be applied after you log out and in again.

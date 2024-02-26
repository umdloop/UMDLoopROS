# UMDLoopROS

This is all written assuming an Ubuntu 22.04 LTS install with ROS2 Humble installed. 

You will need to install these packages additionally.

## Getting Started

You need to install the myactuator_rmd package globally to get started (TODO: Make this not as terrible)
## Installing Packages
``
rosdep install --from-path src --ignore-src -y 
``

## Making Phoenix Not Mad

```
sudo nano /etc/ld.so.conf.d/randomLibs.conf
```

Add the path to the CTRE libs folder. Everything should work just fine after that!

```
/home/mdurrani/ROS/UMDLoopROS/src/diff_drive/hardware/lib/x86-64
```


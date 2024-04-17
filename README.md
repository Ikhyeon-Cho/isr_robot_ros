# ISR M3

## Overview

The repository provides the `isr_m3` ROS driver package implemented for running ISR-M3, the robot manufactured by Intelligent Systems and Robotics (ISR) Lab. 

## Getting Started
### Build
Use the following commands to download and compile the package.
```
cd ~/catkin_ws/src
git clone https://github.com/Ikhyeon-Cho/isr_robot_ros.git
cd ..
catkin build  # or catkin_make
```

### Run the ROS driver
To start running ISR-M3 robot, you should connect to the robot first. After, use command below:

**1. Using keyboard command:**
```
roslaunch isr_m3_bringup keyboard.launch
```
**2. Using joystick command:**
```
roslaunch isr_m3_bringup joy.launch
```

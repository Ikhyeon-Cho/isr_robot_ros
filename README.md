# ISR-M4

## Overview

The repository provides `isr_m4` ROS package implemented for running ISR-M4, the robot manufactured by Intelligent Systems and Robotics (ISR) Lab. 

## Getting Started
### Build Package
Use the following commands to download and compile ISR-M4 ROS driver package.
```
cd ~/catkin_ws/src
git clone https://github.com/Ikhyeon-Cho/isr_robot_ros.git
cd ..
catkin build isr_m4  # or catkin_make
```

### Run the ROS driver
#### 1. run_base.launch
To start running ISR-M4 robot, you should connect to the robot first. After, use the command below:

```
roscd isr_m4 && ./port_authentication.sh
roslaunch isr_m4 run_base.launch    # This starts the robot controller
```
You can give several command-line arguments to the launch files.

- **`use_joy`** (bool, default: false)<br>
    When set to `true`, ISR-M4 will use the joystick with the connected port. By default, `/dev/input/js0`.

- **`publish_odom_tf`** (bool, default: true)<br>
    When set to `false`, ISR-M4 will only publish the wheel odometry messages in nav_msgs/Odometry type.


#### 2. run.launch
To start with the equipped sensors, you should first ensure that the sensor configuration like [Velodyne VLP-16 LiDAR](https://github.com/Ikhyeon-Cho/velodyne_ros_tools) and [Realsense D455 camera](https://github.com/Ikhyeon-Cho/realsense_ros_tools)  has been done in your laptop. After, use the command below:
```
roscd isr_m4 && ./port_authentication.sh
roslaunch isr_m4 run.launch     # Start all sensor drivers and the robot
```
You can also give several command-line arguments to the launch files.

- **`use_joy`** (bool, default: false)<br>
    When set to `true`, ISR-M4 will use the joystick with the connected port. By default, `/dev/input/js0`.

- **`publish_odom_tf`** (bool, default: true)<br>
    When set to `false`, ISR-M4 will only publish the wheel odometry messages in `nav_msgs/Odometry` type.

- **`use_lidar`** (bool, default: true)<br>
    When set to `false`, LiDAR sensor driver will not be activated.

- **`use_camera`** (bool, default: true)<br>
    When set to `false`, RGB-D sensor driver will not be activated.

- **`use_imu`** (bool, default: true)<br>
    When set to `false`, IMU driver will not be activated.

Here is the example command using various launch argument options:
```
# [1] Joystick control
# [2] Don't publish odom_to_baselink transform
# [3] Use RGB-D Camera (and LiDAR, IMU by default)

roslaunch isr_m4 run.launch use_joy:=true publish_odom_tf:=false use_camera:=true
```
#### 3. test_sensor.launch
For testing sensor configuration, the following command can be used. Only the equipped sensors are activated, while the robot remains uncontrolled.
```
roscd isr_m4 && ./port_authentication.sh
roslaunch isr_m4 test_sensor.launch use_lidar:=true use_camera:=true use_imu:=true
```
The followings are possible launch arguments:
- **`use_lidar`** (bool, default: true)<br>
    When set to `false`, LiDAR sensor driver will not be activated.

- **`use_camera`** (bool, default: true)<br>
    When set to `false`, RGB-D sensor driver will not be activated.

- **`use_imu`** (bool, default: true)<br>
    When set to `false`, IMU driver will not be activated.

## Logging the data

Use the following command for recording rosbags:
```
roslaunch isr_m4_tools record_data.launch project_name:=urban_navigation    # assign the name for your project 
```
After, navigate to `~/Downloads/{your-project-name}` amd run the following command:
```
roslaunch isr_m4_tools merge_data.launch project_name:=urban_navigation       # assign the name for your project 
```

## Visualize rosbag
Use the following command to visualize recorded rosbag:
```
cd ~/Downloads/{your-project-name}
roslaunch isr_m4_tools test_playback_data.launch project_name:=urban_navigation      # Rviz visualization
```
or, you can just use the following command for rosbag play.
```
roslaunch isr_m4_tools playback_data.launch project_name:=urban_navigation
```
Possible launch arguments are:
The followings are possible launch arguments:
- **`project_name`** (string, default: "") (Empty string points `~/Downloads/`)<br>
    The default path is `~/Downloads/`. If you give an argument, then `~/Downloads/{argument} `

- **`bag_file`** (string, default: isr_m4_dataset.bag)<br>
    The name of the rosbag file to be played.

- **`playback_speed`** (double, default: 1.0)<br>
    The playback speed of bagfile.

- **`start_time`** (double, default: 1.0)<br>
    The start time of bagfile playback.

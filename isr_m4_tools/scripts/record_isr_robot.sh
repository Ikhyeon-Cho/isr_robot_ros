#!/bin/bash

rosbag record -O ~/Downloads/$1/isr_m4.bag \
  /tf \
  /tf_static \
  /imu/data \
  /odom/wheel \
  /cmd_vel
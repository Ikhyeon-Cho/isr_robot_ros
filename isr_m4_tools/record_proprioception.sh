#!/bin/bash

rosbag record --lz4 -O ~/Downloads/$1/isr_m4.bag /tf /tf_static /imu/data /odom/wheel
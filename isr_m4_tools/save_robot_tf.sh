#!/bin/bash

# Create the directory if it doesn't exist
mkdir -p ~/Downloads/$1

rosbag record -O ~/Downloads/$1/tf.bag /tf
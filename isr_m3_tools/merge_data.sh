#!/bin/bash

# Check if rosbag-merge is installed
if ! command -v rosbag-merge &> /dev/null
then
    echo -e "\033[0;32mrosbag-merge could not be found."
    echo -e "Installing rosbag-merge... \033[0m"
    pip3 install rosbag-merge
fi

cd ~/Downloads/$1 && rosbag-merge --write_bag --outbag_name isr_m3_dataset
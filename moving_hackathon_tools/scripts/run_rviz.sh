#!/bin/bash

set +x

source ~/Autoware/ros/install/local_setup.bash

cd ~/Autoware/ros/src/.config/rviz
#roscd moving_hackathon_tools/rviz
rviz -d default.rviz

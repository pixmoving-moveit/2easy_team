#!/bin/bash

set +x

source ~/Autoware/ros/install/local_setup.bash

cd ~/Autoware/ros/src/.config/rviz
rviz -d default.rviz

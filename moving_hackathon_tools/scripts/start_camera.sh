#!/bin/bash

source ~/camera_ws/devel/setup.bash

# We added the setuid bit to the mindvision binary
# sudo chown root /home/pix/camera_ws/devel/.private/mindvision/lib/mindvision/mindvision_node
# chmod u+s /home/pix/camera_ws/devel/.private/mindvision/lib/mindvision/mindvision_node
# chmod g+s /home/pix/camera_ws/devel/.private/mindvision/lib/mindvision/mindvision_node
roslaunch mindvision mindvision.launch

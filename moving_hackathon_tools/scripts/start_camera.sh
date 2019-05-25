#!/bin/bash

source ~/camera_ws/devel/setup.bash

# We added the setuid bit to the mindvision binary
# sudo chown root mindvision
# chmod u+s mindivision
roslaunch mindvision mindvision.launch

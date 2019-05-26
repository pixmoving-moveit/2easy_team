# Team 2easy repository

This repository contains the code from the team "2easy" composed by:

* Sammy Pfeiffer (Sammy.Pfeiffer at student.uts.edu.au)
* Eduardo Ferrera (eferrera at catec.aero)
* Wang Guangwei (gwwang at gzu.edu.cn)
* Axing Xi (948085419 At qq.com)
* Zihao Liu (360472051 at qq.com)
* HaoLong Fu (1165081323 at qq.com)
* Chang Liu (13726277580 at 163.com)


# Projects

* `competition_sm`: Contains the competition state machine. We divided the state machine in missions, every mission is a state machine. A general state machine executes all the isolated state machines. It uses SMACH states machines, follow [the tutorials here](http://wiki.ros.org/smach/Tutorials).
* `traffic_light_detector`: Contains a detector of the status of a traffic light.
* `stop_detector`: Contains a detector of if there is a traffic stop signal.
* `pedestrian_detector`: Contains a detector if there is a pedestrian in front of the car.

# Setup

Create a ROS workspace and clone this repo:
```bash
mkdir -p moving_2019_ws/src
cd moving_2019_ws/src
git clone https://github.com/pixmoving-moveit/2easy_team
cd ..
catkin build
# or catkin_make, to use catkin build (recommended)
# do: sudo apt-get install python-catkin-tools

# DO IT ON EVERY SHELL YOU WANT TO USE
source devel/setup.bash
```



## Run a mission

```bash
rosrun competition_sm mission_1.py
```

For testing, you can run:
```bash
rostopic pub /traffic_light_status std_msgs/String "data: 'GREEN'"
```

For testing, you can also try:
```bash
roslaunch competition_sm launch_mission_1.launch
```

To make the State Machine advance. The expected output:

```
[ DEBUG ] : Adding state (Move_until_traffic_light, <__main__.MoveUntilTrafficLight object at 0x7faf702f0e10>, {'failed': 'failed', 'succeeded': 'Wait_for_traffic_light_green_status'})
[ DEBUG ] : Adding state 'Move_until_traffic_light' to the state machine.
[ DEBUG ] : State 'Move_until_traffic_light' is missing transitions: {}
[ DEBUG ] : TRANSITIONS FOR Move_until_traffic_light: {'failed': 'failed', 'succeeded': 'Wait_for_traffic_light_green_status'}
[ DEBUG ] : Adding state (Wait_for_traffic_light_green_status, <__main__.WaitForTrafficLightGreenStatus object at 0x7faf702f0e90>, {'got_green': 'Move_curve'})
[ DEBUG ] : Adding state 'Wait_for_traffic_light_green_status' to the state machine.
[ DEBUG ] : State 'Wait_for_traffic_light_green_status' is missing transitions: {}
[ DEBUG ] : TRANSITIONS FOR Wait_for_traffic_light_green_status: {'got_green': 'Move_curve'}
[ DEBUG ] : Adding state (Move_curve, <__main__.MoveCurve object at 0x7faf702f0f10>, {'failed': 'failed', 'succeeded': 'succeeded'})
[ DEBUG ] : Adding state 'Move_curve' to the state machine.
[ DEBUG ] : State 'Move_curve' is missing transitions: {}
[ DEBUG ] : TRANSITIONS FOR Move_curve: {'failed': 'failed', 'succeeded': 'succeeded'}
[  INFO ] : State machine starting in initial state 'Move_until_traffic_light' with userdata: 
    []
[INFO] [1558509822.672012]: Executing state MoveUntilTrafficLight
[  INFO ] : State machine transitioning 'Move_until_traffic_light':'succeeded'-->'Wait_for_traffic_light_green_status'
[INFO] [1558509825.676024]: Executing state WaitForTrafficLightGreenStatus
[  INFO ] : State machine transitioning 'Wait_for_traffic_light_green_status':'got_green'-->'Move_curve'
[INFO] [1558509825.676873]: Executing state MoveCurve
[  INFO ] : State machine terminating 'Move_curve':'succeeded':'succeeded'
```


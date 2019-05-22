# Team 2easy repository

This repository contains the code from the team "2easy" composed by:

* Sammy Pfeiffer (Sammy.Pfeiffer at student.uts.edu.au)
* Eduardo
* ...


# Projects

* `competition_sm`: Contains the competition state machine. We divided the state machine in missions, every mission is a state machine. A general state machine executes all the isolated state machines.
* `traffic_light_detector`:
* `stop_signal_detector`:
* `pedestrian_detector`:


## Run a mission

```bash
rosrun competition_sm mission_1.py
```

For testing, you can run:
```bash
rostopic pub /traffic_light_status std_msgs/String "data: 'GREEN'"
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


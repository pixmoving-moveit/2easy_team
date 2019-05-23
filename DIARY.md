# Diary of the Moving hackathon of May 2019

From the point of view of the team "2easy". Mainly from the perspective of Sammy Pfeiffer and Eduardo Ferrera.

# Day 1 

* We got a safety guidance on how to use the cars and charging them.
* Challenge explanation and arena showcase.
* People introduction and team composition. Everyone came up, wrote their name on the whiteboard and gave a little bio about themselves. We took care of keeping people that wanted to be together, together and mixing up skilled people with people that wanted to learn. We made 3 teams based on the 3 leaders with more experience. Sam, Dr. Kerem & Dr. Ma.
* Identify what initial tasks needed to be done for our team.
* Distribution of the tasks.
* Choose a team name, we went for "2easy" as we were 2 teams joined together (Edu & Sam, and the members of the Chinese university (TODO: fill the name)). And cause it sounds fun.
* In between Sam & Edu, global design of the challenge. We decided to use a State Machine approach where every mission will be a State Machine so we can test every mission separately.
* Prepare Github repository to work together in it. Fill it up with skeleton code of what is expected to be coded. On one side State machines for the `Mission 1` and `Mission 2` and on another side example dummy nodes for the `traffic_light_detector`, `pedestrian_detector` and `stop_sign_detector`. The dummy nodes contain the logic to communicate using the expected messages the State Machines will need (basically `std_msgs/String` with 'GREEN' for the traffic_light_detector, for example) and with a `execute` function with a commented block that says "your magic code here".
* Another subteam installed Autoware on the car's computer (and in their laptops).
* Another subteam installed tensorflow in their laptops to develop detectors.
* Edu & Sam work on getting the drive-by-wire working. Needed to configure the local eth network device to have IP 192.168.1.2. Then needed to run the Autoware GUI `cd ~/Autoware/ros; ./run` and go to the `Interface tab` and click on **Vehicle Gateway**. Then on the `Computing` tab activate `Motion Planning > waypoint_follower > pure_pursuit` and `Motion Planning > waypoint_follower > twist_filter`. Didn't work because we weren't told about the need of putting the car in "Autonomos driving mode".
* Edu & Sam work on getting the velodyne working (just needed to launch `roslaunch velodyne_pointcloud VLP16_points.launch` as it was already configured with the defaults).
* Sam work on getting the mindvision camera working with the provided driver.
* Edu took measurings of the placement of the velodyne and the camera with respect to the base_link of the car (center of back wheel axis) to create a usable TF. Sam configures tf in Autoware for the velodyne and in general tf for the camera.
* We recorded 2 rosbags with the data from the velodyne and the camera while doing the full 7 mission route for creating later on a map and testing our assumptions about the quality of the data. The camera data was not very useful as we were recording by night and the image was very dark.
* The team that were installing Autoware move on to create a map based on the 2nd rosbag we recorded. Using the Autoware GUI, `Simulation` tab, choose the rosbag, hit play. In another shell (cause we captured the rosbag with the topic `/velodyne_points`) run `rosrun topic_tools relay /velodyne_points /points_raw`. Then go to the `Computing` tab and click on `Localization > ndt_mapping`. Clicking on `[app]` let's you save the PCD map once it's finished.
* Edu & Sam got explained how to set the car in "Autonomous driving mode". It involves 1) Turning on the car with pressing the braking pedal and pressing the start button. 2) Setting the car in Neutral gear. 3) Using the remote controller in the left door and pressing the green button once. 4) Setting the car in Driving 'D' gear while pressing the brake pedal. 5) Pushing the ON button next to the gear joystick. Now we can send commands.


# Day 2

* We had terrible breakfast.
* Quick meeting to review work done the last day and work to do today.
* 2 will work on traffic light detector, 2 will work on pedestrian detector and Edu & Sam gather rosbags with example data of the traffic light and the pedestrian for testing.
* Edu & Sam record full arena drive for new map (because last night there were cars blocking some roads).
* Sam try to make Drive-By-Wire work using Autoware waypoints follower & try to teleoperate with a Xbox controller and a keyboard node. Trying to find out which nodes in Autoware need to run and how they need to be configured to actually follow a previously saved set of waypoints. Needs `vel_pose_connect` with `current_velocity` set to `/estimate_twist`. Needs `waypoints loader`, needs `velocity_set`, needs, `pure_pursuit`\*, needs `twist_filter`, needs `path_select` and needs `lattice_velocity_set`. \* pure_pursuit has a bug for our car where it does not send sufficient angular.z. We will need to hack it.
* Edu work on using darknet with YOLO to detect with pre-trained networks traffic lights. Traffic light was very far, so only 1/4 of the pixels were useful. By using cropped image we improved the detections. YOLO is ready to work with our full system.
* The internet/network connectivity in the venue was very bad today. Hours were lost on waiting for things to download.
* We suffered a bug with the ETH-to-CAN board where it apparently overheated and stopped working correctly. The car didn't enter in autonomous mode. Autonomous mode is normally set by turning on the car, setting it in Neutral gear, pressing the green button of the wireless controller, then the big red button just next to the gear stick and then switching to D gear. The car should not move awaiting commands to move. This stopped happening. We are not sure if that was the case for sure, but it is very likely.


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
* The other subteams (not Edu & Sam) didn't have Ubuntu neither ROS installed in their computers so they needed to install it in order to work with the rosbags recorded (and in general, to work).

# Day 3

* The breakfast was a little bit better than yesterday.
* Quick meeting to review the work done the last day and the work to do today.
* We decide to target mission 1 and 2 as goal for today.
* Sam improves a bit the dummy code of the pedestrian detectors to define a path to follow for the chinease fellows.
* Edu starts helping the chinease team members to use ROS and play rosbags. They take a video and focus as a reference and focus in pedestrian detector.
* Edu keeps working in detecting the traffic light. Using the output of Yolo immage was cropped on the most probable traffic light.
* Edu makes a tiny color detector over the image cropped. Boundaries in HSV are used to create masks on the image. The number of ones in the mask is used to stimate probability of red and green. The system works but with some green false positives.
* Edu tunes the detector and filters out false positives. Green signal is only sent when more than 20 green images are seen (1-2 seconds expected in the real car)
* Sam decides to create two launch files to wake up the full autoware without using the gui. Gui gives problems in configuration and errors are expected to be avoided with that.
* Sam creates a rqt_reconfigure system to tune the steering gain.
* Edu & Sam step in the car and start making experiments: 
(from here on everything happens in the car, Edu & Sam point of view)
* The curve after the pedestrian detector was used to tune the steering in life.
* The node of life tuning was not working. Sam lifecodes changes. Several experiments demonstrates that a gain of 25 was necessary.
* A large goal based path was saved around the arena.
* An experiment around the full arena without stopping was made. The U-turn does not work. We decide that we will change the lookahead of the system for that (will require a shorter one).
* The car looses control once achieving more than 12 km/h. A longer lookahead will be required for that. Edu decides that if has time tomorrow he will change the lookahead based in the speed.
* Several small goal based paths where recorded based in roads between experiments. A wider U-turn makes the car drive smooth.
* Edu & Sam returns with the car.
* Edu merges the traffic light detector with the main in git and readys the car for using it (install yolo)
* Sam prepares a system to test the first and second test.
* The Chinease team was working in detecting the person with the camera. They have a bad time installing everything, but with the help of Sam, most of the required instalations are done.
* Edu starts working in the pedestrian detector, afraid that the system will not be ready in time. It system works based in the Velodyne
* We are forced to return at before 0:00 due to a really bad weather forecast.

# Day 4
* Edu finally overcomed the jetlag.
* Sam went have breakfast alone; it was slighly better. There was some green cookies!!
* Quick meeting to review the work done the last day and the work to do today.
* Chinease team will work in pedestrian detector by camera, Edu by velodyne. Sam will keep working in glue everything with a state machine.
* Edu filters out a large ammount of points with a plc_manager over nodelets. Points are seen in as if the velodyne where in a squared tunel. Points behind the car are also out.
* Sam and Edu test the state machine in the car and fixed some bugs.
* Sam keeps working on the state machine. Squeletom of the state machine was made.

# Global Expo Start (Midd day)
* Announcement of the Hackathon was made.   
* We had a lot of fun having pictures with our faces and our names printed in a hudge fotocall.
* Several speaches are given by the organizers.
* Edu gives a speach on behalf of all participants. He mentions the great effort that Nancy made to suppor us and thanks Pixmoving and Global Expo for the organization of the even.

(going back to work)
* Wang Guangwei (chinease professor) joins again the team to support his students. He start playing with Yolo
* Edu creates a second filter to check when there is a pedestrian in the pedestrian area. Is configurable with rqt_reconfigure, so slight changes can be afforded. It is not a generic solution.
* Wang is capable to detect persons with Yolo. Due to the lack of time and necessary implementation, we make him move to detect the stop signal.
* The car was tested again by Edu and Sam.
* Sam test again the state machine.
* Edu's filter is tested with the real car. It works fuk** amazing!!! (Edu is writting this post)
* Sam test the car with the keyboard and with the teleoperation keypad
* Edu and Sam starts to be kind of tired. Some code mistakes where made and idiotic short missundertandings started happening. We decided to go to sleep before make a mistake in the code.

#!/usr/bin/env python

import rospy
from autoware_msgs.msg import LaneArray

rospy.init_node('asdfasdfasdfasddfaf')

while not rospy.is_shutdown():
    print("Waiting for /traffic_waypoints_array...")
    msg = rospy.wait_for_message('/traffic_waypoints_array', LaneArray)
    wpn = len(msg.lanes[0].waypoints)
    print("traffic_waypoints_array has: " + str(wpn) + " waypoints")
    rospy.sleep(0.5)

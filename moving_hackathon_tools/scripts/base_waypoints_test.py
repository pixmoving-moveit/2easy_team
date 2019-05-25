#!/usr/bin/env python

import rospy
from autoware_msgs.msg import Lane

rospy.init_node('asdfaf')

while not rospy.is_shutdown():
	print("Waiting for /base_waypoints...")
	msg = rospy.wait_for_message('/base_waypoints', Lane)
	wpn = len(msg.waypoints)
	print("base_waypoints has: " + str(wpn) + " waypoints")
	rospy.sleep(0.5)
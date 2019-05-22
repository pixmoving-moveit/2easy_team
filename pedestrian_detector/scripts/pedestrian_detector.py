#!/usr/bin/env python

# Author: Eduardo Ferrera <eferrera@catec.aer>                             <----------------- change


# @sort: Checks with the Velodyne if there is a pedestrian in front of the car
# Connect to the velodyne, if there is a pedestrian in front of the car, sends a 
# message showing its presence.

# Python includes
import numpy as np
import math  as m 

# ROS includes                                             
import rospy
from std_msgs.msg import String


# todo lists:
# Write here the list of things to do


class PedestrianDetector():
    """
    Checks in front of the car using the Velodyne sensor if there is a pedestrian or not.
    To do so, checks at d[m] in front of the car a box of dx[m] by dy[m].
    If there is something inside that box that is higher than h[m] says that there is a pedestrian.
    """

    def __init__(self):
        
        # Constant values
        self.update_ratio = 0.1 
        self.topic_name = "pedestrians_on_road"                

        self.pub = rospy.Publisher(self.topic_name, String, queue_size=1)
        return

    def YourCode(self):
        """
        HERE GOES YOUR CODE
        """

        if (True):
            node.PublishMsgString("Pedestrian")
        return


    def PublishMsgString(self, msg):
        """
        Publishes a message in ROS as a string
        """
        rospy.loginfo('Publishing: ' + msg)
        self.pub.publish(msg)

        return



if __name__ == '__main__':
    
    node_name = "pedestrian_detector"                     
    rospy.init_node(node_name)
    rospy.loginfo(node_name + ' is working')

    
    try:
        node = PedestrianDetector()
        rate = rospy.Rate(10) # 10hz
        while not rospy.is_shutdown():
            node.YourCode()
            rate.sleep()
    except rospy.ROSInterruptException:
        rospy.loginfo('Unexpected error')
        pass
#!/usr/bin/env python

# Author: Eduardo Ferrera <eferrera@catec.aer>                             <----------------- change


# @sort: Small explaination of your code
# Extended explaination of your code

# Python includes
import numpy as np
import math  as m 

# ROS includes                                             
import rospy
from std_msgs.msg import String
#import tf                                                      <----------------- Remove them or use it as a example
#import tf2_ros
#import tf2_geometry_msgs
#import sensor_msgs.point_cloud2 as pc2
#from   geometry_msgs.msg      import TransformStamped
#from   geometry_msgs.msg      import PointStamped
#from   sensor_msgs.msg        import PointCloud2
#from   visualization_msgs.msg import Marker 
#from   visualization_msgs.msg import MarkerArray


# todo lists:
# Write here the list of things to do


class Example():
    """
    Line to explain the meaning of the class
    """

    def __init__(self):
        
        # Constant values
        self.update_ratio = 0.1 
        self.topic_name = "topic_name"                # <------------ Change

        self.pub = rospy.Publisher(self.topic_name, String, queue_size=1)
        return

    def YourCode(self):
        """
        HERE GOES YOUR CODE
        """

        if (True):
            node.PublishMsgString("GREEN")
        return


    def PublishMsgString(self, msg):
        """
        Publishes a message in ROS as a string
        """
        rospy.loginfo('Publishing: ' + msg)
        self.pub.publish(msg)

        return



if __name__ == '__main__':
    
    node_name = "Name_of_node"                      #<--------------- Change the name of the node!
    rospy.init_node(node_name)
    rospy.loginfo(node_name + ' is working')



#rostopic pub /traffic_light_status std_msgs/String "data: 'GREEN'"
    
    try:
        rospy.loginfo(node_name + ' is working')
        node = Example()
        rate = rospy.Rate(10) # 10hz
        while not rospy.is_shutdown():
            node.YourCode()
            rate.sleep()
    except rospy.ROSInterruptException:
        rospy.loginfo('Unexpected error')
        pass
#!/usr/bin/env python

# BSD 2-Clause License
#
# Copyright (c) 2019, Pix-Moving Hackathon
# All rights reserved.
# Author: Eduardo Ferrera <eferrera@catec.aer>                             <----------------- change
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
# * Redistributions of source code must retain the above copyright notice, this
#   list of conditions and the following disclaimer.
#
# * Redistributions in binary form must reproduce the above copyright notice,
#   this list of conditions and the following disclaimer in the documentation
#   and/or other materials provided with the distribution.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

# @sort: Small explaination of your code
# Extended explaination of your code

# Python includes
import numpy as np
import math  as m 

# ROS includes                                             
import rospy
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


        return

    def PublishTf(self, msg):
        """
        Publishes a message in ROS
        """


        return



if __name__ == '__main__':
    
    node_name = "Name_of_node"                      #<--------------- Change the name of the node!
    rospy.init_node(node_name)
    rospy.loginfo(node_name + ' is working')
    
    try:
        #node = ComputeTf()
        #rate = rospy.Rate(10) # 10hz
        #while not rospy.is_shutdown():
        #    node.UpdateTf()
        #    node.PublishTf()
        #    rate.sleep()
    except rospy.ROSInterruptException:
        pass
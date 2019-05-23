#!/usr/bin/env python

# Author: Eduardo Ferrera <eferrera@catec.aero>


# Python
import numpy as np
import keyboard
#https://pypi.org/project/keyboard/

# ROS
import rospy
#from std_msgs.msg import String
# from sensor_msgs.msg import Image
#from sensor_msgs.msg import PointCloud2
#from sensor_msgs.point_cloud2 import read_points

# Other imports


class WaypointsRecorder(object):
    """
    Waypoints recorder system, capable of saving a set of waypoints on the fly.
    """

    def __init__(self):
        # Subscribe to current poition
        self.last_position = None
        #self.position_sub = rospy.Subscriber('/position_topic',
        #                                       PointCloud2,
        #                                       self.position_cb,
        #                                       queue_size=1)
        #rospy.loginfo("Subscribing to positioning system at: " +
        #              self.pointcloud_sub.resolved_name)

        new_name =raw_input('Introduce a name for the file:')
        new_name += '.txt'
        self.pf = open(new_name,"w") 

    def __del__(self):
        # Destroy the system
        self.pf.close()

    def position_cb(self, position):
        self.last_position = position

    def run(self):
        rate = rospy.Rate(10)  # Detect at 10hz

        # Waiting to have a first valid position
        #while not rospy.is_shutdown() and self.last_position is None:
        #    rate.sleep()
        rospy.loginfo("Waypoints recorder ready.")
        
        while not rospy.is_shutdown():
            new_point =raw_input('Introduce a name and press enter to save a point (q to quit):')
            rate.sleep()
            if (new_point == "q"):
                return

            rospy.loginfo("Saving the point as '" + new_point + "'")
            


if __name__ == '__main__':
    rospy.init_node("pedestrian_detector")
    wr = WaypointsRecorder()
    wr.run()

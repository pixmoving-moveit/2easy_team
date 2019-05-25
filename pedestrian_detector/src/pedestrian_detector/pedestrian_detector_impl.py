#!/usr/bin/env python

# Author: Eduardo Ferrera <eferrera@catec.aero>


# Python
import numpy as np

# ROS
import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Point
from geometry_msgs.msg import PolygonStamped
from ddynamic_reconfigure_python.ddynamic_reconfigure import DDynamicReconfigure
""" from sensor_msgs.msg import PointCloud2
from sensor_msgs.point_cloud2 import read_points """

# Other imports


class PedestrianDetector(object):
    """
    Pedestrian detector, capable of checking if there is one person un a specific area
    """

    def __init__(self):

        # Publishes the area where the pedestrian pass is
        self.pub_pedestrian_area = rospy.Publisher("/pedestrian_area_reference", PolygonStamped, queue_size=1, latch=True)
        self.pedestrian_center = (42.689,79.109, -0.48)
        self.pedestrian_wide = (48.564 - 42.689,81.691 - 79.109)
        self.publish_pedestrian_area()
        self.pedestrian_area = DDynamicReconfigure("Pedestrian area")
        self.pedestrian_area.add_variable('x pose','x_pose', 
                                    self.pedestrian_center[0], min= 35, max = 85)
        self.pedestrian_area.add_variable('y pose','y_pose', 
                                    self.pedestrian_center[1], min= 70, max = 85)
        self.pedestrian_area.add_variable('x size','x_size', 
                                    self.pedestrian_wide[0], min= 0, max = 20)
        self.pedestrian_area.add_variable('y size','y_size', 
                                    self.pedestrian_wide[1], min= 0, max = 20)
        self.pedestrian_area.start(self.pedestrian_changes_cb)


    def publish_pedestrian_area(self):
        msg = PolygonStamped()
        msg.header.frame_id = "/map"
        p1 = Point()
        p1.x = self.pedestrian_center[0]
        p1.y = self.pedestrian_center[1]
        p1.z = self.pedestrian_center[2]
        msg.polygon.points.append(p1)

        p2 = Point()
        p2.x = self.pedestrian_center[0] + self.pedestrian_wide[0]
        p2.y = self.pedestrian_center[1]
        p2.z = self.pedestrian_center[2]
        msg.polygon.points.append(p2)

        p3 = Point()
        p3.x = self.pedestrian_center[0] + self.pedestrian_wide[0]
        p3.y = self.pedestrian_center[1] + self.pedestrian_wide[1]
        p3.z = self.pedestrian_center[2]
        msg.polygon.points.append(p3)

        p4 = Point()
        p4.x = self.pedestrian_center[0] 
        p4.y = self.pedestrian_center[1] + self.pedestrian_wide[1]
        p4.z = self.pedestrian_center[2]
        msg.polygon.points.append(p4)

        self.pub_pedestrian_area.publish(msg)
        return

    def pedestrian_changes_cb(config, level):
        self.pedestrian_center[0] = config['x_pose']
        self.pedestrian_center[1] = config['y_pose']
        self.pedestrian_wide[0] =   config['x_size']
        self.pedestrian_wide[1] =   config['y_size']
        return config

    
# """         self.pub = rospy.Publisher("/pedestrian_detection",
#                                    String, queue_size=1)
#         rospy.loginfo("Publishing pedestrian detections to: " +
#                       self.pub.resolved_name)

#         # Subscribe to images?
#         self.last_pointcloud = None
#         self.pointcloud_sub = rospy.Subscriber('/velodyne_points',
#                                                PointCloud2,
#                                                self.pointcloud_cb,
#                                                queue_size=1)
#         rospy.loginfo("Subscribing to images at: " +
#                       self.pointcloud_sub.resolved_name) """

# """    def pointcloud_cb(self, pointcloud):
#         self.last_pointcloud = pointcloud

#     def detect_pedestrian(self, pointcloud):
#         # Do magic code here
#         # I would take an area of the pointcloud
#         # that is over the zebra crossing
#         # and check if there is a set of points over it
#         # higher than the ground and in a meaningful amount

#         cloud_points = list(read_points(
#             pointcloud, skip_nans=True, field_names=("x", "y", "z")))

#         # if detection is true
#         return 'CROSSING'
#         # else:
#         # return 'CLEAR'

#     def pub_pedestrian_status(self, status):
#         """
#    """      Publishes the pedestrian status: """
#         'CROSSING', 'CLEAR'
#         """
#         rospy.loginfo('Publishing: ' + status)
#         self.pub.publish(status) """

    def run(self):
        rate = rospy.Rate(10)  # Detect at 10hz
        while not rospy.is_shutdown():
            

# """             if self.last_pointcloud is not None:
#                 result = self.detect_pedestrian(self.last_pointcloud)
#                 self.pub_pedestrian_status(result) """
            rate.sleep()


if __name__ == '__main__':
    rospy.init_node("pedestrian_detector")
    pd = PedestrianDetector()
    pd.run()

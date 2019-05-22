#!/usr/bin/env python

# Author: Eduardo Ferrera <eferrera@catec.aero>


# Python
import numpy as np

# ROS
import rospy
from std_msgs.msg import String
# from sensor_msgs.msg import Image
from sensor_msgs.msg import PointCloud2
from sensor_msgs.point_cloud2 import read_points

# Other imports


class WaypointsRecorder(object):
    """
    Traffic Light Detector based on blabla
    """

    def __init__(self):
        self.pub = rospy.Publisher("/pedestrian_detection",
                                   String, queue_size=1)
        rospy.loginfo("Publishing pedestrian detections to: " +
                      self.pub.resolved_name)

        # Subscribe to images?
        self.last_pointcloud = None
        self.pointcloud_sub = rospy.Subscriber('/velodyne_points',
                                               PointCloud2,
                                               self.pointcloud_cb,
                                               queue_size=1)
        rospy.loginfo("Subscribing to images at: " +
                      self.pointcloud_sub.resolved_name)

    def pointcloud_cb(self, pointcloud):
        self.last_pointcloud = pointcloud

    def detect_pedestrian(self, pointcloud):
        """
        Code that detects if a pedestrian is crossing
        """
        # Do magic code here
        # I would take an area of the pointcloud
        # that is over the zebra crossing
        # and check if there is a set of points over it
        # higher than the ground and in a meaningful amount

        cloud_points = list(read_points(
            pointcloud, skip_nans=True, field_names=("x", "y", "z")))

        # if detection is true
        return 'CROSSING'
        # else:
        # return 'CLEAR'

    def pub_pedestrian_status(self, status):
        """
        Publishes the pedestrian status:
        'CROSSING', 'CLEAR'
        """
        rospy.loginfo('Publishing: ' + status)
        self.pub.publish(status)

    def run(self):
        rate = rospy.Rate(10)  # Detect at 10hz
        while not rospy.is_shutdown():
            if self.last_pointcloud is not None:
                result = self.detect_pedestrian(self.last_pointcloud)
                self.pub_pedestrian_status(result)
            rate.sleep()


if __name__ == '__main__':
    rospy.init_node("pedestrian_detector")
    wr = WaypointsRecorder()
    wr.run()

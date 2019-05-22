#!/usr/bin/env python

# Author: Your name <youremail@gmail.com>


# Python
import numpy as np

# ROS
import rospy
from std_msgs.msg import String
from sensor_msgs.msg import Image

# Other imports


class StopDetector(object):
    """
    Stop Detector based on blabla
    """

    def __init__(self):
        self.pub = rospy.Publisher("/stop_detection_status",
                                   String, queue_size=1)
        rospy.loginfo("Publishing traffic stop detections to: " +
                      self.pub.resolved_name)

        # Subscribe to images?
        self.last_img = None
        self.img_sub = rospy.Subscriber('/car_camera',
                                        Image,
                                        self.img_cb,
                                        queue_size=1)
        rospy.loginfo("Subscribing to images at: " +
                      self.img_sub.resolved_name)

    def img_cb(self, img):
        self.last_img = img

    def detect_traffic_stop(self, image):
        """
        Code that detects the traffic stop signals
        based on ...
        returns 'STOP'
        """
        # Do magic code here

        # if detection is true
        return 'STOP'
        # else:
        # return 'RED'

    def pub_traffic_stop_status(self, status):
        """
        Publishes the traffic stop status:
        'STOP', 'UNKNOWN'
        """
        rospy.loginfo('Publishing: ' + status)
        self.pub.publish(status)

    def run(self):
        rate = rospy.Rate(10)  # Detect at 10hz
        while not rospy.is_shutdown():
            result = self.detect_traffic_stop(self.last_img)
            self.pub_traffic_stop_status(result)
            rate.sleep()


if __name__ == '__main__':
    rospy.init_node("stop_detector")
    tld = StopDetector()
    tld.run()

#!/usr/bin/env python

# Author: Your name <youremail@gmail.com>


# Python
import numpy as np

# ROS
import rospy
from std_msgs.msg import String
from sensor_msgs.msg import Image

# Other imports


class TrafficLightDetector(object):
    """
    Traffic Light Detector based on blabla
    """

    def __init__(self):
        self.pub = rospy.Publisher("/traffic_light_status",
                                   String, queue_size=1)
        rospy.loginfo("Publishing traffic light detections to: " +
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

    def detect_traffic_light(self, image):
        """
        Code that detects the traffic light status
        based on ...
        returns 'RED', 'GREEN', 'UNKNOWN'
        """
        # Do magic code here

        # if detection is true
        return 'GREEN'
        # else:
        # return 'RED'

    def pub_traffic_light_status(self, status):
        """
        Publishes the traffic light status:
        'RED', 'GREEN', 'UNKNOWN'
        """
        rospy.loginfo('Publishing: ' + status)
        self.pub.publish(status)

    def run(self):
        rate = rospy.Rate(10)  # Detect at 10hz
        while not rospy.is_shutdown():
            if self.last_img is not None:
                result = self.detect_traffic_light(self.last_img)
                self.pub_traffic_light_status(result)
            rate.sleep()


if __name__ == '__main__':
    rospy.init_node("traffic_light_detector")
    tld = TrafficLightDetector()
    tld.run()

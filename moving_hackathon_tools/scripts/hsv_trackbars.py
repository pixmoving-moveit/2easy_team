#! /usr/bin/env python
# -*- coding: utf-8 -*-
"""
Created on 7/29/15

@author: sampfeiffer

hsv_trackbars.py contains
a set of sliders done in OpenCV to filter by HSV values
"""

import cv2
import numpy as np
import rospy
from sensor_msgs.msg import Image, CompressedImage
from rostopic import get_topic_type
from cv_bridge import CvBridge
import sys


def nothing(x):
    pass


class ChooseHSVValues(object):
    def __init__(self, topic):
        self.bridge = CvBridge()
        self.last_frame = None
        # Creating a window for later use
        self.w_name = 'HSV color range chooser'
        cv2.namedWindow('result')

        # Creating track bar
        cv2.createTrackbar('lh', 'result', 0, 179, nothing)
        cv2.createTrackbar('ls', 'result', 0, 255, nothing)
        cv2.createTrackbar('lv', 'result', 0, 255, nothing)

        cv2.createTrackbar('hh', 'result', 0, 179, nothing)
        cv2.createTrackbar('hs', 'result', 0, 255, nothing)
        cv2.createTrackbar('hv', 'result', 0, 255, nothing)

        cv2.setTrackbarPos('hh', 'result', 179)
        cv2.setTrackbarPos('hs', 'result', 255)
        cv2.setTrackbarPos('hv', 'result', 255)

        type_name, topic_name, _ = get_topic_type(topic)
        if type_name == 'sensor_msgs/Image':
            self.subscriber = rospy.Subscriber(topic,
                                               Image,
                                               self.img_cb,
                                               queue_size=1)
        elif type_name == 'sensor_msgs/CompressedImage':
            self.subscriber = rospy.Subscriber(topic,
                                               CompressedImage,
                                               self.img_cb,
                                               queue_size=1)
        rospy.loginfo("Subscribing to: " + self.subscriber.resolved_name)

    def stop_node(self):
        exit(0)

    def img_cb(self, data):
        """
        :type data: Image
        """
        self.last_img = data
        self.new_img = True

        if self.subscriber.type == 'sensor_msgs/CompressedImage':
            np_arr = np.fromstring(data.data, np.uint8)
            cvImage = cv2.imdecode(np_arr, cv2.CV_LOAD_IMAGE_COLOR)
        else:
            cvImage = self.bridge.imgmsg_to_cv2(data, "bgr8")
        frame = cvImage

        # converting to HSV
        hsv_img = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        # get info from track bar and appy to result
        lh = cv2.getTrackbarPos('lh', 'result')
        ls = cv2.getTrackbarPos('ls', 'result')
        lv = cv2.getTrackbarPos('lv', 'result')

        hh = cv2.getTrackbarPos('hh', 'result')
        hs = cv2.getTrackbarPos('hs', 'result')
        hv = cv2.getTrackbarPos('hv', 'result')

        # Normal masking algorithm
        lower_color = np.array([lh, ls, lv])
        higher_color = np.array([hh, hs, hv])
        mask = cv2.inRange(hsv_img, lower_color, higher_color)
        result = cv2.bitwise_and(frame, frame, mask=mask)

        cv2.imshow('result', result)
        cv2.imshow('mask', mask)

        print "Your color range is:"
        print "Lower range (H, S, V): "
        print lh, ls, lv
        print "Higher range (H, S, V):"
        print hh, hs, hv
        print

        # Necessary to show the window
        cv2.waitKey(33)


if __name__ == '__main__':
    rospy.init_node('choose_hsv_values')
    cleaned_args = rospy.myargv(sys.argv)
    if len(cleaned_args) != 2:
        print "Usage:"
        print sys.argv[0] + " image_topic"
        print sys.argv[0] + " /xtion/rgb/image_rect_color/compressed"
        exit(0)
    topic = cleaned_args[1]
    c = ChooseHSVValues(topic)
    rospy.spin()

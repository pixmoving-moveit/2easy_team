#!/usr/bin/env python

# Author: Guangwei WANG <gwwang@gzu.edu.cn>
# Python
import numpy as np

# ROS
import rospy
from std_msgs.msg import String
from sensor_msgs.msg import Image

# Autoware message
from autoware_msgs.msg import DetectedObjectArray


class StopDetector(object):
    """
    Stop Detector based on Yolo.
    """

    def __init__(self):
        self.pub = rospy.Publisher("/stop_detection_status",
                                   String, queue_size=1)
        rospy.loginfo("Publishing traffic stop detections to: " +
                      self.pub.resolved_name)

        # stop_sign status
        self.last_obj = None
        self.label_ss = None
        self.x_ss = None
        self.y_ss = None
        self.h_ss = None
        self.w_ss = None       

        self.ped_sub = rospy.Subscriber('detection/image_detector/objects', 
                                        DetectedObjectArray, self.obj_cb, queue_size=1)

        # Publishing new images prepared for Yolo
        self.img_sub = rospy.Subscriber('/camera0/image_raw', Image,                                        self.img2yolo_cb,
                                        queue_size=1)
        rospy.loginfo("Subscribing to images at: " + self.img_sub.resolved_name)        
        self.img2yolo_pub = rospy.Publisher("/camera0/image_for_yolo", Image, queue_size=1)
        rospy.loginfo("Publishes the image to: /camera0/image_for_yolo")


    def obj_cb(self, DetectedObject):   
        self.ped_label = None
        self.x = None
        self.y = None
        self.h_ss = None
        self.w_ss = None
        for obj in DetectedObject.objects:
            if obj.label == 'stop sign':
                self.label_ss = obj.label
                self.x_ss = obj.x
                self.y_ss = obj.y
                self.h_ss = obj.height
                self.w_ss = obj.width
                # print (self.x_ss)
                
        self.last_obj = DetectedObject.objects

    def detect_traffic_stop(self):
        """
        Code that detects the traffic stop signals
        based on ...
        returns 'STOP'
        """
        # Do magic code here
        if self.label_ss == 'stop sign' and self.x_ss > 700:
            return 'STOP'
        else:
            return 'GO'

    def pub_traffic_stop_status(self, status):
        """
        Publishes the traffic stop status:
        'STOP', 'UNKNOWN'
        """
        rospy.loginfo('Publishing: ' + status)
        self.pub.publish(status)

    def img2yolo_cb(self,img):
        # Publishing again the image to yolo
        # Note: This is not the way to do it, but we are 
        # doing it because we are running out of time
        self.img2yolo_pub.publish(img)

    def run(self):
        rate = rospy.Rate(10)  # Detect at 10hz
        while not rospy.is_shutdown():
            if self.last_obj is not None:
                result = self.detect_traffic_stop()
                self.pub_traffic_stop_status(result)
                self.last_obj = None
                self.label_ss = None
            rate.sleep()


if __name__ == '__main__':
    rospy.init_node("stop_detector")
    tld = StopDetector()
    tld.run()

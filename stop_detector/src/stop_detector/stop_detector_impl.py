#!/usr/bin/env python

# Author: Guangwei WANG <gwwang@gzu.edu.cn>
# Colaborator: Eduardo Ferrera <eferrera@catec.aero>
# Python
import numpy as np

# ROS
import rospy
from std_msgs.msg import String
from sensor_msgs.msg import Image
from darknet_ros_msgs.msg import BoundingBoxes

# Autoware message
from autoware_msgs.msg import DetectedObjectArray

# Other imports
import cv2
from cv_bridge import CvBridge, CvBridgeError


class StopDetector(object):
    """
    Stop Detector based on Yolo.
    """

    def __init__(self):
        # It only pays attention to the stop sign if is at the right
        self.reduction_factor = 0.5

        self.pub = rospy.Publisher("/stop_detection_status",
                                   String, queue_size=1)
        rospy.loginfo("Publishing traffic stop detections to: " +
                      self.pub.resolved_name)
        self.bridge = CvBridge()

        # stop_sign status
        self.last_obj = None
        self.label_ss = None
        self.x_ss = None
        self.y_ss = None
        self.h_ss = None
        self.w_ss = None       


        self.ped_sub_yolo3 = rospy.Subscriber('detection/image_detector/objects', 
                                        DetectedObjectArray, self.yolo3_cb, queue_size=1)

        self.img2yolo = None
        self.ped_sub_yolo1 = rospy.Subscriber('/darknet_ros/bounding_boxes', 
                                        BoundingBoxes, self.yolo1_cb, queue_size=1)

        # Publishing new images prepared for Yolo
        self.img_sub = rospy.Subscriber('/camera0/image_raw', Image,                                        self.img2yolo_cb,
                                        queue_size=1)
        rospy.loginfo("Subscribing to images at: " + self.img_sub.resolved_name)        
        self.img2yolo_pub = rospy.Publisher("/camera0/image_for_yolo", Image, queue_size=1)
        rospy.loginfo("Publishes the image to: /camera0/image_for_yolo")

        self.img_seg_pub = rospy.Publisher("camera0/traffic_light_image", Image, queue_size=1)

    def yolo1_cb(self, DetectedObject):  
        self.ped_label = None
        self.x = None
        self.y = None
        self.h_ss = None
        self.w_ss = None
        do_print = False
        for obj in DetectedObject.bounding_boxes:
            if obj.Class == 'stop sign': 
                self.label_ss = obj.Class
                self.x = obj.xmin
                self.y = obj.ymin
                self.h_ss = obj.xmax - obj.xmin
                self.w_ss = obj.ymax - obj.ymin 
                do_print = True
                self.pub_traffic_stop_status(self.detect_traffic_stop())

        if (do_print):
            img = self.img2yolo
            cv_img = self.bridge.imgmsg_to_cv2(img,desired_encoding="bgr8")
            cv_img = cv_img[self.y:(self.y + self.w_ss),self.x:(self.x + self.h_ss)]
            self.img_seg_pub.publish(self.bridge.cv2_to_imgmsg(cv_img, encoding="bgr8"))

    def yolo3_cb(self, DetectedObject): 
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
                # print (self.x_ss)
                
        self.last_obj = DetectedObject.objects

    def detect_traffic_stop(self):
        """
        Code that detects the traffic stop signals
        based on ...
        returns 'STOP'
        """
        # Do magic code here
        if self.label_ss == 'stop sign':
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
        
        # Cropping the image (makes it faster and easier for yolo) and publishing it
        cv_img = self.bridge.imgmsg_to_cv2(img,desired_encoding="bgr8")
        # Grabing the size of the image
        (h, w) = cv_img.shape[:2]
        rf = self.reduction_factor
        cv_img = cv_img[int(0):int(rf*h),int(rf*w):int(1*w)]
        img = self.bridge.cv2_to_imgmsg(cv_img, encoding="bgr8")
        
        self.img2yolo = img
        self.img2yolo_pub.publish(img)

    def run(self):
        rate = rospy.Rate(10)  # Detect at 10hz
        while not rospy.is_shutdown():
            rate.sleep()


if __name__ == '__main__':
    rospy.init_node("stop_detector")
    tld = StopDetector()
    tld.run()

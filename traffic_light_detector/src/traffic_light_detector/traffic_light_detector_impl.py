#!/usr/bin/env python

# Author: Eduardo Ferrera <eferrera@catec.aero>

# Python
import numpy as np

# ROS
import rospy
from std_msgs.msg import String
from sensor_msgs.msg import Image
from darknet_ros_msgs.msg import BoundingBoxes
#darknet_ros_msgs/BoundingBoxes

# Other imports
import cv2
from cv_bridge import CvBridge, CvBridgeError

# Color boundaries in HSV
low_red = np.array([161, 155, 84])
high_red = np.array([179, 255, 255])
low_green = np.array([25, 52, 72])
high_green = np.array([102, 255, 255])

class TrafficLightDetector(object):
    """
    This traffic light detector tries to improve the output of Yolo in a crappy way.
    Reduces the size of the image, to make things easier for Yolo
    Detects the bounding boxes given by Yolo and checks the status of the traffic_light
    """

    def __init__(self):
        # Subscribing to the image
        self.reduction_factor = 0.5

        if (self.reduction_factor >= 1 or self.reduction_factor <= 0):
            rospy.logwarn("Ignoring the reduction factor")
            self.reduction_factor = 0

        self.cv_last_img = None
        self.bridge = CvBridge()
        self.img_sub = rospy.Subscriber('/camera0/image_raw', Image, self.cropper2yolo_cb,
                                        queue_size=1)
        rospy.loginfo("Subscribing to images at: " + self.img_sub.resolved_name)

        # Publishing new images prepared for Yolo
        self.img_pub = rospy.Publisher("/camera0/image_for_yolo", Image, queue_size=1)
        rospy.loginfo("Publishes the image to: /camera0/image_for_yolo")

        # Receives feedback from Yolo (crops this area of the image)
        self.yolo_sub = rospy.Subscriber('/darknet_ros/bounding_boxes', BoundingBoxes, 
                                            self.yolo_output_cb, queue_size=5)
        self.img_seg_pub = rospy.Publisher("camera0/traffic_light_image", Image, queue_size=1)

        # From the cropped image extracts green or red
        self.img_green_pub = rospy.Publisher("camera0/traffic_light_green", Image, queue_size=1)
        self.img_red_pub   = rospy.Publisher("camera0/traffic_light_red"  , Image, queue_size=1)

        # Publishing the result of the node
        self.pub = rospy.Publisher("/traffic_light_status", String, queue_size=1)
        rospy.loginfo("Publishing traffic light detections to: " + self.pub.resolved_name)


    def cropper2yolo_cb(self, img):
        # Cropping the image (makes it faster and easier for yolo) and publishing it
        cv_img = self.bridge.imgmsg_to_cv2(img,desired_encoding="bgr8")
        # Grabing the size of the image
        (h, w) = cv_img.shape[:2]
        rf = self.reduction_factor/2.0
        cv_img = cv_img[int(rf*h):int((1-rf)*h),int(rf*w):int((1-rf)*w)]
        self.cv_last_img = cv_img
        self.img_pub.publish(self.bridge.cv2_to_imgmsg(cv_img, encoding="bgr8"))
        self.wait_for_yolo = True

    def yolo_output_cb(self, msg):
        # Search for the most probable traffic light
        traf_found = None
        for b_box in msg.bounding_boxes:
            if (b_box.Class == "traffic light"):
                if (traf_found == None):
                    traf_found = b_box
                else:
                    if (traf_found.probability > b_box):
                        traf_found = b_box
        if (traf_found != None):
            print traf_found
            cv_img_traf_light = self.cv_last_img[traf_found.ymin:traf_found.ymax, 
                                                 traf_found.xmin:traf_found.xmax]
            cv_green_masked = self.masked_by_color(cv_img_traf_light, low_green, high_green)
            cv_red_masked   = self.masked_by_color(cv_img_traf_light, low_red, high_red)
            self.img_seg_pub.publish(  self.bridge.cv2_to_imgmsg(cv_img_traf_light, encoding="bgr8"))
            self.img_red_pub.publish(  self.bridge.cv2_to_imgmsg(cv_red_masked,     encoding="bgr8"))
            self.img_green_pub.publish(self.bridge.cv2_to_imgmsg(cv_green_masked,   encoding="bgr8"))

    def masked_by_color(self,img, low_mask, high_mask):
        # Applies a mask trying to extract only the specified color
        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

        # Threshold the HSV image to get only blue color
        mask = cv2.inRange(hsv, low_mask, high_mask)
        res = cv2.bitwise_and(img, img, mask=mask)
        res = cv2.medianBlur(res, 5)
        return res


    """
    # Green color
    
    green_mask = cv2.inRange(hsv_frame, low_green, high_green)
    green = cv2.bitwise_and(frame, frame, mask=green_mask)
    """

        

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
        #rospy.loginfo('Publishing: ' + status)
        self.pub.publish(status)

    def run(self):
        rate = rospy.Rate(10)  # Detect at 10hz
        while not rospy.is_shutdown():
            if self.cv_last_img is not None:
                result = self.detect_traffic_light(self.cv_last_img)
                self.pub_traffic_light_status(result)
            rate.sleep()


if __name__ == '__main__':
    rospy.init_node("traffic_light_detector")
    tld = TrafficLightDetector()
    tld.run()

#!/usr/bin/env python

# Author: Your name <gwwang17@gmail.com>

# ROS
import rospy
from std_msgs.msg import String
from sensor_msgs.msg import Image
# Autoware message
from autoware_msgs.msg import DetectedObjectArray

class PedestrianDetector(object):
    """
    Traffic Light Detector based on blabla
    """

    def __init__(self):

        self.pub = rospy.Publisher("/pedestrian_detection",
                                   String, queue_size=1)
        rospy.loginfo("Publishing pedestrian detections to: " +
                      self.pub.resolved_name)

        self.last_obj = None

        # pedestrian status
        self.ped_label = None
        self.x = None
        self.y = None
        self.height = None
        self.width = None
        self.ped_sub = rospy.Subscriber('detection/image_detector/objects', 
                                        DetectedObjectArray, self.ped_cb, queue_size=1)


    def ped_cb(self, DetectedObject):   
        self.ped_label = None
        self.x = None
        self.y = None
        self.height = None
        self.width = None
        for obj in DetectedObject.objects:
            if obj.label == 'person':
                # print ('I found a person!')
                self.ped_label = obj.label
                self.x = obj.x
                self.y = obj.y
                self.height = obj.height
                self.width = obj.width
                # print (self.height, self.width)
                # print (self.x, self.y)
                
        self.last_obj = DetectedObject.objects


    def detect_pedestrian(self):
        """
        Code that detects if a pedestrian is crossing
        """
        # add height and width judgement to get more accurate result
        if self.ped_label == 'person' and self.x<520 and self.height>200:
            return 'CROSSING'
        else:
            return 'CLEAR'


    def pub_pedestrian_status(self, status):
        """
        Publishes the pedestrian status:
        'CROSSING', 'CLEAR'
        """
        # rospy.loginfo('Publishing: ' + status)
        rospy.loginfo(status)
        self.pub.publish(status)

    def run(self):
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            if self.last_obj is not None:
                result = self.detect_pedestrian()
                self.pub_pedestrian_status(result)
                self.last_obj = None
            rate.sleep()


if __name__ == '__main__':
    rospy.init_node("pedestrian_detector")
    pd = PedestrianDetector()
    pd.run()

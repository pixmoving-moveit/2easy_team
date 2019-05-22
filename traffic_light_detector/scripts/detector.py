#!/usr/bin/env python

import rospy
from traffic_light_detector.traffic_light_detector_impl import TrafficLightDetector

if __name__ == '__main__':
    rospy.init_node('traffic_light_detector')
    tld = TrafficLightDetector()
    tld.run()

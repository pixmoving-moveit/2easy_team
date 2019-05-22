#!/usr/bin/env python

import rospy
from stop_detector.stop_detector_impl import StopDetector

if __name__ == '__main__':
    rospy.init_node('stop_detector')
    tld = StopDetector()
    tld.run()

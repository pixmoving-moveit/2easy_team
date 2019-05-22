#!/usr/bin/env python

import rospy
from pedestrian_detector.pedestrian_detector_impl import PedestrianDetector

if __name__ == '__main__':
    rospy.init_node('pedestrian_detector')
    pd = PedestrianDetector()
    pd.run()

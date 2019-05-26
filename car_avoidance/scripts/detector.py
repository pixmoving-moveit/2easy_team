#!/usr/bin/env python

import rospy
from car_avoidance.car_avoidance_impl import CarObstacleDetector

if __name__ == '__main__':
    rospy.init_node('car_avoidance_detector')
    cod = CarObstacleDetector()
    cod.run()

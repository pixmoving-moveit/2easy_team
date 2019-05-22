#!/usr/bin/env python

import rospy

from competition_sm.mission_1 import mission_1
from competition_sm.mission_2 import mission_2

if __name__ == '__main__':
    rospy.init_node('full_competition')

    rospy.loginfo("Executing mission 1")
    mission_1()
    rospy.loginfo("Executing mission 2")
    mission_2()
    # rospy.loginfo("Executing mission 3")
    # mission_3()

    rospy.loginfo("All mission done!")

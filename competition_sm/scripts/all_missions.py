#!/usr/bin/env python

import rospy
import smach
import smach_ros

from competition_sm.mission_1 import get_mission_1_sm
from competition_sm.mission_2 import get_mission_2_and_3_sm
from competition_sm.mission_4 import get_mission_4_sm
from competition_sm.mission_5 import get_mission_5_sm
from competition_sm.mission_6 import get_mission_6_and_7_sm

if __name__ == '__main__':
    rospy.init_node('full_competition')

    Mission1 = get_mission_1_sm()
    Mission2_3 = get_mission_2_and_3_sm()
    Mission4 = get_mission_4_sm()
    Mission5 = get_mission_5_sm()
    Mission6_7 = get_mission_6_and_7_sm()

    # Create a SMACH state machine
    sm = smach.StateMachine(outcomes=['succeeded', 'failed'])

    # Open the container
    with sm:
        # Add states to the container
        smach.StateMachine.add('Mission_1',
                               Mission1,
                               transitions={
                                   'succeeded': 'Mission_2_and_3',
                                   'failed': 'failed'})
        smach.StateMachine.add('Mission_2_and_3',
                               Mission2_3,
                               transitions={
                                   'succeeded': 'Mission_4',
                                   'failed': 'failed'})
        smach.StateMachine.add('Mission_4',
                               Mission4,
                               transitions={
                                   'succeeded': 'Mission_5',
                                   'failed': 'failed'})
        smach.StateMachine.add('Mission_5',
                               Mission5,
                               transitions={
                                   'succeeded': 'Mission_6_and_7',
                                   'failed': 'failed'})
        smach.StateMachine.add('Mission_6_and_7',
                               Mission6_7,
                               transitions={
                                   'succeeded': 'succeeded',
                                   'failed': 'failed'})

    sis = smach_ros.IntrospectionServer('all_missions', sm, '/SM_ROOT')
    sis.start()

    sm.execute()

    rospy.spin()
    sis.stop()

    rospy.loginfo("All mission done!")

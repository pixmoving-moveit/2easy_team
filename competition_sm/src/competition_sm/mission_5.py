#!/usr/bin/env python

import rospy
import smach
import smach_ros
from std_msgs.msg import String
from follow_waypoints import FollowWaypointsFile


class DoUTurnAndGoToStop(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded', 'failed'])

    def execute(self, userdata):
        rospy.loginfo('Executing state ' + self.__class__.__name__)
        # Send a goal to our "Move using waypoints" server and wait until
        # we reach the goal
        fwf = FollowWaypointsFile('mission_5_uturn_new.csv')
        fwf.wait_to_reach_last_waypoint()
        return 'succeeded'
        # if something went wrong
        # return 'failed'


def mission_5():
    sm = get_mission_5_sm()

    sis = smach_ros.IntrospectionServer('mission_5', sm, '/SM_ROOT')
    sis.start()
    # Execute SMACH plan
    outcome = sm.execute()


def get_mission_5_sm():

    # Create a SMACH state machine
    sm = smach.StateMachine(outcomes=['succeeded', 'failed'])

    # Open the container
    with sm:
        smach.StateMachine.add('Do_U_turn_and_go_to_stop',
                               DoUTurnAndGoToStop(),
                               transitions={'succeeded': 'succeeded',
                                            'failed': 'failed'})
    return sm


if __name__ == '__main__':
    rospy.init_node("mission_5_sm")
    mission_5()

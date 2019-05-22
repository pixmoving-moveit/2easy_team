#!/usr/bin/env python

import rospy
import smach
from std_msgs.msg import String


class MoveUntilTrafficLight(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded', 'failed'])

    def execute(self, userdata):
        rospy.loginfo('Executing state ' + self.__class__.__name__)
        # Send a goal to our "Move using waypoints" server and wait until
        # we reach the goal

        rospy.sleep(3)
        return 'succeeded'
        # if something went wrong
        # return 'failed'


class WaitForTrafficLightGreenStatus(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['got_green'])
        self.got_green = False
        self._green_subscriber = rospy.Subscriber('/traffic_light_status',
                                                  String,
                                                  self._traffic_light_status_cb,
                                                  queue_size=1)

    def _traffic_light_status_cb(self, msg):
        status = msg.data.upper()
        if 'G' in status:
            self.got_green = True

    def execute(self, userdata):
        rospy.loginfo('Executing state ' + self.__class__.__name__)
        # Wait for the traffic signal state detector
        # to tell us the light is green
        while not rospy.is_shutdown() and not self.got_green:
            rospy.sleep(0.1)
        return 'got_green'


class MoveCurve(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded', 'failed'])

    def execute(self, userdata):
        rospy.loginfo('Executing state ' + self.__class__.__name__)
        # Send a goal to our "Move using waypoints" server and wait until
        # we reach the goal

        rospy.sleep(3)
        return 'succeeded'
        # if something went wrong
        # return 'failed'


def mission_1():

    # Create a SMACH state machine
    sm = smach.StateMachine(outcomes=['succeeded', 'failed'])

    # Open the container
    with sm:
        # Add states to the container
        smach.StateMachine.add('Move_until_traffic_light',
                               MoveUntilTrafficLight(),
                               transitions={
                                   'succeeded': 'Wait_for_traffic_light_green_status',
                                   'failed': 'failed'})
        smach.StateMachine.add('Wait_for_traffic_light_green_status',
                               WaitForTrafficLightGreenStatus(),
                               transitions={'got_green': 'Move_curve'})
        smach.StateMachine.add('Move_curve',
                               MoveCurve(),
                               transitions={'succeeded': 'succeeded',
                                            'failed': 'failed'})

    # Execute SMACH plan
    outcome = sm.execute()


if __name__ == '__main__':
    rospy.init_node("mission_1_sm")
    mission_1()

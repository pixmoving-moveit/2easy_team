#!/usr/bin/env python

import rospy
import smach
from std_msgs.msg import String


class MoveUntilZebraCrossing(smach.State):
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


class WaitForPedestrianToCross(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['pedestrian_crossed'])
        self.pedestrian_crossed = False
        self._green_subscriber = rospy.Subscriber('/pedestrian_detection',
                                                  String,
                                                  self._pedestrian_status_cb,
                                                  queue_size=1)

    def _pedestrian_status_cb(self, msg):
        status = msg.data.upper()
        if 'CLEAR' in status:
            self.pedestrian_crossed = True

    def execute(self, userdata):
        rospy.loginfo('Executing state ' + self.__class__.__name__)
        # Wait for the traffic signal state detector
        # to tell us the light is green
        rospy.logwarn("Waiting for /pedestrian_detection to give CLEAR")
        while not rospy.is_shutdown() and not self.pedestrian_crossed:
            rospy.sleep(0.1)
        return 'pedestrian_crossed'


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


def mission_2():

    # Create a SMACH state machine
    sm = smach.StateMachine(outcomes=['succeeded', 'failed'])

    # Open the container
    with sm:
        # Add states to the container
        smach.StateMachine.add('Move_until_zebra_crossing',
                               MoveUntilZebraCrossing(),
                               transitions={
                                   'succeeded': 'Wait_for_pedestrian_to_cross',
                                   'failed': 'failed'})
        smach.StateMachine.add('Wait_for_pedestrian_to_cross',
                               WaitForPedestrianToCross(),
                               transitions={'pedestrian_crossed': 'Move_curve'})
        smach.StateMachine.add('Move_curve',
                               MoveCurve(),
                               transitions={'succeeded': 'succeeded',
                                            'failed': 'failed'})

    # Execute SMACH plan
    outcome = sm.execute()


if __name__ == '__main__':
    rospy.init_node("mission_2_sm")
    mission_2()

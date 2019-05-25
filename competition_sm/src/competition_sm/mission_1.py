#!/usr/bin/env python

import rospy
import smach
import smach_ros
from std_msgs.msg import String
from follow_waypoints import FollowWaypointsFile


class MoveUntilTrafficLight(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded', 'failed'])

    def execute(self, userdata):
        rospy.loginfo('Executing state ' + self.__class__.__name__)
        # Send a goal to our "Move using waypoints" server and wait until
        # we reach the goal
        fwf = FollowWaypointsFile('mission_1_until_stop.csv')
        fwf.wait_to_reach_last_waypoint()
        fwf.kill()
        del fwf

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
        if 'GREEN' in status:
            self.got_green = True

    def execute(self, userdata):
        rospy.loginfo('Executing state ' + self.__class__.__name__)
        # Wait for the traffic signal state detector
        # to tell us the light is green
        rospy.logwarn("Waiting for /traffic_light_status to give GREEN")
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
        fwf = FollowWaypointsFile('mission_1_curve_to_pedestrian.csv')
        fwf.wait_to_reach_last_waypoint()
        fwf.kill()
        del fwf

        return 'succeeded'
        # if something went wrong
        # return 'failed'


def get_mission_1_sm():
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
    return sm


def mission_1():
    sm = get_mission_1_sm()
    sis = smach_ros.IntrospectionServer('mission_1', sm, '/SM_ROOT')
    sis.start()

    # Execute SMACH plan
    outcome = sm.execute()


if __name__ == '__main__':
    rospy.init_node("mission_1_sm")
    mission_1()

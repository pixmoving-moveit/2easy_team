#!/usr/bin/env python

import rospy
import smach
import smach_ros
from std_msgs.msg import String
from follow_waypoints import FollowWaypointsFile
from shellcmd import ShellCmd

global detector_process


class StartPedestrianDetector(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded'])

    def execute(self, userdata):
        global detector_process
        rospy.loginfo("Launching detector")
        # detector_process = ShellCmd(
        #     "roslaunch pedestrian_detector pedestrian_detector.launch")
        # detector_process = ShellCmd("rostopic pub /test std_msgs/String test -r 2")

        rospy.sleep(5.0)  # Give some time to person to cross
        return 'succeeded'


class StopPedestrianDetector(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded'])

    def execute(self, userdata):
        global detector_process
        # if detector_process is not None:
        #     rospy.loginfo("Killing detector")
        #     if not detector_process.is_done():
        #         detector_process.kill()
        return 'succeeded'


class WaitForPedestrianToCross(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['pedestrian_crossed'])
        self.pedestrian_crossed = False
        self.saw_pedestrian_once = False
        self._green_subscriber = rospy.Subscriber('/pedestrian_detection',
                                                  String,
                                                  self._pedestrian_status_cb,
                                                  queue_size=1)

    def _pedestrian_status_cb(self, msg):
        status = msg.data.upper()
        if 'CLEAR' in status:
            self.pedestrian_crossed = True
        elif 'CROSSING':
            self.saw_pedestrian_once = True

    def execute(self, userdata):
        rospy.loginfo('Executing state ' + self.__class__.__name__)
        # Wait for the traffic signal state detector
        # to tell us the light is green
        rospy.logwarn("Waiting for /pedestrian_detection to give CROSSING")
        while not rospy.is_shutdown() and not self.saw_pedestrian_once:
            rospy.sleep(0.2)
        rospy.logwarn("Waiting for /pedestrian_detection to give CLEAR")
        while not rospy.is_shutdown() and not self.pedestrian_crossed:
            rospy.sleep(0.2)
        return 'pedestrian_crossed'


class MoveCurveChangeLaneAndStop(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded', 'failed'])

    def execute(self, userdata):
        rospy.loginfo('Executing state ' + self.__class__.__name__)
        # Send a goal to our "Move using waypoints" server and wait until
        # we reach the goal
        fwf = FollowWaypointsFile('mission_2_drive_curve.csv',
                                  consider_done_on_waypoint_id=84)
        fwf.wait_to_reach_last_waypoint()
        return 'succeeded'
        # if something went wrong
        # return 'failed'


def mission_2():
    sm = get_mission_2_and_3_sm()

    sis = smach_ros.IntrospectionServer('mission_2', sm, '/SM_ROOT')
    sis.start()
    # Execute SMACH plan
    outcome = sm.execute()


def get_mission_2_and_3_sm():

    # Create a SMACH state machine
    sm = smach.StateMachine(outcomes=['succeeded', 'failed'])

    # Open the container
    with sm:
        smach.StateMachine.add('Start_pedestrian_detector',
                               StartPedestrianDetector(),
                               transitions={
                                   'succeeded': 'Wait_for_pedestrian_to_cross'})
        smach.StateMachine.add('Wait_for_pedestrian_to_cross',
                               WaitForPedestrianToCross(),
                               transitions={'pedestrian_crossed': 'Move_curve_change_lane_and_stop'})
        smach.StateMachine.add('Move_curve_change_lane_and_stop',
                               MoveCurveChangeLaneAndStop(),
                               transitions={'succeeded': 'Stop_pedestrian_detector',
                                            'failed': 'failed'})
        smach.StateMachine.add('Stop_pedestrian_detector',
                               StopPedestrianDetector(),
                               transitions={
                                   'succeeded': 'succeeded'})
    return sm


if __name__ == '__main__':
    rospy.init_node("mission_2_sm")
    mission_2()

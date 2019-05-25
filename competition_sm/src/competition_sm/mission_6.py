#!/usr/bin/env python

import rospy
import smach
import smach_ros
from std_msgs.msg import String
from follow_waypoints import FollowWaypointsFile
import time
from geometry_msgs.msg import TwistStamped


class DetectStop(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['stop_detected', 'timeout'])
        self.stop_sign_detected = None
        self.subs = rospy.Subscriber('/stop_sign_detected',
                                     String,
                                     self._stop_sign_detected_cb,
                                     queue_size=1)

    def _stop_sign_detected_cb(self, msg):
        self.stop_sign_detected = msg.data

    def execute(self, userdata):
        rospy.loginfo('Executing state ' + self.__class__.__name__)
        # Wait for the traffic signal state detector
        # to tell us the light is green
        rospy.logwarn(
            "Waiting for /stop_sign_detected to maybe give a detection")
        ini_t = time.time()
        while not rospy.is_shutdown() and self.stop_sign_detected is None and time.time() - ini_t < 10.0:
            rospy.sleep(0.1)
        rospy.loginfo("finished waiting! got a detection?: " +
                      str(self.stop_sign_detected))
        if self.stop_sign_detected is None:
            return 'timeout'
        if self.stop_sign_detected.upper() == 'STOP':
            return 'stop_detected'


class GoToStop2(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded', 'failed'])

    def execute(self, userdata):
        rospy.loginfo('Executing state ' + self.__class__.__name__)
        # Send a goal to our "Move using waypoints" server and wait until
        # we reach the goal
        fwf = FollowWaypointsFile('mission_6_stop1_to_stop2.csv')
        fwf.wait_to_reach_last_waypoint()
        del fwf
        # rospy.sleep(20.0)
        return 'succeeded'


class MoveSCones(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded', 'failed'])

    def execute(self, userdata):
        rospy.loginfo('Executing state ' + self.__class__.__name__)
        # Send a goal to our "Move using waypoints" server and wait until
        # we reach the goal
        fwf = FollowWaypointsFile('mission_7_s.csv')
        fwf.wait_to_reach_last_waypoint()
        del fwf
        return 'succeeded'


class StopAtPlace3s(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded'])
        self.pub = rospy.Publisher('/twist_raw', TwistStamped, queue_size=1)

    def execute(self, userdata):
        rospy.loginfo('Executing state ' + self.__class__.__name__)
        ini_t = time.time()
        # Create a timer that effectively spams 0 twist at 50Hz
        timer = rospy.Timer(rospy.Duration(0.02), self.send_0_twist)
        rospy.loginfo("SPAMMING STOP (0 twist to /twist_raw) for 3s")
        while not rospy.is_shutdown() and time.time() - ini_t < 3.0:
            rospy.sleep(0.2)
        timer.shutdown()
        rospy.loginfo("Done!")
        return 'succeeded'

    def send_0_twist(self, *args):
        ts = TwistStamped()
        self.pub.publish(ts)


def mission_6_and_7():
    sm = get_mission_6_and_7_sm()

    sis = smach_ros.IntrospectionServer('mission_6_7', sm, '/SM_ROOT')
    sis.start()
    # Execute SMACH plan
    outcome = sm.execute()


def get_mission_6_and_7_sm():

    # Create a SMACH state machine
    sm = smach.StateMachine(outcomes=['succeeded', 'failed'])

    # Open the container
    with sm:
        stop_sm = smach.StateMachine(outcomes=['succeeded'])
        with stop_sm:
            smach.StateMachine.add('Detect_stop',
                                   DetectStop(),
                                   transitions={'stop_detected':
                                                'Stop_at_place_3s',
                                                'timeout':
                                                'succeeded'})

            smach.StateMachine.add('Stop_at_place_3s',
                                   StopAtPlace3s(),
                                   transitions={
                                       'succeeded': 'succeeded'
                                   })

        def child_term_cb(outcome_map):
            rospy.loginfo("Returning true on child_term_cb")
            # This will stop the other states
            return True

        sm_con = smach.Concurrence(
            outcomes=[
                'finished_waypoints'
            ],
            default_outcome='finished_waypoints',
            outcome_map={'finished_waypoints': {'Detect_and_stop': 'succeeded'},
                         'finished_waypoints': {'Go_to_Stop2': 'succeeded'}},
            child_termination_cb=child_term_cb
            # outcome_cb=out_cb
        )

        with sm_con:
            smach.Concurrence.add('Detect_and_stop',
                                  stop_sm)
            smach.Concurrence.add('Go_to_Stop2',
                                  GoToStop2())

        smach.StateMachine.add('Concurrent_stop_and_go', sm_con,
                               transitions={
                                   'finished_waypoints': 'Do_s_cones'}
                               )

        smach.StateMachine.add('Do_s_cones',
                               MoveSCones(),
                               transitions={'succeeded': 'succeeded',
                                            'failed': 'failed'})

    return sm


if __name__ == '__main__':
    rospy.init_node("mission_6_and_7_sm")
    mission_6_and_7()

#!/usr/bin/env python

import rospy
import smach
import smach_ros
from std_msgs.msg import String
from follow_waypoints import FollowWaypointsFile
import time
from geometry_msgs.msg import TwistStamped

# Stop 1 pose
# - Translation: [-57.978, 112.837, -1.249]
# - Rotation: in Quaternion [-0.017, 0.001, -0.711, 0.703]
#             in RPY (radian) [-0.026, -0.023, -1.581]
#             in RPY (degree) [-1.506, -1.330, -90.586]

# Stop 2 pose
# - Translation: [-53.440, 57.763, 0.672]
# - Rotation: in Quaternion [-0.038, -0.023, -0.704, 0.709]
#             in RPY (radian) [-0.021, -0.087, -1.562]
#             in RPY (degree) [-1.224, -4.979, -89.479]


# Check /ndt_pose PoseStamped for being in that place
# With a radius of 0.5m


class DetectStop(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['stop_detected', 'timeout'])
        self.stop_sign_detected = None
        self.subs = rospy.Subscriber('/stop_detection_status',
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
        while not rospy.is_shutdown() and \
                (self.stop_sign_detected is None or self.stop_sign_detected != 'STOP') and \
                (time.time() - ini_t) < 10.0:
            rospy.logerr("waiting for stop sign... time elapsed: " + str(time.time() - ini_t))
            rospy.loginfo("self.stop_sign_detected is: " + str(self.stop_sign_detected))
            rospy.sleep(0.5)
        rospy.loginfo("finished waiting! got a detection?: " +
                      str(self.stop_sign_detected))
        if self.stop_sign_detected is None or self.stop_sign_detected.upper() == 'GO':
            rospy.logwarn("We got timeout!!")
            return 'timeout'
        if self.stop_sign_detected.upper() == 'STOP':
            rospy.logerr("WE SAW THE STOP SIGN, WE SHOULD GO STOP 3S NOW")
            self.stop_sign_detected = None
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
        rospy.sleep(3.0)
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
        while not rospy.is_shutdown() and (time.time() - ini_t) < 3.0:
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
                                   'finished_waypoints': 'Concurrent_stop_and_go2'}
                               )


        stop_sm2 = smach.StateMachine(outcomes=['succeeded'])
        with stop_sm2:
            smach.StateMachine.add('Detect_stop2',
                                   DetectStop(),
                                   transitions={'stop_detected':
                                                'Stop_at_place_3s2',
                                                'timeout':
                                                'succeeded'})

            smach.StateMachine.add('Stop_at_place_3s2',
                                   StopAtPlace3s(),
                                   transitions={
                                       'succeeded': 'succeeded'
                                   })

        def child_term_cb(outcome_map):
            rospy.loginfo("Returning true on child_term_cb")
            # This will stop the other states
            return True

        sm_con2 = smach.Concurrence(
            outcomes=[
                'finished_waypoints'
            ],
            default_outcome='finished_waypoints',
            outcome_map={'finished_waypoints': {'Detect_and_stop2': 'succeeded'},
                         'finished_waypoints': {'Do_s_cones': 'succeeded'}},
            child_termination_cb=child_term_cb
            # outcome_cb=out_cb
        )

        with sm_con2:
            smach.Concurrence.add('Detect_and_stop2',
                                  stop_sm2)
            smach.Concurrence.add('Do_s_cones',
                                  MoveSCones())

        smach.StateMachine.add('Concurrent_stop_and_go2',
                               sm_con2,
                               transitions={'finished_waypoints': 'succeeded'})

    return sm


if __name__ == '__main__':
    rospy.init_node("mission_6_and_7_sm")
    mission_6_and_7()

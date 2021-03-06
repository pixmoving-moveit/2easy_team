#!/usr/bin/env python

import math
import time
import rospy
import smach
import smach_ros
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped, TwistStamped
from follow_waypoints import FollowWaypointsFile
from dynamic_reconfigure.client import Client

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


class DoUTurnAndGoToStop1Stop2ThenSAndHomeWhileStopping(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded', 'failed'])
        self.last_pose = None
        self.ndt_pose_sub = rospy.Subscriber('/ndt_pose', PoseStamped,
                                             self.ndt_pose_cb,
                                             queue_size=1)

        self.twist_gain_dyn_client = Client('/twist_gains/twist_gains')

    def _stop_sign_detected_cb(self, msg):
        self.last_stop = msg.data

    def ndt_pose_cb(self, msg):
        self.last_pose = msg

    def did_we_see_stop(self):
        msg = rospy.wait_for_message('/stop_detection_status',
                                     String,
                                     4.0)
        if msg.data == 'STOP':
            return True
        else:
            return False

    def are_we_at_stop_1(self):
        stop_1_x = -57.978
        stop_1_y = 112.837

        return self.are_we_at(stop_1_x, stop_1_y)

    def are_we_at_stop_2(self):
        stop_1_x = -53.440
        stop_1_y = 57.763

        return self.are_we_at(stop_1_x, stop_1_y)

    def are_we_at(self, x, y, tolerance=0.5):
        curr_x = self.last_pose.pose.position.x
        curr_y = self.last_pose.pose.position.y

        dist = math.hypot(curr_x - x, curr_y - y)
        rospy.loginfo("We are at " + str(round(dist, 2)) + "m")

        if abs(dist) < tolerance:
            return True
        else:
            return False

    def stop_car_3s(self):
        ini_t = time.time()
        rospy.logerr("======== STOPPING FOR 3S ===========")
        self.twist_gain_dyn_client.update_configuration({'linear_x_gain': 0.0,
                                                         'angular_z_gain': 0.0})
        while not rospy.is_shutdown() and (time.time() - ini_t) < 5.0:
            rospy.sleep(0.05)
        rospy.loginfo("Done!")
        self.twist_gain_dyn_client.update_configuration({'linear_x_gain': 1.0,
                                                         'angular_z_gain': 25.0})

    def execute(self, userdata):
        rospy.loginfo('Executing state ' + self.__class__.__name__)
        # Send a goal to our "Move using waypoints" server and wait until
        # we reach the goal
        fwf = FollowWaypointsFile('mission_5_6_7.csv',
                                  consider_done_on_waypoint_id=489)
        rospy.sleep(2.0)

        # keep checking for our pose to be in the radius of the first stop signal
        while not rospy.is_shutdown() and \
                not self.are_we_at_stop_1():
            rospy.sleep(0.1)

        if self.did_we_see_stop():
            # stop the car 3s
            self.stop_car_3s()

        # wait for the car to move enough to stop seeing the STOP sign and reset
        rospy.sleep(10.0)
        self.last_stop = 'GO'

        # keep checking for our pose to be in the radius of the first stop signal
        while not rospy.is_shutdown() and \
                not self.are_we_at_stop_2():
            rospy.sleep(0.1)

        if self.did_we_see_stop():
            # stop the car 3s
            self.stop_car_3s()

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
        smach.StateMachine.add('DoUTurnAndGoToStop1Stop2ThenSAndHomeWhileStopping',
                               DoUTurnAndGoToStop1Stop2ThenSAndHomeWhileStopping(),
                               transitions={'succeeded': 'succeeded',
                                            'failed': 'failed'})
    return sm


if __name__ == '__main__':
    rospy.init_node("mission_5_sm")
    mission_5()

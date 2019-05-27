#!/usr/bin/env python

import rospy
import smach
import smach_ros
from std_msgs.msg import String
from follow_waypoints import FollowWaypointsFile


class DetectObstacle(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['obstacle_on_left',
                                             'obstacle_on_right'])

    def _obstacle_side_cb(self, msg):
        self.obstacle_side = msg.data

    def execute(self, userdata):
        rospy.loginfo('Executing state ' + self.__class__.__name__)
        self.obstacle_side = None
        self.subs = rospy.Subscriber('/obstacle_place',
                                     String,
                                     self._obstacle_side_cb,
                                     queue_size=1)
        # Wait for the traffic signal state detector
        # to tell us the light is green
        rospy.logwarn("Waiting for /obstacle_side to give left or right")
        while not rospy.is_shutdown() and self.obstacle_side is None:
            rospy.sleep(0.1)
        if self.obstacle_side.upper() == 'FAR':
            return 'obstacle_on_left'
        elif self.obstacle_side.upper() == 'CLOSE':
            return 'obstacle_on_right'
        else:
            rospy.logerr("Got something weird in /obstacle_side: " +
                         str(self.obstacle_side))
            rospy.loginfo("Retrying...")
            self.obstacle_side = None
            self.execute(userdata)


class MoveCurve(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded', 'failed'])

    def execute(self, userdata):
        rospy.loginfo('Executing state ' + self.__class__.__name__)
        # Send a goal to our "Move using waypoints" server and wait until
        # we reach the goal
        fwf = FollowWaypointsFile('mission_4_curve_to_right_lane.csv',
                                  consider_done_on_waypoint_id=38)  # was 40
        fwf.wait_to_reach_last_waypoint()
        # del fwf
        return 'succeeded'


class MoveLeft(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded', 'failed'])

    def execute(self, userdata):
        rospy.loginfo('Executing state ' + self.__class__.__name__)
        # Send a goal to our "Move using waypoints" server and wait until
        # we reach the goal
        fwf = FollowWaypointsFile('mission_4_go_left_lane.csv',
                                  consider_done_on_waypoint_id=161)
        rospy.sleep(1.0)
        fwf.wait_to_reach_last_waypoint()
        del fwf
        return 'succeeded'


class MoveRight(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded', 'failed'])

    def execute(self, userdata):
        rospy.loginfo('Executing state ' + self.__class__.__name__)
        # Send a goal to our "Move using waypoints" server and wait until
        # we reach the goal
        fwf = FollowWaypointsFile('mission_4_go_right_lane.csv',
                                  consider_done_on_waypoint_id=161)
        rospy.sleep(1.0)
        fwf.wait_to_reach_last_waypoint()
        del fwf
        return 'succeeded'


def mission_4_sm():
    sm = get_mission_4_sm()

    sis = smach_ros.IntrospectionServer('mission_4', sm, '/SM_ROOT')
    sis.start()
    # Execute SMACH plan
    outcome = sm.execute()


def get_mission_4_sm():

    # Create a SMACH state machine
    sm = smach.StateMachine(outcomes=['succeeded', 'failed'])

    # Open the container
    with sm:
        smach.StateMachine.add('Move_curve',
                               MoveCurve(),
                               transitions={'succeeded': 'Detect_obstacle_side',
                                            'failed': 'failed'})

        smach.StateMachine.add('Detect_obstacle_side',
                               DetectObstacle(),
                               transitions={
                                   'obstacle_on_left': 'Move_left',
                                   'obstacle_on_right': 'Move_right'
                               })
        smach.StateMachine.add('Move_right',
                               MoveRight(),
                               transitions={'succeeded': 'succeeded',
                                            'failed': 'failed'})

        smach.StateMachine.add('Move_left',
                               MoveLeft(),
                               transitions={'succeeded': 'succeeded',
                                            'failed': 'failed'})

    return sm


if __name__ == '__main__':
    rospy.init_node("mission_4_sm")
    mission_4_sm()

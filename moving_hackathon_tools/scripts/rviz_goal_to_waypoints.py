#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped
try:
    from autoware_msgs.msg import Lane, Waypoint
except Exception as e:
    print("Exception: " + str(e))
    print("autoware_msgs is not in the PATH")
    print("do")
    print("source ~/Autoware/ros/install/local_setup.bash")
    print("And try again")
    exit(-1)


class RvizToWaypoints(object):
    def __init__(self):
        self.pub = rospy.Publisher('/final_waypoints',
                                   Lane,
                                   queue_size=1)
        self.sub = rospy.Subscriber('/move_base_simple/goal',
                                    PoseStamped,
                                    self._cb,
                                    queue_size=1)
        rospy.loginfo("Initialized")

    def _cb(self, msg):
        ln = Lane()
        wp = Waypoint()
        wp.pose = msg
        wp.twist.twist.linear.x = 2.0
        ln.waypoints.append(wp)
        self.pub.publish(ln)
        rospy.loginfo("Published /final_waypoints")


if __name__ == '__main__':
    rospy.init_node('rviz_waypoints')
    r = RvizToWaypoints()
    rospy.spin()

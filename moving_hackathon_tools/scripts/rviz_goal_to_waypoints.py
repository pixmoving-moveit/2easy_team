#!/usr/bin/env python

from copy import deepcopy
import math
import rospy
from geometry_msgs.msg import PoseStamped, PoseArray
from tf.transformations import euler_from_quaternion
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

        self.pub_debug = rospy.Publisher(
            '/rviz_waypoints', PoseArray, queue_size=1)

    def _cb(self, msg):
        ln = Lane()
        wp = Waypoint()
        wp.pose = deepcopy(msg)
        wp.twist.twist.linear.x = 1.94
        # Make a pre-goal pose 0.5m before with 7km/h speed
        o = msg.pose.orientation
        _, _, yaw = euler_from_quaternion([o.x, o.y, o.z, o.w])
        wp.pose.pose.position.x -= math.cos(yaw) * 0.5
        wp.pose.pose.position.y -= math.sin(yaw) * 0.5 
        # Add another point after with 0 speed to stop at the goal
        wp_stop = Waypoint()
        wp_stop.pose = deepcopy(msg)
        ln.waypoints.append(wp)
        ln.waypoints.append(wp_stop)
        self.pub.publish(ln)
        rospy.loginfo("Published /final_waypoints")

        pa = PoseArray()
        pa.header.frame_id = msg.header.frame_id
        pa.poses.append(wp.pose.pose)
        pa.poses.append(wp_stop.pose.pose)
        self.pub_debug.publish(pa)


if __name__ == '__main__':
    rospy.init_node('rviz_waypoints')
    r = RvizToWaypoints()
    rospy.spin()

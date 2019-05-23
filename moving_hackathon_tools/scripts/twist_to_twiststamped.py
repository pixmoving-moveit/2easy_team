#!/usr/bin/env python

from geometry_msgs.msg import Twist, TwistStamped
import rospy

class Twist2TwistStamped(object):
    def __init__(self):
        self.pub = rospy.Publisher('/twist_cmd', TwistStamped, queue_size=1)
        self.sub = rospy.Subscriber('/twist_cmd_test', Twist, self._cb, queue_size=1)
        rospy.loginfo("Initialized Twist2TwistStamped")

    def _cb(self, msg):
        ts = TwistStamped()
        ts.twist = msg
        ts.twist.linear.x *= -1.0
        ts.header.stamp = rospy.Time.now()
        self.pub.publish(ts)
    

if __name__ == '__main__':
    rospy.init_node('twist2twiststamped')
    t2ts = Twist2TwistStamped()
    rospy.spin()
	

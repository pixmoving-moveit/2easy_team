#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist, TwistStamped
from ddynamic_reconfigure_python.ddynamic_reconfigure import DDynamicReconfigure

"""
Class that acts as a intermediate node
getting a TwistStamped in and
outputing a TwistStamped out
that has been applied a set of gains
that can be configured via Dynamic Reconfigure
with rosrun rqt_reconfigure rqt_reconfigure

Author: Sammy Pfeiffer <Sammy.Pfeiffer@student.uts.edu.au>
"""


class TwistStampedGains(object):
    def __init__(self):
        self.pub = rospy.Publisher('/twist_raw',
                                   TwistStamped, queue_size=1)
        rospy.loginfo("")
        self.ddr = DDynamicReconfigure('twist_gains')
        self.ddr.add_variable('linear_x_gain',
                              "gain for twist.linear.x",
                              1.0, min=-5.0, max=5.0)
        self.ddr.add_variable('angular_z_gain',
                              "gain for twist.angular.z",
                              2.0, min=0.0, max=20.0)
        self.ddr.start(self.dyn_rec_callback)

        self.sub = rospy.Subscriber('/twist_raw_tmp',
                                    TwistStamped, self._cb,
                                    queue_size=1)
        rospy.loginfo("TwistStampedGains initialized.")
        rospy.loginfo("Listening to topic: " + str(self.sub.resolved_name))
        rospy.loginfo("Publishing on topic: " + str(self.pub.resolved_name))
        rospy.loginfo("Applying gains: x: " + str(self.linear_x_gain) +
                      " y: " + str(self.angular_z_gain))

    def dyn_rec_callback(self, config, level):
        self.linear_x_gain = config['linear_x_gain']
        self.angular_z_gain = config['angular_z_gain']
        return config

    def _cb(self, msg):
        msg.twist.linear.x *= self.linear_x_gain
        msg.twist.angular.z *= self.angular_z_gain
        self.pub.publish(msg)


if __name__ == '__main__':
    rospy.init_node('twist_gains')
    tsg = TwistStampedGains()
    rospy.spin()

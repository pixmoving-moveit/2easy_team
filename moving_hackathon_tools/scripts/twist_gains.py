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
        self.ddr = DDynamicReconfigure('twist_gains')
        self.ddr.add_variable('linear_x_gain',
                              "gain for twist.linear.x",
                              1.0, min=-5.0, max=5.0)
        self.ddr.add_variable('angular_z_gain',
                              "gain for twist.angular.z",
                              25.0, min=0.0, max=100.0)
        self.ddr.add_variable("max_angular_z", "max +- angular.z",
                              6.0, min=0.0, max=20.0)
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
        self.max_z = config['max_angular_z']
        rospy.loginfo(
            "TwistStampedGains got a reconfigure callback: " + str(config))
        return config

    def _cb(self, msg):
        # rospy.loginfo("in z: " + str(msg.twist.angular.z))
        msg.twist.linear.x *= self.linear_x_gain
        msg.twist.angular.z *= self.angular_z_gain
        if abs(msg.twist.angular.z) > self.max_z:
            if msg.twist.angular.z >= 0.0:
                msg.twist.angular.z = self.max_z
            else:
                msg.twist.angular.z = - self.max_z

        # rospy.loginfo("out z: " + str(msg.twist.angular.z))
        self.pub.publish(msg)


if __name__ == '__main__':
    rospy.init_node('twist_gains')
    tsg = TwistStampedGains()
    rospy.spin()

#!/usr/bin/env python

import rospy
from waypoints_recorder.waypoints_recorder_impl import WaypointsRecorder

if __name__ == '__main__':
    rospy.init_node('waypoints_recorder')
    wr = WaypointsRecorder()
    wr.run()

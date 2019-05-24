#!/usr/bin/env python

from competition_sm.shellcmd import ShellCmd
from rospkg import RosPack
from autoware_msgs.msg import Lane
from std_msgs.msg import Int32
import rospy


class FollowWaypointsFile(object):
    def __init__(self, waypoints_csv):
        rp = RosPack()
        mission_wp_path = rp.get_path(
            'competition_sm') + "/../waypoints_data/missions/"
        rospy.loginfo("mission_wp_path: " + str(mission_wp_path))
        precommand = "source ~/Autoware/ros/install/local_setup.bash; "
        self.wp_loader_cmd = ShellCmd(precommand + "roslaunch waypoint_maker waypoint_loader.launch load_csv:=True multi_lane_csv:=" + mission_wp_path + waypoints_csv +
                                      " replanning_mode:=False realtime_tuning_mode:=False resample_mode:=True resample_interval:=1 replan_curve_mode:=False overwrite_vmax_mode:=False replan_endpoint_mode:=True velocity_max:=20 radius_thresh:=20 radius_min:=6 velocity_min:=4 accel_limit:=0.5 decel_limit:=0.3 velocity_offset:=4 braking_distance:=5 end_point_offset:=1")
        self.path_select_cmd = ShellCmd(precommand +
                                        "sleep 3; rosrun lattice_planner path_select")
        # rospy.sleep(10.0)
        # while not self.wp_loader_cmd.is_done():
        #     rospy.logwarn(self.wp_loader_cmd.get_stdout())
        #     rospy.logerr(self.wp_loader_cmd.get_stderr())
        # rospy.logwarn(self.wp_loader_cmd.get_stdout())
        # rospy.logerr(self.wp_loader_cmd.get_stderr())

    def kill(self):
        if not self.wp_loader_cmd.is_done():
            self.wp_loader_cmd.kill()
        if not self.path_select_cmd.is_done():
            self.path_select_cmd.kill()

    def wait_to_reach_last_waypoint(self, callback=None):
        """
        Wait until we reached the last waypoint.
        if a callback is provided, it will be called
        when reached instead of blocking
        """
        rospy.loginfo("Waiting for /base_waypoints...")
        bw = rospy.wait_for_message('/base_waypoints', Lane)
        self.last_waypoint_id = len(bw.waypoints) - 1
        rospy.loginfo("Got base_waypoints of len: " +
                      str(self.last_waypoint_id))
        self.closest_waypoint = -1
        self.user_callback = callback

        def closest_waypoint_cb(msg):
            self.closest_waypoint = msg.data
            if msg.data == self.last_waypoint_id:
                if self.user_callback is not None:
                    rospy.loginfo("We reached the last waypoint! (callback)")
                    self.user_callback()
        tmp_sub = rospy.Subscriber('/closest_waypoint', Int32,
                                   closest_waypoint_cb, queue_size=1)
        if self.user_callback is None:
            rospy.loginfo("Blocking until we reach last waypoint...")
            while not rospy.is_shutdown() and self.closest_waypoint != self.last_waypoint_id:
                rospy.loginfo("Current closest waypoint ID " + str(self.closest_waypoint) +
                              " (waiting for " + str(self.last_waypoint_id) + ")")
                rospy.sleep(0.1)
            rospy.loginfo("We reached the last waypoint!")
        else:
            rospy.loginfo("Callback provided, not blocking.")

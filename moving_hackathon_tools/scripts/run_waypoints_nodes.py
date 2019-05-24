#!/usr/bin/env python

import subprocess
import tempfile
import os
import signal


class ShellCmd:
    """Helpful class to spawn commands and keep track of them"""

    def __init__(self, cmd):
        self.retcode = None
        self.outf = tempfile.NamedTemporaryFile(mode="w")
        self.errf = tempfile.NamedTemporaryFile(mode="w")
        self.inf = tempfile.NamedTemporaryFile(mode="r")
        self.cmd_line = cmd
        self.process = subprocess.Popen(cmd, shell=True, stdin=self.inf,
                                        stdout=self.outf, stderr=self.errf,
                                        preexec_fn=os.setsid, close_fds=True,
                                        # The default is /bin/sh
                                        executable='/bin/bash')

    def __del__(self):
        if not self.is_done():
            self.kill()
        self.outf.close()
        self.errf.close()
        self.inf.close()

    def get_command_str(self):
        return self.cmd_line

    def get_stdout(self):
        with open(self.outf.name, "r") as f:
            return f.read()

    def get_stderr(self):
        with open(self.errf.name, "r") as f:
            return f.read()

    def get_retcode(self):
        """Get retcode or None if still running"""
        if self.retcode is None:
            self.retcode = self.process.poll()
        return self.retcode

    def is_done(self):
        return self.get_retcode() is not None

    def is_succeeded(self):
        """Check if the process ended with success state (retcode 0)
        If the process hasn't finished yet this will be False."""
        return self.get_retcode() == 0

    def kill(self):
        self.retcode = -1
        os.killpg(self.process.pid, signal.SIGTERM)
        self.process.wait()


# Demonstration of usage
if __name__ == '__main__':
    import time
    precommand = "source ~/Autoware/ros/install/local_setup.bash; "
    commands_list = [
        "roslaunch autoware_connector vel_pose_connect.launch topic_pose_stamped:=/ndt_pose topic_twist_stamped:=/estimate_twist sim_mode:=False",
        # "roslaunch autoware_connector vel_pose_connect.launch topic_pose_stamped:=/ndt_pose topic_twist_stamped:=/current_velocity sim_mode:=False",
        "roslaunch lattice_planner lattice_velocity_set.launch use_crosswalk_detection:=False",
        "roslaunch waypoint_follower twist_filter.launch",
        # pure_pursuit needs the following rostopic pub in order to work
        "roslaunch waypoint_follower pure_pursuit.launch is_linear_interpolation:=True publishes_for_steering_robot:=False param_flag_:=0",
        """sleep 3; rostopic pub /config/waypoint_follower autoware_config_msgs/ConfigWaypointFollower "header:
  seq: 0
  stamp: {secs: 0, nsecs: 0}
  frame_id: ''
param_flag: 0
velocity: 5.0
lookahead_distance: 4.0
lookahead_ratio: 2.0
minimum_lookahead_distance: 6.0
displacement_threshold: 0.0
relative_angle_threshold: 0.0" --once""",

        "roslaunch lane_planner lane_select.launch",
        "roslaunch lane_planner lane_navi.launch velocity:=40 output_file:=/tmp/lane_waypoint.csv",
        "rosrun lane_planner lane_rule",
        "rosrun lane_planner lane_stop",

        # "sleep 3; roslaunch waypoint_maker waypoint_loader.launch load_csv:=True multi_lane_csv:=/home/sam/moving_2019_ws/src/2easy_team/waypoints_data/mission_2_to_pedestrian.csv replanning_mode:=False realtime_tuning_mode:=False resample_mode:=True resample_interval:=1 replan_curve_mode:=False overwrite_vmax_mode:=False replan_endpoint_mode:=True velocity_max:=20 radius_thresh:=20 radius_min:=6 velocity_min:=4 accel_limit:=0.5 decel_limit:=0.3 velocity_offset:=4 braking_distance:=5 end_point_offset:=1",
        # "sleep 10; rosrun lattice_planner path_select"

    ]

    executed_commands = []
    for cmd in commands_list:
        final_cmd = precommand + cmd
        print("Running: '" + str(final_cmd) + "'")
        running_process = ShellCmd(final_cmd)
        executed_commands.append(running_process)

    try:
        already_reported_error = []
        while True:
            for process in executed_commands:
                out = process.get_stdout()
                err = process.get_stderr()
                if err and process.get_command_str() not in already_reported_error:
                    print("In process: " + process.get_command_str())
                    print("We got error:")
                    print(err)
                    print("            ")
                    print("stdout was:")
                    print(out)
                    already_reported_error.append(process.get_command_str())

            time.sleep(1.0)

    except KeyboardInterrupt:
        print("Closing processes...")
        for process in executed_commands:
            if process.is_done():
                print("Process already done: '" +
                      str(process.get_command_str()) +
                      "'")
            else:
                print("Killing process: '" +
                      str(process.get_command_str()) +
                      "'")
                process.kill()

    print("!! Done !!")

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
        # "roslaunch runtime_manager setup_tf.launch x:=1.05 y:=0.0 z:=1.7 yaw:=0.0 pitch:=0.0 roll:=0.0 frame_id:=/base_link child_frame_id:=/velodyne period_in_ms:=10
        "rosrun tf static_transform_publisher 1.05 0.0 1.7 0.0 0.0 0.0 base_link velodyne 50",
        "roslaunch vehicle_description vehicle_model.launch",
        "rosrun map_file points_map_loader noupdate $HOME/moving_2019_ws/src/2easy_team/pcd_maps/day2_map.pcd",
        "roslaunch $HOME/Autoware/ros/src/.config/tf/tf_local.launch",
        # "roslaunch runtime_manager velodyne_vlp16.launch calibration:=",
        "roslaunch points_downsampler points_downsample.launch node_name:=voxel_grid_filter points_topic:=/points_raw",
        """rosparam set localizer velodyne;
rosparam set tf_x 1.05;
rosparam set tf_y 0.0;
rosparam set tf_z 1.7;
rosparam set tf_yaw 0.0;
rosparam set tf_pitch 0.0;
rosparam set tf_roll 0.0;
sleep 5;
roslaunch lidar_localizer ndt_matching.launch method_type:=0 use_odom:=False use_imu:=False imu_upside_down:=False imu_topic:=/imu_raw get_height:=False output_log_data:=False localizer:=velodyne""",
        "rosrun topic_tools relay /velodyne_points /points_raw",
        # For the rosbag with --clock
        "rosparam set /use_sim_time true",
        "rosbag play /media/sam/DATA/moving2019/round2_2019-05-22-20-02-59.bag -r 0.5 --clock --topics /velodyne_points",
        """sleep 10; rostopic pub /initialpose geometry_msgs/PoseWithCovarianceStamped \"header:
  seq: 0
  stamp:
    secs: 0
    nsecs: 0
  frame_id: 'map'
pose:
  pose:
    position: {x: 0.0, y: 0.0, z: 0.0}
    orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}
  covariance: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0]\" --once"""
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

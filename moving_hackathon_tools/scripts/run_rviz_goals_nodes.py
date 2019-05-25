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
        "roslaunch lane_planner lane_select.launch",
        "rosrun moving_hackathon_tools rviz_goal_to_waypoints.py"
    ]

    executed_commands = []
    for cmd in commands_list:
        if 'moving_hackathon_tools' not in cmd:
            final_cmd = precommand + cmd
        else:
            final_cmd = cmd
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

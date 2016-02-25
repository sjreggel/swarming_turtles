#!/usr/bin/env python
import os
import rospkg

import errno
import rospy
import time

import signal

from geometry_msgs.msg import Twist
from stage_ros.msg import Stall
from std_msgs.msg import Int32
from std_srvs.srv import Empty
from subprocess import Popen, PIPE

STALL_RESTART_TIME = 10.  # How long have the robots to be stalled to restart
SHUTDOWN_TIME = 10.  # How long does it take to shutdown everything
START_WAIT = 5.  # How long before sending the start command

MAX_ZERO_CMD = 30  # How many should we see a 0 cmd vel to restart

FOOD_PRINT = 5   # How often food should be printed

SHOW_OUTPUT_FROM_LAUNCH = True

configs = [
    # launchfile (without .launch), repetitions, num_food_runs, num_robots, time_limit in s
    # ('5x5-sim-3-robots', 10, 50, 3, 600),
    ('5x5-sim-6-robots', 10, 50, 6, 900),
    ('5x5-sim-9-robots', 10, 50, 9, 900),
]


def terminate_process_and_children(p):
    try:
        ps_command = Popen("ps -o pid --ppid %d --noheaders" % p.pid, shell=True, stdout=PIPE)
        ps_output = ps_command.stdout.read()
        retcode = ps_command.wait()
        assert retcode == 0, "ps command returned %d" % retcode
        for pid_str in ps_output.split("\n")[:-1]:
                os.kill(int(pid_str), signal.SIGINT)
        p.terminate()
    except Exception as e:
        rospy.logwarn('something went wrong during shutdown %s', e)


def start_environment(launch_file):
    cmd = ["roslaunch", "swarming_turtles_stage", launch_file+".launch"]
    if SHOW_OUTPUT_FROM_LAUNCH:
        process = Popen(cmd)
    else:
        process = Popen(cmd, stdout=PIPE, stderr=PIPE)  # , universal_newlines=True, stdout=None, stderr=PIPE)
    return process


class RunExperiments(object):
    food_listeners = []
    stall_listeners = []
    cmd_listeners = []
    total_food = 0
    food_counts = []
    stalled = []
    cmd_zero_counts = []
    total_food_runs = 0
    start_time = None
    time_limit = 0
    launch_process = None
    rosbag_process = None
    run_completed = False
    restart_run = False
    time_out = False
    run = 0

    def __init__(self):

        rospy.on_shutdown(self.stop_experiment)

        self.start_all = rospy.ServiceProxy('/start_all', Empty)
        rospkg_pack = rospkg.RosPack()
        path = os.path.join(rospkg_pack.get_path('swarming_turtles_stage'), 'bags')

        if not os.path.exists(path):
            try:
                os.makedirs(path)
            except OSError as exception:
                if exception.errno != errno.EEXIST:
                    rospy.logerr('could not create path %s', path)
                    exit(1)
        self.path = path
        rospy.loginfo("Logging to path %s", self.path)

    def food_cb(self, msg, idx):
        self.food_counts[idx] = msg.data
        self.total_food = sum(self.food_counts)
        if self.total_food % FOOD_PRINT == 0:
            rospy.loginfo("Foodruns %d of %d", self.total_food, self.total_food_runs)
        if self.total_food >= self.total_food_runs:
            self.run_completed = True

    def stall_cb(self, msg, idx):
        assert isinstance(msg, Stall)
        if msg.stall:
            if self.stalled[idx] == 0:
                self.stalled[idx] = rospy.Time.now()
            elif (rospy.Time.now()-self.stalled[idx]).to_sec() > STALL_RESTART_TIME:
                self.restart_run = True
        else:
            self.stalled[idx] = 0

    def cmd_cb(self, msg, idx):
        assert isinstance(msg, Twist)
        if rospy.Time.now() > self.start_time + rospy.Duration(0.5):
            if msg.angular.z == 0 and msg.linear.x == 0:
                self.cmd_zero_counts[idx] += 1
                if self.cmd_zero_counts[idx] > MAX_ZERO_CMD:
                    self.restart_run = True
            else:
                self.cmd_zero_counts[idx] = 0

    def init_for_num(self, num_robots):
        self.run_completed = False
        self.restart_run = False

        self.total_food = 0
        self.food_counts = [0] * (num_robots+1)
        self.stalled = [0] * (num_robots+1)
        self.cmd_zero_counts = [0] * (num_robots+1)

        self.update_listeners(num_robots)

    def update_listeners(self, num_robots):
        for l in self.food_listeners:
            assert isinstance(l, rospy.Subscriber)
            l.unregister()

        for l in self.stall_listeners:
            assert isinstance(l, rospy.Subscriber)
            l.unregister()

        for l in self.cmd_listeners:
            assert isinstance(l, rospy.Subscriber)
            l.unregister()

        time.sleep(0.5)
        self.food_listeners = []
        self.stall_listeners = []

        for i in range(1, num_robots+1):
            self.food_listeners.append(rospy.Subscriber("/robot_%d/fooddrops" % i, Int32, self.food_cb, i))
            self.stall_listeners.append(rospy.Subscriber("/robot_%d/stall" % i, Stall, self.stall_cb, i))
            self.cmd_listeners.append(rospy.Subscriber("/robot_%d/cmd_vel" % i, Twist, self.cmd_cb, i))

    def start_experiment(self, launchfile, num_robots):
        self.init_for_num(num_robots)
        self.launch_process = start_environment(launchfile)
        for i in range(1, num_robots + 1):
            try:
                rospy.wait_for_service("/robot_%d/start_smach" % i, 10.)
            except Exception as e:
                rospy.logwarn("Robot not started after 10 seconds, restarting, %s", e)
                self.restart_run = True
                return
        time.sleep(START_WAIT)

        rosbag_file = os.path.join(self.path, launchfile + "_run" + str(self.run) + ".bag")
        rospy.loginfo("rosbag filename: %s", rosbag_file)

        self.rosbag_process = Popen(["rosbag", "record", "/logging", "-O", rosbag_file])
        self.start_time = rospy.Time.now()

        self.start_all()

    def stop_experiment(self, wait=0.1):
        if self.launch_process.poll() is None:
            self.launch_process.send_signal(signal.SIGINT)
        if self.rosbag_process.poll() is None:
            terminate_process_and_children(self.rosbag_process)
        # self.rosbag_process.send_signal(signal.SIGINT)
        self.launch_process.wait()
        self.rosbag_process.wait()
        time.sleep(wait)

        rospy.loginfo("Terminated all processes")

    def run_experiment(self, launchfile, num_robots):
        self.start_experiment(launchfile, num_robots)
        while not self.run_completed and not rospy.is_shutdown():
            if self.restart_run:
                rospy.logwarn("Restart required for run %d, launchfile %s", self.run, launchfile)
                self.stop_experiment(SHUTDOWN_TIME)
                self.start_experiment(launchfile, num_robots)
            if (rospy.Time.now() - self.start_time).to_sec() > self.time_limit:
                rospy.logwarn("Timelimit exceeded for run %d, launchfile %s", self.run, launchfile)
                break
            time.sleep(0.1)
        rospy.loginfo("Run completed foodrun count: %d of %d", self.total_food, self.total_food_runs)
        self.stop_experiment(SHUTDOWN_TIME)

    def run_all_experiments(self):
        for launch_file, repetitions, num_food_runs, num_robots, time_limit in configs:

            self.total_food_runs = num_food_runs
            self.time_limit = time_limit

            for i in range(1, repetitions+1):
                self.run = i
                if rospy.is_shutdown():
                    return
                rospy.loginfo("Starting run %d, for launchfile %s, num_robots %d", self.run, launch_file, num_robots)
                self.run_experiment(launch_file, num_robots)


if __name__ == '__main__':
    rospy.init_node('run_experiments')
    runner = RunExperiments()
    runner.run_all_experiments()

#!/usr/bin/env python
import os
import rospkg

import errno
import rospy
import time

import signal
import math

from geometry_msgs.msg import Twist, Pose2D
from stage_ros.msg import Stall
from std_msgs.msg import Int32
from std_srvs.srv import Empty
from subprocess import Popen, PIPE

STALL_RESTART_TIME = 30.  # How long have the robots to be stalled to restart
SHUTDOWN_TIME = 10.  # How long does it take to shutdown everything
START_WAIT = 5.  # How long before sending the start command

MAX_ZERO_CMD = 30  # How many should we see a 0 cmd vel to restart

MAX_NO_CMD_RECEIVED = 30  # After how long there should be a restart when no cmd is received

FOOD_PRINT = 1   # How often food should be printed

SHOW_OUTPUT_FROM_LAUNCH = False

configs = [
    # launchfile (without .launch), repetitions, num_food_runs, num_robots, time_limit in s, array of future food poses (change after runs, Pose2D)

#    ('5x5-sim-1-robots-shepherd', 5, 50, 1, 1500, []),
#    ('5x5-sim-2-robots-shepherd', 5, 50, 2, 1200, []),
#    ('5x5-sim-3-robots-shepherd', 5, 50, 3, 900, []),
#    ('5x5-sim-4-robots-shepherd', 10, 50, 4, 900, []),
#    ('5x5-sim-5-robots-shepherd', 10, 50, 5, 900, []),
#    ('5x5-sim-6-robots-shepherd', 5, 50, 6, 900, []),
#    ('5x5-sim-7-robots-shepherd', 5, 50, 7, 900, []),
#    ('5x5-sim-8-robots-shepherd', 2, 50, 8, 1200, []),
#    ('5x5-sim-9-robots-shepherd', 4, 50, 9, 1800, []),


#    ('10x10-sim-9-robots-shepherd', 10, 50, 9, 1500, []),
#    ('10x10-sim-9-robots-2food', 8, 100, 9, 2000, [(50, Pose2D(-3.0,4.9, math.radians(-90)))]),
#    ('10x10-sim-9-robots-2food-distx2', 10, 100, 9, 2000, [(50, Pose2D(-3.0,4.9, math.radians(-90)))]),
#    ('10x10-sim-9-robots', 7, 50, 9, 2000, []),
#    ('10x10-sim-9-robots-2food-shepherd', 8, 100, 9, 2000, [(50, Pose2D(-3.0,4.9, math.radians(-90)))]),

    ('10xL-sim-9-robots-3food', 1, 150, 9, 3000, [(50, Pose2D(-2.5,4.9, math.radians(-90))),(100, Pose2D(4.9,-2.5, math.radians(180)))]),


#    ('5x5-sim-9-robots', 10, 50, 9, 1800, []),
#    ('5x5-sim-8-robots', 10, 50, 8, 1200, []),
#    ('5x5-sim-7-robots', 1, 50, 7, 1000, []),
#    ('5x5-sim-6-robots', 1, 50, 6, 1000, []),
#    ('5x5-sim-5-robots', 5, 50, 5, 1000, []),
#    ('5x5-sim-4-robots', 9, 50, 4, 1000, []),
#    ('5x5-sim-3-robots', 9, 50, 3, 1000, []),
#    ('5x5-sim-2-robots', 9, 50, 2, 1200, []),
#    ('5x5-sim-1-robots', 9, 50, 1, 1800, []),





#    ('5x5-sim-4-robots', 1, 50, 4, 1000, [(5, Pose2D(2,3, math.pi))]),
#    ('5x5-sim-3-robots-shepherd', 1, 15, 3, 1000, [(5, Pose2D(-4.0,-0.29, math.radians(-90))),(10, Pose2D(-1.0,-0.29, math.radians(-90)))]),
#    ('5x5-sim-2-robots', 1, 50, 2, 1200, [(5, Pose2D(2,3, math.pi))]),
#    ('5x5-sim-1-robots', 1, 50, 1, 1800, [(5, Pose2D(2,3, math.pi))]),


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
    current_food_pose = 0
    food_poses = []
    total_food = 0
    food_counts = []
    stalled = []
    cmd_zero_counts = []
    last_cmd_vels = []
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
        self.food_pose_pub = rospy.Publisher('/food/set_pose', Pose2D, queue_size=1)
        rospy.loginfo("Logging to path %s", self.path)

    def food_cb(self, msg, idx):
        self.food_counts[idx] = msg.data
        self.total_food = sum(self.food_counts)
        if self.total_food % FOOD_PRINT == 0:
            rospy.loginfo("Foodruns %d of %d", self.total_food, self.total_food_runs)

        if len(self.food_poses) > 0 and self.current_food_pose < len(self.food_poses):  # check if we have more than one food poses, and if the current one is not the last one
            if self.total_food >= self.food_poses[self.current_food_pose][0]:
                rospy.loginfo("Changing food pose after %d runs to %s", self.total_food, str(self.food_poses[self.current_food_pose][1]))
                self.food_pose_pub.publish(self.food_poses[self.current_food_pose][1])
                self.current_food_pose += 1
                try:
                    os.system("sh beep.sh")
                except:
                    pass

        if self.total_food >= self.total_food_runs:
            self.run_completed = True

    def stall_cb(self, msg, idx):
        assert isinstance(msg, Stall)
        if msg.stall:
            if self.stalled[idx] == 0:
                self.stalled[idx] = rospy.Time.now()
            elif (rospy.Time.now()-self.stalled[idx]).to_sec() > STALL_RESTART_TIME:
                rospy.logwarn("Stall Time exceeded in run %d for robot %d", self.run, idx)
                self.restart_run = True
        else:
            self.stalled[idx] = 0

    def cmd_cb(self, msg, idx):
        assert isinstance(msg, Twist)
        self.last_cmd_vels[idx-1] = rospy.Time.now()
        if rospy.Time.now() > self.start_time + rospy.Duration(0.5):  #only start checkin after 0.5 seconds after start
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
        self.current_food_pose = 0
        self.food_counts = [0] * (num_robots+1)
        self.stalled = [0] * (num_robots+1)
        self.cmd_zero_counts = [0] * (num_robots+1)
        self.last_cmd_vels = [rospy.Time(0)] * (num_robots)
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
        try:
            os.system("sh beep.sh")
        except:
            pass
        rosbag_file = os.path.join(self.path, launchfile + "_run" + str(self.run) + ".bag")
        rospy.loginfo("rosbag filename: %s", rosbag_file)

        self.rosbag_process = Popen(["rosbag", "record", "/logging", "-O", rosbag_file])

        self.start_time = rospy.Time.now()

        # set last cmd received to start time for checking if they send cmd_vels
        for i in range(len(self.last_cmd_vels)):
            self.last_cmd_vels[i] = self.start_time
        self.start_all()

    def stop_experiment(self, wait=0.1):
        if self.launch_process.poll() is None:
            self.launch_process.send_signal(signal.SIGINT)
        if self.rosbag_process is not None and self.rosbag_process.poll() is None:
            terminate_process_and_children(self.rosbag_process)
        # self.rosbag_process.send_signal(signal.SIGINT)
        self.launch_process.wait()
        self.rosbag_process.wait()
        time.sleep(wait)

        rospy.loginfo("Terminated all processes")

    def check_cmd_vels(self):
        now = rospy.Time.now()
        for idx, r in enumerate(self.last_cmd_vels):
            if (now - r).to_sec() > MAX_NO_CMD_RECEIVED:
                rospy.logwarn("No cmd_vel received for robot %d for %f seconds", idx+1, (now-r).to_sec())
                return False
        return True

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

            if not self.check_cmd_vels():
                self.restart_run = True
            time.sleep(0.1)
        if not rospy.is_shutdown():
            rospy.loginfo("Run completed foodrun count: %d of %d", self.total_food, self.total_food_runs)
            self.stop_experiment(SHUTDOWN_TIME)
        else:
            rospy.loginfo("rospy shutdown")

    def run_all_experiments(self):
        for launch_file, repetitions, num_food_runs, num_robots, time_limit, next_food_poses in configs:

            self.total_food_runs = num_food_runs
            self.time_limit = time_limit
            self.food_poses = next_food_poses
            for i in range(1, repetitions+1):
                self.run = i
                if rospy.is_shutdown():
                    return
                rospy.loginfo("Starting run %d of %d, for launchfile %s, num_robots %d", self.run, repetitions, launch_file, num_robots)
                self.run_experiment(launch_file, num_robots)


if __name__ == '__main__':
    rospy.init_node('run_experiments')
    runner = RunExperiments()
    runner.run_all_experiments()

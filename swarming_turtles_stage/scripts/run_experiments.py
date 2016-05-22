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
START_WAIT_SVC = 60.  # How long to eait for the service to start


#MAX_ZERO_CMD = 30  # How many should we see a 0 cmd vel to restart
MAX_ZERO_CMD = 30  # How many should we see a 0 cmd vel to restart


MAX_NO_CMD_RECEIVED = 30  # After how long there should be a restart when no cmd is received

FOOD_PRINT = 1   # How often food should be printed

SHOW_OUTPUT_FROM_LAUNCH = True


#----move food ----#
cm = 1.0
pos = -13.0

def move():
    global cm, pos
    pos += cm
    return pos



configs = [
    # launchfile (without .launch), additional_file-naming, repetitions, num_food_runs, num_robots, time_limit in s, array of future food poses (change after runs, Pose2D)


#################### SHEPHERDING #####################

#    ('5x5-sim-1-robots-shepherd','', 5, 50, 1, 1500, []),
#    ('5x5-sim-2-robots-shepherd','', 5, 50, 2, 1200, []),
#    ('5x5-sim-3-robots-shepherd','', 5, 50, 3, 900, []),
#    ('5x5-sim-4-robots-shepherd','', 10, 50, 4, 900, []),
#    ('5x5-sim-5-robots-shepherd','', 10, 50, 5, 900, []),
#    ('5x5-sim-6-robots-shepherd','', 5, 50, 6, 900, []),
#    ('5x5-sim-7-robots-shepherd','', 5, 50, 7, 900, []),
#    ('5x5-sim-8-robots-shepherd','', 2, 50, 8, 1200, []),
#    ('5x5-sim-9-robots-shepherd','', 4, 50, 9, 1800, []),

#    ('10x10-sim-9-robots-shepherd','', 10, 50, 9, 1500, []),

#    ('10x10-sim-9-robots-shepherd','-2food', 8, 100, 9, 2000, [(50, Pose2D(-3.0,4.9, math.radians(-90)))]),

#    ('10xL-sim-9-robots-shepherd','-3food', 1, 150, 9, 3000, [(50, Pose2D(-2.5,4.9, math.radians(-90))),(100, Pose2D(4.9,-2.5, math.radians(180)))]),


##################### SWARMING #####################
#
#---------------------5x5 1 food------------------
#    ('5x5-sim-9-robots','', 10, 50, 9, 1800, []),
#    ('5x5-sim-8-robots','', 10, 50, 8, 1200, []),
#    ('5x5-sim-7-robots','', 10, 50, 7, 1000, []),
#    ('5x5-sim-6-robots','', 10, 50, 6, 1000, []),
#    ('5x5-sim-5-robots','', 10, 50, 5, 1000, []),
#    ('5x5-sim-4-robots','', 10, 50, 4, 1000, []),
#    ('5x5-sim-3-robots','', 10, 50, 3, 1000, []),
#    ('5x5-sim-2-robots','', 10, 50, 2, 1200, []),
#    ('5x5-sim-1-robots','', 10, 50, 1, 1800, []),
#----------------------------------------------------

#-------------------10x10 1 food------------------
#    ('10x10-sim-9-robots','', 10, 50, 9, 2000, []),
#    ('10x10-sim-8-robots','', 1, 50, 8, 2000, []),
#    ('10x10-sim-7-robots','', 1, 50, 7, 2000, []),
#    ('10x10-sim-6-robots','', 10, 50, 6, 2000, []),
#    ('10x10-sim-5-robots','', 10, 50, 5, 2000, []),
#    ('10x10-sim-4-robots','', 1, 50, 4, 2000, []),
#    ('10x10-sim-3-robots','', 10, 50, 3, 2500, []),
#    ('10x10-sim-2-robots','', 10, 50, 2, 3000, []),
#    ('10x10-sim-1-robots','', 1, 50, 1, 4500, []),
#    ('10x10-sim-10-robots','', 1, 50, 10, 4000, []),
#    ('10x10-sim-11-robots','', 9, 50, 11, 4000, []),
#    ('10x10-sim-12-robots','', 9, 50, 12, 4000, []),
#    ('10x10-sim-13-robots','', 1, 50, 13, 4000, []),
#    ('10x10-sim-14-robots','', 9, 50, 14, 4500, []),
#    ('10x10-sim-15-robots','', 1, 50, 15, 4000, []),
#    ('10x10-sim-16-robots','', 1, 50, 16, 4000, []),
#    ('10x10-sim-17-robots','', 1, 50, 17, 4000, []),
#    ('10x10-sim-18-robots','', 1, 50, 18, 4000, []),
#----------------------------------------------------

#-----------------10x10 1 food distx2----------------
#  for distx2 make sure to change the random walk distance parameter 
#  in move_random.py
#----------------------------------------------------
#    ('10x10-sim-10-robots','-dist20m', 4, 50, 10, 4000, []),
#    ('10x10-sim-11-robots','-dist20m', 4, 50, 11, 4000, []),
#    ('10x10-sim-12-robots','-dist20m', 4, 50, 12, 4000, []),
#    ('10x10-sim-13-robots','-dist20m', 4, 50, 13, 4000, []),
#    ('10x10-sim-9-robots','-dist20m', 4, 50, 9, 3000, []),
#    ('10x10-sim-8-robots','-dist20m', 4, 50, 8, 3000, []),
#    ('10x10-sim-7-robots','-dist20m', 4, 50, 7, 3000, []),
#    ('10x10-sim-6-robots','-dist20m', 4, 50, 6, 3000, []),
#    ('10x10-sim-5-robots','-dist20m', 4, 50, 5, 3000, []),
#    ('10x10-sim-4-robots','-dist20m', 4, 50, 4, 3000, []),
#    ('10x10-sim-3-robots','-dist20m', 4, 50, 3, 3000, []),
#    ('10x10-sim-2-robots','-dist20m', 4, 50, 2, 3000, []),
#    ('10x10-sim-1-robots','-dist20m', 4, 50, 1, 5000, []),
#    ('10x10-sim-14-robots','-dist20m', 4, 50, 14, 4000, []),
#----------------------------------------------------


#-----------------18x10 1 food distx2----------------
#  for distx2 make sure to change the random walk distance parameter 
#  in move_random.py
#----------------------------------------------------
    #('18x10-sim-10-robots','-dist10m', 10, 50, 10, 4000, []),
    #('18x10-sim-11-robots','-dist10m', 10, 50, 11, 4000, []),
    #('18x10-sim-12-robots','-dist10m', 10, 50, 12, 4000, []),
    #('18x10-sim-9-robots','-dist10m', 10, 50, 9, 3000, []),
    #('18x10-sim-8-robots','-dist10m', 10, 50, 8, 3000, []),
    #('18x10-sim-7-robots','-dist10m', 10, 50, 7, 3000, []),
    #('18x10-sim-6-robots','-dist10m', 10, 50, 6, 3000, []),
    #('18x10-sim-5-robots','-dist10m', 5, 50, 5, 3000, []),
    #('18x10-sim-4-robots','-dist10m', 10, 50, 4, 3000, []),
    #('18x10-sim-3-robots','-dist10m', 10, 50, 3, 3000, []),
    #('18x10-sim-2-robots','-dist10m', 10, 50, 2, 3000, []),
    #('18x10-sim-13-robots','-dist10m', 10, 50, 13, 4000, []),
    #('18x10-sim-1-robots','-dist10m', 10, 50, 1, 5000, []),
    #('18x10-sim-14-robots','-dist10m', 10, 50, 14, 4000, []),
#----------------------------------------------------


#-----------------20x20 1 food - open environment sqrt same as 10x10, randomwalk 20m ----------------
#  Make sure to change the random walk distance parameter 
#  in move_random.py
#----------------------------------------------------
    #('20x20-sim-10-robots','-dist20m', 1, 50, 10, 4000, []),
    #('20x20-sim-11-robots','-dist20m', 10, 50, 11, 4000, []),
    #('20x20-sim-12-robots','-dist20m', 10, 50, 12, 4000, []),
    #('20x20-sim-9-robots','-dist20m', 10, 50, 9, 3000, []),
    #('20x20-sim-8-robots','-dist20m', 10, 50, 8, 3000, []),
    #('20x20-sim-7-robots','-dist20m', 10, 50, 7, 3000, []),
    #('20x20-sim-6-robots','-dist20m', 10, 50, 6, 3000, []),
    #('20x20-sim-5-robots','-dist20m', 7, 50, 5, 3000, []),
    #('20x20-sim-4-robots','-dist20m', 10, 50, 4, 3000, []),
    #('20x20-sim-3-robots','-dist20m', 10, 50, 3, 3000, []),
    #('20x20-sim-2-robots','-dist20m', 1, 50, 2, 3000, []),
    #('20x20-sim-13-robots','-dist20m', 10, 50, 13, 4000, []),
    #('20x20-sim-1-robots','-dist20m', 10, 50, 1, 5000, []),
    #('20x20-sim-14-robots','-dist20m', 10, 50, 14, 4000, []),
    #----------------------------------------------------

#-----------------20x20 1 food - open environment sqrt same as 10x10, randomwalk 20m ----------------
#  Make sure to change the random walk distance parameter 
#  in move_random.py
#----------------------------------------------------

    #----------------------------------------------------
    #('20x20-sim-10-robots','-dist20m-food7m', 10, 50, 10, 4000, [(0, Pose2D(1.0,1.81, math.radians(-90)))]),
    #('20x20-sim-11-robots','-dist20m-food7m', 10, 50, 11, 4000, [(0, Pose2D(1.0,1.81, math.radians(-90)))]),
    #('20x20-sim-12-robots','-dist20m-food7m', 10, 50, 12, 4000, [(0, Pose2D(1.0,1.81, math.radians(-90)))]),
    #('20x20-sim-9-robots','-dist20m-food7m', 10, 50, 9, 3000, [(0, Pose2D(1.0,1.81, math.radians(-90)))]),
    #('20x20-sim-8-robots','-dist20m-food7m', 2, 50, 8, 3000, [(0, Pose2D(1.0,1.81, math.radians(-90)))]),
    #('20x20-sim-7-robots','-dist20m-food7m', 10, 50, 7, 3000, [(0, Pose2D(1.0,1.81, math.radians(-90)))]),
    #('20x20-sim-6-robots','-dist20m-food7m', 10, 50, 6, 3000, [(0, Pose2D(1.0,1.81, math.radians(-90)))]),
    #('20x20-sim-5-robots','-dist20m-food7m', 10, 50, 5, 3000, [(0, Pose2D(1.0,1.81, math.radians(-90)))]),
    #('20x20-sim-4-robots','-dist20m-food7m', 10, 50, 4, 3000, [(0, Pose2D(1.0,1.81, math.radians(-90)))]),
    #('20x20-sim-3-robots','-dist20m-food7m', 10, 50, 3, 3000, [(0, Pose2D(1.0,1.81, math.radians(-90)))]),
    #('20x20-sim-2-robots','-dist20m-food7m', 10, 50, 2, 3000, [(0, Pose2D(1.0,1.81, math.radians(-90)))]),
    #('20x20-sim-1-robots','-dist20m-food7m', 10, 50, 1, 5000, [(0, Pose2D(1.0,1.81, math.radians(-90)))]),
    #('20x20-sim-13-robots','-dist20m-food7m', 10, 50, 13, 4000, [(0, Pose2D(1.0,1.81, math.radians(-90)))]),
    #('20x20-sim-14-robots','-dist20m-food7m', 10, 50, 14, 4000, [(0, Pose2D(1.0,1.81, math.radians(-90)))]),

    #('20x20-sim-9-robots','-dist20m-food5m-150x', 10, 150, 9, 3000, [(0, Pose2D(-1.0,-0.29, math.radians(-90)))]),
    #('20x20-sim-10-robots','-dist20m-food5m-150x', 2, 150, 10, 4000, [(0, Pose2D(-1.0,-0.29, math.radians(-90)))]),
    #('20x20-sim-11-robots','-dist20m-food5m-150x', 10, 150, 11, 4000, [(0, Pose2D(-1.0,-0.29, math.radians(-90)))]),
    #('20x20-sim-12-robots','-dist20m-food5m-150x', 4, 150, 12, 4000, [(0, Pose2D(-1.0,-0.29, math.radians(-90)))]),
    #('20x20-sim-13-robots','-dist20m-food5m-150x', 10, 150, 13, 4000, [(0, Pose2D(-1.0,-0.29, math.radians(-90)))]),
    #('20x20-sim-14-robots','-dist20m-food5m-150x', 10, 150, 14, 4000, [(0, Pose2D(-1.0,-0.29, math.radians(-90)))]),
    #('20x20-sim-8-robots','-dist20m-food5m-150x', 10, 150, 8, 3000, [(0, Pose2D(-1.0,-0.29, math.radians(-90)))]),
    #('20x20-sim-7-robots','-dist20m-food5m-150x', 10, 150, 7, 3000, [(0, Pose2D(-1.0,-0.29, math.radians(-90)))]),
    #('20x20-sim-6-robots','-dist20m-food5m-150x', 10, 150, 6, 3000, [(0, Pose2D(-1.0,-0.29, math.radians(-90)))]),
    #('20x20-sim-5-robots','-dist20m-food5m-150x', 10, 150, 5, 3000, [(0, Pose2D(-1.0,-0.29, math.radians(-90)))]),
    #('20x20-sim-4-robots','-dist20m-food5m-150x', 10, 150, 4, 3000, [(0, Pose2D(-1.0,-0.29, math.radians(-90)))]),
    #('20x20-sim-3-robots','-dist20m-food5m-150x', 10, 150, 3, 3000, [(0, Pose2D(-1.0,-0.29, math.radians(-90)))]),
    #('20x20-sim-2-robots','-dist20m-food5m-150x', 10, 150, 2, 3000, [(0, Pose2D(-1.0,-0.29, math.radians(-90)))]),
    #('20x20-sim-1-robots','-dist20m-food5m-150x', 10, 150, 1, 5000, [(0, Pose2D(-1.0,-0.29, math.radians(-90)))]),

    #('10x10-sim-10-robots','-dist20m-food5m', 10, 50, 10, 4000, [(0, Pose2D(-1.0,-0.29, math.radians(-90)))]),
    #('10x10-sim-11-robots','-dist20m-food5m', 10, 50, 11, 4000, [(0, Pose2D(-1.0,-0.29, math.radians(-90)))]),
    #('10x10-sim-12-robots','-dist20m-food5m', 10, 50, 12, 4000, [(0, Pose2D(-1.0,-0.29, math.radians(-90)))]),
    #('10x10-sim-8-robots','-dist20m-food5m', 10, 50, 8, 3000, [(0, Pose2D(-1.0,-0.29, math.radians(-90)))]),
    #('10x10-sim-7-robots','-dist20m-food5m', 10, 50, 7, 3000, [(0, Pose2D(-1.0,-0.29, math.radians(-90)))]),
    #('10x10-sim-6-robots','-dist20m-food5m', 10, 50, 6, 3000, [(0, Pose2D(-1.0,-0.29, math.radians(-90)))]),
    #('10x10-sim-5-robots','-dist20m-food5m', 10, 50, 5, 3000, [(0, Pose2D(-1.0,-0.29, math.radians(-90)))]),
    #('10x10-sim-4-robots','-dist20m-food5m', 10, 50, 4, 3000, [(0, Pose2D(-1.0,-0.29, math.radians(-90)))]),
    #('10x10-sim-3-robots','-dist20m-food5m', 1, 50, 3, 3000, [(0, Pose2D(-1.0,-0.29, math.radians(-90)))]),
    #('10x10-sim-2-robots','-dist20m-food5m', 10, 50, 2, 3000, [(0, Pose2D(-1.0,-0.29, math.radians(-90)))]),
    #('10x10-sim-1-robots','-dist20m-food5m', 10, 50, 1, 5000, [(0, Pose2D(-1.0,-0.29, math.radians(-90)))]),
    #('10x10-sim-13-robots','-dist20m-food5m', 3, 50, 13, 4000, [(0, Pose2D(-1.0,-0.29, math.radians(-90)))]),
    #('10x10-sim-9-robots','-dist20m-food5m', 10, 50, 9, 3000, [(0, Pose2D(-1.0,-0.29, math.radians(-90)))]),
#    ('10x10-sim-14-robots','-dist20m-food5m', 10, 50, 14, 4000, [(0, Pose2D(-1.0,-0.29, math.radians(-90)))]),

    #('10x10-sim-10-robots','-dist20m-food7m', 10, 50, 10, 4000, [(0, Pose2D(1.0,1.81, math.radians(-90)))]),
    #('10x10-sim-11-robots','-dist20m-food7m', 10, 50, 11, 4000, [(0, Pose2D(1.0,1.81, math.radians(-90)))]),
    #('10x10-sim-12-robots','-dist20m-food7m', 1, 50, 12, 4000, [(0, Pose2D(1.0,1.81, math.radians(-90)))]),
    #('10x10-sim-13-robots','-dist20m-food7m', 10, 50, 13, 4000, [(0, Pose2D(1.0,1.81, math.radians(-90)))]),
    #('10x10-sim-8-robots','-dist20m-food7m', 10, 50, 8, 3000, [(0, Pose2D(1.0,1.81, math.radians(-90)))]),
    #('10x10-sim-7-robots','-dist20m-food7m', 10, 50, 7, 3000, [(0, Pose2D(1.0,1.81, math.radians(-90)))]),
    #('10x10-sim-6-robots','-dist20m-food7m', 10, 50, 6, 3000, [(0, Pose2D(1.0,1.81, math.radians(-90)))]),
    #('10x10-sim-5-robots','-dist20m-food7m', 10, 50, 5, 3000, [(0, Pose2D(1.0,1.81, math.radians(-90)))]),
    #('10x10-sim-4-robots','-dist20m-food7m', 10, 50, 4, 3000, [(0, Pose2D(1.0,1.81, math.radians(-90)))]),
    #('10x10-sim-3-robots','-dist20m-food7m', 4, 50, 3, 3000, [(0, Pose2D(1.0,1.81, math.radians(-90)))]),
    #('10x10-sim-2-robots','-dist20m-food7m', 10, 50, 2, 3000, [(0, Pose2D(1.0,1.81, math.radians(-90)))]),
    #('10x10-sim-1-robots','-dist20m-food7m', 10, 50, 1, 5000, [(0, Pose2D(1.0,1.81, math.radians(-90)))]),
    #('10x10-sim-9-robots','-dist20m-food7m', 10, 50, 9, 3000, [(0, Pose2D(1.0,1.81, math.radians(-90)))]),
    #('10x10-sim-14-robots','-dist20m-food7m', 10, 50, 14, 4000, [(0, Pose2D(1.0,1.81, math.radians(-90)))]),


#-----------------40x40 1 food - open environment sqrt same as 10x10, randomwalk 40m ----------------
#  Make sure to change the random walk distance parameter 
#  in move_random.py
#----------------------------------------------------
    #('40x40-sim-10-robots','-dist40m', 10, 50, 10, 5000, []),
    #('40x40-sim-11-robots','-dist40m', 10, 50, 11, 5000, []),
    #('40x40-sim-12-robots','-dist40m', 10, 50, 12, 5000, []),
    #('40x40-sim-9-robots','-dist40m', 10, 50, 9, 5000, []),
    #('40x40-sim-8-robots','-dist40m', 10, 50, 8, 5000, []),
    #('40x40-sim-7-robots','-dist40m', 10, 50, 7, 5000, []),
    #('40x40-sim-6-robots','-dist40m', 10, 50, 6, 5000, []),
    #('40x40-sim-5-robots','-dist40m', 10, 50, 5, 5000, []),
    #('40x40-sim-4-robots','-dist40m', 10, 50, 4, 5000, []),
    #('40x40-sim-3-robots','-dist40m', 10, 50, 3, 5000, []),
    #('40x40-sim-2-robots','-dist40m', 10, 50, 2, 5000, []),
    #('40x40-sim-13-robots','-dist40m', 10, 50, 13, 5000, []),
    #('40x40-sim-1-robots','-dist40m', 10, 50, 1, 5000, []),
    #('40x40-sim-14-robots','-dist40m', 10, 50, 14, 5000, []),
    #----------------------------------------------------



#-----------------7x7 1 food ----------------
#  Make sure to change the random walk distance parameter 
#  in move_random.py
#----------------------------------------------------
#    ('7x7-sim-10-robots','', 10, 50, 10, 4000, []),
#    ('7x7-sim-11-robots','', 10, 50, 11, 4000, []),
#    ('7x7-sim-9-robots','', 10, 50, 9, 3000, []),
#    ('7x7-sim-8-robots','', 10, 50, 8, 3000, []),
#    ('7x7-sim-7-robots','', 10, 50, 7, 3000, []),
#    ('7x7-sim-6-robots','', 10, 50, 6, 3000, []),
#    ('7x7-sim-5-robots','', 6, 50, 5, 3000, []),
#    ('7x7-sim-4-robots','', 10, 50, 4, 3000, []),
#    ('7x7-sim-3-robots','', 10, 50, 3, 3000, []),
#    ('7x7-sim-2-robots','', 10, 50, 2, 3000, []),
#    ('7x7-sim-1-robots','', 10, 50, 1, 5000, []),
#    ('7x7-sim-12-robots','', 9, 50, 12, 4000, []),
#    ('7x7-sim-13-robots','', 10, 50, 13, 4000, []),
#    ('7x7-sim-14-robots','', 10, 50, 14, 4000, []),
#----------------------------------------------------



#--------------------- 10x10 x food ----------------


    #('10x10-sim-11-robots','-movingfoodtest', 10, 50, 11, 3000, [(0, Pose2D(move(),4.9, math.radians(-90)))
                                                                #,(23, Pose2D(move(),4.9, math.radians(-90)))
                                                                #,(24, Pose2D(move(),4.9, math.radians(-90)))
                                                                #,(25, Pose2D(move(),4.9, math.radians(-90)))
                                                                #,(26, Pose2D(move(),4.9, math.radians(-90)))
                                                                #,(27, Pose2D(move(),4.9, math.radians(-90)))
                                                                #,(28, Pose2D(move(),4.9, math.radians(-90)))
                                                                #,(29, Pose2D(move(),4.9, math.radians(-90)))
                                                                #,(30, Pose2D(move(),4.9, math.radians(-90)))
                                                                #,(31, Pose2D(move(),4.9, math.radians(-90)))
                                                                #,(32, Pose2D(move(),4.9, math.radians(-90)))
                                                                #,(33, Pose2D(move(),4.9, math.radians(-90)))
                                                                #,(34, Pose2D(move(),4.9, math.radians(-90)))
                                                                #,(35, Pose2D(move(),4.9, math.radians(-90)))
                                                                #,(36, Pose2D(move(),4.9, math.radians(-90)))
                                                                #,(37, Pose2D(move(),4.9, math.radians(-90)))
                                                                #,(38, Pose2D(move(),4.9, math.radians(-90)))
                                                                #,(39, Pose2D(move(),4.9, math.radians(-90)))
                                                                #,(40, Pose2D(move(),4.9, math.radians(-90)))
                                                                #,(41, Pose2D(move(),4.9, math.radians(-90)))
                                                                #,(42, Pose2D(move(),4.9, math.radians(-90)))
                                                                #,(43, Pose2D(move(),4.9, math.radians(-90)))
                                                                #,(44, Pose2D(move(),4.9, math.radians(-90)))
                                                                #,(45, Pose2D(move(),4.9, math.radians(-90)))
                                                                #]),
#----------------------------------------------------
    #('18x10-sim-10-robots','-movingfood-10x', 10, 300, 10, 6000, [(0, Pose2D(move(),4.9, math.radians(-90)))
                                                                #,(25, Pose2D(move(),4.9, math.radians(-90)))
                                                                #,(50, Pose2D(move(),4.9, math.radians(-90)))
                                                                #,(75, Pose2D(move(),4.9, math.radians(-90)))
                                                                #,(100, Pose2D(move(),4.9, math.radians(-90)))
                                                                #,(125, Pose2D(move(),4.9, math.radians(-90)))
                                                                #,(150, Pose2D(move(),4.9, math.radians(-90)))
                                                                #,(175, Pose2D(move(),4.9, math.radians(-90)))
                                                                #,(225, Pose2D(move(),4.9, math.radians(-90)))
                                                                #,(250, Pose2D(move(),4.9, math.radians(-90)))
                                                                #,(275, Pose2D(move(),4.9, math.radians(-90)))
                                                                #]),
#----------------------------------------------------

#--------------------- 10x10 2 food ----------------
    #('10x10-sim-9-robots','-2food', 8, 100, 9, 3000, [(50, Pose2D(-3.0,4.9, math.radians(-90)))]),
    #('10x10-sim-8-robots','-2food', 10, 100, 8, 3000, [(50, Pose2D(-3.0,4.9, math.radians(-90)))]),
    #('10x10-sim-7-robots','-2food', 10, 100, 7, 3000, [(50, Pose2D(-3.0,4.9, math.radians(-90)))]),
    #('10x10-sim-6-robots','-2food', 10, 100, 6, 3000, [(50, Pose2D(-3.0,4.9, math.radians(-90)))]),
    #('10x10-sim-5-robots','-2food', 10, 100, 5, 4000, [(50, Pose2D(-3.0,4.9, math.radians(-90)))]),
    #('10x10-sim-4-robots','-2food', 10, 100, 4, 4000, [(50, Pose2D(-3.0,4.9, math.radians(-90)))]),
    #('10x10-sim-3-robots','-2food', 10, 100, 3, 4000, [(50, Pose2D(-3.0,4.9, math.radians(-90)))]),
    #('10x10-sim-2-robots','-2food', 10, 100, 2, 5000, [(50, Pose2D(-3.0,4.9, math.radians(-90)))]),
    #('10x10-sim-1-robots','-2food', 10, 100, 1, 6000, [(50, Pose2D(-3.0,4.9, math.radians(-90)))]),
    #('10x10-sim-10-robots','-2food', 10, 100, 10, 3000, [(50, Pose2D(-3.0,4.9, math.radians(-90)))]),
    #('10x10-sim-11-robots','-2food', 10, 100, 11, 3000, [(50, Pose2D(-3.0,4.9, math.radians(-90)))]),
    #('10x10-sim-12-robots','-2food', 10, 100, 12, 4000, [(50, Pose2D(-3.0,4.9, math.radians(-90)))]),
    #('10x10-sim-13-robots','-2food', 10, 100, 13, 5000, [(50, Pose2D(-3.0,4.9, math.radians(-90)))]),
    #('10x10-sim-14-robots','-2food', 10, 100, 14, 6000, [(50, Pose2D(-3.0,4.9, math.radians(-90)))]),
#----------------------------------------------------



#-----------------10x10 2 food distx2----------------
#  for distx2 make sure to change the random walk distance parameter 
#----------------------------------------------------
#    ('10x10-sim-9-robots','-2food-distx2', 10, 100, 9, 2000, [(50, Pose2D(-3.0,4.9, math.radians(-90)))]),
#----------------------------------------------------

#-----------------10xL 3 food ----------------
    #('10xL-sim-9-robots','-3food', 10, 150, 9, 5000, [(50, Pose2D(-2.5,4.9, math.radians(-90))),(100, Pose2D(4.9,-2.5, math.radians(180)))]),
    #('10xL-sim-8-robots','-3food', 10, 150, 8, 5000, [(50, Pose2D(-2.5,4.9, math.radians(-90))),(100, Pose2D(4.9,-2.5, math.radians(180)))]),
    #('10xL-sim-7-robots','-3food', 10, 150, 7, 5000, [(50, Pose2D(-2.5,4.9, math.radians(-90))),(100, Pose2D(4.9,-2.5, math.radians(180)))]),
    #('10xL-sim-6-robots','-3food', 10, 150, 6, 5000, [(50, Pose2D(-2.5,4.9, math.radians(-90))),(100, Pose2D(4.9,-2.5, math.radians(180)))]),
    #('10xL-sim-5-robots','-3food', 10, 150, 5, 5000, [(50, Pose2D(-2.5,4.9, math.radians(-90))),(100, Pose2D(4.9,-2.5, math.radians(180)))]),
    #('10xL-sim-4-robots','-3food', 10, 150, 4, 5000, [(50, Pose2D(-2.5,4.9, math.radians(-90))),(100, Pose2D(4.9,-2.5, math.radians(180)))]),
    #('10xL-sim-3-robots','-3food', 8, 150, 3, 5000, [(50, Pose2D(-2.5,4.9, math.radians(-90))),(100, Pose2D(4.9,-2.5, math.radians(180)))]),
    #('10xL-sim-2-robots','-3food', 10, 150, 2, 5000, [(50, Pose2D(-2.5,4.9, math.radians(-90))),(100, Pose2D(4.9,-2.5, math.radians(180)))]),
    #('10xL-sim-1-robots','-3food', 10, 150, 1, 9000, [(50, Pose2D(-2.5,4.9, math.radians(-90))),(100, Pose2D(4.9,-2.5, math.radians(180)))]),
    #('10xL-sim-10-robots','-3food', 10, 150, 10, 5000, [(50, Pose2D(-2.5,4.9, math.radians(-90))),(100, Pose2D(4.9,-2.5, math.radians(180)))]),
    #('10xL-sim-11-robots','-3food', 10, 150, 11, 5000, [(50, Pose2D(-2.5,4.9, math.radians(-90))),(100, Pose2D(4.9,-2.5, math.radians(180)))]),
    #('10xL-sim-12-robots','-3food', 10, 150, 12, 5000, [(50, Pose2D(-2.5,4.9, math.radians(-90))),(100, Pose2D(4.9,-2.5, math.radians(180)))]),
    ('10xL-sim-13-robots','-3food', 5, 150, 13, 5000, [(50, Pose2D(-2.5,4.9, math.radians(-90))),(100, Pose2D(4.9,-2.5, math.radians(180)))]),
    #('10xL-sim-14-robots','-3food', 10, 150, 14, 5000, [(50, Pose2D(-2.5,4.9, math.radians(-90))),(100, Pose2D(4.9,-2.5, math.radians(180)))]),
#----------------------------------------------------


###### EXAMPLES #######

#    ('5x5-sim-4-robots','', 1, 50, 4, 1000, [(5, Pose2D(2,3, math.pi))]),
#    ('5x5-sim-3-robots-shepherd','', 1, 15, 3, 1000, [(5, Pose2D(-4.0,-0.29, math.radians(-90))),(10, Pose2D(-1.0,-0.29, math.radians(-90)))]),
#    ('5x5-sim-2-robots','', 1, 50, 2, 1200, [(5, Pose2D(2,3, math.pi))]),
#    ('5x5-sim-1-robots','', 1, 50, 1, 1800, [(5, Pose2D(2,3, math.pi))]),


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
    foodpick_listeners = []
    stall_listeners = []
    cmd_listeners = []
    current_food_pose = 0
    food_poses = []
    total_food = 0
    food_counts = []
    totalpick_food = 0
    foodpick_counts = []
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

#        if len(self.food_poses) > 0 and self.current_food_pose < len(self.food_poses):  # check if we have more than one food poses, and if the current one is not the last one
#            if self.total_food >= self.food_poses[self.current_food_pose][0]:
#                rospy.loginfo("Changing food pose after %d runs to %s", self.total_food, str(self.food_poses[self.current_food_pose][1]))
#                self.food_pose_pub.publish(self.food_poses[self.current_food_pose][1])
#                self.current_food_pose += 1
#                try:
#                    os.system("sh beep.sh")
#                except:
#                    pass
        if self.total_food >= self.total_food_runs:
            self.run_completed = True

    def foodpick_cb(self, msg, idx):
        self.foodpick_counts[idx] = msg.data
        self.totalpick_food = sum(self.foodpick_counts)
        if self.totalpick_food % FOOD_PRINT == 0:
            rospy.loginfo("Foodpicks %d of %d", self.totalpick_food, self.total_food_runs)

        if len(self.food_poses) > 0 and self.current_food_pose < len(self.food_poses):  # check if we have more than one food poses, and if the current one is not the last one
            if self.totalpick_food >= self.food_poses[self.current_food_pose][0]:
                rospy.loginfo("Changing food pose after %d runs to %s", self.totalpick_food, str(self.food_poses[self.current_food_pose][1]))
                self.food_pose_pub.publish(self.food_poses[self.current_food_pose][1])
                self.current_food_pose += 1



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
#            prev_msg_angular_z = 
#            if msg.linear.x == 0 and (msg.angular.z == prev_msg_angular_z):
#            if msg.linear.x == 0:
                self.cmd_zero_counts[idx] += 1
                if self.cmd_zero_counts[idx] > MAX_ZERO_CMD:
                    self.restart_run = True
            else:
                self.cmd_zero_counts[idx] = 0

    def init_for_num(self, num_robots):
        self.run_completed = False
        self.restart_run = False

        self.total_food = 0
        self.totalpick_food = 0
        self.current_food_pose = 0
        self.food_counts = [0] * (num_robots+1)
        self.foodpick_counts = [0] * (num_robots+1)
        self.stalled = [0] * (num_robots+1)
        self.cmd_zero_counts = [0] * (num_robots+1)
        self.last_cmd_vels = [rospy.Time(0)] * (num_robots)
        self.update_listeners(num_robots)

    def update_listeners(self, num_robots):
        for l in self.food_listeners:
            assert isinstance(l, rospy.Subscriber)
            l.unregister()

        for l in self.foodpick_listeners:
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
        self.foodpick_listeners = []
        self.stall_listeners = []

        for i in range(1, num_robots+1):
            self.foodpick_listeners.append(rospy.Subscriber("/robot_%d/foodpicks" % i, Int32, self.foodpick_cb, i))
            self.food_listeners.append(rospy.Subscriber("/robot_%d/fooddrops" % i, Int32, self.food_cb, i))
            self.stall_listeners.append(rospy.Subscriber("/robot_%d/stall" % i, Stall, self.stall_cb, i))
            self.cmd_listeners.append(rospy.Subscriber("/robot_%d/cmd_vel" % i, Twist, self.cmd_cb, i))

    def start_experiment(self, launchfile,nameext, num_robots):
        self.init_for_num(num_robots)
        self.launch_process = start_environment(launchfile)
        for i in range(1, num_robots + 1):
            try:
                rospy.wait_for_service("/robot_%d/start_smach" % i, START_WAIT_SVC)
            except Exception as e:
                rospy.logwarn("Robot not started after %s seconds, restarting, %s", START_WAIT_SVC, e)
                self.restart_run = True
                return
        time.sleep(START_WAIT)
#        try:
#            os.system("sh beep.sh")
#        except:
#            pass
        rosbag_file = os.path.join(self.path, launchfile + nameext + "_run" + str(self.run) + ".bag")
        rospy.loginfo("rosbag filename: %s", rosbag_file)

        self.rosbag_process = Popen(["rosbag", "record", "/logging", "-O", rosbag_file])

        self.start_time = rospy.Time.now()
    
        # move the initial food position with configuration parameter (0 fooddrops)
        if len(self.food_poses) > 0 and self.current_food_pose < len(self.food_poses):  # check if we have more than one food poses, and if the current one is not the last one
            if self.totalpick_food >= self.food_poses[self.current_food_pose][0]:
                rospy.loginfo("Changing food pose after %d runs to %s", self.totalpick_food, str(self.food_poses[self.current_food_pose][1]))
                self.food_pose_pub.publish(self.food_poses[self.current_food_pose][1])
                self.current_food_pose += 1
                
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

    def run_experiment(self, launchfile, nameext, num_robots):
        self.start_experiment(launchfile, nameext, num_robots)
        while not self.run_completed and not rospy.is_shutdown():
            if self.restart_run:
                rospy.logwarn("Restart required for run %d, launchfile %s%s", self.run, launchfile, nameext)
                self.stop_experiment(SHUTDOWN_TIME)
                self.start_experiment(launchfile, nameext, num_robots)
            if (rospy.Time.now() - self.start_time).to_sec() > self.time_limit:
                rospy.logwarn("Timelimit exceeded for run %d, launchfile %s%s", self.run, launchfile,nameext)
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
        for launch_file, name_ext, repetitions, num_food_runs, num_robots, time_limit, next_food_poses in configs:

            self.total_food_runs = num_food_runs
            self.time_limit = time_limit
            self.food_poses = next_food_poses
            for i in range(1, repetitions+1):
                self.run = i
                if rospy.is_shutdown():
                    return
                rospy.loginfo("Starting run %d of %d, for launchfile %s%s, num_robots %d", self.run, repetitions, launch_file, name_ext, num_robots)
                self.run_experiment(launch_file, name_ext, num_robots)


if __name__ == '__main__':
    rospy.init_node('run_experiments')
    runner = RunExperiments()
    runner.run_all_experiments()

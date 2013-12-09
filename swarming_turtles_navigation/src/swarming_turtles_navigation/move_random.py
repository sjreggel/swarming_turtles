#!/usr/bin/env python
import rospy
import random

from geometry_msgs.msg import Twist, PoseStamped, Vector3, Quaternion
from kobuki_msgs.msg import SensorState
from sensor_msgs.msg import LaserScan

from swarming_turtles_navigation.srv import GetCollvoidTwist

LEFT = 0
CENTER = 1
RIGHT = 2

ROTATE_RIGHT = -1
ROTATE_LEFT = 1

#config
ROTATION_SPEED = 1.3
FORWARD_SPEED = 0.3


get_twist_srv = None
cmd_pub = None
cur_goal = None

MIN_DIST_LASER = 0.5
EPS_ALIGN_THETA = 0.2 #alignment precision
EPS_ALIGN_XY = 0.1 #alignment precision

active = False
RATE = 20
base_frame = '/base_link'
hive = '/hive'

min_dist_laser = 2*MIN_DIST_LASER

count_low_speed = 0

def init_globals():
    global get_twist, cmd_pub
    cmd_pub = rospy.Publisher('cmd_vel_mux/input/navi', Twist) #publish Twist
    get_twist_srv = rospy.ServiceProxy('get_collvoid_twist', GetCollvoidTwist)
    rospy.Subscriber('/mobile_base/sensors/core', SensorState, cb_sensors) #for bumpers
    rospy.Subscriber('/scan_obst', LaserScan, cb_laser_scan) #for min dist

    rospy.Service('move_random_start', Empty, move_random_start)
    rospy.Service('move_random_stop', Empty, move_random_stop)

    
def cb_laser_scan(msg):
    global min_dist_laser
    min_dist_laser_tmp = msg.range_max
    #todo maybe limit range to front view only?

    for i in msg.ranges:
        if i < min_dist_laser_tmp:
            min_dist_laser_tmp = i
    min_dist_laser = min_dist_laser_tmp

def cb_sensors(msg):
    global bumpers
    bumpers[LEFT] = msg.bumper >> 2 & 1
    bumpers[CENTER] = msg.bumper >> 1 & 1
    bumpers[RIGHT] = msg.bumper & 1
    

def obstacle_left():
    return bumpers[LEFT]

def obstacle_right():
    return bumpers[RIGHT]

def obstacle_front():
    return bumpers[CENTER] or min_dist_laser < MIN_DIST_LASER

def obstacle():
    return obstacle_left() or obstacle_right or obstacle_front()

def stop():
    for i in xrange(3):
        cmd_pub.publish(Twist())


#helpers
def transformPose(pose_in, time_in = None, frame = hive):
    if tfListen.frameExists(pose_in.header.frame_id) and tfListen.frameExists(frame):
        time = tfListen.getLatestCommonTime(odom, pose_in.header.frame_id)
        if not time_in:
            pose_in.header.stamp = time
        else:
            pose_in.header.stamp = time_in
        pose = tfListen.transformPose(frame, pose_in)
        return pose
    return None


def get_own_pose():
    pose_stamped = PoseStamped()
    pose_stamped.header.stamp = rospy.Time.now()
    pose_stamped.header.frame_id = base_frame
    pose_stamped.pose.orientation.w = 1.0
    return transformPose(pose_stamped)

    
def quat_msg_to_array(quat):
    return [quat.x, quat.y, quat.z, quat.w]

def get_jaw(orientation):
    quat = quat_msg_to_array(orientation)
    r,p,theta = tf.transformations.euler_from_quaternion(quat)
    return theta

        
def rotate_to_ang(ang):
    twist = Twist()
    twist.linear.x = 0
    twist.angular.z = rotate_side(ang) * ROTATION_SPEED
    cmd_pub.publish(twist)
            
def rotate_side(ang):
    own_pose = get_own_pose()
    theta = get_jaw(own_pose.pose.orientation)
    res = ang - theta
    if res > math.pi:
        res -= 2 * math.pi
    if res < -math.pi:
        res += 2* math.pi
    if (res) < 0:
        return ROTATE_RIGHT
    else:
        return ROTATE_LEFT

def rotation_aligned(ang):
    own_pose = get_own_pose()
    theta = get_jaw(own_pose.pose.orientation)
    return  abs(ang - theta) < EPS_ALIGN_THETA
        
def get_random_walk():
    dist = random.random() * 2.5
    ang = random.random() * 2. * math.pi 
    return dist, ang


def create_goal(dist):
    global cur_goal
    req = GetCollvoidTwistRequest()
    goal = PoseStamped()
    goal.header.stamp = rospy.Time.now()
    goal.header.frame_id = base_frame
    goal.pose.position.x = dist
    goal.pose.orientation.w = 1.0

    req.goal = transformPose(goal)

    cur_goal = req

def dist_vec(a, b):
    d = diff_vec(a,b)
    return math.sqrt(d.x * d.x + d.y * d.y)

def diff_vec(a,b):
    res = Vector3()
    res.x = b.x - a.x
    res.y = b.y - a.y
    return res

    
def at_goal():
    own_pose = get_own_pose()
    goal = cur_goal.goal
    dist = dist_vec(own_pose.pose.position, goal.pose.position)
    ang_diff = abs(get_jaw(own_pose.pose.orientation) - get_jaw(goal.pose.orientation))
    return dist < EPS_ALIGN_XY and ang_diff < EPS_ALIGN_THETA
    

def reset_counts():
    global count_low_speed
    count_low_speed = 0

def move_random():
    reset_counts()

    dist, ang = get_random_walk()

    jaw = get_jaw(get_own_pose().pose.orientation)

    print dist, ang, jaw
    
    while active and not rotation_aligned(ang):
        rotate_to_ang(ang)

    create_goal(dist)
    r = rospy.Rate(RATE)

    while active and not at_goal():
        twist = get_twist()

        if obstacle():
            twist = Twist()
            while obstacle():
                if obstacle_left():
                    twist.angular.z = ROTATE_RIGHT * ROTATION_SPEED
                else:
                    twist.angular.z = ROTATE_LEFT * ROTATION_SPEED
                cmd_pub.publish(twist)
                r.sleep()
            return
        cmd_pub.publish(twist)
        r.sleep()
        
def get_twist():
    try:
        twist = get_twist_srv(cur_goal)
    except:
        print "get twist call failed"
        twist = Twist()
    return twist
        
        
def move_random_start(req):
    global active
    active = True

def move_random_stop(req):
    global active
    active = False

    
def main():
    rospy.init_node("move_random")
    init_globals()
    r = rospy.Rate(RATE)

    while not rospy.is_shutdown():
        if active:
            move_random()
        r.sleep()
    

if __name__ == "__main__":
    main()

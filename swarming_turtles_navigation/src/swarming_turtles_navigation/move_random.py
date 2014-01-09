#!/usr/bin/env python
import rospy
import random
import math
import tf
import actionlib
from actionlib_msgs.msg import *


from geometry_msgs.msg import Twist, PoseStamped, Vector3, Quaternion
from kobuki_msgs.msg import SensorState
from sensor_msgs.msg import LaserScan
from move_base_msgs.msg import *

from std_srvs.srv import *
from swarming_turtles_navigation.srv import *

bumpers = [0,0,0]
LEFT = 0
CENTER = 1
RIGHT = 2

ROTATE_RIGHT = -1
ROTATE_LEFT = 1

tfListen = None

#config
ROTATION_SPEED = 1.5



get_twist_srv = None
cmd_pub = None
goal_pub = None
cur_goal = None

action_server = None

MIN_DIST_LASER = 0.5
EPS_ALIGN_THETA = 0.3 #alignment precision
EPS_ALIGN_XY = 0.5 #alignment precision

STAND_STILL_XY = 0.02
STAND_STILL_THETA = 0.02

active = False
RATE = 10
base_frame = '/base_link'
hive = '/hive'

min_dist_laser = 2*MIN_DIST_LASER

count_low_speed = 0
MAX_COUNT = 5
EPS_SPEED = 0.1

obstacle_front_bool = False

MIN_BELOW_MAX = 20


def init_globals():
    global tfListen, cmd_pub
    tfListen = tf.TransformListener()
    rospy.sleep(1)
    cmd_pub = rospy.Publisher('cmd_vel_mux/input/navi', Twist) #publish Twist

    
def cb_laser_scan(msg):
    global min_dist_laser, obstacle_front_bool
    min_dist_laser_tmp = msg.range_max
    #todo maybe limit range to front view only?

    count_below_max = 0
    for i in msg.ranges:
        
        if i < min_dist_laser_tmp:
            min_dist_laser_tmp = i
        if i < msg.range_max:
            count_below_max += 1

    obstacle_front_bool = count_below_max < MIN_BELOW_MAX
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
    return bumpers[CENTER] or min_dist_laser < MIN_DIST_LASER or obstacle_front_bool

def obstacle():
    return obstacle_left() or obstacle_right() or obstacle_front()

def stop():
    for i in xrange(3):
        cmd_pub.publish(Twist())


#helpers
def transformPose(pose_in, time_in = None, frame = hive):
    if tfListen.frameExists(pose_in.header.frame_id) and tfListen.frameExists(frame):
        time = tfListen.getLatestCommonTime(frame, pose_in.header.frame_id)
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

def rotation_aligned(ang, eps = None):
    own_pose = get_own_pose()
    theta = get_jaw(own_pose.pose.orientation)

    
    diff = abs(ang-theta)

    if diff > math.pi:
        diff -= 2*math.pi

    #print diff
    if eps is None:
        return  abs(diff) < EPS_ALIGN_THETA
    else:
        return  abs(diff) < eps
    
def get_random_walk():
    dist = random.random() * 2.5
    ang = random.random() * 2. * math.pi 
    return dist, ang


def create_goal_from_pose(pose):
    global cur_goal
    req = GetCollvoidTwistRequest()
    print "creating goal from target"
    pose.header.stamp = rospy.Time.now()
    req.goal = transformPose(pose)
    cur_goal = req
    goal_pub.publish(req.goal)

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
    goal_pub.publish(req.goal)

    
def dist_vec(a, b):
    d = diff_vec(a,b)
    return math.sqrt(d.x * d.x + d.y * d.y)

def diff_vec(a,b):
    res = Vector3()
    res.x = b.x - a.x
    res.y = b.y - a.y
    return res

def rotate_vec_by_angle(v, ang):
    res = Vector3()
    cos_a = math.cos(ang)
    sin_a = math.sin(ang)
    res.x = cos_a * v.x - sin_a * v.y
    res.y = cos_a * v.y + sin_a * v.x
    return res

def standing_still(old_pose):
    own_pose = get_own_pose()
    dist = dist_vec(own_pose.pose.position, old_pose.pose.position)
    ang = abs(get_jaw(own_pose.pose.orientation) - get_jaw(old_pose.pose.orientation))
    return dist < STAND_STILL_XY and ang < STAND_STILL_THETA

def dist_aligned():
    own_pose = get_own_pose()
    goal = cur_goal.goal
    dist = dist_vec(own_pose.pose.position, goal.pose.position)
    return dist < EPS_ALIGN_XY

def at_goal():
    #own_pose = get_own_pose()
    goal = cur_goal.goal
    ang = get_jaw(goal.pose.orientation)

    return dist_aligned() and rotation_aligned(ang)
    

def reset_counts():
    global count_low_speed
    count_low_speed = 0

    
def move_random():
    global count_low_speed
    reset_counts()

    dist, ang = get_random_walk()

    jaw = get_jaw(get_own_pose().pose.orientation)

    #print dist, ang, jaw

    r = rospy.Rate(RATE)

    while active and not rotation_aligned(ang):
        rotate_to_ang(ang)
        r.sleep()
    stop()
    create_goal(dist)

    while active and not at_goal() and not count_low_speed > MAX_COUNT:
        if obstacle():
            twist = Twist()
            while active and obstacle():
                #print "obstacle"
                if obstacle_left():
                    twist.angular.z = ROTATE_RIGHT * ROTATION_SPEED
                else:
                    twist.angular.z = ROTATE_LEFT * ROTATION_SPEED
                cmd_pub.publish(twist)
                r.sleep()

            dist, ang = get_random_walk()
            create_goal(dist)
        twist = get_twist()
        cmd_pub.publish(twist)
        if twist.linear.x < EPS_SPEED and abs(twist.angular.z) < EPS_SPEED:
            count_low_speed +=1
        
        r.sleep()

def move_to_goal_cb(goal):
    global action_server
    count_low_speed = 0
    create_goal_from_pose(goal.target_pose)
    r = rospy.Rate(RATE)

    while not dist_aligned():
        if count_low_speed > MAX_COUNT or action_server.is_preempt_requested():
            if action_server.is_new_goal_available():
                goal = action_server.accept_new_goal()
                create_goal_from_pose(goal.target_pose)
            else:
                action_server.set_preempted()
                #action_server.set_aborted()
                stop()
                return
            
        if sum(bumpers)>0:
            twist = Twist()
            while sum(bumpers)>0:
                #print "obstacle"
                if obstacle_left():
                    twist.angular.z = ROTATE_RIGHT * ROTATION_SPEED
                else:
                    twist.angular.z = ROTATE_LEFT * ROTATION_SPEED
                cmd_pub.publish(twist)
                r.sleep()
            create_goal_from_pose(goal.target_pose)

        twist = get_twist()
        if abs(abs(twist.angular.z) - 1.3) < 0.01:
            ang = get_jaw(goal.target_pose)
            rotate_to_ang(ang)
        else:
            cmd_pub.publish(twist)
        if twist.linear.x < EPS_SPEED and abs(twist.angular.z) < EPS_SPEED:
            count_low_speed +=1
        r.sleep()


    action_server.set_succeeded()


        
def get_twist():
    global get_twist_srv
    try:
        res = get_twist_srv(cur_goal)
        #print res
        return res.twist
    except rospy.ServiceException as exc:
        print("Service did not process request: " + str(exc))
        print "get twist call failed"
        get_twist_srv.close()
        print 'reconnecting'
        rospy.wait_for_service('SwarmCollvoid/get_collvoid_twist')
        get_twist_srv = rospy.ServiceProxy('SwarmCollvoid/get_collvoid_twist', GetCollvoidTwist, persistent = True)
        print 'done'
        twist = Twist()
    return twist
        
        
def move_random_start(req):
    global active
    active = True
    return EmptyResponse()

def move_random_stop(req):
    global active
    active = False
    return EmptyResponse()

    



def move_location(pose, x = 0, y = 0):
    pose_stamped = PoseStamped()
    pose_stamped.header.frame_id = pose.header.frame_id
    ang = get_jaw(pose.pose.orientation)
    vec = Vector3()
    vec.x = x
    vec.y = y
    vec = rotate_vec_by_angle(vec, ang)
    
    pose_stamped.pose.position.x = pose.pose.position.x + vec.x
    pose_stamped.pose.position.y = pose.pose.position.y + vec.y
    pose_stamped.pose.orientation = pose.pose.orientation
        
    return pose_stamped
    
def move_location_inwards(pose, dist):
    pose_stamped = PoseStamped()
    pose_stamped.header.frame_id = pose.header.frame_id
    ang = get_jaw(pose.pose.orientation)
    vec = Vector3()
    vec.x = dist
    #vec.y = -Y_OFFSET
    vec = rotate_vec_by_angle(vec, ang-math.pi/2.0)

    pose_stamped.pose.position.x = pose.pose.position.x + vec.x
    pose_stamped.pose.position.y = pose.pose.position.y + vec.y

    q = tf.transformations.quaternion_from_euler(0,0,ang+math.pi/2.0,axes = 'sxyz')
   
    pose_stamped.pose.orientation = Quaternion(*q)
        
    return pose_stamped

def create_goal_message(goal):
    goal_msg = MoveBaseGoal()
    goal_msg.target_pose.pose = goal.pose
    goal_msg.target_pose.header.frame_id = hive
    goal_msg.target_pose.header.stamp = rospy.Time.now()
    return goal_msg

        
        
def main():
    global action_server, get_twist_srv, goal_pub
    rospy.init_node("move_random")
    init_globals()

    rospy.Subscriber('/mobile_base/sensors/core', SensorState, cb_sensors) #for bumpers
    rospy.Subscriber('/scan_obst', LaserScan, cb_laser_scan) #for min dist

    rospy.Service('move_random_start', Empty, move_random_start)
    rospy.Service('move_random_stop', Empty, move_random_stop)

    goal_pub = rospy.Publisher('cur_goal', PoseStamped)
    get_twist_srv = rospy.ServiceProxy('SwarmCollvoid/get_collvoid_twist', GetCollvoidTwist, persistent = True)
    rospy.loginfo("wait for service")
    rospy.wait_for_service('SwarmCollvoid/get_collvoid_twist')
    rospy.loginfo("done")

    
    r = rospy.Rate(RATE)
    action_server = actionlib.simple_action_server.SimpleActionServer('move_to_goal', MoveBaseAction, move_to_goal_cb, False)
    action_server.start()

    

    while not rospy.is_shutdown():
        if active:
            move_random()
        r.sleep()
        
    get_twist_srv.close()
    
if __name__ == "__main__":
    main()

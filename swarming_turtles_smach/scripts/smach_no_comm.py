#!/usr/bin/env python

#import roslib; roslib.load_manifest('swarming_turtles_smach')
import rospy
import smach
import smach_ros
import tf
import math
import actionlib
import copy
from ar_track_alvar.msg import AlvarMarkers
from geometry_msgs.msg import Twist, PoseStamped, Vector3, Quaternion
from kobuki_msgs.msg import SensorState
from actionlib_msgs.msg import *
from move_base_msgs.msg import *


locations = {}
markers = {}

bumper = False
tfListen = None
cmd_pub = None

hive_pub = None
food_pub = None

#config
ROTATION_SPEED = 1
FORWARD_SPEED = 0.3
SEARCH_TIMEOUT = 10
DIST_OFFSET = 0.2

EPS_TARGETS = 0.1 #if targets are further away than that resend goal

INWARDS = 0.4

RATE = 30
EPS = 0.1

odom = "/odom"
base_frame = "/base_link"


found = ''


def init_globals():
    global markers, tfListen, cmd_pub, hive_pub, food_pub
    markers['food'] = [201, 202]
    markers['hive'] = [200, 199]
    tfListen = tf.TransformListener()
    rospy.sleep(1)

    cmd_pub = rospy.Publisher('cmd_vel_mux/input/navi', Twist)

    food_pub = rospy.Publisher('cur_food', PoseStamped)

    hive_pub = rospy.Publisher('cur_hive', PoseStamped)

    rospy.Subscriber('ar_pose_marker', AlvarMarkers, cb_ar_marker)
    rospy.Subscriber('/mobile_base/sensors/core', SensorState, cb_sensors)


def rotate_vec_by_angle(v, ang):
    res = Vector3()
    cos_a = math.cos(ang)
    sin_a = math.sin(ang)
    
    res.x = cos_a * v.x - sin_a * v.y
    res.y = cos_a * v.y + sin_a * v.x
    return res


def dist_vec(a, b):
    d = diff_vec(a,b)
    return math.sqrt(d.x * d.x + d.y * d.y)

def diff_vec(a,b):
    res = Vector3()
    res.x = b.x - a.x
    res.y = b.y - a.y
    return res

def cb_sensors(msg):
    global bumper
    if msg.bumper > 0:
        bumper = True
    else:
        bumper = False
    

def cb_ar_marker(msg):
    global found
    for marker in msg.markers: #check all markers
        for loc in markers.keys(): #check all locations
            if marker.id in markers[loc]: #marker id in marker list
                found = loc
                update_location(loc, marker)
                return

def create_goal_message(goal):
    goal_msg = MoveBaseGoal()
    goal_msg.target_pose.pose = goal.pose
  
    goal_msg.target_pose.header.frame_id = odom
    goal_msg.target_pose.header.stamp = rospy.Time.now()
    return goal_msg

            
def stop():
    for i in xrange(3):
        cmd_pub.publish(Twist())

        
def move_random():
        #todo move forward turn on bumper detect
    twist = Twist()
    if bumper or True:
        twist.linear.x = 0
        twist.angular.z = ROTATION_SPEED
    else:
       twist.linear.x = FORWARD_SPEED
       twist.angular.z = 0
        
    
    cmd_pub.publish(twist)


def update_location(loc, msg):
    global locations
    if not loc in locations:
        locations[loc] = {}
    locations[loc]['frame'] = msg.header.frame_id
    locations[loc]['pose'] = move_location_inwards(msg.pose, INWARDS)
    locations[loc]['time'] = msg.header.stamp
    
    pose = locations[loc]['pose']
    pose.header.frame_id = locations[loc]['frame']
    pose.header.stamp = locations[loc]['time']

    if loc == 'hive':
        hive_pub.publish(pose)
    else:
        food_pub.publish(pose)
    #print locations[loc]

def quat_msg_to_array(quat):
    return [quat.x, quat.y, quat.z, quat.w]

def get_jaw(orientation):
    quat = quat_msg_to_array(orientation)
    r,p,theta = tf.transformations.euler_from_quaternion(quat)
    return theta

    
def move_location_inwards(pose, dist):
    pose_stamped = PoseStamped()
    pose_stamped.header.frame_id = pose.header.frame_id
    ang = get_jaw(pose.pose.orientation)
    vec = Vector3()
    vec.x = dist
    vec = rotate_vec_by_angle(vec, ang-math.pi/2.0)

    pose_stamped.pose.position.x = pose.pose.position.x + vec.x
    pose_stamped.pose.position.y = pose.pose.position.y + vec.y

    q = tf.transformations.quaternion_from_euler(0,0,ang+math.pi/2.0,axes = 'sxyz')
   
    pose_stamped.pose.orientation = Quaternion(*q)
        
    return pose_stamped
    

def transformPose(pose_in):
    if tfListen.frameExists(pose_in.header.frame_id) and tfListen.frameExists(odom):
        time = tfListen.getLatestCommonTime(odom, pose_in.header.frame_id)
        pose_in.header.stamp = time
        pose = tfListen.transformPose(odom, pose_in)
        return pose
    return None


def get_own_pose():
    pose_stamped = PoseStamped()
    pose_stamped.header.stamp = rospy.Time.now()
    pose_stamped.header.frame_id = base_frame
    pose_stamped.pose.orientation.w = 1.0
    
    return transformPose(pose_stamped)


class Explore(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['found_hive', 'found_food'])
        self.locs = ['food', 'hive']

    def execute(self, userdata):
        global found
        found = ''

        rate = rospy.Rate(RATE)
        while found=='':
            move_random()
            rate.sleep()
        stop()
        if found == 'hive':
            return 'found_hive'
        else:
            return 'found_food'
    

class SearchLocations(smach.State):
    def __init__(self, looking_for):
        smach.State.__init__(self, outcomes=['found', 'not_found'])
        self.loc = looking_for

    def execute(self, userdata):
        global found
        found = ''

        start = rospy.Time.now()
        rate = rospy.Rate(RATE)
        while not found in self.loc:
            if (rospy.Time.now()-start).to_sec() > SEARCH_TIMEOUT:
                stop()
                return 'not_found'
            move_random()
            rate.sleep()
        stop()
        return 'found'

class PreSearchLocation(smach.State):
    def __init__(self, loc):
        smach.State.__init__(self, outcomes=['known', 'not_known'])
        self.loc = loc
        
    def execute(self, userdata):
        if self.loc in locations.keys():
            return 'known'
        else:
            return 'not_known'
    

class MoveToLocation(smach.State):
    def __init__(self, loc):
        smach.State.__init__(self, outcomes=['failed', 'success'])
        self.loc = loc
        self.client = actionlib.SimpleActionClient('SwarmCollvoid/swarm_nav_goal', MoveBaseAction)
        self.client.wait_for_server()


    def rotate_to_ang(self, ang):
        twist = Twist()
        twist.linear.x = 0
        twist.angular.z = self.rotate_side(ang) * ROTATION_SPEED

        rate = rospy.Rate(RATE)
        while not self.rotation_aligned(ang):
            cmd_pub.publish(twist)
        stop()


        
    def rotate_side(self, ang):
        own_pose = get_own_pose(pose_stamped)
        quat = [own_pose.pose.orientation.x, own_pose.pose.orientation.y, own_pose.pose.orientation.z,own_pose.pose.orientation.w]
        r,p,theta = tf.transformations.euler_from_quaternion(quat)
        if (ang - theta) < 0:
            return -1
        else:
            return 1

    def rotation_aligned(self, ang):
        own_pose = get_own_pose(pose_stamped)
        quat = [own_pose.pose.orientation.x, own_pose.pose.orientation.y, own_pose.pose.orientation.z,own_pose.pose.orientation.w]
        r,p,theta = tf.transformations.euler_from_quaternion(quat)
        return  abs(ang - theta) < EPS

       
    def drive_to_straight(self, dist):
        twist = Twist()
        twist.linear.x = FORWARD_SPEED
        twist.angular.z = 0
        rate = rospy.Rate(RATE)
        start_pose = get_own_pose(pose_stamped)
        while not self.dist_achieved(start_pose, dist):
            cmd_pub.publish(twist)
        stop()

    def dist_achieved(self,start_pose, dist):
        own_pose = get_own_pose(pose_stamped)
        diff_x =  start_pose.pose.position.x - own_pose.pose.position.x
        diff_y = start_pose.pose.position.y - own_pose.pose.position.y
        dist_cur = math.sqrt(diff_y*diff_y + diff_x*diff_x)
        return abs(dist_cur - dist) < EPS


    
    def execute(self, userdata):

        #return 'success'
        own_pose = get_own_pose()

        target = copy.deepcopy(locations[self.loc]['pose'])

        #print target
        
        #diff_x = target.pose.position.x - own_pose.pose.position.x
        #diff_y = target.pose.position.y - own_pose.pose.position.y

        #ang = math.atan2(diff_y, diff_x)
        
        #self.rotate_to_goal(ang)
        #self.drive_to_goal(dist - DIST_OFFSET)
        goal = create_goal_message(target)

        self.client.send_goal(goal)

        rate = rospy.Rate(RATE)
        
        while True:
            if (dist_vec(locations[self.loc]['pose'].pose.position, target.pose.position) > EPS_TARGETS):

                print "resending goal"
                self.client.cancel_all_goals()
                target = copy.deepcopy(locations[self.loc]['pose'])
                goal = create_goal_message(target)
                self.client.send_goal(goal)
                
            if self.client.get_state() == GoalStatus.SUCCEEDED:
                self.client.cancel_all_goals()
                stop()
                return 'success'
            if self.client.get_state() == GoalStatus.PREEMPTED:
                self.client.cancel_all_goals()
                stop()
                return 'failed'
            rate.sleep()
    
def main():
    rospy.init_node('swarming_turtles_machine')
    init_globals()
    # create a SMACH state machine
    sm = smach.StateMachine(outcomes=['end'])

    with sm:

        #smach.StateMachine.add("Explore", Explore(), transitions = {'found_hive':'end', 'found_food':'GoToFood'})

        #smach.StateMachine.add("Explore", Explore(), transitions = {'found_hive':'Explore', 'found_food':'Explore'})

        
        smach.StateMachine.add("Explore", Explore(), transitions = {'found_hive':'SearchFood', 'found_food':'SearchHive'})

        #Hive states
        
        smach.StateMachine.add("PreSearchHive", PreSearchLocation('hive'), transitions = {'known':'GoToHive', 'not_known':'SearchHive'})
        smach.StateMachine.add("SearchHive", SearchLocations(['hive']), transitions = {'found':'PreSearchFood', 'not_found':'SearchHive'})
        smach.StateMachine.add("GoToHive", MoveToLocation('hive'), transitions = {'failed':'SearchHive', 'success':'PreSearchFood'})

        # #food states
        smach.StateMachine.add("PreSearchFood", PreSearchLocation('food'), transitions = {'known':'GoToFood', 'not_known':'SearchFood'})
        smach.StateMachine.add("SearchFood", SearchLocations(['food']), transitions = {'found':'PreSearchHive', 'not_found':'SearchFood'})
        smach.StateMachine.add("GoToFood", MoveToLocation('food'), transitions = {'failed':'SearchFood', 'success':'PreSearchHive'})

        #smach.StateMachine.add("GoToFood", MoveToLocation('food'), transitions = {'failed':'end', 'success':'end'})
       
        
    # Create and start the introspection server
    sis = smach_ros.IntrospectionServer('server_name', sm, '/swarming_turtles')
    sis.start()
    rospy.loginfo("starting!")

    # Execute SMACH plan
    outcome = sm.execute()

    rospy.spin()
    sis.stop()

    
if __name__ == '__main__':
    main()

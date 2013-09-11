#!/usr/bin/env python

#import roslib; roslib.load_manifest('swarming_turtles_smach')

import rospy
import smach
import smach_ros
import tf
import math
import random
import actionlib
import copy
import thread
import subprocess
from socket import gethostname

import rosgraph

from ar_track_alvar.msg import AlvarMarkers
from geometry_msgs.msg import Twist, PoseStamped, Vector3, Quaternion
from kobuki_msgs.msg import SensorState
from actionlib_msgs.msg import *
from move_base_msgs.msg import *

from swarming_turtles_msgs.msg import Turtles, Turtle, CommunicationProtocol
from swarming_turtles_communicate.communicate  import make_master_uri

from swarming_turtles_communicate.master_sync import MasterSync, check_master

#confirm Location

locations = {}

locations_received = {} #TODO: distinguish between locations seen and reveived

received_msg = None


markers = {}

LEFT = 0
CENTER = 1
RIGHT = 2

bumpers = [False] * 3
tfListen = None
cmd_pub = None

name = ''
topic = '/communication'
topic_in = '/communication_in'

topic_out = '/communication_out'

hive_pub = None
food_pub = None
comm_pub = None

#config
ROTATION_SPEED = 2.0
FORWARD_SPEED = 0.3
SEARCH_TIMEOUT = 15

LAST_SEEN = 1.0 #check last seen for other turtle

EPS_TARGETS = 0.1 #if targets are further away than that resend goal

INWARDS = 0.4 #move loc xx meters inwards from detected marker locations
Y_OFFSET = 0.3

MIN_DIST = 1.5 #how close to include in asking?

LAST_USED = 20.0 #how long vefore closing connection

MAX_RETRY = 3

RATE = 10
EPS = 0.1

odom = "/odom"
base_frame = "/base_link"


found = ''
closest = ''
received = ''
sending = False

turtles = {}

open_cons = {}

def init_globals():
    global markers, tfListen, cmd_pub, hive_pub, comm_pub, food_pub, name
    markers['food'] = [201, 202]
    markers['hive'] = [200, 199]
    tfListen = tf.TransformListener()

    name = gethostname()
    rospy.sleep(1)

    comm_pub = rospy.Publisher(topic_out, CommunicationProtocol)

    cmd_pub = rospy.Publisher('cmd_vel_mux/input/navi', Twist)

    food_pub = rospy.Publisher('cur_food', PoseStamped)

    hive_pub = rospy.Publisher('cur_hive', PoseStamped)

    rospy.Subscriber('ar_pose_marker', AlvarMarkers, cb_ar_marker)
    rospy.Subscriber('/mobile_base/sensors/core', SensorState, cb_sensors)
    rospy.Subscriber('/found_turtles', Turtles, cb_found_turtles)
    
    rospy.Subscriber(topic, CommunicationProtocol, cb_communication)

    cmd = ["rosrun", "foreign_relay", "unreliable_relay", topic_in, topic]
    #print cmd
    subprocess.Popen(cmd, stdout=subprocess.PIPE)
    thread.start_new_thread(check_open_connections, ())

def connect(foreign_master_uri):
    #master_uri = make_master_uri(foreign_master)
    m = rosgraph.Master(rospy.get_name(), master_uri=foreign_master_uri)
    if not check_master(m):
        return False
    
    cmd = ["rosrun", "foreign_relay", "foreign_relay", "adv", foreign_master_uri, topic_in, topic_out]

    relay = subprocess.Popen(cmd)
    
    global open_cons
    if not foreign_master_uri in open_cons.keys():
        open_cons[foreign_master_uri] = {}
    open_cons[foreign_master_uri]['process'] = relay
    return True
    
def disconnect(foreign_master_uri):
    global open_cons
    #print "closing connection", foreign_master_uri
       
    if foreign_master_uri in open_cons.keys():
        print "closing connection", foreign_master_uri
        open_cons[foreign_master_uri]['process'].kill()
        open_cons[foreign_master_uri]['process'].wait()

        open_cons.pop(foreign_master_uri, None)
    
def check_open_connections():
    global open_cons
    r = rospy.Rate(10)
    while True:
        dict_copy = dict(open_cons)
        keys = open_cons.keys()
        for con in keys:
            if (rospy.Time.now() - open_cons[con]['last_used']).to_sec() > LAST_USED:
                try:
                    disconnect(con)
                except Exception as e:
                    print e
        r.sleep()

def create_pose_msg_from_received(loc, pose_in):
    pose_dict = dict(locations[loc])
    pose = pose_dict['pose']
    pose.header.frame_id = pose_dict['frame']
    pose = transform_to_baseframe(pose) #, time_in=pose_in.header.stamp)

    pose.pose.position.x = pose.pose.position.x + pose_in.pose.position.x
    pose.pose.position.y = pose.pose.position.y + pose_in.pose.position.y
    
    yaw = get_jaw(pose.pose.orientation) + get_jaw(pose_in.pose.orientation)
    q = tf.transformations.quaternion_from_euler(0,0,yaw, axes = "sxyz")
    pose.pose.orientation = Quaternion(*q)
    return pose
        
def cb_communication(msg):
    global received, received_msg, location_received
    if not msg.receiver == name:
        return
    req = msg.request.split(' ')
    if "request" == req[0]: #handle reqest
        if req[1] in locations.keys():
            pose = create_pose_msg_from_received(req[1], msg.location)
            answer(msg.sender, req[1], pose)
            
    elif "answer" == req[0]:
        print "GOT ANSWER", req[1]
        received_msg = msg #handle message in search states
        received = req[1]  #say we received something

def answer(receiver, loc_name, pose):
    msg = CommunicationProtocol()
    msg.sender = name
    msg.receiver = receiver
    msg.request = "answer %s"%(loc_name)
    msg.location = pose
    #thread.start_new_thread(send,(receiver, msg))

    send(receiver, msg)
   
def request(receiver, loc_name):
    msg = CommunicationProtocol()
    msg.sender = name
    msg.receiver = receiver
    msg.request = "request %s"%(loc_name)
    msg.location = turtles[receiver]
    #thread.start_new_thread(send,(receiver, msg))
    send(receiver, msg)
    

def send(receiver, msg):
    global open_cons, sending
    sending = True
    foreign_master_uri = make_master_uri(receiver)
    try:
        print open_cons
        if not foreign_master_uri in open_cons.keys():
            if connect(foreign_master_uri):
                print "connected"
                open_cons[foreign_master_uri]['last_used'] = rospy.Time.now()
                rospy.sleep(1.5)
            else:
                print "failed"

        rospy.sleep(1)
        for i in xrange(2):
            comm_pub.publish(msg)
            rospy.sleep(0.2)
        open_cons[foreign_master_uri]['last_used'] = rospy.Time.now()
       
    except Exception as e:
        print "exception", e

    sending = False
        
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
    global bumpers
    
    bumpers[LEFT] = msg.bumper >> 2 & 1
    bumpers[CENTER] = msg.bumper >> 1 & 1
    bumpers[RIGHT] = msg.bumper & 1


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

def get_random_walk():
    dist = random.random() * 1.5
    ang = random.random() * 2. * math.pi 
    return dist, ang




def move_random(client):
        #todo move forward turn on bumper detect
            
    
    #cmd_pub.publish(twist)
        
    if sending:
        client.cancel_all_goals()
        stop()
        while sending:
            rospy.sleep(0.1)

        goal = get_own_pose()
        dist, ang = get_random_walk()
                
        v = Vector3()
        v.x = dist
        vect = rotate_vec_by_angle(v, ang)

        goal.pose.position.x += vect.x
        goal.pose.position.y += vect.y

        goal = create_goal_message(goal)
        client.send_goal(goal)
       
        
    
    if sum(bumpers)>0 or  client.get_state() == GoalStatus.SUCCEEDED or client.get_state == GoalStatus.PREEMPTED:
        client.cancel_all_goals()
        goal = get_own_pose()
        dist, ang = get_random_walk()
        print "random walk dist, ang", dist, ang

        
        v = Vector3()
        v.x = dist
        vect = rotate_vec_by_angle(v, ang)

        goal.pose.position.x += vect.x
        goal.pose.position.y += vect.y

        goal = create_goal_message(goal)

        twist = Twist()
        twist.linear.x = 0
    
        while sum(bumpers) > 0:
            if bumpers[LEFT]:
                twist.angular.z = -ROTATION_SPEED
            else:
                twist.angular.z = ROTATION_SPEED

            if not sending:
                cmd_pub.publish(twist)
            
            rospy.sleep(0.1)
            
        client.send_goal(goal)
        

    

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

def quat_msg_to_array(quat):
    return [quat.x, quat.y, quat.z, quat.w]

def get_jaw(orientation):
    quat = quat_msg_to_array(orientation)
    r,p,theta = tf.transformations.euler_from_quaternion(quat)
    return theta


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
    vec.y = -Y_OFFSET
    vec = rotate_vec_by_angle(vec, ang-math.pi/2.0)

    pose_stamped.pose.position.x = pose.pose.position.x + vec.x
    pose_stamped.pose.position.y = pose.pose.position.y + vec.y

    q = tf.transformations.quaternion_from_euler(0,0,ang+math.pi/2.0,axes = 'sxyz')
   
    pose_stamped.pose.orientation = Quaternion(*q)
        
    return pose_stamped

                   
def transform_to_baseframe(pose_in, time_in = None):
    if tfListen.frameExists(pose_in.header.frame_id) and tfListen.frameExists(base_frame):
        time = tfListen.getLatestCommonTime(base_frame, pose_in.header.frame_id)
        if not time_in:
            pose_in.header.stamp = time
        else:
            pose_in.header.stamp = time_in
        pose = tfListen.transformPose(base_frame, pose_in)
        return pose
    return None

def transformPose(pose_in, time_in = None):
    if tfListen.frameExists(pose_in.header.frame_id) and tfListen.frameExists(odom):
        time = tfListen.getLatestCommonTime(odom, pose_in.header.frame_id)
        if not time_in:
            pose_in.header.stamp = time
        else:
            pose_in.header.stamp = time_in
        pose = tfListen.transformPose(odom, pose_in)
        return pose
    return None


def get_own_pose():
    pose_stamped = PoseStamped()
    pose_stamped.header.stamp = rospy.Time.now()
    pose_stamped.header.frame_id = base_frame
    pose_stamped.pose.orientation.w = 1.0
  
    return transformPose(pose_stamped)



def cb_found_turtles(msg):
    global closest, turtles
    if len(msg.turtles) == 0:
        closest = ''
        return
    closest_tmp = ''
    closest_dist = MIN_DIST
    for turtle in msg.turtles:
        pose = transform_to_baseframe(turtle.position)
        dist = dist_vec(pose.pose.position, Vector3())
        if dist < closest_dist:
            closest_dist = dist
            closest_tmp = turtle.name
        turtles[turtle.name] = turtle.position
    closest = closest_tmp

def process_msg(loc, msg):
    if msg.sender not in turtles.keys():
        return False
    turtle = turtles[msg.sender]
    if (rospy.Time.now() - turtle.header.stamp).to_sec() > LAST_SEEN:
        return False
        #turtle pose in base_link
    #pose = transform_to_baseframe(turtle)
    
    #res_pose = PoseStamped()
    #res_pose.header.frame_id = base_frame
    #res_pose.pose.position.x = pose.pose.position.x + msg.location.pose.position.x
    #res_pose.pose.position.y = pose.pose.position.y + msg.location.pose.position.y
    
    #yaw = get_jaw(pose.pose.orientation) + get_jaw(msg.location.pose.orientation)
    #q = tf.transformations.quaternion_from_euler(0,0,yaw, axes = "sxyz")
    #res_pose.pose.orientation = Quaternion(*q)
    msg.location.header.frame_id = odom
    global locations
    
    if not loc in locations:
        locations[loc] = {}
    locations[loc]['frame'] = odom
    locations[loc]['pose'] = msg.location #transformPose(res_pose)
    locations[loc]['time'] = msg.location.header.stamp

    if loc == 'hive':
        hive_pub.publish(locations[loc]['pose'])
    else:
        food_pub.publish(locations[loc]['pose'])

    return True



class Explore(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['found_hive', 'found_food'])
        self.locs = ['food', 'hive']
        self.client = actionlib.SimpleActionClient('SwarmCollvoid/swarm_nav_goal', MoveBaseAction)
        self.client.wait_for_server()

    def execute(self, userdata):
        global found, received, closest
        found = ''
        received = ''
        closest = ''
       
        send_msg = []
        start = rospy.Time.now()
        rate = rospy.Rate(RATE)


        goal = get_own_pose()
        dist, ang = get_random_walk()
        print "dist, ang", dist, ang

        v = Vector3()
        v.x = dist
        vect = rotate_vec_by_angle(v, ang)

        goal.pose.position.x += vect.x
        goal.pose.position.y += vect.y

        goal = create_goal_message(goal)
        self.client.send_goal(goal)


        while found=='':
            self.closest = copy.deepcopy(closest)
            if received in self.locs:
                if process_msg(received, received_msg):
                    print 'RECEIVED', received
                    return 'found_%s'%(received)
            if not self.closest=='':#and closest not in send_msg:
                send_msg.append(self.closest)
                for loc in self.locs:
                    print "asking ", self.closest, loc
                    request(self.closest, loc)
            move_random(self.client)
            rate.sleep()
        self.client.cancel_all_goals()
        stop()
        
        if found == 'hive':
            return 'found_hive'
        else:
            return 'found_food'
    

class SearchLocations(smach.State):
    def __init__(self, looking_for):
        smach.State.__init__(self, outcomes=['found', 'not_found'])
        self.loc = looking_for
        self.client = actionlib.SimpleActionClient('SwarmCollvoid/swarm_nav_goal', MoveBaseAction)
        self.client.wait_for_server()

    def execute(self, userdata):
        global found, received, closest
        found = ''
        received = ''
        closest = ''
        send_msg = []
        start = rospy.Time.now()
        rate = rospy.Rate(RATE)

        goal = get_own_pose()
        dist, ang = get_random_walk()

        print "dist, ang", dist, ang

        v = Vector3()
        v.x = dist
        vect = rotate_vec_by_angle(v, ang)

        goal.pose.position.x += vect.x
        goal.pose.position.y += vect.y

        goal = create_goal_message(goal)
        self.client.send_goal(goal)

        while not found in self.loc:
            self.closest = copy.deepcopy(closest)

            if (rospy.Time.now()-start).to_sec() > SEARCH_TIMEOUT:
                self.client.cancel_all_goals()
                stop()
                return 'not_found'
            if received in self.loc:
                print 'RECEIVED', received
                if process_msg(received, received_msg):
                    self.client.cancel_all_goals()
                    stop()
                    return 'found'
            if not self.closest=='': # and self.closest not in send_msg:
                send_msg.append(self.closest)
                for loc in self.loc:
                    print "asking ", self.closest, loc
                    request(self.closest, loc)
            move_random(self.client)
            rate.sleep()
        self.client.cancel_all_goals()
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


class CheckIfAtLocation(smach.State):
    def __init__(self, loc):
        smach.State.__init__(self, outcomes=['failed', 'success'])
        self.loc = loc

    def rotate_to_ang(self, ang):
        twist = Twist()
        twist.linear.x = 0
        twist.angular.z = self.rotate_side(ang) * ROTATION_SPEED
        cmd_pub.publish(twist)
        
    def rotate_side(self, ang):
        own_pose = get_own_pose()
        quat = [own_pose.pose.orientation.x, own_pose.pose.orientation.y, own_pose.pose.orientation.z,own_pose.pose.orientation.w]
        r,p,theta = tf.transformations.euler_from_quaternion(quat)
        
        res = ang - theta
        if res > math.pi:
            res -= 2 * math.pi
        if res < -math.pi:
            res += 2* math.pi
        if (res) < 0:
            return -1
        else:
            return 1

    def rotation_aligned(self, ang):
        own_pose = get_own_pose()
        quat = [own_pose.pose.orientation.x, own_pose.pose.orientation.y, own_pose.pose.orientation.z,own_pose.pose.orientation.w]
        r,p,theta = tf.transformations.euler_from_quaternion(quat)
        return  abs(ang - theta) < EPS

    def execute(self, userdata):
        global found
        found = ''

        own_pose = get_own_pose()
        target = copy.deepcopy(locations[self.loc]['pose'])
        #diff_x = target.pose.position.x - own_pose.pose.position.x
        #diff_y = target.pose.position.y - own_pose.pose.position.y

        #ang = math.atan2(diff_y, diff_x)

        ang = get_jaw(target.pose.orientation)
        rate = rospy.Rate(RATE)

        while not found == self.loc:
            if self.rotation_aligned(ang):
                return 'failed'
            else:
                if not sending:
                    self.rotate_to_ang(ang)
            rate.sleep()
        return 'success'

    
class MoveToOutLocation(smach.State):
    def __init__(self, loc):
        smach.State.__init__(self, outcomes=['failed', 'success'])
        self.loc = loc
        self.client = actionlib.SimpleActionClient('SwarmCollvoid/swarm_nav_goal', MoveBaseAction)
        self.client.wait_for_server()

    def execute(self, userdata):
        own_pose = get_own_pose()
        target = copy.deepcopy(locations[self.loc]['pose'])


        goal = move_location(target, y = -3 *Y_OFFSET)
        goal = create_goal_message(goal)

        self.client.send_goal(goal)

        rate = rospy.Rate(RATE)

        while True:
            if self.client.get_state() == GoalStatus.SUCCEEDED:
                self.client.cancel_all_goals()
                return 'success'
            if self.client.get_state() == GoalStatus.PREEMPTED:
                return 'failed'
            if sum(bumpers)>0:
                self.client.cancel_all_goals()
                twist = Twist()

                while sum(bumpers) > 0:
                    if bumpers[LEFT]:
                        twist.angular.z = -ROTATION_SPEED
                    else:
                        twist.angular.z = ROTATION_SPEED
                    cmd_pub.publish(twist)
                    rate.sleep()
                
                self.client.send_goal(goal)
                
            rate.sleep()
       

    
class MoveToLocation(smach.State):
    def __init__(self, loc):
        smach.State.__init__(self, outcomes=['failed', 'success'])
        self.loc = loc
        self.client = actionlib.SimpleActionClient('SwarmCollvoid/swarm_nav_goal', MoveBaseAction)
        self.client.wait_for_server()

       
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

        self.retry = 0
        while True:
            if sending:
                self.client.cancel_all_goals()
                stop()
                while sending:
                    rate.sleep()
                self.client.send_goal(goal)
                
            if (dist_vec(locations[self.loc]['pose'].pose.position, target.pose.position) > EPS_TARGETS):

                print "resending goal"
                self.client.cancel_all_goals()
                target = copy.deepcopy(locations[self.loc]['pose'])
                goal = create_goal_message(target)
                self.client.send_goal(goal)
                
            if self.client.get_state() == GoalStatus.SUCCEEDED:
                self.client.cancel_all_goals()
                return 'success'
            if self.client.get_state() == GoalStatus.PREEMPTED:
                self.client.cancel_all_goals()
                if self.retry < MAX_RETRY:
                    self.retry +=1
                    self.client.send_goal(goal)
                else:                    
                    return 'failed'
            if sum(bumpers)>0:
                self.client.cancel_all_goals()
                twist = Twist()

                while sum(bumpers) > 0:
                    if bumpers[LEFT]:
                        twist.angular.z = -ROTATION_SPEED
                    else:
                        twist.angular.z = ROTATION_SPEED

                    if not sending:
                        cmd_pub.publish(twist)
                    rate.sleep()
                
                self.client.send_goal(goal)
                
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
        smach.StateMachine.add("SearchHive", SearchLocations(['hive']), transitions = {'found':'GoToHive', 'not_found':'SearchHive'})
        smach.StateMachine.add("GoToHive", MoveToLocation('hive'), transitions = {'failed':'SearchHive', 'success':'AtHive'})
        smach.StateMachine.add("AtHive", CheckIfAtLocation('hive'), transitions = {'failed':'SearchHive', 'success':'GoToHiveOut'})
        smach.StateMachine.add("GoToHiveOut", MoveToOutLocation('hive'), transitions = {'failed':'PreSearchFood', 'success':'PreSearchFood'})
        
        
        # #food states
        smach.StateMachine.add("PreSearchFood", PreSearchLocation('food'), transitions = {'known':'GoToFood', 'not_known':'SearchFood'})
        smach.StateMachine.add("SearchFood", SearchLocations(['food']), transitions = {'found':'GoToFood', 'not_found':'SearchFood'})
        smach.StateMachine.add("GoToFood", MoveToLocation('food'), transitions = {'failed':'SearchFood', 'success':'AtFood'})
        smach.StateMachine.add("AtFood", CheckIfAtLocation('food'), transitions = {'failed':'SearchFood', 'success':'GoToFoodOut'})
        smach.StateMachine.add("GoToFoodOut", MoveToOutLocation('food'), transitions = {'failed':'PreSearchHive', 'success':'PreSearchHive'})

        
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

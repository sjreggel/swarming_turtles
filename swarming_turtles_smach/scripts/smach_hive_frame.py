#!/usr/bin/env python

import rospy
import smach
import smach_ros
import tf
import math
import random
import actionlib
import copy

from socket import gethostname

from geometry_msgs.msg import Twist, PoseStamped, Vector3, Quaternion
from move_base_msgs.msg import *

from swarming_turtles_msgs.msg import Turtles, Turtle
from swarming_turtles_navigation.srv import GetCollvoidTwist
from std_srvs.srv import Empty

turtles = {}
closest = ''

tfListen = None

name = '' #hostname used for communication

#config

SEARCH_TIMEOUT = 15
ASK_TIMEOUT = 1.0 

LAST_SEEN = 3.0 #check last seen for other turtle
EPS_TARGETS = 0.2 #if targets are further away than that resend goal

INWARDS = 0.4 #move loc xx meters inwards from detected marker locations

Y_OFFSET = 0.5 #move loc xx towards exit

MIN_DIST_ASK = 2.0 #how close to include in asking?

RATE = 10

hive = "/hive"
odom = "/odom"
base_frame = "/base_link"


def init_globals():
    global markers, tfListen, cmd_pub, name, get_twist
    tfListen = tf.TransformListener()
    rospy.sleep(1)

    name = gethostname()

    cmd_pub = rospy.Publisher('cmd_vel_mux/input/navi', Twist) #publish Twist
    rospy.Subscriber('/found_turtles', Turtles, cb_found_turtles) #which turtles are near?

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
    
def rotate_vec_by_angle(v, ang):
    res = Vector3()
    cos_a = math.cos(ang)
    sin_a = math.sin(ang)
    res.x = cos_a * v.x - sin_a * v.y
    res.y = cos_a * v.y + sin_a * v.x
    return res


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
                  

def cb_found_turtles(msg):
    global closest, turtles
    if len(msg.turtles) == 0:
        closest = ''
        return
    closest_tmp = ''
    closest_dist = MIN_DIST_ASK
    for turtle in msg.turtles:
        pose = transformPose(turtle.position, frame = base_frame)
        dist = dist_vec(pose.pose.position, Vector3())
        if dist < closest_dist:
            closest_dist = dist
            closest_tmp = turtle.name
        turtles[turtle.name] = turtle.position
    closest = closest_tmp




class SearchFood(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['found', 'not_found'])
        self.get_food_srv = rospy.ServiceProxy('get_location', GetLocation)
        self.get_received_location_srv = rospy.ServiceProxy('get_received_location', GetLocation)
        self.ask_food_srv = rospy.ServiceProxy('ask_food', GetLocation)

        self.move_random_start = rospy.ServiceProxy('move_random_start', Empty)
        self.move_random_stop = rospy.ServiceProxy('move_random_stop', Empty)
        
        rospy.wait_for_service(self.get_food_srv)
        rospy.wait_for_service(self.get_received_location_srv)

        
    def ask_food(self, name):
        try:
            resp = self.ask_food_srv(location = name)
            return resp.pose
        except:
            rospy.logerr("service call ask food failed")
            return None

        
    def get_food(self):
        try:
            resp = self.get_food_srv()
            return resp.pose
        except:
            rospy.logerr("service call to get food failed")
            return None

    def get_received_location(self,asked_turtles):
        try:
            resp = self.get_received_location_srv()
            if resp.res in asked_turtles and (rospy.Time.now()-resp.pose.header.stamp).to_sec() < ASK_TIMEOUT:
                return resp.pose
            else:
                return None
        except:
            rospy.logerr("service call to receive locaiton failed")
            return None

        
    def execute(self, userdata):
        global closest
        closest = ''
        asked_turtles = []
        start = rospy.Time.now()
        rate = rospy.Rate(RATE)
        found = False
        
        self.move_random_start()

        while not found:
            if (rospy.Time.now()-start).to_sec() > SEARCH_TIMEOUT:
                self.move_random_stop()
                return 'not_found'
            pose = self.get_food()
            if pose is not None:
                found = True
            pose = self.get_received_location()
            if pose is not None:
                found = True
            if not closest==''  and closest not in asked_turtles:
                asked_turles.append(closest)
                print "asking ", closest 
                self.ask_food(closest)
            rate.sleep()

        self.move_random_stop()
        userdata.pose_out = pose
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

def create_goal_message(goal):
    goal_msg = MoveBaseGoal()
    goal_msg.target_pose.pose = goal.pose
  
    goal_msg.target_pose.header.frame_id = odom
    goal_msg.target_pose.header.stamp = rospy.Time.now()
    return goal_msg

    
class MoveToHiveOut(smach.State):
    def __init__(self, loc):
        smach.State.__init__(self, outcomes=['failed', 'success'])
        self.client = actionlib.SimpleActionClient('SwarmCollvoid/swarm_nav_goal', MoveBaseAction)
        self.client.wait_for_server()

    def execute(self, userdata):
        own_pose = get_own_pose()
        target = 
        
        goal = move_location(target, y = Y_OFFSET)
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
        return abs(dist_cur - dist) < EPS_ALIGN


    
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

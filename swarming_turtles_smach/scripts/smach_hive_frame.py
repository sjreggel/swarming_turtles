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
from actionlib_msgs.msg import *

from swarming_turtles_detect.srv import GetLocation, ForgetLocation

from swarming_turtles_msgs.msg import Turtles, Turtle
from swarming_turtles_navigation.srv import GetCollvoidTwist
import swarming_turtles_navigation.move_random as utils
from std_srvs.srv import Empty

turtles = {}
closest = ''

tfListen = None

name = '' #hostname used for communication

hive_loc = None
move_random_stop = None
move_random_start = None
get_food_srv = None
get_hive_srv = None

#config
MAX_RETRY = 5
SEARCH_TIMEOUT = 15
ASK_TIMEOUT = 1.0 

FIND_TIMEOUT = 0.5

LAST_SEEN = 3.0 #check last seen for other turtle
EPS_TARGETS = 0.2 #if targets are further away than that resend goal

INWARDS = 0.4 #move loc xx meters inwards from detected marker locations

Y_OFFSET = 1.5 #move loc xx towards exit

MIN_DIST_ASK = 2.0 #how close to include in asking?

RATE = 10

hive = "/hive"
odom = "/odom"
base_frame = "/base_link"

MAX_DIST = 1.5

def init_globals():
    global name, hive, hive_loc, move_random_stop, move_random_start, get_food_srv, get_hive_srv
    utils.init_globals()


    hive_loc = PoseStamped()
    hive_loc.header.frame_id = hive
    hive_loc.pose.orientation.w = 1
    hive_loc = utils.move_location_inwards(hive_loc, INWARDS)
    
    get_food_srv = rospy.ServiceProxy('get_location', GetLocation)
    get_hive_srv = rospy.ServiceProxy('get_hive', GetLocation)

    move_random_start = rospy.ServiceProxy('move_random_start', Empty)
    move_random_stop = rospy.ServiceProxy('move_random_stop', Empty)

    
    name = gethostname()

    rospy.Subscriber('/found_turtles', Turtles, cb_found_turtles) #which turtles are near?


def at_hive():
    try:
        resp = get_hive_srv()
        own_pose = utils.get_own_pose()
        target = PoseStamped()
        dist = utils.dist_vec(own_pose.pose.position, target.pose.position)
        return not resp.res == '' and (rospy.Time.now()-resp.pose.header.stamp).to_sec() < FIND_TIMEOUT and dist < MAX_DIST
    except:
        rospy.logerr("service call to get hive failed")
        return False


def at_food():
    try:
        resp = get_food_srv()
        own_pose = utils.get_own_pose()
        target = resp.pose
        dist = utils.dist_vec(own_pose.pose.position, target.pose.position)

        return not resp.res == '' and (rospy.Time.now()-resp.pose.header.stamp).to_sec() < FIND_TIMEOUT and dist < MAX_DIST
    except:
        rospy.logerr("service call to get food failed")
        return False

def get_food():
    try:
        resp = get_food_srv()
        if not resp.res == '':
            #pose = resp.pose
            return resp.pose
        else:
            return None
    except:
        rospy.logerr("service call to get food failed")
        return None

    
def cb_found_turtles(msg):
    global closest, turtles
    if len(msg.turtles) == 0:
        closest = ''
        return
    closest_tmp = ''
    closest_dist = MIN_DIST_ASK
    for turtle in msg.turtles:
        pose = utils.transformPose(turtle.position, frame = base_frame)
        dist = utils.dist_vec(pose.pose.position, Vector3())
        if dist < closest_dist:
            closest_dist = dist
            closest_tmp = turtle.name
        turtles[turtle.name] = turtle.position
    closest = closest_tmp


class SearchFood(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['found', 'not_found'], output_keys = ['pose_out'])
        self.get_received_location_srv = rospy.ServiceProxy('get_received_location', GetLocation)
        self.ask_food_srv = rospy.ServiceProxy('ask_food', GetLocation)

        
        #rospy.wait_for_service(self.ask_food_srv)
        #rospy.wait_for_service(self.get_received_location_srv)

        
    def ask_food(self, name):
        try:
            resp = self.ask_food_srv(location = name)
            return resp.pose
        except:
            rospy.logerr("service call ask food failed")
            return None

    def get_received_location(self,asked_turtles):
        try:
            resp = self.get_received_location_srv(location = '')
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
        
        move_random_start()
        userdata.pose_out = None
        pose = None
        while not found:
            if (rospy.Time.now()-start).to_sec() > SEARCH_TIMEOUT:
                move_random_stop()
                return 'not_found'
            pose = get_food()
            if pose is not None:
                found = True
            pose = self.get_received_location(asked_turtles)
            if pose is not None:
                found = True
            if not closest==''  and closest not in asked_turtles:
                asked_turtles.append(closest)
                print "asking ", closest 
                self.ask_food(closest)
            rate.sleep()

        userdata.pose_out = pose

        move_random_stop()
        return 'found'
           

class PreSearchFoodLocation(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['known', 'not_known'])
        
    def execute(self, userdata):
        if get_food() is not None:
            return 'known'
        else:
            return 'not_known'


class CheckIfAtLocation(smach.State):
    def __init__(self, loc):
        smach.State.__init__(self, outcomes=['failed', 'success'])
        self.loc = loc
        self.forget_food = rospy.ServiceProxy('forget_location', ForgetLocation)
      
    def execute(self, userdata):
        target = None
        found = None
        if self.loc =='hive':
            target = hive_loc
            found = at_hive
        else: #food
            target = get_food()
            if target is None:
                try:
                    self.forget_food()
                except:
                    print "forget_food failed"
                return 'failed'
            found = at_food
            target = utils.move_location_inwards(target, INWARDS)
        

        ang = utils.get_jaw(target.pose.orientation)
        rate = rospy.Rate(RATE)

        while not found():
            if utils.rotation_aligned(ang):
                if self.loc == 'food':
                    try:
                        self.forget_food()
                    except:
                        print "forget_food failed"

                return 'failed'
            utils.rotate_to_ang(ang)
            rate.sleep()
        return 'success'




class SearchHive(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['success'])
        
    def execute(self, userdata):
        rate = rospy.Rate(RATE)
        start = rospy.Time.now()
        move_random_start()
        while True:
            if (rospy.Time.now()-start).to_sec() > 2*SEARCH_TIMEOUT:
                move_random_stop()
                return 'success'
         
            if at_hive():
                move_random_stop()
                return 'success'
            rate.sleep()



class MoveToInLocation(smach.State):
    def __init__(self, loc):
        smach.State.__init__(self, outcomes=['failed', 'success'])
        self.client = actionlib.SimpleActionClient('move_to_goal', MoveBaseAction)
        self.client.wait_for_server()
        self.loc = loc
        
    def execute(self, userdata):
        target = None
        if self.loc =='hive':
            target = hive_loc
            at_loc = at_hive
        else:
            target = get_food()
            if target is None:
                return 'failed'
            at_loc = at_food
            target = utils.move_location_inwards(target, INWARDS)

        if at_loc():
            return 'success'
        
        goal = utils.move_location(target, x = -0.5, y = -0.5)
        goal = utils.create_goal_message(goal)

        self.client.send_goal(goal)

        rate = rospy.Rate(RATE)

        start = rospy.Time.now()
        
        
        while True:
            if self.client.get_state() == GoalStatus.SUCCEEDED:
                self.client.cancel_all_goals()
                return 'success'
            if self.client.get_state() == GoalStatus.PREEMPTED:
                self.client.cancel_all_goals()
                return 'failed'
            rate.sleep()


            
class MoveToOutLocation(smach.State):
    def __init__(self, loc):
        smach.State.__init__(self, outcomes=['failed', 'success'])
        self.client = actionlib.SimpleActionClient('move_to_goal', MoveBaseAction)
        self.client.wait_for_server()
        self.loc = loc
        self.TIME_OUT = 5.0
        
    def execute(self, userdata):
        target = None
        if self.loc =='hive':
            target = hive_loc
        else:
            target = get_food()
            if target is None:
                return 'failed'
            target = utils.move_location_inwards(target, INWARDS)
            
        goal = utils.move_location(target, y = Y_OFFSET)
        goal = utils.create_goal_message(goal)

        self.client.send_goal(goal)

        rate = rospy.Rate(RATE)

        start = rospy.Time.now()
        
        
        while True:
            if (rospy.Time.now()-start).to_sec() > self.TIME_OUT:
                self.client.cancel_all_goals()
                return 'failed' 
            if self.client.get_state() == GoalStatus.SUCCEEDED:
                self.client.cancel_all_goals()
                return 'success'
            if self.client.get_state() == GoalStatus.PREEMPTED:
                self.client.cancel_all_goals()
                return 'failed'
            rate.sleep()
       
class MoveToHiveLocation(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['failed', 'success'])
        self.client = actionlib.SimpleActionClient('move_to_goal', MoveBaseAction)
        self.client.wait_for_server()

    def execute(self, userdata):
        target = hive_loc
        goal = utils.create_goal_message(target)
        self.client.send_goal(goal)
        rate = rospy.Rate(RATE)

        self.retry = 0
        while True:
            if self.client.get_state() == GoalStatus.SUCCEEDED:
                self.client.cancel_all_goals()
                return 'success'
            if self.client.get_state() == GoalStatus.PREEMPTED:
                self.client.cancel_all_goals()
                if self.retry < 2*MAX_RETRY:
                    self.retry +=1
                    self.client.send_goal(goal)
                else:
                    self.client.cancel_all_goals()
                    return 'failed'
            rate.sleep()

            
    
class MoveToFoodLocation(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['failed', 'success'], input_keys = ['pose_in'])
        self.client = actionlib.SimpleActionClient('move_to_goal', MoveBaseAction)
        self.client.wait_for_server()
        self.forget_food = rospy.ServiceProxy('forget_location', ForgetLocation)
      
        
    def execute(self, userdata):
        target = get_food()

        if target is None and userdata.pose_in is not None:
            target = userdata.pose_in

        if target is None:
            return 'failed'
        goal = utils.create_goal_message(utils.move_location_inwards(target, INWARDS))
        self.client.send_goal(goal)

        rate = rospy.Rate(RATE)

        self.retry = 0
        while True:
            pose = get_food()
            if pose is not None and (utils.dist_vec(pose.pose.position, target.pose.position) > EPS_TARGETS):
                print "resending goal"
                self.client.cancel_all_goals()
                target = pose
                goal = utils.create_goal_message(utils.move_location_inwards(target, INWARDS))
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
                    try:
                        self.forget_food(location = "")
                    except:
                        print "forget_food failed"
                    return 'failed'
            rate.sleep()

def main():
    rospy.init_node('swarming_turtles_machine')
    init_globals()
    # create a SMACH state machine
    sm = smach.StateMachine(outcomes=['end'])
    sm.userdata.pose = None
    with sm:
        #Hive states
        smach.StateMachine.add("SearchHive", SearchHive(), transitions = {'success':'GoToHiveIn'})

        smach.StateMachine.add("GoToHiveIn", MoveToInLocation('hive'), transitions = {'failed':'GoToHive', 'success':'GoToHive'})
        smach.StateMachine.add("GoToHive", MoveToHiveLocation(), transitions = {'failed':'SearchHive', 'success':'AtHive'})
       
        smach.StateMachine.add("AtHive", CheckIfAtLocation('hive'), transitions = {'failed':'SearchHive', 'success':'GoToHiveOut'})
        smach.StateMachine.add("GoToHiveOut", MoveToOutLocation('hive'), transitions = {'failed':'PreSearchFood', 'success':'PreSearchFood'})
        
        
        # #food states
        smach.StateMachine.add("PreSearchFood", PreSearchFoodLocation(), transitions = {'known':'GoToFoodIn', 'not_known':'SearchFood'})
        smach.StateMachine.add("SearchFood", SearchFood(), transitions = {'found':'GoToFoodIn', 'not_found':'SearchFood'}, remapping = {'pose_out':'pose'})

        smach.StateMachine.add("GoToFoodIn", MoveToInLocation('food'), transitions = {'failed':'GoToFood', 'success':'GoToFood'})
        smach.StateMachine.add("GoToFood", MoveToFoodLocation(), transitions = {'failed':'SearchFood', 'success':'AtFood'}, remapping = {'pose_in':'pose'})
        smach.StateMachine.add("AtFood", CheckIfAtLocation('food'), transitions = {'failed':'SearchFood', 'success':'GoToFoodOut'})
        smach.StateMachine.add("GoToFoodOut", MoveToOutLocation('food'), transitions = {'failed':'GoToHiveIn', 'success':'GoToHiveIn'})

        
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

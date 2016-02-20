#!/usr/bin/env python
from socket import gethostname
import math

import rospy
import smach
import smach_ros
import actionlib
from geometry_msgs.msg import Twist, PoseStamped, Vector3
from move_base_msgs.msg import *
from actionlib_msgs.msg import *
from swarming_turtles_detect.srv import GetLocation, ForgetLocation
from swarming_turtles_msgs.msg import Turtles
from std_srvs.srv import Empty, EmptyResponse
from stage_ros.msg import Stall
import swarming_turtles_navigation.move_random as utils
from std_msgs.msg import Int32, String

#disable ros_info messages from SMACH
import logging
class Filter(logging.Filter):
    def filter(self, record):
        return 'State machine' not in record.msg
class Filter2(logging.Filter):
    def filter2(self, record):
        return 'neighbor' not in record.msg


logging.getLogger('rosout').addFilter(Filter())
logging.getLogger('rosout').addFilter(Filter2())

 #transitioning


turtles = {}
closest = ''
count_fooddeliveries = 0
robot_has_food = False
robot_is_foraging = False
robot_in_collision = False

tfListen = None

own_name = ''  # hostname used for communication

hive_loc = None
move_random_stop = None
move_random_start = None
get_food_srv = None
get_hive_srv = None

log_publisher = None


get_received_location_srv = None

prev_xpos = 0
prev_ypos = 0
prev_zpos = 0

# config
MAX_RETRY = 5
SEARCH_TIMEOUT = 15
ASK_TIMEOUT = 1.0

MOVE_RANDOM_TIME = 3.0

STAND_STILL_TIMES = 10

FIND_TIMEOUT = 1.0

LAST_SEEN = 3.0  # check last seen for other turtle
EPS_TARGETS = 0.2  # if targets are further away than that resend goal

INWARDS = 0.4  # move loc xx meters inwards from detected marker locations

Y_OFFSET_EXIT = -0.5  # move loc xx towards exit
X_OFFSET_EXIT = 0.2  # move loc xx towards exit

Y_OFFSET_ENTRY = 0.5  # move loc xx towards exit
X_OFFSET_ENTRY = 0.2  # move loc xx towards exit

MIN_DIST_ASK = 2.0  # how close to include in asking?

RATE = 10

hive = rospy.get_param('hive_frame', '/hive')

global_frame = rospy.get_param('global_frame', '/odom')
base_frame = rospy.get_param('base_frame', '/base_link')

MAX_DIST = 1.5
move_action_server = None

# offset = rospy.get_param('marker_offset', math.pi/2.0)
offset = 0

started = False




def rob_debug():
    global own_name
    global count_fooddeliveries
    global robot_has_food
    global robot_is_foraging
    global robot_in_collision
    now = rospy.get_rostime()
    time_now = float(now.secs) + (float(now.nsecs) / 1000000000)
    pos = utils.get_own_pose()
    x = format(pos.pose.position.x,'.3f')
    y = format(pos.pose.position.y,'.3f')
    z = format(pos.pose.orientation.z,'.3f')
    w = format(pos.pose.orientation.w,'.3f')
    return time_now, own_name, x, y, z, w, robot_is_foraging, count_fooddeliveries, robot_has_food, robot_in_collision

def init_globals():
    global own_name, hive, hive_loc, move_random_stop, move_random_start, get_food_srv, get_hive_srv, move_action_server, get_received_location_srv
    utils.init_globals()

    move_action_server = actionlib.SimpleActionClient('move_to_goal', MoveBaseAction)
    move_action_server.wait_for_server()

    hive_loc = PoseStamped()
    hive_loc.header.frame_id = hive
    hive_loc.pose.orientation.w = 1
    hive_loc = utils.move_location_inwards(hive_loc, INWARDS, offset=offset)

    get_food_srv = rospy.ServiceProxy('get_location', GetLocation, persistent=True)
    get_hive_srv = rospy.ServiceProxy('get_hive', GetLocation, persistent=True)

    get_received_location_srv = rospy.ServiceProxy('get_received_location', GetLocation, persistent=True)

    move_random_start = rospy.ServiceProxy('move_random_start', Empty)
    move_random_stop = rospy.ServiceProxy('move_random_stop', Empty)

    own_name = rospy.get_namespace()
    if own_name == "/":
        own_name = gethostname()
    else:
        own_name = own_name.replace('/', '')
    own_name = rospy.get_param('~name', own_name)

    rospy.Subscriber('found_turtles', Turtles, cb_found_turtles)  # which turtles are near?
    rospy.Subscriber('stall', Stall, cb_stall)  # turtle in collision?



def seen_hive():
    global get_hive_srv
    try:
        resp = get_hive_srv()
        return not resp.res == '' and (rospy.Time.now() - resp.pose.header.stamp).to_sec() < FIND_TIMEOUT
    except:
        rospy.logerr("service call to get hive failed")
        get_hive_srv.close()
        get_hive_srv = rospy.ServiceProxy('get_hive', GetLocation, persistent=True)

        return False


def at_hive():
    global get_hive_srv
    try:
        resp = get_hive_srv()
        own_pose = utils.get_own_pose()
        target = PoseStamped()
        dist = utils.dist_vec(own_pose.pose.position, target.pose.position)
        return not resp.res == '' \
               and (rospy.Time.now() - resp.pose.header.stamp).to_sec() < FIND_TIMEOUT \
               and dist < MAX_DIST
    except:
        rospy.logerr("service call to get hive failed")
        get_hive_srv.close()
        get_hive_srv = rospy.ServiceProxy('get_hive', GetLocation, persistent=True)

        return False


def at_food():
    global get_food_srv
    try:
        resp = get_food_srv()
        own_pose = utils.get_own_pose()
        target = resp.pose
        dist = utils.dist_vec(own_pose.pose.position, target.pose.position)

        return not resp.res == '' \
               and (rospy.Time.now() - resp.pose.header.stamp).to_sec() < FIND_TIMEOUT \
               and dist < MAX_DIST
    except:
        rospy.logerr("service call to get food failed")
        get_food_srv.close()
        get_food_srv = rospy.ServiceProxy('get_location', GetLocation, persistent=True)
        return False


def get_food():
    global get_food_srv
    try:
        resp = get_food_srv()
        if not resp.res == '':
            # pose = resp.pose
            return resp.pose
        else:
            return None
    except:
        rospy.logerr("service call to get food failed")
        get_food_srv.close()
        get_food_srv = rospy.ServiceProxy('get_location', GetLocation, persistent=True)

        return None


def cb_found_turtles(msg):
    global closest, turtles
    if len(msg.turtles) == 0:
        closest = ''
        return
    closest_tmp = ''
    closest_dist = MIN_DIST_ASK
    for turtle in msg.turtles:
        if turtle.name == own_name:
            continue
        # TODO: check angle if in view (for simulation)
        pose = utils.transformPose(turtle.position, frame=base_frame)
        dist = utils.dist_vec(pose.pose.position, Vector3())
        if dist < closest_dist:
            closest_dist = dist
            closest_tmp = turtle.name
        turtles[turtle.name] = turtle.position
    closest = closest_tmp

def cb_stall(msg):
    global robot_in_collision
    robot_in_collision = msg.stall



def get_received_location(asked_turtles):
    global get_received_location_srv
    try:
        resp = get_received_location_srv(location='')
        if (resp.res in asked_turtles or resp.res == 'mitro') and (
                    rospy.Time.now() - resp.pose.header.stamp).to_sec() < ASK_TIMEOUT:
            return resp.pose
        else:
            return None
    except rospy.ServiceException:
        rospy.logerr("service call to receive locaiton failed")
        get_received_location_srv.close()
        get_received_location_srv = rospy.ServiceProxy('get_received_location', GetLocation, persistent=True)

        return None

class SearchFoodNoAsking(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['found', 'not_found'], output_keys=['pose_out'])

    def execute(self, userdata):
        global robot_is_foraging
        robot_is_foraging = False
        start = rospy.Time.now()
        rate = rospy.Rate(RATE)
        found = False
        move_random_start()
        userdata.pose_out = None
        pose = None

        while not found and not rospy.is_shutdown():
            if (rospy.Time.now() - start).to_sec() > SEARCH_TIMEOUT:
                move_random_stop()
                return 'not_found'
            pose = get_food()
            if pose is not None:
                found = True
                break
            #rospy.loginfo("%s -> %s ", rob_debug(), type(self).__name__)
            log_publisher.publish("%s -> %s" % (str(rob_debug()),type(self).__name__))
            rate.sleep()

        userdata.pose_out = pose
        move_random_stop()
        return 'found'




class SearchFood(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['found', 'not_found'], output_keys=['pose_out'])
        self.ask_food_srv = rospy.ServiceProxy('ask_food', GetLocation, persistent=True)

    def ask_food(self, name):
        try:
            resp = self.ask_food_srv(location=name)
            return resp.pose
        except:
            rospy.logerr("service call ask food failed")
            self.ask_food_srv.close()
            self.ask_food_srv = rospy.ServiceProxy('ask_food', GetLocation, persistent=True)
            return None

    def execute(self, userdata):
        global closest
        global robot_is_foraging
        asked_robot = None
        robot_is_foraging = False
        closest = ''
        asked_turtles = []
        start = rospy.Time.now()
        rate = rospy.Rate(RATE)
        found = False
        move_random_start()
        userdata.pose_out = None
        pose = None
        while not found and not rospy.is_shutdown():
            if (rospy.Time.now() - start).to_sec() > SEARCH_TIMEOUT:
                move_random_stop()
                return 'not_found'
            pose = get_food()
            if pose is not None:
                found = True
                break
            pose = get_received_location(asked_turtles)
            if pose is not None:
                found = True
                break
            if not closest == '' and closest not in asked_turtles:
                asked_turtles.append(closest)
                #rospy.loginfo("%s -> asking %s for food", rob_debug(), closest)
                log_publisher.publish("%s -> asking %s for food" % (str(rob_debug()),closest))
                asked_robot = closest
                #print "asking ", closest
                self.ask_food(closest)
            else:
                asked_robot = None
            #rospy.loginfo("%s -> %s ", rob_debug(), type(self).__name__)
            log_publisher.publish("%s -> %s" % (str(rob_debug()),type(self).__name__))
            rate.sleep()

        if asked_robot == closest:
            #rospy.loginfo("%s -> Foodlocation received from %s", rob_debug(), closest)
            log_publisher.publish("%s -> Foodlocation received from %s" % (str(rob_debug()),closest))
            asked_robot = None

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
        smach.State.__init__(self, outcomes=['failed', 'success'], input_keys=['pose_in'])
        self.loc = loc
        self.forget_food = rospy.ServiceProxy('forget_location', ForgetLocation)

    def execute(self, userdata):
        global count_fooddeliveries
        global robot_has_food
        global robot_is_foraging

        drop_pub = rospy.Publisher('fooddrops', Int32, queue_size=10)


        target = None
        found = None
        if self.loc == 'hive':
            target = hive_loc
            found = at_hive
            if robot_has_food is True:
                robot_has_food = False # Food is deliverd
                count_fooddeliveries += 1
                #rospy.loginfo("%s -> Delivered_Food", rob_debug())
                log_publisher.publish("%s -> Delivered_Food" % (str(rob_debug())))
                drop_pub.publish(count_fooddeliveries)
                robot_is_foraging = True
        else:  # food
            target = get_food()

            if target is None and userdata.pose_in is not None:
                target = userdata.pose_in

            if target is None:
                try:
                    self.forget_food()
                except:
                    print "forget_food failed"
                    robot_is_foraging = False
                return 'failed'
            found = at_food
   
            target = utils.move_location_inwards(target, INWARDS, offset=offset)

        ang = utils.get_jaw(target.pose.orientation) + math.pi
        rate = rospy.Rate(RATE)
        start = rospy.Time.now()

        while not found() and not rospy.is_shutdown():
            if utils.rotation_aligned(ang, eps=0.05):
                if self.loc == 'food':
                    try:
                        self.forget_food()
                    except:
                        print "forget_food failed"
                robot_is_foraging = False
                return 'failed'
            utils.rotate_to_ang(ang)
            if (rospy.Time.now() - start).to_sec() > SEARCH_TIMEOUT:
                #move_action_server.cancel_all_goals()
                try:
                    self.forget_food()
                except:
                    pass
                robot_is_foraging = False
                return 'failed'	    

            rate.sleep()
        if self.loc == 'food' and target is not None:
            robot_has_food = True
            #print ">>>>>>>>", own_name, "Aquired_food" ,"<<<<<<<<"
            #rospy.loginfo("%s -> Aquired_Food", rob_debug())
            log_publisher.publish("%s -> Aquired_Food" % (str(rob_debug())))
        return 'success'


class SearchHive(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['success'])
        global robot_is_foraging
        robot_is_foraging = False

    def execute(self, userdata):
        global prev_xpos
        global prev_ypos
        global prev_zpos
        rate = rospy.Rate(RATE)
        start = rospy.Time.now()
        move_random_start()
        while not rospy.is_shutdown():
            if (rospy.Time.now() - start).to_sec() > 2 * SEARCH_TIMEOUT:
                move_random_stop()
                return 'success'
            if seen_hive():
                move_random_stop()
                return 'success'
            rate.sleep()
        # only loginfo when robot moved
        pose = utils.get_own_pose()
        if (prev_xpos != pose.pose.position.x) or (prev_ypos != pose.pose.position.y) or (prev_zpos != pose.pose.position.z):
            prev_xpos = pose.pose.position.x
            prev_ypos = pose.pose.position.y
            prev_zpos = pose.pose.position.z
        #rospy.loginfo("%s -> %s ", rob_debug(), type(self).__name__)
        log_publisher.publish("%s -> %s" % (str(rob_debug()),type(self).__name__))


class InitHive(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['success'])

    def execute(self, userdata):
        rate = rospy.Rate(RATE)
        start = rospy.Time.now()
        twist = Twist()
        twist.angular.z = utils.ROTATION_SPEED
        while not rospy.is_shutdown():
            if True: #seen_hive(): #hive at startup has not to be in visual range
                utils.stop()
                rospy.sleep(1.0)
                return 'success'
            utils.cmd_pub.publish(twist)
            rate.sleep()


class MoveToInLocation(smach.State):
    def __init__(self, loc):
        smach.State.__init__(self, outcomes=['failed', 'success'], input_keys=['pose_in'], output_keys=['pose_out'])
        self.loc = loc
        self.TIME_OUT = 60.0

    def execute(self, userdata):
        global move_action_server
        global prev_xpos
        global prev_ypos
        global prev_zpos

        target = None

        if self.loc == 'hive':
            target = hive_loc
            at_loc = at_hive
        else:
            target = get_food()

            if target is None and userdata.pose_in is not None:
                target = userdata.pose_in
                userdata.pose_out = target
            if target is None:
                return 'failed'
            at_loc = at_food
            target = utils.move_location_inwards(target, INWARDS, offset=offset)
        if at_loc():
            return 'success'

        goal = utils.move_location(target, x=X_OFFSET_ENTRY, y=Y_OFFSET_ENTRY)
        goal = utils.create_goal_message(goal)

        move_action_server.send_goal(goal)

        rate = rospy.Rate(RATE)

        start = rospy.Time.now()
        stand_still = 0
        self.retry = 0
        old_pose = utils.get_own_pose()
        retry = MAX_RETRY - 2

        while not rospy.is_shutdown():
            if stand_still > STAND_STILL_TIMES:
                move_action_server.cancel_all_goals()
                print own_name, "standing still too long for moving to in location"
                move_action_server = actionlib.SimpleActionClient('move_to_goal', MoveBaseAction)
                move_action_server.wait_for_server()
                if self.retry < MAX_RETRY:
                    self.retry += 1
                    move_action_server.send_goal(goal)
                else:
                    move_action_server.cancel_all_goals()
                    return 'failed'

            if utils.standing_still(old_pose):
                stand_still += 1
            else:
                stand_still = 0
                old_pose = utils.get_own_pose()
            rec_pose = get_received_location([])
            if rec_pose is not None:
                move_action_server.cancel_all_goals()
                goal = utils.move_location_inwards(rec_pose, INWARDS, offset=offset)
                goal = utils.move_location(goal, x=X_OFFSET_ENTRY, y=Y_OFFSET_ENTRY)
                goal = utils.create_goal_message(goal)
                move_action_server.send_goal(goal)
                rate.sleep()
                continue

            if (rospy.Time.now() - start).to_sec() > self.TIME_OUT:
                move_action_server.cancel_all_goals()
                return 'failed'


            if move_action_server.get_state() == GoalStatus.SUCCEEDED:
                move_action_server.cancel_all_goals()
                return 'success'
            
            if move_action_server.get_state() == GoalStatus.PREEMPTED:
                move_action_server.cancel_all_goals()
                if retry > MAX_RETRY:
                    return 'failed'
                else:
                    retry += 1
                    move_random_start()
                    rospy.sleep(MOVE_RANDOM_TIME)
                    move_random_stop()
                    move_action_server.send_goal(goal)
        
            # only loginfo when robot moved
            pose = utils.get_own_pose()
            if (prev_xpos != pose.pose.position.x) or (prev_ypos != pose.pose.position.y) or (prev_zpos != pose.pose.position.z):
                prev_xpos = pose.pose.position.x
                prev_ypos = pose.pose.position.y
                prev_zpos = pose.pose.position.z
            #rospy.loginfo("%s -> %s %s ", rob_debug(), type(self).__name__, self.loc)
            log_publisher.publish("%s -> %s %s " % (str(rob_debug()),type(self).__name__, self.loc)) 
            rate.sleep()


class MoveToOutLocation(smach.State):
    def __init__(self, loc):
        smach.State.__init__(self, outcomes=['failed', 'success'])
        self.loc = loc
        self.TIME_OUT = 3.0

    def execute(self, userdata):
        global prev_xpos
        global prev_ypos
        global prev_zpos
        target = None
        if self.loc == 'hive':
            target = hive_loc
        else:
            target = get_food()
            if target is None:
                return 'failed'
            target = utils.move_location_inwards(target, INWARDS, offset)

        goal = utils.move_location(target, x=X_OFFSET_EXIT, y=Y_OFFSET_EXIT)
        goal = utils.create_goal_message(goal)

        move_action_server.send_goal(goal)

        rate = rospy.Rate(RATE)

        start = rospy.Time.now()

        stand_still = 0
        old_pose = utils.get_own_pose()

        while not rospy.is_shutdown():
            if stand_still > STAND_STILL_TIMES:
                move_action_server.cancel_all_goals()
                return 'failed'
            if utils.standing_still(old_pose):
                stand_still += 1
            else:
                stand_still = 0
                old_pose = utils.get_own_pose()

            if (rospy.Time.now() - start).to_sec() > self.TIME_OUT:
                move_action_server.cancel_all_goals()
                return 'failed'
            if move_action_server.get_state() == GoalStatus.SUCCEEDED:
                move_action_server.cancel_all_goals()
                return 'success'
            if move_action_server.get_state() == GoalStatus.PREEMPTED:
                move_action_server.cancel_all_goals()
                return 'failed'
            # only loginfo when robot moved
            pose = utils.get_own_pose()
            if (prev_xpos != pose.pose.position.x) or (prev_ypos != pose.pose.position.y) or (prev_zpos != pose.pose.position.z):
                prev_xpos = pose.pose.position.x
                prev_ypos = pose.pose.position.y
                prev_zpos = pose.pose.position.z
            #rospy.loginfo("%s -> %s %s ", rob_debug(), type(self).__name__, self.loc)
            log_publisher.publish("%s -> %s %s " % (str(rob_debug()),type(self).__name__, self.loc)) 
            rate.sleep()


class MoveToHiveLocation(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['failed', 'success'])
        self.TIME_OUT = 3.0

    def execute(self, userdata):
        global move_action_server
        global robot_is_foraging
        global prev_xpos
        global prev_ypos
        global prev_zpos

        target = hive_loc
        goal = utils.create_goal_message(target)
        move_action_server.send_goal(goal)
        rate = rospy.Rate(RATE)

        start = rospy.Time.now()

        self.retry = 0
        stand_still = 0
        old_pose = utils.get_own_pose()

        while not rospy.is_shutdown():
            if stand_still > STAND_STILL_TIMES:
                move_action_server.cancel_all_goals()
                if at_hive():
                    print 'standing still too long, but close to hive'
                    return 'success'
                else:
                    print 'standing still to long'
                    move_action_server.cancel_all_goals()
                    move_action_server = actionlib.SimpleActionClient('move_to_goal', MoveBaseAction)
                    move_action_server.wait_for_server()

                    if self.retry < MAX_RETRY:
                        self.retry += 1
                        move_action_server.send_goal(goal)
                    else:
                        move_action_server.cancel_all_goals()
                        return 'failed'

            if utils.standing_still(old_pose):
                stand_still += 1
            else:
                stand_still = 0
                old_pose = utils.get_own_pose()
            if (rospy.Time.now() - start).to_sec() > self.TIME_OUT:
                move_action_server.cancel_all_goals()
                return 'failed'
            if get_received_location([]) is not None:
                move_action_server.cancel_all_goals()
                return 'success'
            if move_action_server.get_state() == GoalStatus.SUCCEEDED:
                move_action_server.cancel_all_goals()
                return 'success'
            if move_action_server.get_state() == GoalStatus.PREEMPTED:
                move_action_server.cancel_all_goals()
                if self.retry < 2 * MAX_RETRY:
                    self.retry += 1
                    move_action_server.send_goal(goal)
                else:
                    move_action_server.cancel_all_goals()
                    robot_is_foraging = False
                    return 'failed'
            # only loginfo when robot moved
            pose = utils.get_own_pose()
            if (prev_xpos != pose.pose.position.x) or (prev_ypos != pose.pose.position.y) or (prev_zpos != pose.pose.position.z):
                prev_xpos = pose.pose.position.x
                prev_ypos = pose.pose.position.y
                prev_zpos = pose.pose.position.z
            #rospy.loginfo("%s -> %s", rob_debug(), type(self).__name__)
            log_publisher.publish("%s -> %s" % (str(rob_debug()),type(self).__name__))
            rate.sleep()


class MoveToFoodLocation(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['failed', 'success'], input_keys=['pose_in'], output_keys=['pose_out'])
        self.forget_food = rospy.ServiceProxy('forget_location', ForgetLocation)
        self.TIME_OUT = 3.0

    def execute(self, userdata):
        global move_action_server
        global robot_is_foraging
        global prev_xpos
        global prev_ypos
        global prev_zpos

        target = get_food()

        if target is None and userdata.pose_in is not None:
            target = userdata.pose_in
            userdata.pose_out = target

        if target is None:
            try:
                self.forget_food(location="")
            except:
                print "forget_food failed"
            print "target is None"
            robot_is_foraging = False
            return 'failed'
        goal = utils.create_goal_message(utils.move_location_inwards(target, INWARDS, offset=offset))
        move_action_server.send_goal(goal)

        rate = rospy.Rate(RATE)

        start = rospy.Time.now()

        self.retry = 0
        stand_still = 0
        old_pose = utils.get_own_pose()

        while not rospy.is_shutdown():
            if stand_still > STAND_STILL_TIMES:
                move_action_server.cancel_all_goals()
                move_action_server = actionlib.SimpleActionClient('move_to_goal', MoveBaseAction)
                move_action_server.wait_for_server()

                if at_food():
                    print "standing still too long, but close to food"
                    return 'success'
                else:
                    print "standing still too long, failed to go to food"

                    try:
                        self.forget_food(location="")
                    except:
                        print "forget_food failed"
                    robot_is_foraging = False
                    return 'failed'
            rec_pose = get_received_location([])
            if rec_pose is not None:
                move_action_server.cancel_all_goals()
                goal = utils.create_goal_message(utils.move_location_inwards(rec_pose, INWARDS, offset=offset))
                move_action_server.send_goal(goal)
                rate.sleep()
                continue

            if (rospy.Time.now() - start).to_sec() > self.TIME_OUT:
                move_action_server.cancel_all_goals()
                try:
                    self.forget_food()
                except:
                    pass
                robot_is_foraging = False
                return 'failed'

            if utils.standing_still(old_pose):
                stand_still += 1
            else:
                stand_still = 0
                old_pose = utils.get_own_pose()
            pose = get_food()
            if pose is not None and (utils.dist_vec(pose.pose.position, target.pose.position) > EPS_TARGETS):
                print "resending goal"
                move_action_server.cancel_all_goals()
                target = pose
                goal = utils.create_goal_message(utils.move_location_inwards(target, INWARDS, offset=offset))
                move_action_server.send_goal(goal)

            if move_action_server.get_state() == GoalStatus.SUCCEEDED:
                move_action_server.cancel_all_goals()
                return 'success'
            if move_action_server.get_state() == GoalStatus.PREEMPTED:
                move_action_server.cancel_all_goals()
                if self.retry < MAX_RETRY:
                    self.retry += 1
                    move_action_server.send_goal(goal)
                else:
                    try:
                        self.forget_food(location="")
                    except:
                        print "forget_food failed"
                    robot_is_foraging = False
                    return 'failed'
            # only loginfo when robot moved
            pose = utils.get_own_pose()
            if (prev_xpos != pose.pose.position.x) or (prev_ypos != pose.pose.position.y) or (prev_zpos != pose.pose.position.z):
                prev_xpos = pose.pose.position.x
                prev_ypos = pose.pose.position.y
                prev_zpos = pose.pose.position.z
            #rospy.loginfo("%s -> %s", rob_debug(), type(self).__name__)
            log_publisher.publish("%s -> %s" % (str(rob_debug()),type(self).__name__)) 
            rate.sleep()


def start_srv_cb(req):
    global started
    started = True
    return EmptyResponse()


def main():
    rospy.init_node('swarming_turtles_machine')
    init_globals()
 
    global log_publisher

    log_publisher = rospy.Publisher("/logging", String, queue_size=10)
 
    # create a SMACH state machine
    sm = smach.StateMachine(outcomes=['end'])
    sm.userdata.pose = None
    with sm:
        # Hive states
        smach.StateMachine.add("InitHive", InitHive(), transitions={'success': 'GoToHiveOut'})
        smach.StateMachine.add("SearchHive", SearchHive(), transitions={'success': 'GoToHiveIn'})

        smach.StateMachine.add("GoToHiveIn", MoveToInLocation('hive'),
                               transitions={'failed': 'GoToHive', 'success': 'GoToHive'})
        smach.StateMachine.add("GoToHive", MoveToHiveLocation(),
                               transitions={'failed': 'SearchHive', 'success': 'AtHive'})

        smach.StateMachine.add("AtHive", CheckIfAtLocation('hive'),
                               transitions={'failed': 'SearchHive', 'success': 'GoToHiveOut'})
        smach.StateMachine.add("GoToHiveOut", MoveToOutLocation('hive'),
                               transitions={'failed': 'PreSearchFood', 'success': 'PreSearchFood'})


        # #food states
        smach.StateMachine.add("PreSearchFood", PreSearchFoodLocation(),
                               transitions={'known': 'GoToFoodIn', 'not_known': 'SearchFood'})
        smach.StateMachine.add("SearchFoodNoAsking", SearchFoodNoAsking(),
                               transitions={'found': 'SearchFood', 'not_found': 'SearchFood'},
                               remapping={'pose_out': 'pose'})
        smach.StateMachine.add("SearchFood", SearchFood(),
                               transitions={'found': 'GoToFoodIn', 'not_found': 'SearchFood'},
                               remapping={'pose_out': 'pose'})
        smach.StateMachine.add("GoToFoodIn", MoveToInLocation('food'),
                               transitions={'failed': 'GoToFood', 'success': 'GoToFood'},
                               remapping={'pose_in': 'pose', 'pose_out': 'pose'})
        smach.StateMachine.add("GoToFood", MoveToFoodLocation(),
                               transitions={'failed': 'SearchFoodNoAsking', 'success': 'AtFood'},
                               remapping={'pose_in': 'pose', 'pose_out': 'pose'})
     
        smach.StateMachine.add("AtFood", CheckIfAtLocation('food'),
                               transitions={'failed': 'SearchFoodNoAsking', 'success': 'GoToFoodOut'},
                               remapping={'pose_in': 'pose'})
        smach.StateMachine.add("GoToFoodOut", MoveToOutLocation('food'),
                               transitions={'failed': 'GoToHiveIn', 'success': 'GoToHiveIn'})






 

    # Create and start the introspection server
    sis = smach_ros.IntrospectionServer('server_name', sm, '/swarming_turtles')
    sis.start()

    rospy.Service('start_smach', Empty, start_srv_cb)
    r = rospy.Rate(10)
    while not rospy.is_shutdown() and not started:
        r.sleep()
    # Execute SMACH plan

    rospy.loginfo("%s -> Starting Experiment", rob_debug())
    log_publisher.publish("%s -> Starting Experiment" % str(rob_debug()))

    outcome = sm.execute()

    rospy.spin()
    sis.stop()


if __name__ == '__main__':
    main()
    

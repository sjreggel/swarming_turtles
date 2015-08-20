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
from std_srvs.srv import Empty
import swarming_turtles_navigation.move_random as utils

turtles = {}
closest = ''

tfListen = None

own_name = ''  # hostname used for communication

hive_loc = None
move_random_stop = None
move_random_start = None
get_food_srv = None
get_hive_srv = None

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

#offset = rospy.get_param('marker_offset', math.pi/2.0)
offset = 0

def init_globals():
    global own_name, hive, hive_loc, move_random_stop, move_random_start, get_food_srv, get_hive_srv, move_action_server
    utils.init_globals()

    move_action_server = actionlib.SimpleActionClient('move_to_goal', MoveBaseAction)
    move_action_server.wait_for_server()

    hive_loc = PoseStamped()
    hive_loc.header.frame_id = hive
    hive_loc.pose.orientation.w = 1
    hive_loc = utils.move_location_inwards(hive_loc, INWARDS, offset=offset)

    get_food_srv = rospy.ServiceProxy('get_location', GetLocation, persistent=True)
    get_hive_srv = rospy.ServiceProxy('get_hive', GetLocation, persistent=True)

    move_random_start = rospy.ServiceProxy('move_random_start', Empty)
    move_random_stop = rospy.ServiceProxy('move_random_stop', Empty)

    own_name = rospy.get_namespace()
    if own_name == "/":
        own_name = gethostname()
    else:
        own_name = own_name.replace('/', '')
    own_name = rospy.get_param('~name', own_name)

    rospy.Subscriber('found_turtles', Turtles, cb_found_turtles)  # which turtles are near?


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


class SearchFood(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['found', 'not_found'], output_keys=['pose_out'])
        self.get_received_location_srv = rospy.ServiceProxy('get_received_location', GetLocation, persistent=True)
        self.ask_food_srv = rospy.ServiceProxy('ask_food', GetLocation, persistent=True)


        # rospy.wait_for_service(self.ask_food_srv)
        # rospy.wait_for_service(self.get_received_location_srv)

    def ask_food(self, name):
        try:
            resp = self.ask_food_srv(location=name)
            return resp.pose
        except:
            rospy.logerr("service call ask food failed")
            self.ask_food_srv.close()
            self.ask_food_srv = rospy.ServiceProxy('ask_food', GetLocation, persistent=True)

            return None

    def get_received_location(self, asked_turtles):
        try:
            resp = self.get_received_location_srv(location='')
            if (resp.res in asked_turtles or resp.res == 'mitro') and (rospy.Time.now() - resp.pose.header.stamp).to_sec() < ASK_TIMEOUT:
                return resp.pose
            else:
                return None
        except:
            rospy.logerr("service call to receive locaiton failed")
            self.get_received_location_srv.close()
            self.get_received_location_srv = rospy.ServiceProxy('get_received_location', GetLocation, persistent=True)

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
        while not found and not rospy.is_shutdown():
            if (rospy.Time.now() - start).to_sec() > SEARCH_TIMEOUT:
                move_random_stop()
                return 'not_found'
            pose = get_food()
            if pose is not None:
                found = True
                break
            pose = self.get_received_location(asked_turtles)
            if pose is not None:
                found = True
                break
            if not closest == '' and closest not in asked_turtles:
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
        smach.State.__init__(self, outcomes=['failed', 'success'], input_keys=['pose_in'])
        self.loc = loc
        self.forget_food = rospy.ServiceProxy('forget_location', ForgetLocation)

    def execute(self, userdata):
        target = None
        found = None
        if self.loc == 'hive':
            target = hive_loc
            found = at_hive
        else:  # food
            target = get_food()

            if target is None and userdata.pose_in is not None:
                target = userdata.pose_in

            if target is None:
                try:
                    self.forget_food()
                except:
                    print "forget_food failed"
                return 'failed'
            found = at_food
            target = utils.move_location_inwards(target, INWARDS, offset=offset)

        ang = utils.get_jaw(target.pose.orientation) + math.pi
        rate = rospy.Rate(RATE)

        while not found() and not rospy.is_shutdown():
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
        while not rospy.is_shutdown():
            if (rospy.Time.now() - start).to_sec() > 2 * SEARCH_TIMEOUT:
                move_random_stop()
                return 'success'

            if seen_hive():
                move_random_stop()
                return 'success'
            rate.sleep()


class InitHive(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['success'])

    def execute(self, userdata):
        rate = rospy.Rate(RATE)
        start = rospy.Time.now()
        twist = Twist()
        twist.angular.z = utils.ROTATION_SPEED
        while not rospy.is_shutdown():
            if seen_hive():
                utils.stop()
                rospy.sleep(1.0)
                return 'success'
            utils.cmd_pub.publish(twist)
            rate.sleep()


class MoveToInLocation(smach.State):
    def __init__(self, loc):
        smach.State.__init__(self, outcomes=['failed', 'success'], input_keys=['pose_in'], output_keys=['pose_out'])
        self.loc = loc

    def execute(self, userdata):
        global move_action_server

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
        old_pose = utils.get_own_pose()

        retry = MAX_RETRY - 2
        while not rospy.is_shutdown():
            if stand_still > STAND_STILL_TIMES:
                move_action_server.cancel_all_goals()
                print "standing still too long for moviing to in location"
                move_action_server = actionlib.SimpleActionClient('move_to_goal', MoveBaseAction)
                move_action_server.wait_for_server()

                return 'failed'
            if utils.standing_still(old_pose):
                stand_still += 1
            else:
                stand_still = 0
                old_pose = utils.get_own_pose()

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
            rate.sleep()


class MoveToOutLocation(smach.State):
    def __init__(self, loc):
        smach.State.__init__(self, outcomes=['failed', 'success'])
        self.loc = loc
        self.TIME_OUT = 3.0

    def execute(self, userdata):
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
            rate.sleep()


class MoveToHiveLocation(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['failed', 'success'])

    def execute(self, userdata):
        global move_action_server

        target = hive_loc
        goal = utils.create_goal_message(target)
        move_action_server.send_goal(goal)
        rate = rospy.Rate(RATE)

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
                    return 'failed'
            rate.sleep()


class MoveToFoodLocation(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['failed', 'success'], input_keys=['pose_in'], output_keys=['pose_out'])
        self.forget_food = rospy.ServiceProxy('forget_location', ForgetLocation)

    def execute(self, userdata):
        global move_action_server

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
            return 'failed'
        goal = utils.create_goal_message(utils.move_location_inwards(target, INWARDS, offset=offset))
        move_action_server.send_goal(goal)

        rate = rospy.Rate(RATE)

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
                    return 'failed'
            rate.sleep()


def main():
    rospy.init_node('swarming_turtles_machine')
    init_globals()
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
        smach.StateMachine.add("SearchFood", SearchFood(),
                               transitions={'found': 'GoToFoodIn', 'not_found': 'SearchFood'},
                               remapping={'pose_out': 'pose'})

        smach.StateMachine.add("GoToFoodIn", MoveToInLocation('food'),
                               transitions={'failed': 'GoToFood', 'success': 'GoToFood'},
                               remapping={'pose_in': 'pose', 'pose_out': 'pose'})
        smach.StateMachine.add("GoToFood", MoveToFoodLocation(),
                               transitions={'failed': 'SearchFood', 'success': 'AtFood'},
                               remapping={'pose_in': 'pose', 'pose_out': 'pose'})
        smach.StateMachine.add("AtFood", CheckIfAtLocation('food'),
                               transitions={'failed': 'SearchFood', 'success': 'GoToFoodOut'},
                               remapping={'pose_in': 'pose'})
        smach.StateMachine.add("GoToFoodOut", MoveToOutLocation('food'),
                               transitions={'failed': 'GoToHiveIn', 'success': 'GoToHiveIn'})


        # smach.StateMachine.add("GoToFood", MoveToLocation('food'), transitions = {'failed':'end', 'success':'end'})


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

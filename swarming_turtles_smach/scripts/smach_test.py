#!/usr/bin/env python

#import roslib; roslib.load_manifest('swarming_turtles_smach')
import rospy
import smach
import smach_ros
import tf
import math
from ar_track_alvar.msg import AlvarMarkers
from geometry_msgs.msg import Twist, PoseStamped
from kobuki_msgs.msg import SensorState

locations = {}
markers = {}

bumper = False
tfListen = None
cmd_pub = None

#config
ROTATION_SPEED = 1
FORWARD_SPEED = 0.3
SEARCH_TIMEOUT = 10
DIST_OFFSET = 0.2

RATE = 30
EPS = 0.1

odom = "/odom"
base_frame = "/base_link"


found = ''


def init_globals():
    global markers, tfListen, cmd_pub
    markers['food'] = [201, 202]
    markers['hive'] = [200, 199]
    tfListen = tf.TransformListener()
    rospy.sleep(1)

    cmd_pub = rospy.Publisher('cmd_vel_mux/input/navi', Twist)
    rospy.Subscriber('ar_pose_marker', AlvarMarkers, cb_ar_marker)
    rospy.Subscriber('/mobile_base/sensors/core', SensorState, cb_sensors)


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
    locations[loc]['pose'] = msg.pose
    locations[loc]['time'] = msg.header.stamp
    #print locations[loc]


def transformPose(pose_in):
    if tfListen.frameExists(base_frame) and tfListen.frameExists(odom):
        time = tfListen.getLatestCommonTime(odom, base_frame)
        pose_in.header.stamp = time
        pose = tfListen.transformPose(odom, pose_in)
        return pose
    return None
    
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


    def rotate_to_goal(self, ang):
        twist = Twist()
        twist.linear.x = 0
        twist.angular.z = self.rotate_side(ang) * ROTATION_SPEED

        rate = rospy.Rate(RATE)
        while not self.rotation_aligned(ang):
            cmd_pub.publish(twist)
        stop()


    def rotate_side(self, ang):
        pose_stamped = PoseStamped()
        pose_stamped.header.stamp = rospy.Time.now()
        pose_stamped.header.frame_id = base_frame
        pose_stamped.pose.orientation.w = 1.0
        own_pose = transformPose(pose_stamped)
        quat = [own_pose.pose.orientation.x, own_pose.pose.orientation.y, own_pose.pose.orientation.z,own_pose.pose.orientation.w]

        r,p,theta = tf.transformations.euler_from_quaternion(quat)

        #print ang - theta
        
        if (ang - theta) < 0:
            return -1
        else:
            return 1

        

    def rotation_aligned(self, ang):
        pose_stamped = PoseStamped()
        pose_stamped.header.stamp = rospy.Time.now()
        pose_stamped.header.frame_id = base_frame
        pose_stamped.pose.orientation.w = 1.0
        own_pose = transformPose(pose_stamped)
        quat = [own_pose.pose.orientation.x, own_pose.pose.orientation.y, own_pose.pose.orientation.z,own_pose.pose.orientation.w]

        r,p,theta = tf.transformations.euler_from_quaternion(quat)

        return  abs(ang - theta) < EPS

        
        
    def drive_to_goal(self, dist):
        twist = Twist()
        twist.linear.x = FORWARD_SPEED
        twist.angular.z = 0
        rate = rospy.Rate(RATE)

        pose_stamped = PoseStamped()
        pose_stamped.header.stamp = rospy.Time.now()
        pose_stamped.header.frame_id = base_frame
        pose_stamped.pose.orientation.w = 1.0

        start_pose = transformPose(pose_stamped)

        
        while not self.dist_achieved(start_pose, dist):
            cmd_pub.publish(twist)
        stop()

    def dist_achieved(self,start_pose, dist):
        pose_stamped = PoseStamped()
        pose_stamped.header.stamp = rospy.Time.now()
        pose_stamped.header.frame_id = base_frame
        pose_stamped.pose.orientation.w = 1.0
        own_pose = transformPose(pose_stamped)
        
        diff_x =  start_pose.pose.position.x - own_pose.pose.position.x
        diff_y = start_pose.pose.position.y - own_pose.pose.position.y
        
        dist_cur = math.sqrt(diff_y*diff_y + diff_x*diff_x)

        return abs(dist_cur - dist) < EPS
        
    def execute(self, userdata):
        pose_stamped = PoseStamped()
        pose_stamped.header.stamp = rospy.Time.now()
        pose_stamped.header.frame_id = base_frame
        pose_stamped.pose.orientation.w = 1.0

        own_pose = transformPose(pose_stamped)
        target = locations[self.loc]['pose']

        diff_x =  target.pose.position.x - own_pose.pose.position.x
        diff_y = target.pose.position.y - own_pose.pose.position.y

        ang = math.atan2(diff_y, diff_x)
        
        quat = [own_pose.pose.orientation.x, own_pose.pose.orientation.y, own_pose.pose.orientation.z,own_pose.pose.orientation.w]

        r,p,theta = tf.transformations.euler_from_quaternion(quat)

        diff_ang = ang - theta

        
        self.rotate_to_goal(ang)
        dist = math.sqrt(diff_y*diff_y + diff_x*diff_x)

        print diff_ang, dist
        self.drive_to_goal(dist - DIST_OFFSET)
        
        return 'success'

    
def main():
    rospy.init_node('swarming_turtles_machine')
    init_globals()
    # create a SMACH state machine
    sm = smach.StateMachine(outcomes=['end'])

    with sm:

        #smach.StateMachine.add("Explore", Explore(), transitions = {'found_hive':'end', 'found_food':'GoToFood'})

        
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

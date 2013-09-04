#!/usr/bin/env python


import rospy
import tf
import math
import copy
from swarming_turtles_msgs.msg import Turtle,Turtles, PositionShare, PositionShares
from geometry_msgs.msg import PoseStamped, Vector3


pub = None
tfListen = None

turtles = {}

RATE = 10
RADIUS = 0.2

MIN_SPEED = 0.1

TIME_THRESHOLD = 1.0 #seconds not seen
DIST_THRESHOLD = 2.5 #m too far away to include

MIN_TIME_DIFF = 0.1

output_frame = '/odom'
base_frame = '/base_link'

updating = False
publishing = False

def rotate_vec_by_angle(v, ang):
    res = Vector3()
    cos_a = math.cos(ang)
    sin_a = math.sin(ang)
    
    res.x = cos_a * v.x - sin_a * v.y
    res.y = cos_a * v.y + sin_a * v.x
    return res

def transform_pose(pose_in):
    if pose_in.header.frame_id == output_frame:
        return pose_in
        
    if tfListen.frameExists(output_frame) and tfListen.frameExists(pose_in.header.frame_id):
        time = tfListen.getLatestCommonTime(pose_in.header.frame_id, output_frame)
        pose_in.header.stamp = time
        #tfListen.waitForTransform(output_frame, pose_in.header.frame_id, time, rospy.Duration(0.2))
        pose = tfListen.transformPose(output_frame, pose_in)
        return pose
    return None

def quat_msg_to_array(quat):
    return [quat.x, quat.y, quat.z, quat.w]

def get_jaw(orientation):
    quat = quat_msg_to_array(orientation)
    r,p,theta = tf.transformations.euler_from_quaternion(quat)
    return theta

def dist_between(a, b):
    return math.sqrt(math.pow(a.position.x - b.position.x, 2) + math.pow(a.position.y - b.position.y, 2))

def cb_found_turtles(msg):
    global updating
    if updating:
        return
    updating = True
    for turtle in msg.turtles:
        update_pose(turtle)
    updating = False
        
def create_own_pose():
    own_pose = PoseStamped()
    own_pose.header.stamp = rospy.Time.now()
    own_pose.header.frame_id = base_frame
    own_pose.pose.orientation.w = 1.0
    transform = transform_pose(own_pose)
    return transform

def publish_poses():
    global turtles, publishing
    if publishing:
        return
    publishing = True
    pose_array = PositionShares()
    own_pose = create_own_pose()
    delete_keys = []
    for t in turtles:
        msg = create_pose_msg(own_pose, turtles[t])
        if msg is not None:
            pose_array.positions.append(msg)
        else:
            delete_keys.append(t)
    for t in delete_keys:
        del turtles[t]
    pub.publish(pose_array)
    publishing = False

def create_pose_msg(own_pose, t):
    if (rospy.Time.now() - t['stamps'][-1]).to_sec() > TIME_THRESHOLD:
        return None
    if dist_between(own_pose.pose, t['poses'][-1]) > DIST_THRESHOLD:
        return None
    msg = PositionShare()

    msg.frame_id = output_frame
    msg.name = t['name']
    msg.radius = RADIUS
    msg.holo_robot = False
    msg.controlled = True
    p, v = predict_pos_vel(t['poses'], t['stamps'])

    msg.position = p
    msg.velocity = v

    return msg
    
def predict_pos_vel(poses, times, steps = 5):
    if len(poses) == 1:
        pos = Vector3()
        pos.x = poses[-1].position.x
        pos.y = poses[-1].position.y
        vel = Vector3()
        vel.x = MIN_SPEED

        ang = get_jaw(poses[-1].orientation)
                
        return pos, rotate_vec_by_angle(vel, ang)
    
    if len(poses)<steps:
        steps = len(poses)

    avg_lin_speed = 0
    avg_rot_speed = 0
    for i in xrange(steps-1,0,-1):
        cur = poses[-i]
        prev = poses[-(i+1)]

        time_dif = (times[-i] - times[-(i+1)]).to_sec()

        #print times
        avg_lin_speed += 1./(steps-1) * dist_between(cur, prev)/time_dif
        avg_rot_speed += 1./(steps-1) * abs(get_jaw(cur.orientation) - get_jaw(prev.orientation))/time_dif

    #print avg_lin_speed, avg_rot_speed
    time_to_now = (rospy.Time.now() - times[-1]).to_sec()

    jaw = get_jaw(poses[-1].orientation)
    dif_ang = time_to_now * avg_rot_speed
    dif_x = avg_lin_speed * math.cos(dif_ang / 2.0)
    dif_y = avg_lin_speed * math.sin(dif_ang / 2.0)

    vel = Vector3()
    vel.x = dif_x
    vel.y = dif_y

    vel = rotate_vec_by_angle(vel, jaw + dif_ang)

    pos = Vector3()
    pos.x = poses[-1].position.x + vel.x
    pos.y = poses[-1].position.y + vel.y

    return pos, vel
    
        
def update_pose(turtle):
    global turtles
    name = turtle.name

    output_pose = transform_pose(turtle.position)
    if output_pose is None:
        return

    if not name in turtles.keys():
        turtles[name] = {}
        turtles[name]['name'] = name
        turtles[name]['poses'] = []
        turtles[name]['stamps'] = []


    if len(turtles[name]['stamps']) > 1 and (turtle.position.header.stamp - turtles[name]['stamps'][-1]).to_sec() < MIN_TIME_DIFF:
        return

    turtles[name]['poses'].append(output_pose.pose)
    turtles[name]['stamps'].append(turtle.position.header.stamp)
        

def main():
    global pub, tfListen
    rospy.init_node("position_share")
    tfListen = tf.TransformListener()
    rospy.sleep(1)
    
    rospy.Subscriber("found_turtles", Turtles, cb_found_turtles)
    pub = rospy.Publisher("position_share", PositionShares)
    rate = rospy.Rate(RATE)
    while not rospy.is_shutdown():
        publish_poses()
        rate.sleep()

if __name__ == '__main__':
    main()



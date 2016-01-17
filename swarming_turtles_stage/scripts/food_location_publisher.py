#!/usr/bin/env python
import math
import rospy
import tf
from geometry_msgs.msg import PoseStamped, Vector3
from nav_msgs.msg import Odometry
__author__ = 'danielclaes'

HIVE_TOPIC = '/hive/base_pose_ground_truth'
FOOD_TOPICS = ['/food/base_pose_ground_truth']

WORLD_FRAME = '/world'
HIVE_FRAME = rospy.get_param('hive_frame', '/hive')
BASE_FRAME = rospy.get_param('base_frame', '/base_link')

RATE = 10

transform = {}  # transform of world to hive

own_name = 'food'

prev_xpos = 0
prev_ypos = 0
prev_zpos = 0

def rob_debug(pos):
    global own_name
    x = format(pos.pose.position.x,'.3f')
    y = format(pos.pose.position.y,'.3f')
    z = format(pos.pose.orientation.z,'.3f')
    w = format(pos.pose.orientation.w,'.3f')
    return own_name, x, y, z, w, False, 0, False, False


def quat_msg_to_array(quat):
    return [quat.x, quat.y, quat.z, quat.w]


class FoodPublisher(object):
    def __init__(self):
        self.tf = tf.TransformListener()
        self.food_pub = rospy.Publisher('/food_location', PoseStamped, queue_size=10)
        self.hive_location = PoseStamped()
        rospy.Subscriber(HIVE_TOPIC, Odometry, self.hive_cb)
        for idx, food_topic in enumerate(FOOD_TOPICS):
            rospy.Subscriber(food_topic, Odometry, self.food_cb, idx)

    def food_cb(self, msg, idx):
	global prev_xpos
	global prev_ypos
	global prev_zpos
        p = PoseStamped()
        p.pose = msg.pose.pose
        p.header = msg.header
        p = self.transform_pose(p, HIVE_FRAME, time_in=msg.header.stamp)
        if p is not None:
	    # only loginfo when food moved
	    if (prev_xpos != p.pose.position.x) or (prev_ypos != p.pose.position.y) or (prev_zpos != p.pose.position.z):
	      prev_xpos = p.pose.position.x
	      prev_ypos = p.pose.position.y
	      prev_zpos = p.pose.position.z
	      rospy.loginfo("%s -> Food Position", rob_debug(p))
              # Here you could also do:
              # rospy.loginfo("Food Position %.2f, %.2f", p.pose.position.x, p.pose.position.y)
	    self.food_pub.publish(p)

    def transform_pose(self, pose_in, frame, time_in=None):
        if self.tf.frameExists(pose_in.header.frame_id) and self.tf.frameExists(frame):
            if time_in is None:
                pose_in.header.stamp = rospy.Time.now()
            else:
                pose_in.header.stamp = time_in
            try:
                self.tf.waitForTransform(pose_in.header.frame_id, frame, pose_in.header.stamp, rospy.Duration(0.2))
                pose = self.tf.transformPose(frame, pose_in)
                return pose
            except tf.Exception as e:
                rospy.logwarn("transform failed: %s", e)
                return None
        return None

    def hive_cb(self, msg):
        global transform
        self.hive_location.pose = msg.pose.pose
        self.hive_location.header = msg.header
        transform['pose'] = (self.hive_location.pose.position.x, self.hive_location.pose.position.y, 0)
        transform['quat'] = tuple(quat_msg_to_array(self.hive_location.pose.orientation))
        transform['stamp'] = rospy.Time.now()


def main():
    global transform

    rospy.init_node('fake_food_hive_detection')
    food_pub = FoodPublisher()

    tf_br = tf.TransformBroadcaster()
    transform['pose'] = (0, 0, 0)
    transform['quat'] = (0, 0, 0, 1)
    transform['stamp'] = rospy.Time.now()

    r = rospy.Rate(RATE)
    while not rospy.is_shutdown():
        tf_br.sendTransform(transform['pose'],
                            transform['quat'],
                            rospy.Time.now(),
                            HIVE_FRAME,
                            WORLD_FRAME)
        r.sleep()


if __name__ == '__main__':
    main()

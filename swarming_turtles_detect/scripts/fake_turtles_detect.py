#!/usr/bin/env python
__author__ = 'danielclaes'
import rospy
import tf
from geometry_msgs.msg import PoseStamped
from swarming_turtles_msgs.msg import Turtles, Turtle

#RATE = 20
RATE = 10

MAX_NUM_TURTLES = 20  # TODO Make param

TARGET_FRAME = rospy.get_param('hive_frame', '/hive')
own_name = ''


def quat_msg_to_array(quat):
    return [quat.x, quat.y, quat.z, quat.w]


class FakeTurtlesDetect(object):
    def __init__(self):
        self.tf = tf.TransformListener()
        self.turtles_pub = rospy.Publisher('found_turtles', Turtles, queue_size=1)

    def publish_turtles(self):
        turtles_msg = Turtles()
        time = rospy.Time.now()
        for i in range(MAX_NUM_TURTLES):
            frame = "/robot_%d/base_footprint" % i
            if not self.tf.frameExists(frame):
                # rospy.logwarn("turtle frame not found %s", frame)
                break
            turtle = Turtle()
            turtle.name = 'robot_%d' % i
            if own_name == turtle.name:
                continue
            turtle.position = self.get_turtle_pose(frame, time)
            if turtle.position is not None:
                turtles_msg.turtles.append(turtle)
        self.turtles_pub.publish(turtles_msg)

    # helpers
    def transform_pose(self, pose_in, frame, time_in=None):
        if self.tf.frameExists(pose_in.header.frame_id) and self.tf.frameExists(frame):
            #
            if time_in is None:
                pose_in.header.stamp = rospy.Time.now()
            else:
                pose_in.header.stamp = time_in
                # print pose_in.header.stamp
            try:
                self.tf.waitForTransform(pose_in.header.frame_id, frame, pose_in.header.stamp, rospy.Duration(0.2))
                pose = self.tf.transformPose(frame, pose_in)
                return pose
            except tf.Exception as e:
                rospy.logwarn("transform in Fake turtles detect failed: %s", e)
                return None
        return None

    def get_turtle_pose(self, frame, time=None):
        pose_stamped = PoseStamped()
        if time is not None:
            pose_stamped.header.stamp = time
        else:
            pose_stamped.header.stamp = rospy.Time.now()
        pose_stamped.header.frame_id = frame
        pose_stamped.pose.orientation.w = 1.0
        return self.transform_pose(pose_stamped, TARGET_FRAME, time)


def main():
    global own_name
    rospy.init_node('fake_food_hive_detection')
    fake_turtles_detect = FakeTurtlesDetect()

    own_name = rospy.get_namespace()
    own_name = own_name.replace('/', '')

    r = rospy.Rate(RATE)
    while not rospy.is_shutdown():
        fake_turtles_detect.publish_turtles()
        r.sleep()


if __name__ == '__main__':
    main()

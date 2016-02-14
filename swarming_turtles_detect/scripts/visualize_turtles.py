#!/usr/bin/env python
import rospy
import tf
import math
import copy
from swarming_turtles_msgs.msg import Turtle, Turtles
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker, MarkerArray

pub = None
pub_me = None

RADIUS = 0.2

BASE_FRAME = rospy.get_param('base_frame', '/base_link')


def cb_found_turtles(msg):
    marker_array = MarkerArray()
    for turtle in msg.turtles:
        add_markers(marker_array, turtle)
    pub.publish(marker_array)
    publish_me()


def add_markers(marker_array, turtle, me=False):
    marker = Marker()
    marker.header.frame_id = turtle.position.header.frame_id
    marker.header.stamp = turtle.position.header.stamp
    marker.ns = BASE_FRAME
    marker.action = Marker.ADD
    marker.type = Marker.SPHERE
    marker.scale.x = 2 * RADIUS
    marker.scale.y = 2 * RADIUS
    marker.scale.z = 0.1
    if me:
        marker.color.b = 1.0
    else:
        marker.color.r = 1.0
    marker.color.a = 1.0
    marker.id = len(marker_array.markers)
    marker.lifetime = rospy.Duration(1)
    marker.pose = turtle.position.pose
    marker_array.markers.append(marker)

    marker = Marker()
    marker.header.frame_id = turtle.position.header.frame_id
    marker.header.stamp = turtle.position.header.stamp
    marker.ns = BASE_FRAME
    marker.action = Marker.ADD
    marker.type = Marker.TEXT_VIEW_FACING
    marker.text = turtle.name
    marker.scale.z = 0.3
    marker.color.r = 1.0
    marker.color.g = 1.0
    marker.color.b = 1.0
    marker.color.a = 1.0
    marker.id = len(marker_array.markers)
    marker.lifetime = rospy.Duration(1)
    marker.pose = copy.deepcopy(turtle.position.pose)

    marker.pose.position.z = 0.2

    marker_array.markers.append(marker)

    marker = Marker()
    marker.lifetime = rospy.Duration(1)

    marker.header.frame_id = turtle.position.header.frame_id
    marker.header.stamp = turtle.position.header.stamp
    marker.ns = BASE_FRAME
    marker.action = Marker.ADD
    marker.type = Marker.ARROW
    marker.scale.x = 0.1
    marker.scale.y = 0.2
    marker.scale.z = 0.1
    if me:
        marker.color.b = 1.0
    else:
        marker.color.r = 1.0
    marker.color.a = 1.0
    marker.id = len(marker_array.markers)
    quat = turtle.position.pose.orientation
    q = [quat.x, quat.y, quat.z, quat.w]
    r, p, theta = tf.transformations.euler_from_quaternion(q)

    p = Point()
    p = turtle.position.pose.position

    marker.points.append(p)

    p = Point()
    p.x = turtle.position.pose.position.x + 2 * RADIUS * math.cos(theta)
    p.y = turtle.position.pose.position.y + 2 * RADIUS * math.sin(theta)
    marker.points.append(p)

    marker_array.markers.append(marker)


def publish_me():
    turtle = Turtle()
    turtle.name = "me"
    turtle.position.header.frame_id = BASE_FRAME
    turtle.position.header.stamp = rospy.Time.now()
    turtle.position.pose.orientation.w = 1.0

    marker_array = MarkerArray()

    add_markers(marker_array, turtle, me=True)

    pub_me.publish(marker_array)


def main():
    global pub, pub_me
    rospy.init_node("visualize_turtles")
    pub = rospy.Publisher("neighbors", MarkerArray, queue_size=1)
    pub_me = rospy.Publisher("me", MarkerArray, queue_size=1)
    rospy.sleep(0.1)
    rospy.Subscriber("found_turtles", Turtles, cb_found_turtles)

    rospy.spin()


if __name__ == '__main__':
    main()

#!/usr/bin/env python
import rospy
import actionlib
import tf
from actionlib_msgs.msg import *
from move_base_msgs.msg import *
from geometry_msgs.msg import *


def create_goal_message(goal):
    goal_msg = MoveBaseGoal()

    
    #goal_msg.target_pose.pose.position.x = goal["x"]
    #goal_msg.target_pose.pose.position.y = goal["y"]
    #if 'theta' in goal.keys():
    #    q = tf.transformations.quaternion_from_euler(0,0, goal["theta"], axes='sxyz')
  #      goal_msg.target_pose.pose.orientation.x = q[0]
   #     goal_msg.target_pose.pose.orientation.y = q[1]
   #     goal_msg.target_pose.pose.orientation.z = q[2]
    #    goal_msg.target_pose.pose.orientation.w = q[3]
    #else:
     #   rospy.logwarn("no theta set, defaulting to 0")
     #   goal_msg.target_pose.pose.orientation.w = 1.0
    goal_msg.target_pose.pose = goal.pose

    goal_msg.target_pose.header.frame_id = "/base_link"
    goal_msg.target_pose.header.stamp = rospy.Time.now()
    return goal_msg


def main():
    rospy.init_node("test_action")
    client = actionlib.SimpleActionClient('SwarmCollvoid/swarm_nav_goal', MoveBaseAction)
    client.wait_for_server()

    goal = PoseStamped()

    theta = 0
    
    q = tf.transformations.quaternion_from_euler(0,0, theta, axes='sxyz')
    goal.pose.position.x = 1.0

    quat = Quaternion(*q)
    goal.pose.orientation = quat

    goal_msg = create_goal_message(goal)

    client.send_goal(goal_msg)
    client.wait_for_result(rospy.Duration(5.0))


if __name__ == '__main__':
    main()

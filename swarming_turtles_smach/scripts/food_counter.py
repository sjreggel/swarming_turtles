#!/usr/bin/env python
import rospy
from std_msgs.msg import Int32

NUM_ROBOTS = 10
food_counts = [0] * NUM_ROBOTS
total_food = 0


def rob_cb(data, idx):
    global food_counts, total_food
    food_counts[idx-1] = data.data
    total_food = sum(food_counts)
    print "rob%d_food = %d total = %d" % (idx, food_counts[idx-1], total_food)


def listener():
    # In ROS, nodes are uniquely named. If two nodes with the same
    # node are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.

    rospy.init_node('listener', anonymous=True)

    for i in range(1, NUM_ROBOTS+1):
        rospy.Subscriber("/robot_%d/fooddrops" % i, Int32, rob_cb, i)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()


if __name__ == '__main__':
    listener()
    print "alles", food_counts

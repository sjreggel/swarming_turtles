#!/usr/bin/env python
import rospy
from std_srvs.srv import Empty, EmptyResponse
__author__ = 'daniel'

MAX_NUM_TURTLES = 10

services = []


def start_all(req):
    for s in services:
        s()
    return EmptyResponse()


if __name__ == '__main__':
    rospy.init_node('start_smach_helper')
    for i in xrange(1, MAX_NUM_TURTLES):
        services.append(rospy.ServiceProxy("/robot_%d/start_smach" % i, Empty))
    rospy.Service('/start_all', Empty, start_all)
    rospy.spin()

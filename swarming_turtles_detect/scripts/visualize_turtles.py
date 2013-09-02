#!/usr/bin/env python


import rospy

from swarming_turtles_msgs.msg import Turtle,Turtles
from visualization_msgs.msg import Marker, MarkerArray



def main():
    rospy.init_node("visualize_turtles")
    

    rospy.spin()



if __name__ == '__main__':
    main()



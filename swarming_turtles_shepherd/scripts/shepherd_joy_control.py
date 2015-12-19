#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist
from std_srvs.srv import Empty
from swarming_turtles_shepherd.srv import SetShepherdingStatus, SetShepherdingStatusRequest, SetShepherdingStatusResponse

__author__ = 'daniel'

SELECT = 0
START = 3

UP_BUTTON = 4
LEFT_BUTTON = 7

TRIANGLE_BUTTON = 12
CIRCLE_BUTTON = 13
CROSS_BUTTON = 14
SQUARE_BUTTON = 15

SHEPHERDING_TOGGLE = CROSS_BUTTON


BUTTONS_USED = [START, CROSS_BUTTON]

class ShepherdJoyControl(object):
    def __init__(self):
        self.buttons_pressed_before = [False] * len(BUTTONS_USED)

        self.last_joy_msg = rospy.Time.now()
        rospy.Subscriber('joy', Joy, self.joy_cb)
        #rospy.Subscriber('cmd_vel', Twist, self.twist_cb)
        self.set_shepherding_status = rospy.ServiceProxy('/shepherd/set_shepherding_status', SetShepherdingStatus)
	self.start_all_robots = rospy.ServiceProxy('/start_all', Empty)


    def joy_cb(self, msg):
        # Handle pressed buttons
       for idx, button in enumerate(BUTTONS_USED):
            if msg.buttons[button]:
		if self.buttons_pressed_before[idx] == False:
                 if button == SHEPHERDING_TOGGLE:
                    self.toggle_shepherding(active=True)
		 if button == START:
		    self.start_robots(active=True)
                 self.buttons_pressed_before[idx] = True
            elif self.buttons_pressed_before[idx] == True:
                if button == SHEPHERDING_TOGGLE:
                    self.toggle_shepherding(active=False)
		if button == START:
		    self.start_robots(active=False)
                self.buttons_pressed_before[idx] = False


    def start_robots(self, active):
	try: 
	    res = self.start_all_robots()
	    print "starting robots"
        except rospy.ServiceException as e:
            print "Service call failed: %s" % e


    def toggle_shepherding(self, active):
        req = SetShepherdingStatusRequest()
        req.target_status = active
        try:
            res = self.set_shepherding_status(req)
            print res
        except rospy.ServiceException as e:
            print "Service call failed: %s" % e


            
if __name__ == '__main__':
    rospy.init_node('SetShepherdingStatus')
    ShepherdJoyControl()
    rospy.spin()

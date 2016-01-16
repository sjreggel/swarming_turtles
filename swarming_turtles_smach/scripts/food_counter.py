#!/usr/bin/env python
import rospy
from std_msgs.msg import Int32

total_food = 0
rob1_food = 0
rob2_food = 0
rob3_food = 0
rob4_food = 0
rob5_food = 0
rob6_food = 0
rob7_food = 0
rob8_food = 0
rob9_food = 0
rob10_food = 0


def rob1_cb(data):
    global rob1_food, rob2_food, rob3_food, rob4_food, rob5_food, rob6_food, rob7_food, rob8_food, rob9_food, rob10_food
    rob1_food = data.data
    total_food = rob1_food + rob2_food + rob3_food + rob4_food + rob5_food + rob6_food + rob7_food + rob8_food + rob9_food + rob10_food
    print "rob1_food =",rob1_food, "total =", total_food

def rob2_cb(data):
    global rob1_food, rob2_food, rob3_food, rob4_food, rob5_food, rob6_food, rob7_food, rob8_food, rob9_food, rob10_food
    rob2_food = data.data
    total_food = rob1_food + rob2_food + rob3_food + rob4_food + rob5_food + rob6_food + rob7_food + rob8_food + rob9_food + rob10_food
    print "rob2_food =",rob2_food, "total =", total_food

    
def rob3_cb(data):
    global rob1_food, rob2_food, rob3_food, rob4_food, rob5_food, rob6_food, rob7_food, rob8_food, rob9_food, rob10_food
    rob3_food = data.data
    total_food = rob1_food + rob2_food + rob3_food + rob4_food + rob5_food + rob6_food + rob7_food + rob8_food + rob9_food + rob10_food
    print "rob3_food =",rob3_food, "total =", total_food
    
def rob4_cb(data):
    global rob1_food, rob2_food, rob3_food, rob4_food, rob5_food, rob6_food, rob7_food, rob8_food, rob9_food, rob10_food
    rob4_food = data.data
    total_food = rob1_food + rob2_food + rob3_food + rob4_food + rob5_food + rob6_food + rob7_food + rob8_food + rob9_food + rob10_food
    print "rob4_food =",rob4_food, "total =", total_food
    
def rob5_cb(data):
    global rob1_food, rob2_food, rob3_food, rob4_food, rob5_food, rob6_food, rob7_food, rob8_food, rob9_food, rob10_food
    rob5_food = data.data
    total_food = rob1_food + rob2_food + rob3_food + rob4_food + rob5_food + rob6_food + rob7_food + rob8_food + rob9_food + rob10_food
    print "rob5_food =",rob5_food, "total =", total_food
    
def rob6_cb(data):
    global rob1_food, rob2_food, rob3_food, rob4_food, rob5_food, rob6_food, rob7_food, rob8_food, rob9_food, rob10_food
    rob6_food = data.data
    total_food = rob1_food + rob2_food + rob3_food + rob4_food + rob5_food + rob6_food + rob7_food + rob8_food + rob9_food + rob10_food
    print "rob6_food =",rob6_food, "total =", total_food
    
def rob7_cb(data):
    global rob1_food, rob2_food, rob3_food, rob4_food, rob5_food, rob6_food, rob7_food, rob8_food, rob9_food, rob10_food
    rob7_food = data.data
    total_food = rob1_food + rob2_food + rob3_food + rob4_food + rob5_food + rob6_food + rob7_food + rob8_food + rob9_food + rob10_food
    print "rob7_food =",rob7_food, "total =", total_food
    
def rob8_cb(data):
    global rob1_food, rob2_food, rob3_food, rob4_food, rob5_food, rob6_food, rob7_food, rob8_food, rob9_food, rob10_food
    rob8_food = data.data
    total_food = rob1_food + rob2_food + rob3_food + rob4_food + rob5_food + rob6_food + rob7_food + rob8_food + rob9_food + rob10_food
    print "rob8_food =",rob8_food, "total =", total_food
    
def rob9_cb(data):
    global rob1_food, rob2_food, rob3_food, rob4_food, rob5_food, rob6_food, rob7_food, rob8_food, rob9_food, rob10_food
    rob9_food = data.data
    total_food = rob1_food + rob2_food + rob3_food + rob4_food + rob5_food + rob6_food + rob7_food + rob8_food + rob9_food + rob10_food
    print "rob9_food =",rob9_food, "total =", total_food
    
def rob10_cb(data):
    global rob1_food, rob2_food, rob3_food, rob4_food, rob5_food, rob6_food, rob7_food, rob8_food, rob9_food, rob10_food
    rob10_food = data.data
    total_food = rob1_food + rob2_food + rob3_food + rob4_food + rob5_food + rob6_food + rob7_food + rob8_food + rob9_food + rob10_food
    print "rob10_food =",rob10_food, "total =", total_food
    


    
def listener():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # node are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('listener', anonymous=True)

    rospy.Subscriber("/robot_1/fooddrops", Int32, rob1_cb)
    rospy.Subscriber("/robot_2/fooddrops", Int32, rob2_cb)
    rospy.Subscriber("/robot_3/fooddrops", Int32, rob3_cb)
    rospy.Subscriber("/robot_4/fooddrops", Int32, rob4_cb)
    rospy.Subscriber("/robot_5/fooddrops", Int32, rob5_cb)
    rospy.Subscriber("/robot_6/fooddrops", Int32, rob6_cb)
    rospy.Subscriber("/robot_7/fooddrops", Int32, rob7_cb)
    rospy.Subscriber("/robot_8/fooddrops", Int32, rob8_cb)
    rospy.Subscriber("/robot_9/fooddrops", Int32, rob9_cb)
    rospy.Subscriber("/robot_10/fooddrops", Int32, rob10_cb)
   
    
    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()
    print "alles", rob1_food


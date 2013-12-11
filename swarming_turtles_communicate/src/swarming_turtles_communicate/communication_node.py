#!/usr/bin/env python
import rospy
from swarming_turtles_msgs.msg import CommunicationProtocol
from swarming_turtles_detect.srv import *
from socket import gethostname


topic = '/communication'
MAX_TIME = 1.0
location_received = {} 
name = ''
comm_pub = ''

def cb_communication(msg):
    global received, received_msg, location_received
    if not msg.receiver == name:
        return
    req = msg.request.split(' ')
    if "request" == req[0]: #handle reqest
        if req[1] == 'food':
            pose = get_food()
            if pose is None:
                return
        else: #hive asked do nothing yet hive in baseframe
            return
        answer(msg.sender, req[1], pose)
            
    elif "answer" == req[0]:
        print "GOT ANSWER", req[1]
        process_msg(msg)

def process_msg(msg):
    global location_received
    #if msg.sender not in turtles.keys():
    #    return False
    #turtle = turtles[msg.sender]
    #if (rospy.Time.now() - turtle.header.stamp).to_sec() > LAST_SEEN:
    #    print 'message too old'
    #    return False
    location_received['from'] = msg.sender
    location_received['pose'] = msg.location 

def get_food():
    try:
        resp = get_food_srv()
        return resp.pose
    except:
        print "service call failed"
        return None

def answer(receiver, loc_name, pose):
    msg = CommunicationProtocol()
    msg.sender = name
    msg.receiver = receiver
    msg.request = "answer %s"%(loc_name)
    msg.location = pose
    msg.location.header.stamp = rospy.Time.now()
    comm_pub.publish(msg)


def get_received_location(req):
    res = GetLocationResponse()
    if location_received is not None and 'from' in location_received.keys():
        res.res = location_received['from']
        res.pose = location_received['pose']
    return res
    

def ask_hive(req):
    request(req.location, 'hive')
    return GetLocationResponse()
    
def ask_food(req):
    request(req.location, 'food')
    return GetLocationResponse()
    
def request(receiver, loc_name):
    msg = CommunicationProtocol()
    msg.sender = name
    msg.receiver = receiver
    msg.request = "request %s"%(loc_name)
    #msg.location = get_own_pose()

    comm_pub.publish(msg)


def main():
    global get_food_srv, name, comm_pub
    name = gethostname()
    rospy.init_node("communicate_node")
    rospy.Subscriber(topic, CommunicationProtocol, cb_communication)
    get_food_srv = rospy.ServiceProxy('get_location', GetLocation)

    comm_pub = rospy.Publisher(topic, CommunicationProtocol)

    ask_food_srv = rospy.Service('ask_food', GetLocation, ask_food)
    ask_hive_srv = rospy.Service('ask_hive', GetLocation, ask_hive)

    
    received_loc_srv = rospy.Service('get_received_location', GetLocation, get_received_location)

    rospy.wait_for_service(get_food_srv)
    rospy.spin()
    

if __name__ == "__main__":
    main()

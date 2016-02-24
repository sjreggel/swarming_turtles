#!/usr/bin/env python
import rospy
from swarming_turtles_msgs.msg import CommunicationProtocol
from swarming_turtles_detect.srv import *
from socket import gethostname

topic = '/communication'
MAX_TIME = 1.0
location_received = {}
own_name = ''
comm_pub = None
get_food_srv = None
forget_food_srv = None
set_hive_srv = None
get_forget_food_time_srv = None

def cb_communication(msg):
    assert isinstance(msg, CommunicationProtocol)
    if not msg.receiver == own_name:
        return
    req = msg.request.split(' ')
    if "request" == req[0]:  # handle reqest
        pose = None
        if req[1] == 'food':
            pose = get_food()
            if pose is None:
                return
        else:  # hive asked do nothing yet hive in baseframe
            return
        if pose is not None:
            answer(msg.sender, req[1], pose)
    elif "answer" == req[0]:
        rospy.loginfo("GOT ANSWER %s", req[1])
        process_msg(msg)
    elif "mitro_message" == req[0]:
        print "GOT MESSAGE FROM MITRO"
        process_mitro_msg(msg)


def process_mitro_msg(msg):
    global location_received
    try:
        forget_food_srv()
    except rospy.ServiceException as e:
        print "could not forget food", e

    location_received['from'] = msg.sender
    location_received['pose'] = msg.food_location

    robot_location = msg.robot_location
    try:
        set_hive_srv(robot_location)
    except rospy.ServiceException as e:
        print "could not set Hive", e


def process_msg(msg):
    assert isinstance(msg, CommunicationProtocol)
    global location_received
    # if (rospy.Time.now() - turtle.header.stamp).to_sec() > LAST_SEEN:
    #    print 'message too old'
    #    return False
    # Check if food is newer than last forget time
    if compare_with_last_forget_food(msg.food_location.header.stamp):
        rospy.loginfo("Time is newer then last forget time using received food location")

        location_received['from'] = msg.sender
        location_received['pose'] = msg.food_location
        location_received['pose'].header.stamp = rospy.Time.now()
    else:
        rospy.loginfo("Disregarding last food location, since time is too old")


def compare_with_last_forget_food(time):
    try:
        resp = get_forget_food_time_srv()
        assert isinstance(resp, GetLastForgetTimeResponse)
        if time > resp.forget_time:
            return True
        else:
            return False

    except rospy.ServiceException as e:
        print "Communication service call to get food forget time failed", e

        return True


def get_food():
    try:
        resp = get_food_srv()
        if resp.res == '':
            return None
        return resp.pose
    except rospy.ServiceException as e:
        print "Communication service call to get food location failed", e
        return None


def answer(receiver, loc_name, pose):
    msg = CommunicationProtocol()
    msg.sender = own_name
    msg.receiver = receiver
    msg.request = "answer %s" % (loc_name)
    msg.food_location = pose
    # msg.food_location.header.stamp = rospy.Time.now()
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
    msg.sender = own_name
    msg.receiver = receiver
    msg.request = "request %s" % (loc_name)
    # msg.location = get_own_pose()

    comm_pub.publish(msg)


def main():
    global get_food_srv, forget_food_srv, comm_pub, set_hive_srv, own_name, get_forget_food_time_srv
    rospy.init_node("communicate_node")
    rospy.Subscriber(topic, CommunicationProtocol, cb_communication)

    own_name = rospy.get_namespace()
    if own_name == "/":
        own_name = gethostname()
    else:
        own_name = own_name.replace('/', '')
    own_name = rospy.get_param('~name', own_name)

    get_food_srv = rospy.ServiceProxy('get_location', GetLocation)
    set_hive_srv = rospy.ServiceProxy('set_hive', SetHive)

    forget_food_srv = rospy.ServiceProxy('forget_location', ForgetLocation)
    get_forget_food_time_srv = rospy.ServiceProxy('get_last_forget_time', GetLastForgetTime)

    comm_pub = rospy.Publisher(topic, CommunicationProtocol, queue_size=1)

    rospy.Service('ask_food', GetLocation, ask_food)
    rospy.Service('ask_hive', GetLocation, ask_hive)

    rospy.Service('get_received_location', GetLocation, get_received_location)

    #rospy.wait_for_service(get_food_srv, 2.0)
    rospy.loginfo('Started communcation node with name: %s'%own_name)
    rospy.spin()


if __name__ == "__main__":
    main()

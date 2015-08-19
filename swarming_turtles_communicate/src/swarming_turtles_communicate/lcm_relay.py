#!/usr/bin/env python
import rospy
from swarming_turtles_msgs.msg import CommunicationProtocol

from socket import gethostname

import lcm
import time
import StringIO

#from msg import communication_msg

lc = None
topic = '/communication'
own_name = None
lcm_sub = None
pub = None

def init_globals():
    global lc, own_name, pub, lcm_sub
    lc = lcm.LCM("udpm://224.1.1.1:5007?ttl=2")
    name = gethostname()
    lcm_sub = lc.subscribe(name, udp_callback)
    pub = rospy.Publisher(topic, CommunicationProtocol)
    rospy.Subscriber(topic, CommunicationProtocol, handle_msg)

def udp_callback(channel, data):
    msg = CommunicationProtocol()
    msg.deserialize(data)
    if msg.receiver == own_name:
        pub.publish(msg)
    
def handle_msg(msg):
    if msg.receiver == own_name:
        return
    send(msg)
    
def main():
    rospy.init_node('swarming_turtles_machine')
    init_globals()
    
    while not rospy.is_shutdown():
        lc.handle()
    lc.unsubscribe(lcm_sub)



def send(msg, repeats = 1):
    buff = StringIO.StringIO()
    msg.serialize(buff)
    for i in xrange(repeats):
        lc.publish(msg.receiver, buff.getvalue())
    
    

if __name__ == '__main__':
    main()

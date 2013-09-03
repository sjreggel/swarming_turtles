#!/usr/bin/env python

#import roslib; roslib.load_manifest('swarming_turtles_smach')
import rospy
import smach
import smach_ros
import tf
import math
from socket import gethostname

from swarming_turtles_msgs.msg import Turtles, Turtle, CommunicationProtocol
from swarming_turtles_communicate.communicate  import connect, disconnect, make_master_uri


class Communicate(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['done'])
        self.topic = '/communication'
        self.publisher = rospy.Publisher(self.topic, CommunicationProtocol)

        self.id = gethostname()
        print self.id
       
    def execute(self, userdata):
        found = 'tb05'
        foreign_master_uri = make_master_uri(found)

        try:
            msg = CommunicationProtocol()
            msg.sender = self.id
            msg.receiver = found
            msg.request = 'hive'

            connect(self.topic, foreign_master_uri)
            rospy.sleep(0.2)
            self.publisher.publish(msg)
            disconnect(self.topic, foreign_master_uri)
            
            
        except:
            print "exception"
            disconnect(self.topic, foreign_master_uri)
            
        return 'done'


def main():
    rospy.init_node('swarming_turtles_comm')
    # create a SMACH state machine
    sm = smach.StateMachine(outcomes=['end'])

    with sm:
        smach.StateMachine.add("comm", Communicate(), transitions = {'done':'end'})

        # Create and start the introspection server
    sis = smach_ros.IntrospectionServer('server_name', sm, '/swarming_turtles')
    sis.start()
    rospy.loginfo("starting!")

    # Execute SMACH plan
    outcome = sm.execute()

    rospy.spin()
    sis.stop()

    
if __name__ == '__main__':
    main()

from geometry_msgs.msg import PoseStamped
import tf

__author__ = 'danielclaes'
import rospy
from swarming_turtles_detect.srv import *
from nav_msgs.msg import Odometry

HIVE_TOPIC = '/hive/base_pose_ground_truth'
FOOD_TOPICS = ['/food/base_pose_ground_truth']

WORLD_FRAME = '/world'
HIVE_FRAME = '/hive'

RATE = 20

transform = {}  # transform of world to hive


def quat_msg_to_array(quat):
    return [quat.x, quat.y, quat.z, quat.w]


class FakeFoodHiveDetect(object):
    def __init__(self):
        self.food_locations = {}
        self.hive_location = PoseStamped()
        rospy.Subscriber(HIVE_TOPIC, Odometry, self.hive_cb)
        for idx, food_topic in enumerate(FOOD_TOPICS):
            rospy.Subscriber(food_topic, Odometry, self.food_cb, idx)

        rospy.Service('get_hive', GetLocation, self.get_hive)
        rospy.Service('get_location', GetLocation, self.get_food)
        rospy.Service('forget_location', ForgetLocation, self.forget_food_location)

    def food_cb(self, msg, idx):
        """
        :param msg:
        :type msg: Odometry
        :param idx:
        :type idx: int
        """
        key = "food" + "_" + str(idx)
        self.food_locations[key] = PoseStamped()
        self.food_locations[key].pose = msg.pose.pose
        self.food_locations[key].header = msg.header

    def hive_cb(self, msg):
        global transform
        """
        :param msg:
        :type msg: Odometry
        """
        self.hive_location.pose = msg.pose.pose
        self.hive_location.header = msg.header
        transform['pose'] = (self.hive_location.pose.position.x, self.hive_location.pose.position.y, 0)
        transform['quat'] = tuple(quat_msg_to_array(self.hive_location.pose.orientation))
        transform['stamp'] = rospy.Time.now()


    def get_hive(self, req):
        """
        :param req:
        :type req: GetLocationRequest
        :return:
        :rtype: GetLocationResponse
        """
        res = GetLocationResponse()
        res.res = "last_seen"
        res.pose = self.hive_location
        return res

    def get_food(self, req):
        """
        :param req:
        :type req: GetLocationRequest
        :return:
        :rtype: GetLocationResponse
        """
        res = GetLocationResponse()
        if req.location == '':
            for l in self.food_locations.keys():
                if self.food_locations[l] is not None:
                    res.res = l
                    res.pose = self.food_locations[l]
                    break
        else:
            if req.location in self.food_locations.keys():
                res.res = req.location
                res.pose = self.food_locations[req.location]
        return res

    def forget_food_location(self, req):
        """
        :param req:
        :type req: ForgetLocationRequest
        :return:
        :rtype: ForgetLocationResponse
        """
        if req.location == '':
            self.food_locations = {}
        else:
            if req.location in self.food_locations.keys():
                self.food_locations[req.location] = None
        return ForgetLocationResponse()


def main():
    global transform

    rospy.init_node('fake_food_hive_detection')
    fake_food_hive_detect = FakeFoodHiveDetect()

    tf_br = tf.TransformBroadcaster()
    transform['pose'] = (0, 0, 0)
    transform['quat'] = (0, 0, 0, 1)
    transform['stamp'] = rospy.Time.now()

    r = rospy.Rate(RATE)
    while not rospy.is_shutdown():
        tf_br.sendTransform(transform['pose'],
                            transform['quat'],
                            rospy.Time.now(),
                            HIVE_FRAME,
                            WORLD_FRAME)
        r.sleep()


if __name__ == '__main__':
    main()

#!/usr/bin/env python
import rospy
import tf
import math
from ar_track_alvar_msgs.msg import AlvarMarkers
from geometry_msgs.msg import PoseStamped, Quaternion, Point
from swarming_turtles_detect.srv import *

import cv2.cv as cv

odom = "/odom"
hive = "/hive"
base_frame = "/base_link"
RATE = 20

transform = {}

markers_hive = [200, 199, 198, 197]  # front, right, back, left

MARKER_ANGS = [0, 90, 180, 270]
MARKER_VEC_TO_CENTER = [[0, 0.05], [0, 0.05], [0, 0.05], [0, 0.05]]

MAX_ANGLE = math.pi / 4.0
MAX_DIST = 1.5

last_seen = None


def quat_msg_to_array(quat):
    return [quat.x, quat.y, quat.z, quat.w]


class DetectHive:
    def __init__(self):

        self.tfListen = tf.TransformListener()
        rospy.sleep(1.0)

        self.kalman = cv.CreateKalman(3, 3, 0)
        self.kalman_state = cv.CreateMat(3, 1, cv.CV_32FC1)
        self.kalman_process_noise = cv.CreateMat(3, 1, cv.CV_32FC1)
        self.kalman_measurement = cv.CreateMat(3, 1, cv.CV_32FC1)

        self.kalman.state_pre[0, 0] = 0  # first x
        self.kalman.state_pre[1, 0] = 0  # first y
        self.kalman.state_pre[2, 0] = 0  # first theta

        # set kalman transition matrix
        self.kalman.transition_matrix[0, 0] = 1
        self.kalman.transition_matrix[1, 1] = 1
        self.kalman.transition_matrix[2, 2] = 1

        # set Kalman Filter
        cv.SetIdentity(self.kalman.measurement_matrix, cv.RealScalar(1))
        cv.SetIdentity(self.kalman.process_noise_cov, cv.RealScalar(1e-3))
        cv.SetIdentity(self.kalman.measurement_noise_cov, cv.RealScalar(1e-1))
        cv.SetIdentity(self.kalman.error_cov_post, cv.RealScalar(1))

        self.get_hive = rospy.Service('get_hive', GetLocation, self.get_hive)
        rospy.Subscriber('large_markers', AlvarMarkers, self.cb_ar_marker)

    def get_hive(self, req):
        res = GetLocationResponse()
        res.res = "last_seen"
        res.pose.header.stamp = transform['stamp']
        return res

    def get_own_pose(self):
        pose_stamped = PoseStamped()
        pose_stamped.header.stamp = rospy.Time.now()
        pose_stamped.header.frame_id = base_frame
        pose_stamped.pose.orientation.w = 1.0

        return self.transform_pose(self, pose_stamped)

    def transform_pose(self, pose_in, output_frame=odom):
        if pose_in.header.frame_id == output_frame:
            return pose_in

        if self.tfListen.frameExists(output_frame) and self.tfListen.frameExists(pose_in.header.frame_id):
            time = self.tfListen.getLatestCommonTime(pose_in.header.frame_id, output_frame)
            pose_in.header.stamp = time
            pose = self.tfListen.transformPose(output_frame, pose_in)
            return pose
        return None

    def cb_ar_marker(self, msg):
        markers_detected = []
        for marker in msg.markers:  # check all markers
            if not marker.id in markers_hive:
                continue
            m_detect = {}
            m_detect['id'] = 200 - marker.id  # position hack
            m_detect['pose'] = marker.pose
            m_detect['pose'].header = marker.header
            markers_detected.append(m_detect)
        self.calc_position(markers_detected)

    def check_distance(self, marker):
        pose = self.transform_pose(marker, output_frame=base_frame)

        if pose is None:
            return False

        quat = quat_msg_to_array(pose.pose.orientation)
        r, p, theta = tf.transformations.euler_from_quaternion(quat)

        d = pose.pose.position

        if abs(theta + math.pi / 2.0) > MAX_ANGLE:
            return False

        if math.sqrt(d.x * d.x + d.y * d.y) > MAX_DIST:
            return False

        return True

    def wrap_theta(self, previous, theta):
        if previous - theta > math.pi:
            theta += 2. * math.pi
        if previous - theta < -math.pi:
            theta -= 2. * math.pi
        return theta

    def wrap_ang(self, ang):
        while ang > 2. * math.pi:
            ang -= 2. * math.pi
        while ang < -2. * math.pi:
            ang += 2. * math.pi
        return ang

    def predict_pose(self, pose):
        quat = quat_msg_to_array(pose.pose.orientation)
        r, p, theta = tf.transformations.euler_from_quaternion(quat)

        if self.kalman.state_post[0, 0] == 0 and self.kalman.state_post[1, 0] == 0:
            self.kalman.state_post[0, 0] = pose.pose.position.x
            self.kalman.state_post[1, 0] = pose.pose.position.y
            self.kalman.state_post[2, 0] = theta

        self.kalman.state_post[2, 0] = self.wrap_ang(self.kalman.state_post[2, 0])
        theta = self.wrap_theta(self.kalman.state_post[2, 0], theta)

        kalman_prediction = cv.KalmanPredict(self.kalman)

        self.kalman_measurement[0, 0] = pose.pose.position.x
        self.kalman_measurement[1, 0] = pose.pose.position.y
        self.kalman_measurement[2, 0] = theta

        kalman_estimated = cv.KalmanCorrect(self.kalman, self.kalman_measurement)

        point = (kalman_estimated[0, 0], kalman_estimated[1, 0], kalman_estimated[2, 0])

        pose.pose.position.x = point[0]
        pose.pose.position.y = point[1]

        q = tf.transformations.quaternion_from_euler(0, 0, point[2])
        pose.pose.orientation = Quaternion(*q)

        return pose

    def calc_position(self, markers_detected):
        global transform
        # put here the prediction for multiple markers
        for marker in markers_detected:
            if not self.check_distance(marker['pose']):
                transform['stamp'] = rospy.Time.now()
                return

            pose = marker['pose']

            pose = self.transform_pose(pose)

            quat = quat_msg_to_array(pose.pose.orientation)
            r, p, theta = tf.transformations.euler_from_quaternion(quat)
            q = tf.transformations.quaternion_from_euler(0, 0, theta)
            pose.pose.orientation = Quaternion(*q)
            pose.pose.position.z = 0

            # print r,p,theta

            pose = self.predict_pose(pose)

            print pose.pose.position

            transform['pose'] = (pose.pose.position.x, pose.pose.position.y, 0)
            transform['quat'] = tuple(quat_msg_to_array(pose.pose.orientation))
            transform['stamp'] = rospy.Time.now()


def main():
    global transform
    rospy.init_node("detect_hive")
    tf_br = tf.TransformBroadcaster()
    transform['pose'] = (0, 0, 0)
    transform['quat'] = (0, 0, 0, 1)
    transform['stamp'] = rospy.Time.now()

    detect = DetectHive()

    r = rospy.Rate(RATE)
    while not rospy.is_shutdown():
        tf_br.sendTransform(transform['pose'],
                            transform['quat'],
                            rospy.Time.now(),
                            hive,
                            odom)
        r.sleep()


if __name__ == "__main__":
    main()

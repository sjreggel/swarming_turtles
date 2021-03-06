#!/usr/bin/env python
import rospy
import tf
import math
from ar_track_alvar.msg import AlvarMarkers
from geometry_msgs.msg import PoseStamped, Quaternion, Point
import cv2.cv as cv
from swarming_turtles_detect.srv import *

odom = "/odom"
hive = "/hive"
base_frame = "/base_link"
RATE = 20

food_locations = {}

markers_food = [201]

MAX_ANGLE = math.pi / 4.0
MAX_DIST = 2

def quat_msg_to_array(quat):
    return [quat.x, quat.y, quat.z, quat.w]



class DetectFood:
    def __init__(self):
        self.tfListen = tf.TransformListener()
        rospy.sleep(0.5)
        rospy.Subscriber('large_markers', AlvarMarkers, self.cb_ar_marker)
        self.food_pub = rospy.Publisher('cur_food', PoseStamped)

        self.init_kalman()

        self.get_loc = rospy.Service('get_location', GetLocation, self.get_location)
        self.forget_loc = rospy.Service('forget_location', ForgetLocation, self.forget_location)




    def init_kalman(self):
        self.kalman = cv.CreateKalman(3,3,0)
        self.kalman_state = cv.CreateMat(3,1, cv.CV_32FC1)
        self.kalman_process_noise = cv.CreateMat(3,1, cv.CV_32FC1)
        self.kalman_measurement = cv.CreateMat(3,1, cv.CV_32FC1)
        
        self.kalman.state_pre[0,0]  = 0 #first x
        self.kalman.state_pre[1,0]  = 0 #first y
        self.kalman.state_pre[2,0]  = 0 #first theta

        # set kalman transition matrix
        self.kalman.transition_matrix[0,0] = 1
        self.kalman.transition_matrix[1,1] = 1
        self.kalman.transition_matrix[2,2] = 1
        
        # set Kalman Filter
        cv.SetIdentity(self.kalman.measurement_matrix, cv.RealScalar(1))
        cv.SetIdentity(self.kalman.process_noise_cov, cv.RealScalar(1e-3))
        cv.SetIdentity(self.kalman.measurement_noise_cov, cv.RealScalar(1e-1))
        cv.SetIdentity(self.kalman.error_cov_post, cv.RealScalar(1))

        
    def get_location(self, req):
        res = GetLocationResponse()
        if req.location == '':
            for l in food_locations.keys():
                if food_locations[l] is not None:
                    res.res = l
                    res.pose = food_locations[l]
        else:
            if req.location in food_locations.keys():
                res.res = req.location
                res.pose = food_locations[req.location]
        return res
                    

    def forget_location(self, req):
        global food_locations
        if req.location == '':
            food_locations = {}
            self.init_kalman()
        else:
            if req.location in food_locations.keys():
                food_locations[req.location] = None
                self.init_kalman()
        return ForgetLocationResponse()
    
    def get_own_pose(self):
        pose_stamped = PoseStamped()
        pose_stamped.header.stamp = rospy.Time.now()
        pose_stamped.header.frame_id = base_frame
        pose_stamped.pose.orientation.w = 1.0
       
        return transformPose(self,pose_stamped)
        
        
    def transform_pose(self,pose_in, time_in = None, output_frame = hive):
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
        for marker in msg.markers: #check all markers
            if not marker.id in markers_food:
                continue
            m_detect = {}
            m_detect['name'] = str(marker.id) #position hack
            m_detect['pose'] = marker.pose
            m_detect['pose'].header = marker.header
            markers_detected.append(m_detect)
        self.calc_position(markers_detected)


    def check_distance(self, marker):
        pose = self.transform_pose(marker, output_frame = base_frame)

        if pose is None:
            return False
        
        quat = quat_msg_to_array(pose.pose.orientation)
        r,p,theta = tf.transformations.euler_from_quaternion(quat)

        d = pose.pose.position

        if abs(theta+math.pi/2.0) > MAX_ANGLE:
            return False


        if math.sqrt(d.x * d.x + d.y * d.y) > MAX_DIST:
            return False

        return True


    def wrap_theta(self, previous, theta):
        if previous - theta > math.pi:
            theta += 2.*math.pi
        if previous - theta < -math.pi:
            theta -= 2. * math.pi
        return theta

    def wrap_ang(self, ang):
        while ang > 2.*math.pi:
            ang -= 2. * math.pi
        while ang < -2.*math.pi:
            ang += 2. * math.pi
        return ang

    
    def predict_pose(self, pose):
        quat = quat_msg_to_array(pose.pose.orientation)
        r,p,theta = tf.transformations.euler_from_quaternion(quat)

        if self.kalman.state_post[0,0] == 0 and self.kalman.state_post[1,0] == 0:
            self.kalman.state_post[0,0] = pose.pose.position.x
            self.kalman.state_post[1,0] = pose.pose.position.y
            self.kalman.state_post[2,0] = theta



        self.kalman.state_post[2,0] = self.wrap_ang(self.kalman.state_post[2,0])
        theta = self.wrap_theta(self.kalman.state_post[2,0], theta)

        kalman_prediction = cv.KalmanPredict(self.kalman)

        self.kalman_measurement[0,0] = pose.pose.position.x
        self.kalman_measurement[1,0] = pose.pose.position.y
        self.kalman_measurement[2,0] = theta

        kalman_estimated = cv.KalmanCorrect(self.kalman, self.kalman_measurement)

        point  = (kalman_estimated[0,0], kalman_estimated[1,0], kalman_estimated[2,0])

        pose.pose.position.x = point[0]
        pose.pose.position.y = point[1]

        q = tf.transformations.quaternion_from_euler(0, 0, point[2])
        pose.pose.orientation = Quaternion(*q)

        pose.header.stamp = rospy.Time.now()
        
        return pose
        
    def calc_position(self,markers_detected):
        global food_locations
        #put here the prediction for multiple markers
        for marker in markers_detected:
            if not self.check_distance(marker['pose']):
                return
            pose = marker['pose']

            pose = self.transform_pose(pose)

            quat = quat_msg_to_array(pose.pose.orientation)
            r,p,theta = tf.transformations.euler_from_quaternion(quat)


            q = tf.transformations.quaternion_from_euler(0, 0, theta)

            pose.pose.position.z = 0
            pose.pose.orientation = Quaternion(*q)

            pose = self.predict_pose(pose)

            food_locations[marker['name']] = pose

            self.food_pub.publish(pose)


def main():
    rospy.init_node("detect_food")
    detect = DetectFood()

    rospy.spin()
    

if __name__ == "__main__":
    main()

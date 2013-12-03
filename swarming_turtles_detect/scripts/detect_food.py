#!/usr/bin/env python
import rospy
import tf
import math
from ar_track_alvar.msg import AlvarMarkers
from geometry_msgs.msg import PoseStamped, Quaternion, Point
import cv2.cv as cv

odom = "/odom"
hive = "/hive"
base_frame = "/base_link"
RATE = 20

food_locations = {}

markers_food = [201]

MAX_ANGLE = math.pi / 4.0
MAX_DIST = 1.5

def quat_msg_to_array(quat):
    return [quat.x, quat.y, quat.z, quat.w]



class DetectFood:
    def __init__(self):
        self.tfListen = tf.TransformListener()
        rospy.sleep(0.5)
        rospy.Subscriber('ar_pose_marker', AlvarMarkers, self.cb_ar_marker)
        self.food_pub = rospy.Publisher('cur_food', PoseStamped)

        self.kalman = cv.CreateKalman(6,3,0)
        self.kalman_state = cv.CreateMat(6,1, cv.CV_32FC1)
        self.kalman_process_noise = cv.CreateMat(6,1, cv.CV_32FC1)
        self.kalman_measurement = cv.CreateMat(3,1, cv.CV_32FC1)
        
        self.kalman.state_pre[0,0]  = 0 #first x
        self.kalman.state_pre[1,0]  = 0 #first y
        self.kalman.state_pre[2,0]  = 0 #first theta

        self.kalman.state_pre[3,0]  = 0
        self.kalman.state_pre[4,0]  = 0
        self.kalman.state_pre[5,0]  = 0

        # set kalman transition matrix
        self.kalman.transition_matrix[0,0] = 1
        self.kalman.transition_matrix[1,1] = 1
        self.kalman.transition_matrix[2,2] = 1
        self.kalman.transition_matrix[3,3] = 1
        self.kalman.transition_matrix[4,4] = 1
        self.kalman.transition_matrix[5,5] = 1

        
        # set Kalman Filter
        cv.SetIdentity(self.kalman.measurement_matrix, cv.RealScalar(1))
        cv.SetIdentity(self.kalman.process_noise_cov, cv.RealScalar(1e-3))
        cv.SetIdentity(self.kalman.measurement_noise_cov, cv.RealScalar(1e-1))
        cv.SetIdentity(self.kalman.error_cov_post, cv.RealScalar(1))
        
    def get_own_pose(self):
        pose_stamped = PoseStamped()
        pose_stamped.header.stamp = rospy.Time.now()
        pose_stamped.header.frame_id = base_frame
        pose_stamped.pose.orientation.w = 1.0
       
        return transformPose(self,pose_stamped)

        
        
    def transform_pose(self,pose_in, output_frame = hive):
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
    
    def predict_pose(self, pose):
        quat = quat_msg_to_array(pose.pose.orientation)
        r,p,theta = tf.transformations.euler_from_quaternion(quat)

        if self.kalman.state_pre[0,0] == 0 and self.kalman.state_pre[1,0] == 0:
            self.kalman.state_pre[0,0] = pose.pose.position.x
            self.kalman.state_pre[1,0] = pose.pose.position.y
            self.kalman.state_pre[2,0] = theta

            
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

        
        return pose
        
    def calc_position(self,markers_detected):
        global food_locations
        #put here the prediction for multiple markers
        for marker in markers_detected:
            if not self.check_distance(marker['pose']):
                return
            pose = marker['pose']

            quat = quat_msg_to_array(pose.pose.orientation)
            r,p,theta = tf.transformations.euler_from_quaternion(quat)
            q = tf.transformations.quaternion_from_euler(0, 0, theta)

            pose.pose.position.z = 0
            pose.pose.orientation = Quaternion(*q)
            pose = self.transform_pose(pose)

            pose = self.predict_pose(pose)

            food_locations[marker['name']] = pose

            self.food_pub.publish(pose)


def main():
    global transform
    rospy.init_node("detect_food")
    detect = DetectFood()

    rospy.spin()
    

if __name__ == "__main__":
    main()

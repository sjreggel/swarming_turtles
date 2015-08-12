#!/usr/bin/env python
import rospy
import tf
import math
from ar_track_alvar_msgs.msg import AlvarMarkers, AlvarMarker
from geometry_msgs.msg import PoseStamped, Quaternion, Point, PoseArray
import cv2.cv as cv


odom = "/odom"
#hive = "/hive"
base_frame = "/base_link"
RATE = 20
TIME_OUT = 0.2
markers = {}


def quat_msg_to_array(quat):
    return [quat.x, quat.y, quat.z, quat.w]

class FilterMarkers:
    def __init__(self):
        self.tfListen = tf.TransformListener()
        rospy.sleep(1.0)
        rospy.Subscriber('ar_pose_marker', AlvarMarkers, self.cb_ar_marker)
        self.marker_pub = rospy.Publisher('ar_pose_marker_filtered', AlvarMarkers)
        self.array_pub = rospy.Publisher('marker_poses', PoseArray)
        
    def init_kalman(self, pose):
        kalman = cv.CreateKalman(6,3,0)
        #kalman_state = cv.CreateMat(6,1, cv.CV_32FC1)
        #kalman_process_noise = cv.CreateMat(3,1, cv.CV_32FC1)

        quat = quat_msg_to_array(pose.pose.orientation)
        r,p,theta = tf.transformations.euler_from_quaternion(quat)
        
        kalman.state_post[0,0]  = pose.pose.position.x #first x
        kalman.state_post[1,0]  = pose.pose.position.y #first y
        kalman.state_post[2,0]  = theta #first theta
        kalman.state_post[3,0]  = 0 #delta x
        kalman.state_post[4,0]  = 0 #delta y
        kalman.state_post[5,0]  = 0 #delta theta
        
        # set kalman transition matrix
        kalman.transition_matrix[0,0] = 1
        #kalman.transition_matrix[0,3] = 1

        kalman.transition_matrix[1,1] = 1
        #kalman.transition_matrix[1,4] = 1
        
        kalman.transition_matrix[2,2] = 1
        #kalman.transition_matrix[2,5] = 1
       
        kalman.transition_matrix[3,3] = 1
        kalman.transition_matrix[4,4] = 1
        kalman.transition_matrix[5,5] = 1

        
        # set Kalman Filter
        cv.SetIdentity(kalman.measurement_matrix, cv.RealScalar(1))
        cv.SetIdentity(kalman.process_noise_cov, cv.RealScalar(1e-3))
        cv.SetIdentity(kalman.measurement_noise_cov, cv.RealScalar(1e-1))
        cv.SetIdentity(kalman.error_cov_post, cv.RealScalar(1))

        return kalman
        
    def transform_pose(self,pose_in, time_in = None, output_frame = odom):
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
            m_detect = {}
            m_detect['name'] = str(marker.id) #position hack
            m_detect['pose'] = marker.pose
            m_detect['pose'].header = marker.header
            markers_detected.append(m_detect)
        
        self.calc_position(markers_detected)


    def check_marker(self, marker):
        global markers
        if marker not in markers.keys():
            return False
        if (rospy.Time.now() - markers[marker]['last_seen']).to_sec() > TIME_OUT:
            del markers[marker]
            return False
        return True
    
        
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

    
    def predict_pose(self, kalman, pose):
        quat = quat_msg_to_array(pose.pose.orientation)
        r,p,theta = tf.transformations.euler_from_quaternion(quat)

        kalman.state_post[2,0] = self.wrap_ang(kalman.state_post[2,0])
        theta = self.wrap_theta(kalman.state_post[2,0], theta)

        kalman_prediction = cv.KalmanPredict(kalman)

        kalman_measurement = cv.CreateMat(3,1, cv.CV_32FC1)
        kalman_measurement[0,0] = pose.pose.position.x
        kalman_measurement[1,0] = pose.pose.position.y
        kalman_measurement[2,0] = theta

        kalman_estimated = cv.KalmanCorrect(kalman, kalman_measurement)

        point  = (kalman_estimated[0,0], kalman_estimated[1,0], kalman_estimated[2,0])

        pose.pose.position.x = point[0]
        pose.pose.position.y = point[1]

        q = tf.transformations.quaternion_from_euler(0, 0, point[2])
        pose.pose.orientation = Quaternion(*q)

        pose.header.stamp = rospy.Time.now()
        pose.header.frame_id = odom
        return pose

    def create_new_marker(self, marker):
        global markers
        name = marker['name']
        pose = marker['pose']
        pose = self.transform_pose(pose)
        markers[name] = {}
        markers[name]['last_seen'] = pose.header.stamp
        markers[name]['kalman'] = self.init_kalman(pose)
    
    def calc_position(self,markers_detected):
        global markers
        msg = AlvarMarkers()
        pose_array = PoseArray()
        #put here the prediction for multiple markers
        for marker in markers_detected:
            #print marker
            if not self.check_marker(marker['name']):
                self.create_new_marker(marker)
            pose = marker['pose']
            name = marker['name']
            
            quat = quat_msg_to_array(pose.pose.orientation)
            r,p,theta = tf.transformations.euler_from_quaternion(quat)
            q = tf.transformations.quaternion_from_euler(0, 0, theta)
            pose.pose.position.z = 0
            pose.pose.orientation = Quaternion(*q)
            pose = self.predict_pose(markers[name]['kalman'],pose)
            markers[name]['last_seen'] = rospy.Time.now()

            marker_msg = AlvarMarker()
            marker_msg.pose = pose
            marker_msg.header = pose.header
            marker_msg.id = int(marker['name'])
            msg.markers.append(marker_msg)
            pose_array.poses.append(pose.pose)
            
        pose_array.header.stamp = rospy.Time.now()
        pose_array.header.frame_id = odom
        self.array_pub.publish(pose_array)
        
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = odom
        self.marker_pub.publish(msg)

def main():
    rospy.init_node("filter_markers")
    filter_markers = FilterMarkers()

    rospy.spin()
    

if __name__ == "__main__":
    main()

#!/usr/bin/env python
import rospy
import tf
import math
from ar_track_alvar.msg import AlvarMarkers
from geometry_msgs.msg import PoseStamped, Quaternion, Point

odom = "/odom"
hive = "/hive"
base_frame = "/base_link"
RATE = 20

transform = {}

markers_hive = [200, 199, 198, 197] #front, right, back, left

MARKER_ANGS = [0, 90, 180, 270]
MARKER_VEC_TO_CENTER = [[0, 0.05],[0, 0.05],[0, 0.05],[0, 0.05]]

MAX_ANGLE = math.pi / 8.0
MAX_DIST = 0.7

def quat_msg_to_array(quat):
    return [quat.x, quat.y, quat.z, quat.w]



class DetectHive:
    def __init__(self):
 
        self.tfListen = tf.TransformListener()
        rospy.sleep(0.5)
        rospy.Subscriber('ar_pose_marker', AlvarMarkers, self.cb_ar_marker)

    def get_own_pose(self):
        pose_stamped = PoseStamped()
        pose_stamped.header.stamp = rospy.Time.now()
        pose_stamped.header.frame_id = base_frame
        pose_stamped.pose.orientation.w = 1.0
       
        return transformPose(self,pose_stamped)

        
        
    def transform_pose(self,pose_in, output_frame = odom):
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
            if not marker.id in markers_hive:
                continue
            m_detect = {}
            m_detect['id'] = 200 - marker.id #position hack
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
    
        
    def calc_position(self,markers_detected):
        global transform
        #put here the prediction for multiple markers
        for marker in markers_detected:
            if not self.check_distance(marker['pose']):
                return

            pose = marker['pose'].pose
            transform['pose'] = (pose.position.x, pose.position.y, 0)
            transform['orientation'] = tuple(quat_msg_to_array(pose.orientation))
    

def main():
    global transform
    rospy.init_node("detect_hive")
    tf_br = tf.TransformBroadcaster()
    transform['pose'] = (0,0,0)
    transform['quat'] = (0,0,0,1)
    
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

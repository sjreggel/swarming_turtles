#!/usr/bin/env python
import rospy
import tf
import math
from ar_track_alvar.msg import AlvarMarkers
from geometry_msgs.msg import PoseStamped, Quaternion
from swarming_turtles_msgs.msg import Turtle,Turtles

base_frame = "/base_link"
odom = "/odom"

output_frame = '/odom'

MARKER_ANGS = [180, 0, 0, 180, 90, 90] #left out, left in, right out, right in,center out, center in (center inside up side down)
MARKER_VEC_TO_CENTER = [[0, -0.125], [0, -0.125],[0, 0.125], [0, 0.125], [0.055, 0],[0.055, 0]]

NUM_MARKERS = 6
NUM_TURTLES = 30

def quat_msg_to_array(quat):
    return [quat.x, quat.y, quat.z, quat.w]

def get_jaw(orientation):
    quat = quat_msg_to_array(orientation)
    r,p,theta = tf.transformations.euler_from_quaternion(quat)
    #if theta < 0:
    #    theta += 2. * math.pi
    return theta


def limit_ang(ang):
    while ang < 0:
        ang += 2*math.pi
    while ang > 2*math.pi:
        ang -= math.pi
    return ang

class DetectTurtles:
    def __init__(self):
        self.tfListen = tf.TransformListener()
        self.turtle_pub = rospy.Publisher('found_turtles', Turtles)
        rospy.sleep(0.5)
        rospy.Subscriber('ar_pose_marker_filtered', AlvarMarkers, self.cb_ar_marker)
       
        
    def transform_pose(self,pose_in):
        if pose_in.header.frame_id == output_frame:
            return pose_in
        
        if self.tfListen.frameExists(output_frame) and self.tfListen.frameExists(pose_in.header.frame_id):
            time = self.tfListen.getLatestCommonTime(pose_in.header.frame_id, output_frame)
            pose_in.header.stamp = time
            pose = self.tfListen.transformPose(output_frame, pose_in)
            return pose
        return None


    def cb_ar_marker(self, msg):
        turtles = {}
        
        for marker in msg.markers: #check all markers
            turtle_id = int(marker.id / NUM_MARKERS) + 1
            if turtle_id > NUM_TURTLES:
                continue
            id_str = "tb%02d" % turtle_id
            if id_str not in turtles.keys():
                turtles[id_str] = []
            turtle = {}
            turtle['id'] = marker.id % NUM_MARKERS
            turtle['frame'] = marker.header.frame_id
            turtle['pose'] = marker.pose
            turtle['time'] = marker.header.stamp
            turtles[id_str].append(turtle)
        #print turtles
        self.calc_positions(turtles)


    def predict_center(self, marker_id, pose_in, frame_id):
        
        pose_in.header.frame_id = frame_id
              
        pose = self.transform_pose(pose_in)  #transform to output_frame
#calculate angle of other robot in baseframe
        marker_angle = get_jaw(pose.pose.orientation)
        marker_vec = MARKER_VEC_TO_CENTER[marker_id]
        angle = marker_angle + MARKER_ANGS[marker_id] * math.pi / 180.0
        angle = limit_ang(angle)

        #rotate vector poiting to center
        rotated_vec = []
        rotated_vec.append(math.cos(angle) * marker_vec[0] - math.sin(angle) * marker_vec[1])
        rotated_vec.append(math.sin(angle) * marker_vec[0] + math.cos(angle) * marker_vec[1])


        #calculate center
        center = PoseStamped()
        center.pose.position.x = pose.pose.position.x + rotated_vec[0]
        center.pose.position.y = pose.pose.position.y + rotated_vec[1]

        #orientation
        q = tf.transformations.quaternion_from_euler(0, 0, angle)
        orientation = Quaternion(*q)
        center.pose.orientation = orientation
        center.header.frame_id = output_frame
        #print center, angle/math.pi * 180

        return center


    def check_distance(self, average, new_pose):
        #disallow too far away
        return True
    
    
    def average_pose(self, average, new_pose, count):
        if average is None:
            return new_pose
        elif not self.check_distance(average, new_pose):
            return None
        else:
            average.pose.position.x = (count-1.0)/count * average.pose.position.x + 1.0 / count * new_pose.pose.position.x 
            average.pose.position.y = (count-1.0)/count * average.pose.position.y + 1.0 / count * new_pose.pose.position.y 
            new_jaw = (count-1.0)/count * get_jaw(average.pose.orientation) + 1.0 / count * get_jaw(new_pose.pose.orientation)
            quat = tf.transformations.quaternion_from_euler(0,0,new_jaw)
            average.pose.orientation = Quaternion(*quat)

            return average
                                                      
            
    def calc_positions(self,turtles):
        turtle_poses = []
        for key in turtles.keys():
            avg_pose = None
            count = 0
            
            for marker in turtles[key]:
                center = self.predict_center(marker['id'], marker['pose'], marker['frame'])
                count += 1
                avg_pose = self.average_pose(avg_pose, center, count)
                avg_pose.header.stamp = marker['time']
                if avg_pose is None:
                    break

            if avg_pose is not None:
                turtle = {}
                turtle['name'] = key
                turtle['pose'] = avg_pose
                
                turtle_poses.append(turtle)

        self.publish_turtles(turtle_poses)

    def publish_turtles(self, turtles):
        msg = Turtles()
        for t in turtles:
            turtle = Turtle()
            turtle.name = t['name']
            turtle.position = t['pose']
            msg.turtles.append(turtle)
        self.turtle_pub.publish(msg)

def main():
    rospy.init_node("detect_turtles")
    detect = DetectTurtles()
    rospy.spin()


if __name__ == '__main__':
    main()


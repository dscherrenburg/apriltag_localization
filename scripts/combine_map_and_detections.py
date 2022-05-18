#!/usr/bin/python

from hashlib import new
from genpy import Duration
import rospy
import tf
import yaml
import rosparam
import tf.transformations as tft
import numpy as np

from gazebo_msgs.msg import ModelStates
from apriltag_ros.msg import AprilTagDetectionArray
from geometry_msgs.msg import Pose, TransformStamped, Transform
from tf.msg import tfMessage
from quaternion_avg import averageQuaternions

class Tag:
    def __init__(self, id, world_pose=None, det_buffer_size=10, max_time_diff=rospy.Duration(0.2)):
        self.id = id
        self.world_pose = world_pose
        self.latest_detection = None
        self.detections = []
        self.det_buffer_size = det_buffer_size
        self.max_time_diff = max_time_diff
    
    def detected(self, detection):
        """adds the latest detection to the buffer and checks if the buffer is full"""
        if self.latest_detection is not None and (rospy.get_rostime().to_sec() - self.latest_detection.to_sec()) > 0.2:     #check if the time difference is too big and clear the buffer
            self.detections = []
            # rospy.loginfo("Detection buffer cleared" + " Tag: " + str(self.id))
            
        self.latest_detection = rospy.get_rostime()      #update the latest detection time
        self.detections.append(detection)                   #add the detection to the buffer
        if len(self.detections) > self.det_buffer_size:     #if the buffer is too big, remove the oldest detection
            self.detections = self.detections[-self.det_buffer_size:]
        # rospy.loginfo(str(self))
    
    def moving_avg(self):
        """calculates the moving average of the detections. Returns the average pose or none if the buffer is empty"""
        # if self.latest_detection is not None and self.check_timediff(rospy.get_rostime()):     #check if the time difference is too big and clear the buffer
        #     self.detections = []
        #     return None
        
        sum_x, sum_y, sum_z, q = 0, 0, 0, []
        for tf in self.detections:
            sum_x += tf[0][0]
            sum_y += tf[0][1]
            sum_z += tf[0][2]
            q.append([tf[1][3], tf[1][0], tf[1][1], tf[1][2]])
            # sum_rx = tf[1][0]
            # sum_ry = tf[1][1]
            # sum_rz = tf[1][2]
            # sum_rw = tf[1][3]
        
        moving_avg = ([0, 0, 0], [0, 0, 0, 0, 0])
        moving_avg[0][0] = sum_x / len(self.detections)
        moving_avg[0][1] = sum_y / len(self.detections)
        moving_avg[0][2] = sum_z / len(self.detections)

        # calculate the average of the quaternions in the buffer
        q_avg = averageQuaternions(np.matrix(q))

        moving_avg[1][3] = q_avg[0]
        moving_avg[1][0] = q_avg[1]
        moving_avg[1][1] = q_avg[2]
        moving_avg[1][2] = q_avg[3]

        return moving_avg
        
    
    # def check_timediff(self, time):
    #     return time - self.latest_detection > self.max_time_diff
    

    def __str__(self):
        return "Detected Tag " + str(self.id) + "-> Time: " + str(self.latest_detection) + " at " + str(self.detections[-1].transform.translation)


class Robot:
    def __init__(self):
        self.position = None


class VisualLocalization:
    def __init__(self, moving_avg_len=5, buffer_len=10):
        rospy.loginfo("Open tag location yaml file")
        rospy.loginfo(rosparam.list_params("apriltag_localization"))
        self.world_loc_tags = rosparam.get_param('apriltag_localization/tags')
        rospy.loginfo(self.world_loc_tags)
        
        
        self.tags = {'tag_0': Tag(0, self.world_loc_tags['tag_0']), 
                     'tag_1': Tag(1, self.world_loc_tags['tag_1']), 
                     'tag_2': Tag(2, self.world_loc_tags['tag_2']), 
                     'tag_3': Tag(3, self.world_loc_tags['tag_3']), 
                     'tag_4': Tag(4, self.world_loc_tags['tag_4']), 
                     'tag_5': Tag(5, self.world_loc_tags['tag_5']), 
                     'tag_6': Tag(6, self.world_loc_tags['tag_6']), 
                     'tag_7': Tag(7, self.world_loc_tags['tag_7']), 
                     'tag_8': Tag(8, self.world_loc_tags['tag_8']), 
                     'tag_9': Tag(9, self.world_loc_tags['tag_9'])
                     }
        rospy.loginfo("Created tag objects")
        
        self.robot = Robot()
        rospy.loginfo("Create robot object")
        
        # get location of tags from simulation
        # self.model_states = rospy.wait_for_message('gazebo/model_states', ModelStates)
        # self.apriltag_poses = {'tag_' + str(int(name[-3:])) : self.model_states.pose[i] for i, name in enumerate(self.model_states.name) if 'Apriltag' in name}

        # subscibers
        # self.tag_detections_sub = rospy.Subscriber('/tag_detections', AprilTagDetectionArray, self.detection_callback)
        # self.transform_sub = rospy.Subscriber('/tf', tfMessage, self.transformer_callback)
        self.transform_listener = tf.TransformListener() 
        
        # tf publisher
        self.broadcaster = tf.TransformBroadcaster()
        
        while not rospy.is_shutdown():
            for tag in self.tags:
                try:
                    t = self.transform_listener.getLatestCommonTime('base_link', tag)
                    t_now = rospy.Time().now()
                    if t_now.to_sec() - t.to_sec() < 0.3:
                        self.transform_listener.waitForTransform('base_link', tag, rospy.Time(0), rospy.Duration(0.3))
                        tf_odom_to_tag = self.transform_listener.lookupTransform('base_link', tag, rospy.Time(0))
                        
                        self.tags[tag].detected(tf_odom_to_tag)
                        rospy.loginfo(str(self.tags[tag].detections) + '\n')
                        moving_avg_tag = self.tags[tag].moving_avg()
                        
                        world_to_odom = self.calculate_world_position(tag, moving_avg_tag)
                        
                        # publish world position estimation
                        new_tf = TransformStamped()
                        new_tf.header.stamp = rospy.Time.now()
                        new_tf.header.frame_id = "map"
                        new_tf.child_frame_id = "world_position"
                        
                        new_tf.transform.translation.x = world_to_odom[0][3]
                        new_tf.transform.translation.y = world_to_odom[1][3]
                        new_tf.transform.translation.z = world_to_odom[2][3]
                        new_tf.transform.rotation.w = 1.0
                        
                        self.broadcaster.sendTransformMessage(new_tf)
                        
                        new_tf_tag = TransformStamped()
                        new_tf_tag.header.stamp = rospy.Time.now()
                        new_tf_tag.header.frame_id = "map"
                        new_tf_tag.child_frame_id = tag + "_test"
                        tag_obj = self.tags[tag]
                        new_tf_tag.transform.translation.x = tag_obj.world_pose['x']
                        new_tf_tag.transform.translation.y = tag_obj.world_pose['y']
                        new_tf_tag.transform.translation.z = tag_obj.world_pose['z']
                        q = tft.quaternion_from_euler(tag_obj.world_pose['qx'], tag_obj.world_pose['qy'], tag_obj.world_pose['qz'], 'rxyz')
                        new_tf_tag.transform.rotation.w = q[3]
                        new_tf_tag.transform.rotation.x = q[0]
                        new_tf_tag.transform.rotation.y = q[1]
                        new_tf_tag.transform.rotation.z = q[2]
                        
                        self.broadcaster.sendTransformMessage(new_tf_tag)
                        
                        
                        
                except tf.LookupException or tf.Exception:
                    continue
                
        
        # set for all tags and dictionary for latest tag transforms
        self.all_tags = set()
        self.latest_tag_transforms = {}
        self.buffer_len = buffer_len

    def calculate_world_position(self, tag_name, tf_odom_to_tag):
        """calculates the map position of the robot in the map frame"""
        tag = self.tags[tag_name]       # get the tag object
        
        # calculate rotation matrix from quaternion
        translation_mat_world_to_tag = tft.euler_matrix(tag.world_pose['qx'], tag.world_pose['qy'], tag.world_pose['qz'], 'rxyz')
        translation_mat_odom_to_tag = tft.quaternion_matrix(tf_odom_to_tag[1])
    
        # add the translation to the rotation matrix to get the transformation matrix
        translation_mat_world_to_tag[0][3] = tag.world_pose['x']
        translation_mat_world_to_tag[1][3] = tag.world_pose['y']
        translation_mat_world_to_tag[2][3] = tag.world_pose['z']

        translation_mat_odom_to_tag[0][3] = tf_odom_to_tag[0][0]
        translation_mat_odom_to_tag[1][3] = tf_odom_to_tag[0][1]
        translation_mat_odom_to_tag[2][3] = tf_odom_to_tag[0][2]
        
        # get the inverse of the odom to tag transformation matrix
        translation_mat_tag_to_odom = tft.inverse_matrix(translation_mat_odom_to_tag)

        # calculate the translation matrix from the world to odom
        translation_mat_world_to_odom = np.matmul(translation_mat_world_to_tag, translation_mat_tag_to_odom)
        
        
        # rospy.loginfo("World to tag matrix: " + str(translation_mat_world_to_tag) + "\n\n")
        # rospy.loginfo("Odom to tag matrix: " + str(translation_mat_odom_to_tag) + "\n\n")
        # rospy.loginfo("Tag to odom matrix: " + str(translation_mat_tag_to_odom) + "\n\n")
        # rospy.loginfo("\n" + str(tag_name) + "  -  World to Odom matrix: " + "\n" + str(translation_mat_world_to_odom) + "\n")
        
        return translation_mat_world_to_odom
    

    def transformer_callback(self, tf_msgs):    
        """Called every time the tf topic is published. Saves the latest transform of each visible tag in self.latest_tag_transforms."""
        for tf_msg in tf_msgs.transforms:
            if 'xtion_rgb_optical_frame' in tf_msg.header.frame_id:
                self.tags[tf_msg.child_frame_id].detected(tf_msg)
            


    def get_tag_transform(self, tag_id):
        """returns the latest transform of the tag with the given id"""
        return self.latest_tag_transforms[tag_id]['transform'][-1]


    def get_tag_latest_update_time(self, tag_id):
        """Returns the last time a tag was updated."""
        return self.latest_tag_transforms[tag_id]['last_updated']


    def get_tag_to_world(self, tag_id):
        """Returns the tag pose relative to the world."""
        for tag in self.latest_tag_transforms:
            pass


    def get_robot_to_tag(self):
        """Returns the robot pose relative to the tag."""
        pass


    def get_robot_to_world(self):
        """Returns the robot pose relative to the world."""
        pass



if __name__ == '__main__':
    rospy.init_node('tag_localization')
    rospy.loginfo("Start up node")
    tag_localization = VisualLocalization()
    
    while not rospy.is_shutdown():

        rospy.spin()
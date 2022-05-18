#!/usr/bin/python
import rospy
import tf
import rosparam
import tf.transformations as tft
import numpy as np

from geometry_msgs.msg import Pose, TransformStamped, Transform
from quaternion_avg import averageQuaternions

class Tag:
    """Creates a Tag with information needed for calculations"""
    def __init__(self, id, world_pose=None, max_time_diff=0.2, buffer_size=10):
        """id: the number of the tag_id
           world_pose: the position and orientation in the world
           max_time_diff: the maximum time a detection is kept in the buffer
           buffer_size: the maximum number of detections per tag in the buffer"""

        self.id = id
        self.world_pose = world_pose
        self.latest_detection = None
        self.detections = []
        self.max_time_diff = max_time_diff
        self.buffer_size = buffer_size
    
    def detected(self, detection):
        """adds the latest detection to the buffer and checks if the buffer is full
           detection: the transform of the detection of this specific tag"""

        if self.latest_detection is not None and (rospy.get_rostime().to_sec() - self.latest_detection.to_sec()) > self.max_time_diff:     #check if the time difference is too big and clear the buffer
            self.detections = []
            rospy.loginfo("Detection buffer cleared" + " Tag: " + str(self.id))
            
        self.latest_detection = rospy.get_rostime()         #update the latest detection time
        self.detections.append(detection)                   #add the detection to the buffer
        if len(self.detections) > self.buffer_size:     #if the buffer is too big, remove the oldest detection
            self.detections = self.detections[-self.buffer_size:]
    
    def moving_avg(self):
        """calculates the moving average of the detections. Returns the average pose or none if the buffer is empty"""
        
        sum_x, sum_y, sum_z, q = 0, 0, 0, []

        for tf in self.detections:
            sum_x += tf[0][0]
            sum_y += tf[0][1]
            sum_z += tf[0][2]
            q.append([tf[1][3], tf[1][0], tf[1][1], tf[1][2]])

        # calculate the moving average of the detections
        moving_avg = ([0, 0, 0], [0, 0, 0, 0, 0])
        moving_avg[0][0] = sum_x / len(self.detections)
        moving_avg[0][1] = sum_y / len(self.detections)
        moving_avg[0][2] = sum_z / len(self.detections)

        # calculate the average of the quaternions in the detections
        q_avg = averageQuaternions(np.matrix(q))

        moving_avg[1][3] = q_avg[0]
        moving_avg[1][0] = q_avg[1]
        moving_avg[1][1] = q_avg[2]
        moving_avg[1][2] = q_avg[3]

        return moving_avg
    
    def __str__(self):
        return "Detected Tag " + str(self.id) + "-> Time: " + str(self.latest_detection) + " at " + str(self.detections[-1].transform.translation)


class Robot:
    def __init__(self):
        self.position = None


class VisualLocalization:
    """Class for the calculation of the position of the robot in the world,
       using the position of the tags and the transforms from the robot to the tags."""
    def __init__(self):
        # Imports the location of the tags in the world
        self.world_loc_tags = rosparam.get_param('apriltag_localization/tags')
        rospy.loginfo("Opened tag location yaml file")
        
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
        
        # subscibers
        self.transform_listener = tf.TransformListener() 
        
        # publishers
        self.broadcaster = tf.TransformBroadcaster()
        
    def run_localization(self):
        """ Runs the localization algorithm """
        for tag in self.tags:
            tf_odom_to_tag = self.get_tf_robot_to_tag(tag, 'odom')
            if tf_odom_to_tag is None:
                continue
            
            self.tags[tag].detected(tf_odom_to_tag)
            rospy.loginfo(str(self.tags[tag].detections) + '\n')
            moving_avg = self.tags[tag].moving_avg()
            
            world_to_odom = self.calculate_world_position(tag, moving_avg)
            
            # publish world position estimation and ground truth of visible tags
            self.publish_tf_map_to_tag(tag)
            self.publish_tf_map_to_robot(world_to_odom)
    
    
    def get_tf_robot_to_tag(self, tag, robot_frame):
        """Returns the transform from the robot_frame to the tag_frame
           tag:         Tag object
           robot_frame: the frame in which you want to calculate the transform to the tag"""
        try:
            t = self.transform_listener.getLatestCommonTime(robot_frame, tag)
            t_now = rospy.Time().now()
            if t_now.to_sec() - t.to_sec() < 0.3:
                self.transform_listener.waitForTransform(robot_frame, tag, rospy.Time(0), rospy.Duration(0.3))
                tf_robot_to_tag = self.transform_listener.lookupTransform(robot_frame, tag, rospy.Time(0))
                return tf_robot_to_tag
            else:
                rospy.loginfo(tag + ": Latest available transform is to old!")
                return None
        except tf.LookupException or tf.Exception:
            rospy.loginfo(tag + ": Could not get the transform!")
            return None

    def publish_tf_map_to_robot(self, position_transform):
        """Publishes the position_transform from the map to the robot as a TransformMessage
           position_transform: the transform of the robot in the map frame"""
        new_tf = TransformStamped()
        new_tf.header.stamp = rospy.Time.now()
        new_tf.header.frame_id = "map"
        new_tf.child_frame_id = "world_position"
        
        new_tf.transform.translation.x = position_transform[0][3]
        new_tf.transform.translation.y = position_transform[1][3]
        new_tf.transform.translation.z = position_transform[2][3]
        new_tf.transform.rotation.w = 1.0
        
        self.broadcaster.sendTransformMessage(new_tf)
        
    def publish_tf_map_to_tag(self, tag_name):
        """Publishes the transform from the map to the tag_name that is inserted
           tag_name: Name of the tag; e.g. 'tag_0'"""
        new_tf_tag = TransformStamped()
        new_tf_tag.header.stamp = rospy.Time.now()
        new_tf_tag.header.frame_id = "map"
        new_tf_tag.child_frame_id = tag + "_test"
        tag_obj = self.tags[tag_name]
        
        new_tf_tag.transform.translation.x = tag_obj.world_pose['x']
        new_tf_tag.transform.translation.y = tag_obj.world_pose['y']
        new_tf_tag.transform.translation.z = tag_obj.world_pose['z']
        q = tft.quaternion_from_euler(tag_obj.world_pose['qx'], tag_obj.world_pose['qy'], tag_obj.world_pose['qz'], 'rxyz')
        new_tf_tag.transform.rotation.w = q[3]
        new_tf_tag.transform.rotation.x = q[0]
        new_tf_tag.transform.rotation.y = q[1]
        new_tf_tag.transform.rotation.z = q[2]
        
        self.broadcaster.sendTransformMessage(new_tf_tag)
    
    def calculate_world_position(self, tag_name, tf_odom_to_tag):
        """calculates the position and orientation of the robot in the map frame
           tag_name: Name of the tag used for calculation
           tf_odom_to_tag: The transform from odom to tag (created with def get_tf_robot_to_tag)"""
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
    



if __name__ == '__main__':
    rospy.init_node('tag_localization')
    rospy.loginfo("Start up node")
    tag_localization = VisualLocalization()
    rate = rospy.Rate(60)
    while not rospy.is_shutdown():
        tag_localization.run_localization()
        rate.sleep()
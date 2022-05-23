#!/usr/bin/python
import rospy
import tf
import rosparam
import tf.transformations as tft
import localization_transforms as lt
import numpy as np
import utility


from geometry_msgs.msg import TransformStamped, Transform, PoseWithCovarianceStamped
from quaternion_avg import averageQuaternions

class Tag:
    """Creates a Tag with information needed for calculations"""
    def __init__(self, id, world_pose=None, max_time_diff=0.3, buffer_size=20):
        """id: the number of the tag_id
           world_pose: the position and orientation in the world
           max_time_diff: the maximum time a detection is kept in the buffer
           buffer_size: the maximum number of detections per tag in the buffer
           """

        self.id = id
        self.world_pose = world_pose
        self.latest_detection = None
        self.detections = []
        self.max_time_diff = max_time_diff
        self.buffer_size = buffer_size
        self.latest_position_estimate = None

    def valid_latest_detection(self):
        if self.latest_detection is None:
            return False
        return (rospy.get_rostime().to_sec() - self.latest_detection.to_sec()) < self.max_time_diff
    
    def detected(self, detection):
        """
        adds the latest detection to the buffer and checks if the buffer is full
        detection: the transform of the detection of this specific tag
        """
        if not self.valid_latest_detection():               #check if the time difference is too big and clear the buffer
            self.detections = []
            
        self.latest_detection = rospy.get_rostime()         #update the latest detection time
        self.detections.append(detection)                   #add the detection to the buffer
        
        if len(self.detections) > self.buffer_size:         #if the buffer is too big, remove the oldest detection
            self.detections = self.detections[-self.buffer_size:]

    @staticmethod
    def median_outlier_filter(points, max_error=0.1):
        n_params = len(points[0])

        lsts = []
        filtered_lsts = []
        medians = []

        for point in points:
            for i in range(n_params):
                if i >= len(lsts):
                    lsts.append([point[i]])
                else:
                    lsts[i].append(point[i])
        
        for lst in lsts:
            lst.sort()
            medians.append(lst[int(len(lst)/2)])

        for point in points:
            for i in range(n_params):
                if abs(medians[i] - point[i]) < max_error:
                    if i >= len(filtered_lsts):
                        filtered_lsts.append([point[i]])
                    else:
                        filtered_lsts[i].append(point[i])
                else:
                    break
            
            for i in range(len(filtered_lsts)):
                if len(filtered_lsts[i]) > len(filtered_lsts[-1]):
                    rospy.loginfo(f"List {i} longer then last list:   \n List {filtered_lsts[i]} \n last list: {filtered_lsts[-1]}")
                    rospy.loginfo(f"Deleting last item from list: {filtered_lsts[i][-1]}")

                    del filtered_lsts[i][-1]
        
        rospy.loginfo("Filtered lists:   " + str(filtered_lsts))

        return filtered_lsts        


    
    def moving_avg(self, max_error=0.05):
        """
        calculates the moving average of the detections. Returns the average pose or none if the buffer is empty
        """
        
        sum_z, q, list_x, list_y, filtered_x, filtered_y, points = 0, [], [], [], [], [], []

        for tf in self.detections:
            # list_x.append(tf[0][0])
            # list_y.append(tf[0][1])
            # sum_z += tf[0][2]
            points.append([tf[0][0], tf[0][1], tf[0][2]])
            q.append([tf[1][3], tf[1][0], tf[1][1], tf[1][2]])
        
        filtered = Tag.median_outlier_filter(points)
        filtered_x, filtered_y, filtered_z = filtered
        
        # if len(list_x) > 0:
        #     list_x.sort()
        #     list_y.sort()
        #     median_x = list_x[int(len(list_x)/2)]
        #     median_y = list_y[int(len(list_y)/2)]

        # for i in range(len(list_x)):
        #     err_x = abs(list_x[i] - median_x)
        #     err_y = abs(list_y[i] - median_y)
        #     if err_x < max_error and err_y < max_error:
        #         filtered_x.append(list_x[i])
        #         filtered_y.append(list_y[i])
        #     else:
        #         rospy.loginfo("Measurement deleted from moving average. Value: " + str((list_x[i], list_y[i])))

        # calculate the moving average of the detections
        moving_avg = ([0, 0, 0], [0, 0, 0, 0])
        moving_avg[0][0] = sum(filtered_x) / len(filtered_x)
        moving_avg[0][1] = sum(filtered_y) / len(filtered_y)
        moving_avg[0][2] = sum(filtered_z) / len(filtered_z)
        rospy.loginfo("Moving average: " + str((moving_avg[0][0], moving_avg[0][1], moving_avg[0][2])))

        # calculate the average of the quaternions in the detections
        q_avg = averageQuaternions(np.matrix(q))

        moving_avg[1][3] = q_avg[0]
        moving_avg[1][0] = q_avg[1]
        moving_avg[1][1] = q_avg[2]
        moving_avg[1][2] = q_avg[3]

        return moving_avg
    
    def __str__(self):
        return "Detected Tag " + str(self.id) + "-> Time: " + str(self.latest_detection) + " at " + str(self.detections[-1].transform.translation)
    
    def get_tf_msg(self):
        new_tf_tag = TransformStamped()
        new_tf_tag.header.stamp = rospy.Time.now()
        new_tf_tag.header.frame_id = "map"
        new_tf_tag.child_frame_id = "tag" + str(self.id) + "_test"
        
        new_tf_tag.transform.translation.x = self.world_pose['x']
        new_tf_tag.transform.translation.y = self.world_pose['y']
        new_tf_tag.transform.translation.z = self.world_pose['z']
        q = tft.quaternion_from_euler(self.world_pose['qx'], self.world_pose['qy'], self.world_pose['qz'], 'rxyz')
        new_tf_tag.transform.rotation.w = q[3]
        new_tf_tag.transform.rotation.x = q[0]
        new_tf_tag.transform.rotation.y = q[1]
        new_tf_tag.transform.rotation.z = q[2]
        return new_tf_tag

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
        self.pose_publisher = rospy.Publisher("/initialpose", PoseWithCovarianceStamped, queue_size=10)
        
        self.position_estimator = lt.RobotPositionEstimator()

    def run_localization(self):
        """ Runs the localization algorithm. """
        for tag in self.tags:
            tf_odom_to_tag = self.get_tf_robot_to_tag(tag, 'base_footprint')
            if tf_odom_to_tag is None:
                continue
             
            self.tags[tag].detected(tf_odom_to_tag)
            moving_avg = self.tags[tag].moving_avg()
            
            world_to_odom = self.calculate_world_position(tag, moving_avg)
            
            # self.tags[tag].latest_position_estimate = lt.SimpleRobotPosition.from_tuple_of_lists(world_to_odom)
            # combined_world_to_odom = self.combining_visible_tags()
            
            # publish world position estimation and ground truth of visible tags
            self.publish_tf_map_to_tag(tag)
            self.publish_tf_map_to_robot(world_to_odom)


    def calculate_world_position(self, tag_name, tf_odom_to_tag):
        """calculates the position and orientation of the robot in the map frame \n
        args:
            tag_name: Name of the tag used for calculation
            tf_odom_to_tag: The transform from odom to tag (created with def get_tf_robot_to_tag)
            """
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
    
        return translation_mat_world_to_odom
    
    # NOT USED RIGHT NOW
    def combining_visible_tags(self):
        position_estimates = []
        
        for tag in self.tags:
            tag_obj = self.tags[tag]
            if tag_obj.valid_latest_detection():
                if tag_obj.latest_position_estimate is not None:
                    position_estimates.append(tag_obj.latest_position_estimate)
        
        final_position_estimate = self.position_estimator.estimate_position(position_estimates)
        
        return final_position_estimate
    
    def get_tf_robot_to_tag(self, tag, robot_frame):
        """
        Returns the transform from the robot_frame to the tag_frame \n
        args:
            tag:         Tag object
            robot_frame: the frame in which you want to calculate the transform to the tag
        """
        try:
            t = self.transform_listener.getLatestCommonTime(robot_frame, tag)
            t_now = rospy.Time().now()
            if t_now.to_sec() - t.to_sec() < 0.3:
                self.transform_listener.waitForTransform(robot_frame, tag, rospy.Time(0), rospy.Duration(0.3))
                tf_robot_to_tag = self.transform_listener.lookupTransform(robot_frame, tag, rospy.Time(0))
                return tf_robot_to_tag
            else:
                return None
        except tf.LookupException or tf.Exception:
            return None
    
    
    def publish_tf_map_to_robot(self, transformation_matrix):
        """
        Publishes the transform from the map frame to the robot frame. Takes a transformation matrix \n
        args:
            transformation_matrix: The transformation matrix from the map frame to the robot frame
        """
        transform_msg = utility.from_matrix_to_transform_stamped(transformation_matrix, "map", "world_position")
        self.broadcaster.sendTransformMessage(transform_msg)


    def publish_tf_map_to_tag(self, tag_name):
        """
        Publishes the transform from the map to the tag_name that is inserted \n
        args:
            tag_name: Name of the tag; e.g. 'tag_0'
        """
        tag_obj = self.tags[tag_name]
        tf_tag_msg = tag_obj.get_tf_msg()
        self.broadcaster.sendTransformMessage(tf_tag_msg)
    
    

if __name__ == '__main__':
    rospy.init_node('tag_localization')
    rospy.loginfo("Start up node")
    tag_localization = VisualLocalization()
    rate = rospy.Rate(60)
    while not rospy.is_shutdown():
        tag_localization.run_localization()
        rate.sleep()
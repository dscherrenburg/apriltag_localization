#!/usr/bin/python
import rospy
import tf
import rosparam
import tf.transformations as tft
import numpy as np
import utility


from geometry_msgs.msg import TransformStamped, PoseWithCovarianceStamped
from utility import averageQuaternions
from gazebo_msgs.msg import ModelStates

class Tag:
    """Creates a Tag with information needed for calculations"""
    def __init__(self, id, world_pose=None, max_time_diff=0.3, buffer_size=20):
        """
        -   id: the number of the tag_id
        -   world_pose: the position and orientation in the world
        -   max_time_diff: the maximum time a detection is kept in the buffer
        -   buffer_size: the maximum number of detections per tag in the buffer
        """
        self.id = id
        self.world_pose = world_pose
        self.latest_detection = None
        self.detections = []
        self.max_time_diff = max_time_diff
        self.buffer_size = buffer_size
        self.latest_position_estimate = None
        self.latest_distance_to_tag = None

    def valid_latest_detection(self):
        """Checks the latest detection has not expired"""   
        if self.latest_detection is None:
            return False
        return (rospy.get_rostime().to_sec() - self.latest_detection.to_sec()) < self.max_time_diff
    
    def detected(self, detection):
        """
        Adds the latest detection to the buffer and checks if the buffer is full
        -   detection: the transform of the detection of this specific tag
        """
        if not self.valid_latest_detection():               #check if the time difference is too big and clear the buffer
            self.detections = []
            
        self.latest_detection = rospy.get_rostime()         #update the latest detection time
        self.detections.append(detection)                   #add the detection to the buffer
        
        if len(self.detections) > self.buffer_size:         #if the buffer is too big, remove the oldest detection
            self.detections = self.detections[-self.buffer_size:]

    @staticmethod
    def median_outlier_filter(points, max_error=0.1):
        """Deletes the outliers from the list of points.
        -   points: list of points (each point is a list of translations)
        -   max_error: the maximum distance from the median"""
        
        n_params = len(points[0]) 
        lsts, filtered_lsts, medians = [], [], []

        # Looping through points and adding x, y, z to separate lists
        for point in points:
            for i in range(n_params):
                if i >= len(lsts):
                    lsts.append([point[i]])
                else:
                    lsts[i].append(point[i])
        
        # Looping through parameter lists, sorting them to determine the median
        for lst in lsts:
            lst.sort()
            medians.append(lst[int(len(lst)/2)])

        # Looping through points to check if the error from the median is too big. If so it deletes the entire point.
        for point in points:
            for i in range(n_params):
                if i == len(filtered_lsts):
                        filtered_lsts.append([])
                
                if abs(medians[i] - point[i]) < max_error:
                    filtered_lsts[i].append(point[i])
                else:
                    rospy.loginfo("Error larger than allowed sow deleting the point.")
                    break
            
            for i in range(len(filtered_lsts)):
                if len(filtered_lsts[i]) > len(filtered_lsts[-1]):
                    del filtered_lsts[i][-1]
        
        # To make sure that the number of lists is always the same
        while len(filtered_lsts) < n_params:
            filtered_lsts.append([])
        
        # If there where no valid points it returns just the points without filtering
        if len(filtered_lsts[0]) == 0:
            filtered_lsts = lsts
        
        # rospy.loginfo("Filtered lists:   " + str(filtered_lsts))

        return filtered_lsts        

    def moving_avg(self, max_error=0.1):
        """
        calculates the moving average of the detections. Returns the average pose or none if the buffer is empty.
        -   max_error: the maximum error for the median outlier filter
        """
        # Generates seperate lists with the positions and with the orientation in quaternions
        position, q = [], []
        for tf in self.detections:
            position.append([tf[0][0], tf[0][1], tf[0][2]])
            q.append([tf[1][3], tf[1][0], tf[1][1], tf[1][2]])
        
        # Filters the outliers
        filtered = Tag.median_outlier_filter(position, max_error)
        filtered_x, filtered_y, filtered_z = filtered

        # calculate the moving average of the detections
        moving_avg = ([0, 0, 0], [0, 0, 0, 0])
        if len(filtered_x) == 0:
            return 0
        moving_avg[0][0] = sum(filtered_x) / len(filtered_x)
        moving_avg[0][1] = sum(filtered_y) / len(filtered_y)
        moving_avg[0][2] = sum(filtered_z) / len(filtered_z)

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
    def __init__(self, tag_combination_mode="weighted_average", filter_mode="median_outlier", buffer_size=10, max_time_diff=0.2, max_error=0.1):
        """
        -   tag_combination_mode: method for the combining of the estimations from seperate tags. ("weighted average", "average")
        -   filter_mode: method for filtering outliers. ("median_outlier")
        """
        self.tag_combination_mode = tag_combination_mode
        self.buffer_size = buffer_size
        self.max_time_diff = max_time_diff
        self.max_error = max_error
        
        # Imports the location of the tags in the world
        self.world_loc_tags = rosparam.get_param('apriltag_localization/tags')
        rospy.loginfo("Opened tag location yaml file")
        
        # self.model_states = rospy.wait_for_message('gazebo/model_states', ModelStates)
        # self.apriltag_poses = {'tag_' + str(int(name[-3:])) : self.model_states.pose[i] for i, name in enumerate(self.model_states.name) if 'Apriltag' in name}
        # rospy.loginfo(self.apriltag_poses)
        
        self.tags = {'tag_1': Tag(1, self.world_loc_tags['tag_1'], max_time_diff=self.max_time_diff, buffer_size=self.buffer_size),
                     'tag_3': Tag(3, self.world_loc_tags['tag_3'], max_time_diff=self.max_time_diff, buffer_size=self.buffer_size),
                     'tag_4': Tag(4, self.world_loc_tags['tag_4'], max_time_diff=self.max_time_diff, buffer_size=self.buffer_size),
                     'tag_5': Tag(5, self.world_loc_tags['tag_5'], max_time_diff=self.max_time_diff, buffer_size=self.buffer_size),
                     'tag_7': Tag(7, self.world_loc_tags['tag_7'], max_time_diff=self.max_time_diff, buffer_size=self.buffer_size),
                     'tag_8': Tag(8, self.world_loc_tags['tag_8'], max_time_diff=self.max_time_diff, buffer_size=self.buffer_size),
                     'tag_9': Tag(9, self.world_loc_tags['tag_9'], max_time_diff=self.max_time_diff, buffer_size=self.buffer_size),
                     'tag_10': Tag(10, self.world_loc_tags['tag_10'], max_time_diff=self.max_time_diff, buffer_size=self.buffer_size), 
                     'tag_11': Tag(11, self.world_loc_tags['tag_11'], max_time_diff=self.max_time_diff, buffer_size=self.buffer_size),
                     'tag_12': Tag(12, self.world_loc_tags['tag_12'], max_time_diff=self.max_time_diff, buffer_size=self.buffer_size),
                     'tag_13': Tag(13, self.world_loc_tags['tag_13'], max_time_diff=self.max_time_diff, buffer_size=self.buffer_size),
                     'tag_15': Tag(15, self.world_loc_tags['tag_15'], max_time_diff=self.max_time_diff, buffer_size=self.buffer_size),
                     'tag_16': Tag(16, self.world_loc_tags['tag_16'], max_time_diff=self.max_time_diff, buffer_size=self.buffer_size),
                     'tag_17': Tag(17, self.world_loc_tags['tag_17'], max_time_diff=self.max_time_diff, buffer_size=self.buffer_size),
                     'tag_18': Tag(18, self.world_loc_tags['tag_18'], max_time_diff=self.max_time_diff, buffer_size=self.buffer_size)
                     }
        rospy.loginfo("Created tag objects")
        
        # subscibers
        self.transform_listener = tf.TransformListener() 

        # publishers
        self.broadcaster = tf.TransformBroadcaster()
        self.pose_publisher = rospy.Publisher("/initialpose", PoseWithCovarianceStamped, queue_size=10)

    def run_localization(self):
        """ Runs the localization algorithm. """
        for tag in self.tags:
            tf_odom_to_tag = self.get_tf_robot_to_tag(tag, 'base_footprint')
            if tf_odom_to_tag is None:
                continue
             
            self.tags[tag].detected(tf_odom_to_tag)
            moving_avg = self.tags[tag].moving_avg(max_error=self.max_error)
            
            world_to_odom = self.calculate_world_position(tag, moving_avg)
            self.tags[tag].latest_distance_to_tag = utility.distance_robot_tag(moving_avg)
            self.tags[tag].latest_position_estimate = world_to_odom
            
            combined_world_to_odom = self.combining_visible_tags()
            
            self.publish_tf_map_to_tag(tag)
            self.publish_tf_map_to_robot(combined_world_to_odom)    


    def calculate_world_position(self, tag_name, tf_odom_to_tag):
        """ Calculates the position and orientation of the robot in the map frame
        -    tag_name: name of the tag used for calculation
        -    tf_odom_to_tag: the transform from odom to tag (created with def get_tf_robot_to_tag)
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

    
    def combining_visible_tags(self):
        """ Combines the position of the tags using an average. \n"""
        position_estimates, distances = [], []
        
        for tag in self.tags:
            tag_obj = self.tags[tag]
            if tag_obj.valid_latest_detection():
                if tag_obj.latest_position_estimate is not None:
                    distances.append(tag_obj.latest_distance_to_tag)
                    position_estimates.append(tag_obj.latest_position_estimate)
        
        if len(position_estimates) == 0:
            return
        
        translations, rotations = [], []
        for position in position_estimates:
            translation, rotation = utility.from_matrix_to_tuple(position)
            translations.append(translation)
            rotations.append(list(rotation))

        if self.tag_combination_mode == "weighted_average":
            weights = utility.distance_weights(distances)
            avg_x = sum([translations[i][0]*weights[i] for i in range(len(translations))])
            avg_y = sum([translations[i][1]*weights[i] for i in range(len(translations))])
            avg_z = sum([translations[i][2]*weights[i] for i in range(len(translations))])
        elif self.tag_combination_mode == "average":
            avg_x = sum([t[0] for t in translations]) / len(translations)
            avg_y = sum([t[1] for t in translations]) / len(translations)
            avg_z = sum([t[2] for t in translations]) / len(translations)
        else:
            rospy.loginfo("############# Not a valid tag_combination_mode #############")
        
        avg_q = averageQuaternions(np.matrix(rotations))
        avg_tf_matrix = tft.quaternion_matrix(avg_q)
        avg_tf_matrix[0][3] = avg_x
        avg_tf_matrix[1][3] = avg_y
        avg_tf_matrix[2][3] = avg_z
        
        return avg_tf_matrix
    
    
    def get_tf_robot_to_tag(self, tag, robot_frame):
        """
        Returns the transform from the robot_frame to the tag_frame
        -    tag:         Tag object
        -    robot_frame: the frame in which you want to calculate the transform to the tag
        """
        try:
            t = self.transform_listener.getLatestCommonTime(robot_frame, tag)
            t_now = rospy.Time().now()
            if t_now.to_sec() - t.to_sec() < self.max_time_diff:
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
        transform_msg = utility.from_matrix_to_transform_stamped(transformation_matrix, "map", "world_location")
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
    
    localization_method = rospy.get_param("~localization_method")
    buffer_size = rospy.get_param("~buffer_size")
    max_time_diff = rospy.get_param("~max_time_diff")
    max_error = rospy.get_param("~max_error")
    
    rospy.loginfo("\nStarted localization with parameters: \n" + \
                  "Localization method: " + localization_method + "\n" + \
                  "Buffer size: " + str(buffer_size) + "\n" + \
                  "Max time diff: " + str(max_time_diff) + "\n" + \
                  "Max error: " + str(max_error))
    
    
    tag_localization = VisualLocalization(tag_combination_mode=localization_method, 
                                          buffer_size=buffer_size, 
                                          max_time_diff=max_time_diff,
                                          max_error=max_error)
    
    rate = rospy.Rate(60)
    while not rospy.is_shutdown():
        tag_localization.run_localization()
        rate.sleep()
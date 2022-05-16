#!/usr/bin/python

from os import TMP_MAX
import rospy
import tf

from gazebo_msgs.msg import ModelStates
from apriltag_ros.msg import AprilTagDetectionArray
from geometry_msgs.msg import Pose, TransformStamped
from tf.msg import tfMessage


class Tag:
    def __init__(self, id, world_pose=None, det_buffer_size=10, max_time_diff=rospy.Duration(0.1)):
        self.id = id
        self.world_pose = world_pose
        self.latest_detection = None
        self.detections = []
        self.det_buffer_size = det_buffer_size
        self.max_time_diff = max_time_diff
    
    def detected(self, detection):
        """adds the latest detection to the buffer and checks if the buffer is full"""
        if self.check_timediff(detection.header.stamp):     #check if the time difference is too big and clear the buffer
            self.detections = []
            
        self.latest_detection = detection.header.stamp      #update the latest detection time
        self.detections.append(detection)                   #add the detection to the buffer
        if len(self.detections) > self.det_buffer_size:     #if the buffer is too big, remove the oldest detection
            self.detections = self.detections[-self.det_buffer_size:]
        rospy.loginfo(str(self))
        
    
    def check_timediff(self, time):
        return time - self.latest_detection > self.max_time_diff
    

    def __str__(self):
        return "Detected Tag " + str(self.id) + "-> Time: " + str(self.latest_detection) + " at " + str(self.detections[-1].transform.translation)


class VisualLocalization:

    def __init__(self, moving_avg_len=5, buffer_len=10):
        self.tags = {'tag_0': Tag(0), 
                     'tag_1': Tag(1), 
                     'tag_2': Tag(2), 
                     'tag_3': Tag(3), 
                     'tag_4': Tag(4), 
                     'tag_5': Tag(5), 
                     'tag_6': Tag(6), 
                     'tag_7': Tag(7), 
                     'tag_8': Tag(8), 
                     'tag_9': Tag(9)
                     }

        # get location of tags from simulation
        self.model_states = rospy.wait_for_message('gazebo/model_states', ModelStates)
        self.apriltag_poses = {name : self.model_states.pose[i] for i, name in enumerate(self.model_states.name) if 'Apriltag' in name}

        # subscibers
        self.tag_detections_sub = rospy.Subscriber('/tag_detections', AprilTagDetectionArray, self.detection_callback)
        self.transform_sub = rospy.Subscriber('/tf', tfMessage, self.transformer_callback)
        
        # transformer
        self.transformer = tf.TransformerROS()

        # set for all tags and dictionary for latest tag transforms
        self.all_tags = set()
        self.latest_tag_transforms = {}
        self.buffen_len = buffer_len

        # ---OLD---
        self.tag_history = {}
        self.moving_avg_len = moving_avg_len


    def transformer_callback(self, tf_msgs):
        """Called every time the tf topic is published. Saves the latest transform of each visible tag in self.latest_tag_transforms."""
        for tf_msg in tf_msgs.transforms:
            self.transformer.setTransform(tf_msg) # add transform message to transformer

            if 'tag' in tf_msg.child_frame_id:          # filter for only tag frames
                self.all_tags.add(tf_msg.child_frame_id)
                
                self.tags[tf_msg.child_frame_id].detected(tf_msg)
                
                # save the latest transform of each tag and update the latest update time. If the list of transforms is to long it removes the oldest transforms.
                if tf_msg.child_frame_id not in self.latest_tag_transforms:
                    self.latest_tag_transforms[tf_msg.child_frame_id] = {'last_updated': tf_msg.header.stamp.to_sec(), 'transform': [tf_msg.transform]}
                else:
                    self.latest_tag_transforms[tf_msg.child_frame_id]['transform'].append(tf_msg.transform)
                    self.latest_tag_transforms[tf_msg.child_frame_id]['last_updated'] = tf_msg.header.stamp.to_sec()
                    
                    if len(self.latest_tag_transforms[tf_msg.child_frame_id]['transform']) > self.buffen_len:
                        self.latest_tag_transforms[tf_msg.child_frame_id]['transform'] = self.latest_tag_transforms[tf_msg.child_frame_id]['transform'][-self.buffen_len:]


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


    def detection_callback(self, data):
        """Called every time the tag_detection topic is published. Saves the tag_detection message in self.tag_history. 
        Then calculates the moving average of each tag.
        """

        # save the latest relative position of each vissible tag
        for tag in data.detections:
            id = tag.id[0]
            if id not in self.tag_history:
                self.tag_history[id] = []
            self.tag_history[id].append(tag.pose.pose.pose)
            if len(self.tag_history[id]) > self.moving_avg_len:     
                self.tag_history[id] = self.tag_history[id][-self.moving_avg_len:]
        
        # calculate the moving average
        new_moving_avg = self.moving_avg()
      
        self.calculate_robot_pose_per_tag(new_moving_avg, self.apriltag_poses['Apriltag36_11_00009'])

        
    def moving_avg(self):
        """calculates the moving average of each tag and returns a dicionary of Pose message types"""
        avgs = {}
        for tag_id in self.tag_history:
            tag_pos = self.tag_history[tag_id]
            avg_x = sum([pos.position.x for pos in tag_pos]) / self.moving_avg_len
            avg_y = sum([pos.position.y for pos in tag_pos]) / self.moving_avg_len
            avg_z = sum([pos.position.z for pos in tag_pos]) / self.moving_avg_len
            avg_rot_w = sum([pos.orientation.w for pos in tag_pos]) / self.moving_avg_len
            avg_rot_x = sum([pos.orientation.x for pos in tag_pos]) / self.moving_avg_len
            avg_rot_y = sum([pos.orientation.y for pos in tag_pos]) / self.moving_avg_len
            avg_rot_z = sum([pos.orientation.z for pos in tag_pos]) / self.moving_avg_len

            avg_pose = Pose()
            avg_pose.position.x = avg_x
            avg_pose.position.y = avg_y
            avg_pose.position.z = avg_z
            avg_pose.orientation.w = avg_rot_w
            avg_pose.orientation.x = avg_rot_x
            avg_pose.orientation.y = avg_rot_y
            avg_pose.orientation.z = avg_rot_z

            avgs[tag_id] = avg_pose
        return avgs

    def calculate_robot_pose_per_tag(self, tf_tag_world, tf_tag_robot):
        """Calculates the robot pose relative to the world."""
        tf_world_robot = Pose()
        tf_world_robot.position.x = tf_tag_world.position.x - tf_tag_robot.position.x
        tf_world_robot.position.y = tf_tag_world.position.y - tf_tag_robot.position.y
        tf_world_robot.position.z = tf_tag_world.position.z - tf_tag_robot.position.z

        #rospy.loginfo(tf_world_robot)



if __name__ == '__main__':
    rospy.init_node('tag_localization')
    tag_localization = VisualLocalization()

    while not rospy.is_shutdown():

        rospy.spin()

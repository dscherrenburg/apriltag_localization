#!/usr/bin/python

import rospy
import tf

from gazebo_msgs.msg import ModelStates
from tf.msg import tfMessage


class TagToWorld():
    def __init__(self):
        self.model_states = rospy.wait_for_message('gazebo/model_states', ModelStates)
        self.apriltag_poses = {'tag_' + str(int(name[-3:])) : self.model_states.pose[i] for i, name in enumerate(self.model_states.name) if 'Apriltag' in name}
        
        self.broadcaster = tf.TransformBroadcaster()
        self.publishrate = rospy.Rate(5)
        while  not rospy.is_shutdown():
            for tag in self.apriltag_poses:
                self.broadcaster.sendTransform(translation=TagToWorld.pos_msg_to_tuple(self.apriltag_poses[tag].position),
                                                rotation=TagToWorld.rot_msg_to_tuple(self.apriltag_poses[tag].orientation),
                                                time=rospy.get_rostime(),
                                                child=tag,
                                                parent='world')
            rospy.loginfo("Published transforms")
            self.publishrate.sleep()
    
    @staticmethod
    def pos_msg_to_tuple(position_msg):
        """Converts a geometry_msgs/Point message to a tuple"""
        return (position_msg.x, position_msg.y, position_msg.z)
    
    @staticmethod
    def rot_msg_to_tuple(rotation_msg):
        """Converts a geometry_msgs/Quaternion message to a tuple"""
        return (rotation_msg.x, rotation_msg.y, rotation_msg.z, rotation_msg.w)
        

if __name__ == '__main__':
    rospy.init_node('tag_to_world_tf_publisher')
    tag_localization = TagToWorld()
    while not rospy.is_shutdown():
        rospy.spin()
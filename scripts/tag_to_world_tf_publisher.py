#!/usr/bin/python

import rospy
import tf

from gazebo_msgs.msg import ModelStates
from tf.msg import tfMessage


class TagToWorldBroadcaster():
    def __init__(self):
        self.model_states = rospy.wait_for_message('gazebo/model_states', ModelStates)
        self.apriltag_poses = {'tag_' + str(int(name[-3:])) : self.model_states.pose[i] for i, name in enumerate(self.model_states.name) if 'Apriltag' in name}
        rospy.loginfo(self.apriltag_poses)
        
        self.broadcaster = tf.TransformBroadcaster()
        
        self.broadcaster.sendTransform(translation=self.apriltag_poses['tag_0'].position,
                                       rotation=self.apriltag_poses['tag_0'].orientation,
                                       time=rospy.get_rostime(),
                                       child='tag_0',
                                       parent='world')
        
        
        

if __name__ == '__main__':
    rospy.init_node('tag_to_world_tf_publisher')
    tag_localization = TagToWorldBroadcaster()
    while not rospy.is_shutdown():
        rospy.spin()
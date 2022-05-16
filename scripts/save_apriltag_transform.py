#!/usr/bin/python

#ros service to give transforms from tag to world

import rospy
import rosservice

from gazebo_msgs.msg import ModelStates, ModelState

# def get_gazebo_apriltag_model_state():
#     rospy.wait_for_service('/gazebo/get_model_state')
#     get_model_state = rospy.ServiceProxy('/gazebo/get_model_state')
#     rospy.loginfo(get_model_state)

def callback(data):
    apriltag_poses = {name : data.pose[i] for i, name in enumerate(data.name) if 'Apriltag' in name}
    rospy.loginfo(apriltag_poses)
    # get_gazebo_apriltag_model_state()

def listener_transforms():
    rospy.init_node('gazebo_transform_listener')
    rospy.Subscriber('/gazebo/model_states', ModelStates, callback)
    rospy.spin()
    

if __name__ == '__main__':
    listener_transforms()
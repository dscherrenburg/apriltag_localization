#!/usr/bin/python

import rospy
import tf
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry

class FilteredOdomPublisher:
    def __init__(self, frame_id, child_frame_id):
        self.broadcaster = tf.TransformBroadcaster()
        self.subscriber = rospy.Subscriber('/odometry/filtered', Odometry, self.callback)
        
        self.frame_id = frame_id
        self.child_frame_id = child_frame_id
    
    def callback(self, data):
        self.broadcaster.sendTransform((data.pose.pose.position.x, data.pose.pose.position.y, data.pose.pose.position.z),
                                       (data.pose.pose.orientation.x, data.pose.pose.orientation.y, data.pose.pose.orientation.z, data.pose.pose.orientation.w),
                                       data.header.stamp,
                                       self.child_frame_id,
                                       self.frame_id)

if __name__ == "__main__":
    rospy.init_node('filtered_odom_publisher')
    filtered_odom_publisher = FilteredOdomPublisher("map", "filtered_location")
    rate = rospy.Rate(20)
    while not rospy.is_shutdown():
        rate.sleep()